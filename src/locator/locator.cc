#include "locator.h"
#include <cmath>
#include <opencv2/calib3d/calib3d.hpp>       
#include <opencv2/highgui/highgui.hpp>

// used just for debug
#include <glog/logging.h> 
#include <time.h>
#include <opencv2/imgproc/imgproc.hpp>

namespace rcll{
    
void Locator::Initilize(const std::vector< CrossPointPtr >& reference_map, 
                        const LocatorParam& param){
    // 1. Initilize sampler
    requry_sampler_.reset(new RequryPtSampler());
    
    map_sampler_.reset(new MapPtSampler());
    map_sampler_->Initialize(reference_map,
                             param.cos_angle_distance,
                             param.max_sample_distance);
    priority_index_ = map_sampler_->GetSamplePriorityIndex();
    
    // 2. Initilize estimator
    H_estimator_.reset(new HomographyEstimator);
    
    // 3. Initialize similarity evaluator
    similarity_evaluator_.reset(new SimilarityEvaluator);
    similarity_evaluator_->Initilize(reference_map);
    
    // 4. Save all the params
    locator_params_ = param;
}

void Locator::SetRequryImg(const cv::Mat& img){
    img.copyTo(requry_img_);
}

void Locator::SetMapImg(const cv::Mat& img){
    img.copyTo(map_img_);
}



int Locator::Locate(const std::vector< CrossPointPtr >& requry_pts, 
                     cv::Mat &best_H,
                     std::vector<CrossPointPtr> &best_matching_points,
                     double &best_similarity ){  
    best_similarity = 0;
    best_matching_points.clear();
    
    if(requry_pts.size()<locator_params_.min_requry_pt_num)
        return -1;
    
    cv::Mat current_H;  
    std::vector<CrossPointPtr> current_matching_points;
    double current_similarity = 0;
    double current_consistent_num = 0;
    int run_round = 0;
    
    std::vector<CrossPointPtr> sample_map_pts;
    std::vector<cv::Mat> HMats;
    
    ////////////////////////////
    std::vector<cv::Point3d> query_tangents;
    std::vector<cv::Point3d> map_tangnets;
    
    double p_fail_sum = 1 - locator_params_.location_success_confidence;
    double p_fail_map = 1 - locator_params_.query_location_confidence;
    double p_query_success = locator_params_.query_location_success_possibility;
    int min_query_sample_num = std::log10(p_fail_sum)/
                                            std::log10(1 - p_query_success + p_query_success * p_fail_map);             // remain to calculate
    int query_sample_num = 0;
    
    // debug
    clock_t start, end;
    // start = clock();
    requry_sampler_->Initilize(requry_pts, 
                               priority_index_,
                               locator_params_.query_sample_min_diatance,
                               locator_params_.query_sample_max_distance,
                               locator_params_.query_sample_min_cos_angle_distance,
                               locator_params_.max_tangent_error);
    // end = clock();
    // LOG(INFO) << "\n\n@@@@@@@@\nquery_sampler_ initilize time: "<<(end - start)/(CLOCKS_PER_SEC/1000.0)<<" ms\n";
    
    CrossPtTuple query_sample_data;
    int total_round_num = 0;
    while(query_sample_num <= min_query_sample_num){
        // 1. sample query points 
        // start = clock();
        int result = requry_sampler_->Sample(query_sample_data);
               
        if(result == -1)
            return 0;                       // all datas have been sample with no correct datas, which means location fails
        if(result == 0)
            continue;
        
        CrossPtTuple map_sample_data;
        int match_type;
        int min_map_sample_num = 100;           
        int map_sample_num = 0;
        while(map_sample_num++ < min_map_sample_num){
            // 2. sample map points
            start = clock();
            int current_match_num = map_sampler_->Sample(query_sample_data,
                                                         locator_params_.max_cross_ratio_relative_error,
                                                         map_sample_data,
                                                         match_type,
                                                         !(map_sample_num-1));
            end = clock();
            
            // LOG(INFO) << "map sample time: "<<(end - start)/(CLOCKS_PER_SEC/1000.0)<<" ms\n";
            
            if(current_match_num == 0)
                break;
            
            if(map_sample_num==1){
                {
                    std::vector<CrossPointPtr> query_sample_pts = query_sample_data.get_pts();
                }
                min_map_sample_num = current_match_num * locator_params_.query_location_confidence;
            }
                
            
            // 3. calculate model
            start = clock();
            map_sample_data.GetSortedTangents(0, map_tangnets);
            query_sample_data.GetSortedTangents(match_type, query_tangents);
            
            cv::Mat tmp_H = H_estimator_->findHomography(query_tangents, map_tangnets);
            end = clock();
           
            if(tmp_H.empty())
                continue;
            
            HMats.clear();
            HMats.push_back(tmp_H);
       
            // 4. similarity evaluate
            // start = clock();
            current_consistent_num = similarity_evaluator_->Evaluate(requry_pts, 
                                                                 HMats, 
                                                                 locator_params_.max_inliner_distance, 
                                                                 current_H,
                                                                 current_matching_points);
            current_similarity = current_consistent_num / requry_pts.size();
            
            if(current_similarity>best_similarity){                 
                // local optimistic
                if(current_consistent_num>4){
                    std::vector<cv::Point2d> query_inlier_pts;
                    std::vector<cv::Point2d> reference_inlier_pts;
                    cv::Mat status;
                    for(int i=0; i<current_matching_points.size(); ++i){
                        if(current_matching_points[i]!=nullptr){
                            query_inlier_pts.push_back(requry_pts[i]->get_center());
                            reference_inlier_pts.push_back(current_matching_points[i]->get_center());
                        }
                    }
                    
                    cv::Mat optimistic_current_H = cv::findHomography(query_inlier_pts, 
                                                                      reference_inlier_pts, 
                                                                      status, 
                                                                      CV_RANSAC,
                                                                      locator_params_.max_inliner_distance/2);
                    if(!optimistic_current_H.empty()) {
                        std::vector<cv::Mat> optimistic_current_H_set;
                        optimistic_current_H_set.push_back(optimistic_current_H);
                        current_consistent_num = similarity_evaluator_->Evaluate(requry_pts, 
                                                                             optimistic_current_H_set, 
                                                                             locator_params_.max_inliner_distance, 
                                                                             current_H,
                                                                             current_matching_points);
                        current_similarity = current_consistent_num / requry_pts.size();
                       
                    } 
                }
                
                if(current_similarity>best_similarity){
                    best_similarity = current_similarity;
                    current_H.copyTo(best_H);
                    best_matching_points = current_matching_points;
                }     
            }
            
            if(best_similarity>=locator_params_.inliner_rate_threshold && 
                current_consistent_num>=locator_params_.min_inliner_point_num){
                return 1;
            }  
            
            ++total_round_num;
            if(total_round_num>locator_params_.max_iterate_num){
                return 0;                
            }

            // end = clock();
            // std::cout<<(end - start)/(CLOCKS_PER_SEC/1000.0)<<" ms passed"<<std::endl;
            // LOG(INFO) << "similarity evaluate time: "<<(end - start)/(CLOCKS_PER_SEC/1000.0)<<" ms\n";
        }
        
        ++query_sample_num;
    }
    return 0; 
}


void Locator::ShowSample(const std::string& window_name, 
                            const std::vector< CrossPointPtr> &samplers, 
                            int kind, 
                            int wait_time){
    cv::Mat result_img;
    if(kind==0)
        map_img_.copyTo(result_img);
    else
        requry_img_.copyTo(result_img);
    if(samplers.size()>=2){
        samplers[0]->Draw(result_img, cv::Scalar(255,0,0));
        samplers[1]->Draw(result_img, cv::Scalar(255,255,0));
    }
    std::cout<<"Press any key to continue..."<<std::endl;
    cv::resize(result_img, result_img, cv::Size(1000, 1000));
    cv::imshow(window_name, result_img);
    cv::waitKey(wait_time);
}


    
}   // namespace rcll
