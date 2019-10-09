#include "similarity_evaluator.h"
// #include <functional>
// #include <algorithm>
// #include <ctime>

namespace rcll{
void SimilarityEvaluator::Initilize(const std::vector< CrossPointPtr >& reference_pts){
    cross_points_ = reference_pts;
    
    // Get all the center point to build the search tree 
    std::vector<cv::Point2d> pt_centers;
    for(int i=0; i<reference_pts.size(); i++){
        pt_centers.push_back(reference_pts[i]->get_center());
    }
    
    search_tree_.reset(new RoadMapTree);
    search_tree_->BuildKDTree(pt_centers);
}

// 可以通过选储存每一种可能转换下的相似度对应的数据，
// 最后再进行相似度比较，来简化时间复杂度，代价时增大了空间复杂度
double SimilarityEvaluator::Evaluate(const std::vector< CrossPointPtr >& requry_points, 
                                     const std::vector< cv::Mat >& H_candidates, 
                                     const double threshold, 
                                     cv::Mat& H, 
                                     std::vector<CrossPointPtr> &matching_points){
    matching_points.clear();
    
    cv::Mat H_best;
    double similarity_best = -1;
    double similarity_current;
    std::vector<CrossPointPtr> current_matching_point_set;
    for(int i=0; i<H_candidates.size(); i++){
       CrossPointPtr current_matching_point;
       similarity_current = 0;
       current_matching_point_set.clear();
       
        for(int j=0; j<requry_points.size(); j++){
            if(IsInliner(requry_points[j], H_candidates[i], threshold, current_matching_point)){        // check matching point exist,避免多个点映射到一个点
                bool matching_point_existed = false;
                for(int k=0; k<current_matching_point_set.size();k++){
                    if(current_matching_point!=nullptr && current_matching_point_set[k]!=nullptr
                        && current_matching_point->get_id() == current_matching_point_set[k]->get_id()){
                        matching_point_existed = true;
                        break;
                    }
                }
                
                if(!matching_point_existed)
                    ++similarity_current;
                else
                    current_matching_point = nullptr;
            }    
            current_matching_point_set.push_back(current_matching_point);
        }
        
        if(similarity_current>similarity_best){
            similarity_best = similarity_current;
            H_candidates[i].copyTo(H_best);
            matching_points = current_matching_point_set;
        }
    }
    H_best.copyTo(H);
    return double(similarity_best);
}


bool SimilarityEvaluator::IsInliner(const CrossPointPtr& requry_point, 
                                    const cv::Mat& H, 
                                    const double threshold, 
                                    CrossPointPtr &matching_point){
    // 1. Transform requry_point to destination coordination
    cv::Point2d center = requry_point->get_center();
    cv::Mat center_homograph(3,1,CV_64F);
    center_homograph.at<double>(0,0) = center.x;
    center_homograph.at<double>(1,0) = center.y;
    center_homograph.at<double>(2,0) = 1;
    
    cv::Mat transformed_center_homograph = H * center_homograph;
    cv::Point2d transformed_center;
    double x = transformed_center_homograph.at<double>(0,0);
    double y = transformed_center_homograph.at<double>(1,0);
    double z = transformed_center_homograph.at<double>(2,0);
    transformed_center.x = x / z;
    transformed_center.y = y / z;
    
    // 2. search for points within threshold
    std::vector<cv::Point2d> requry_pts;
    requry_pts.push_back(transformed_center);
    
    std::vector<std::vector<std::pair<size_t,double> > > ret_matches;
    search_tree_->RadiusSearch(requry_pts,
                               threshold * threshold,
                               ret_matches );
    // 3. check type
    CrossPointPtr transformed_point_tmp;
    for(int i=0; i<ret_matches[0].size(); i++){
        transformed_point_tmp = cross_points_[ ret_matches[0][i].first ];
        if(transformed_point_tmp->get_braches_num() == requry_point->get_braches_num() 
           && transformed_point_tmp->get_tangents_num() == requry_point->get_tangents_num()){
            matching_point = transformed_point_tmp;
            return true;
        }     
    }
    
    matching_point = nullptr;
    return false;
}

}   // namespace rcll
