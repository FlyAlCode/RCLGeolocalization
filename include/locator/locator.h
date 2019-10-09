#ifndef RCLL_LOCATOR_H_
#define RCLL_LOCATOR_H_

#include <memory>
#include "cross_point.h"
#include "sampler.h"
#include "similarity_evaluator.h"
#include "estimator.h"
#include "matcher.h"
#include "cross_ratio.h"

namespace rcll{
struct LocatorParam{
    double max_sample_distance;                             // MapPtSampler
    double max_inliner_distance;                            // within this distance, a point is treated as a inliner 
    double threshold_diatance;                              // min distance for two requry_pts
    double cos_angle_distance;                              // 
    double inliner_rate_threshold;                          // min inliner rate for a success location
    double min_inliner_point_num;                           // minimun inliner points number for a success location
    int max_iterate_num;                                    // the max try num 
    int max_requry_sample_try_num;                          // max sample number for requry points
    int min_requry_pt_num;                                  // min requry points to perform location
    double max_cross_ratio_relative_error;                  // threshold for performing cross ratio check
    double max_tangent_error;                               // the max allowed tangent error for a cross point in query image to be allowed to generate a hypothesis
    
    double query_location_success_possibility;              // the possibility for finding the transformation with a sample query tuple
    double query_location_confidence;
    double location_success_confidence;                    // 
    
    double query_sample_min_diatance;
    double query_sample_max_distance;
    double query_sample_min_cos_angle_distance;
};

class Locator{
public:
    
    void Initilize(const std::vector<CrossPointPtr> &reference_map, 
                   const LocatorParam &param );
    
    // return value: -1 no enough cross points; 0 fail; 1 success
    int Locate(const std::vector<CrossPointPtr> &requry_pts,
                cv::Mat &best_H,
                std::vector<CrossPointPtr> &best_matching_points,
                double &best_similarity);
    
    // used just for debug
    void SetRequryImg(const cv::Mat &img);
    void SetMapImg(const cv::Mat &img);
    const cv::Mat & GetQueryImg() {return requry_img_;}
    const cv::Mat & GetMapImg() {return map_img_;}
    
private:
    std::unique_ptr<HomographyEstimator> H_estimator_;
    std::unique_ptr<RequryPtSampler> requry_sampler_;
    std::unique_ptr<MapPtSampler> map_sampler_;
    std::unique_ptr<SimilarityEvaluator> similarity_evaluator_;
    std::unique_ptr<Matcher> matcher_;
    
    LocatorParam locator_params_; 
    
    std::vector<int> priority_index_;
    
    // used just for debug
    /* kind=0 --- map 
     * kind=1 --- requry 
     */
    void ShowSample(const std::string &window_name, 
                    const std::vector<CrossPointPtr> &samplers, 
                    int kind=0,
                    int wait_time = 0);
    cv::Mat requry_img_;
    cv::Mat map_img_;
};

}   // namespace rcll

#endif
