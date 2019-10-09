#ifndef RCLL_SIMILARITY_EVALUATOR_H_
#define RCLL_SIMILARITY_EVALUATOR_H_

#include <vector>
#include <opencv2/core/core.hpp>
#include "cross_point.h"
#include "cross_point_tree.h"

namespace rcll{
class SimilarityEvaluator{
public:
    /* Set reference cross_points_ and build a search_tree_
     */
    void Initilize(const std::vector<CrossPointPtr> &reference_pts);

    /* Evaluate the similarity between requry points set and reference points set under 
     * given transformation models, and return the best matching model and similarity.
     * input:
     *   requry_points --- the requry cross point set
     *   H_candidates --- the candidate models
     *   threshold --- a max distance for a points to regard as a inliner
     * output:
     *   H --- the model with the max similarity
     *   matching_points --- the matching points, if no matching point, then the corresponding points is set to NULL
     */
    double Evaluate(const std::vector<CrossPointPtr> &requry_points, 
                    const std::vector<cv::Mat> &H_candidates, 
                    const double threshold, 
                    cv::Mat &H,
                    std::vector<CrossPointPtr> &matching_points);
    
private:
    /* 1. Transform the requry_point to destination coordinate with H
     * 2. Search for points within threshold using search_tree_
     * 3. Check whether a point with the same type as requry_point exists
     */
    bool IsInliner(const CrossPointPtr &requry_point, 
                   const cv::Mat &H, 
                   const double threshold, 
                   CrossPointPtr &matching_point);
    
    std::unique_ptr<RoadMapTree> search_tree_;
    std::vector<CrossPointPtr> cross_points_;
};    
    

}   // namespace rcll

#endif