#ifndef RCLL_MATCHER_H_
#define RCLL_MATCHER_H_
#include <opencv2/core/core.hpp>
#include "cross_point.h"
#include "cross_ratio.h"

namespace rcll{
    
struct LinePair{
    std::vector<cv::Point3d> first_line_set_;
    std::vector<cv::Point3d> second_line_set_;
};

class Matcher{
public:
    explicit Matcher(double cross_ratio_threshold);
    bool Match(const std::vector<CrossPointPtr> &requry_pts, 
               const std::vector<CrossPointPtr> &reference_pts, 
               std::vector<LinePair> &match_lines);
    
private:
    /* Calculate the likely matching lines 
     * input:
     *   requry_pt -- the requry point 
     *   reference_pt -- the reference point
     *   requry_base_line -- the tangents of connection line between the first requry_pt and second requry_pt
     *   reference_base_line -- the tangents of connection line between the first reference_pt and second reference_pt
     * output:
     *   result -- all possible line matching
     */
    bool MatchOnePoint(const CrossPointPtr &requry_pt, 
                       const CrossPointPtr &reference_pt, 
                       const cv::Point2d &requry_base_line, 
                       const cv::Point2d &reference_base_line, 
                       std::vector<LinePair> &result);
    
    void SortTangentsFromBaseLine(const CrossPointPtr &pt, 
                                  const cv::Point2d &base_line, 
                                  std::vector<cv::Point2d> &clockwise_sorted_lines);
    
    /* Given a point on the line, and the tangent of the line,
    * estimate the equation of the line.
    * input:
    *   point --- the point on the line
    *   tangent --- the tangent of the line
    * return:
    *   line --- the params of the line(Ax+By+C=0)
    */
    cv::Point3d EstimateLine(const cv::Point2d &point, 
                  const cv::Point2d &tangent);

    double cross_ratio_threshold_;                      // relative error for cross ratio
};

}   // namespace rcll

#endif


/* 说明：任何情况下，对于一个点，仅仅存在两种可能的线匹配方案，这是因为，一旦两个点的连线确定下来，
 * 其余的线由于相对于连线的先后顺序数保持不变的，因此就仅仅存在“顺时针”和“逆时针”两种匹配方案
 */