#include "matcher.h"

namespace rcll{
Matcher::Matcher(double cross_ratio_threshold){
    cross_ratio_threshold_ = cross_ratio_threshold;
}

bool Matcher::Match(const std::vector< CrossPointPtr >& requry_pts, 
                    const std::vector< CrossPointPtr >& reference_pts, 
                    std::vector< LinePair >& match_lines){
    match_lines.clear();
    
    cv::Point2d requry_base_line = requry_pts[0]->get_center() 
                                    -requry_pts[1]->get_center();
    cv::Point2d reference_base_line = reference_pts[0]->get_center() 
                                        - reference_pts[1]->get_center();
    
    std::vector<LinePair> first_match_result, second_match_result;                                    
    bool first_pt_match = MatchOnePoint(requry_pts[0], 
                                        reference_pts[0], 
                                        requry_base_line, 
                                        reference_base_line, 
                                        first_match_result);
    if(!first_pt_match)
        return false;
    
    bool second_pt_match = MatchOnePoint(requry_pts[1], 
                                        reference_pts[1], 
                                        requry_base_line, 
                                        reference_base_line, 
                                        second_match_result);
    
    if(second_pt_match){
        for(int i=0; i<first_match_result.size(); i++){
            for(int j=0; j<second_match_result.size(); j++){
                LinePair tmp;
                tmp.first_line_set_ = first_match_result[i].first_line_set_;
                for(int k=0; k<second_match_result[j].first_line_set_.size(); k++)
                    tmp.first_line_set_.push_back(second_match_result[j].first_line_set_[k]);
                
                tmp.second_line_set_ = first_match_result[i].second_line_set_;
                for(int k=0; k<second_match_result[j].second_line_set_.size(); k++)
                    tmp.second_line_set_.push_back(second_match_result[j].second_line_set_[k]);
                
                match_lines.push_back(tmp);
            }
        }
    }
    return second_pt_match;
}

bool Matcher::MatchOnePoint(const CrossPointPtr& requry_pt, 
                            const CrossPointPtr& reference_pt, 
                            const cv::Point2d& requry_base_line, 
                            const cv::Point2d& reference_base_line, 
                            std::vector< LinePair >& result){
    if(requry_pt->get_braches_num()!=reference_pt->get_braches_num()
       || requry_pt->get_tangents_num() != reference_pt->get_tangents_num())
        return false;
        
    result.clear();
    
    // sort the tangents from the connection clockwise 
    std::vector<cv::Point2d> requry_tangents_clockwise;
    cv::Point2d requry_base_line_normal = requry_base_line/cv::norm(requry_base_line);
    SortTangentsFromBaseLine(requry_pt, requry_base_line_normal, requry_tangents_clockwise);
    // sort the tangents from the connection counterclockwise 
    std::vector<cv::Point2d> requry_tangents_counterclockwise;
    for(int i=requry_tangents_clockwise.size()-1; i>-1; --i){
        requry_tangents_counterclockwise.push_back(requry_tangents_clockwise[i]);
    }
    
    // sort the tangents of reference pt clockwise
    std::vector<cv::Point2d> reference_tangents_clockwise;
    cv::Point2d reference_base_line_normal = reference_base_line/cv::norm(reference_base_line);
    SortTangentsFromBaseLine(reference_pt, reference_base_line_normal, reference_tangents_clockwise);
    
    // perform cross ratio check
    requry_tangents_clockwise.push_back(requry_base_line_normal);
    requry_tangents_counterclockwise.push_back(requry_base_line_normal);
    reference_tangents_clockwise.push_back(reference_base_line_normal);
        
    bool cross_ratio_consistent1 = CheckCrossRatioConsistency(requry_tangents_clockwise, 
                                                             reference_tangents_clockwise, 
                                                             cross_ratio_threshold_);
    
    bool cross_ratio_consistent2 = CheckCrossRatioConsistency(requry_tangents_clockwise, 
                                                             reference_tangents_clockwise, 
                                                             cross_ratio_threshold_);
    
    if(cross_ratio_consistent1){
        LinePair tmp;
        for(int i=0; i<requry_tangents_clockwise.size()-1;i++){
            tmp.first_line_set_.push_back(EstimateLine(requry_pt->get_center(), requry_tangents_clockwise[i]));
            tmp.second_line_set_.push_back(EstimateLine(reference_pt->get_center(), reference_tangents_clockwise[i]));
        }
        result.push_back(tmp);
    }
    
    if(cross_ratio_consistent2){
        LinePair tmp;
        for(int i=0; i<requry_tangents_counterclockwise.size();i++){
            tmp.first_line_set_.push_back(EstimateLine(requry_pt->get_center(), requry_tangents_counterclockwise[i]));
            tmp.second_line_set_.push_back(EstimateLine(reference_pt->get_center(), reference_tangents_clockwise[i]));
        }
        result.push_back(tmp);
    }
    
    return cross_ratio_consistent1 || cross_ratio_consistent2;
}

void Matcher::SortTangentsFromBaseLine(const CrossPointPtr& pt, 
                                       const cv::Point2d& base_line, 
                                       std::vector< cv::Point2d >& clockwise_sorted_lines){
    clockwise_sorted_lines.clear();
    
    clockwise_sorted_lines = pt->get_tangents();
    
    
    // calculate the angle between the x coordination and all the tangents
    std::vector<double> cos_angle;
    for(int i=0; i<clockwise_sorted_lines.size(); i++){
        double cross_tmp = base_line.cross(clockwise_sorted_lines[i]);
        double dot_tmp = base_line.dot(clockwise_sorted_lines[i]);
        if(cross_tmp>=0)
            cos_angle.push_back(dot_tmp);
        else
            cos_angle.push_back(-dot_tmp);
    }
  
    // optimized bubble sorting
    int start=0, end=cos_angle.size();
    while(start<end){
        int index = start;
        int current_end = start;
        while(++index<end){                             
            if(cos_angle[index-1]<cos_angle[index]){
                double tmp_cos_angle = cos_angle[index];
                cos_angle[index] = cos_angle[index-1];
                cos_angle[index-1] = tmp_cos_angle;
                
                cv::Point2d tangent_tmp = clockwise_sorted_lines[index];
                clockwise_sorted_lines[index] = clockwise_sorted_lines[index-1];
                clockwise_sorted_lines[index-1] = tangent_tmp;
                
                current_end = index;
            }
        }
        end = current_end;
    }
    
}

cv::Point3d Matcher::EstimateLine(const cv::Point2d &point, 
                  const cv::Point2d &tangent){
    cv::Point3d line; 
    line.z = tangent.x * point.y - tangent.y * point.x;
   
    line.x = tangent.y;
    line.y = - tangent.x;
    
    return line;
    
}




}   // namespace rcll
