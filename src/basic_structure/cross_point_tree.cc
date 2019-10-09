#include "cross_point_tree.h"

namespace rcll{
    
void RoadMapTree::BuildKDTree(const std::vector<cv::Point2d> &points ){
    FillData(points);      
    tree_.reset(new RoadMapTree_(2, pts_, nanoflann::KDTreeSingleIndexAdaptorParams(20 /* max leaf */)));
    tree_->buildIndex();
}
    
void RoadMapTree::RadiusSearch(const std::vector<cv::Point2d> &requry_pts,
                  const double threshold_distance,
                  std::vector<std::vector<std::pair<size_t,double> > > &ret_matches){
    ret_matches.clear();
    ret_matches.resize(requry_pts.size());
    
    for(int i=0; i<requry_pts.size(); i++){
        double query_pt[2] = {requry_pts[i].x, requry_pts[i].y};
        
        nanoflann::SearchParams params;

        const size_t nMatches = tree_->radiusSearch(&query_pt[0], threshold_distance, ret_matches[i], params);            
    }
}

void RoadMapTree::FillData(const std::vector<cv::Point2d> &points){
    pts_.pts.clear();
    pts_.pts.resize(points.size());
    for(int i=0; i<points.size(); i++){
        pts_.pts[i].x = points[i].x;
        pts_.pts[i].y = points[i].y;
    }
}



    
    
}   // namespace rcll
