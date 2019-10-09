#ifndef DRAW_SHP_ELEMENTS_H_
#define DRAW_SHP_ELEMENTS_H_
#include <string>
#include <../../opt/ros/kinetic/include/opencv-3.3.1-dev/opencv2/core.hpp>
#include <../../opt/ros/kinetic/include/opencv-3.3.1-dev/opencv2/core/types.hpp>

#include "cross_point.h"

class ShpDrawer{
public:
    typedef std::vector<cv::Point2d> pts;
    ShpDrawer(double scale, 
              double area_width, 
              double area_height);
    
    void ShpToImg(const std::string &shp_file_name, const std::string &offset_file_name);
    void MakeAreaMap(const std::string &shp_file_name,
                     const cv::Vec4d &area_bound,
                     cv::Mat &result_map);

    void DetectCrossPoints(const std::string &shp_file_name, 
                        std::vector<rcll::CrossPointPtr> &cross_pts,            // express in corrected geo coordinate
                        const cv::Point2d &geo_offset = cv::Point2d(0,0));

    void ShowImageOnMap(const std::string &shp_file_name,                       // the shp file name
                        const std::vector<cv::Mat> &src_imgs,                   // images to draw on the map
                        const std::vector<cv::Mat> &Hs,                         // 将图像点转换到地理坐标系点的单应矩阵
                        const std::vector<cv::Point> &padding_sizes,                          // 图像外部的地图区域
                        std::vector<cv::Mat> &draw_imgs);                       // 最后的结果图片
    
private:
    void ShpPtsToPixels(const std::vector<pts> &shp_pts,                        // 
                        const cv::Rect2d &bound,
                        std::vector<std::vector<cv::Point> >&pixels);           // 在整个范围内的像素坐标
    
    // 这里需要进行地理坐标的补偿
    cv::Point2d PixelToShpPt(const cv::Point2d & pixel_pt,
                               const cv::Rect2d &bound,                             // shp的bound
                               const cv::Point2d &geo_offset=cv::Point2d(0,0));                     
   
    // 将图像坐标系下的向量转换到地理坐标系下，由于向量的可移动性，因此这里只需考虑两个坐标系的相对旋转
    inline cv::Point2d ImgVectorToGeoVector(const cv::Point2d &img_vector){
        return cv::Point2d(img_vector.x, -img_vector.y);
    }          
    
    int CreateGrid(std::vector<cv::Point> &start_pts);
    
    void DrawShpInOneGrid(const cv::Point &start_pts, 
                          const std::vector<std::vector<cv::Point> >&pixels,
                          cv::Mat &result_map,
                          int line_width = 5);
    
    void DrawShpOfCertainArea(const cv::Rect2d &shp_bound,
                              const cv::Vec4d &area_bound,           // area_bound = {min_x, min_y, max_x, max_y}
                              const std::vector<std::vector<cv::Point> >&pixels,
                              cv::Mat &result_map,
                              cv::Point2d &img_start_pt) ;          // 所画区域的左上角点在整个区域图像中的坐标

    void DrawShpOfCertainArea(const cv::Rect2d &shp_bound,
                              const cv::Vec4d &area_bound,           // area_bound = {min_x, min_y, max_x, max_y}
                              const std::vector<std::vector<cv::Point> >&pixels,
                              cv::Mat &result_map) ;          // 所画区域的左上角点在整个区域图像中的坐标

    cv::Point2d GetHTransformedPt(const cv::Point2d &src_pt, const cv::Mat &H);

    bool MergeWithMap(const cv::Mat &query_img,               
               const cv::Mat &map_img,                            
               const cv::Mat &H, 
               const cv::Point &padding_size,                        
               cv::Mat &draw_img);
    
    
    double scale_;                  // m/pixel
    double area_width_;             // unit: meter
    double area_height_;
    double area_width_piexl_;       
    double area_height_piexl_;
    int whole_img_width_;           // 在给定的分辨率下的整个图像的尺寸     
    int whole_img_height_;

    // cv::Point2d geo_offset_;        // 为矫正地理坐标系偏移引入的变量
};
#endif 

