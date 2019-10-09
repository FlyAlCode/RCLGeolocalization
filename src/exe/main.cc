#include <iostream>
#include <fstream>
#include <string>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>

#include "locator.h"
#include "cross_point_feature_creator.h"

// debug
#include <glog/logging.h>
#include <time.h>
#include <sys/stat.h>

#include <string.h> //包含strcmp的头文件,也可用: #include <ctring>
#include <dirent.h>

#include <sstream>
#include <algorithm>

void getFileNames(const std::string path, std::vector<std::string>& filenames, const std::string suffix = ""){    
    DIR *pDir;    
    struct dirent* ptr;    
    if (!(pDir = opendir(path.c_str())))        
        return;
    
    while ((ptr = readdir(pDir))!=0) {        
        if (strcmp(ptr->d_name, ".") != 0 && strcmp(ptr->d_name, "..") != 0){            
            std::string file = path + "/" + ptr->d_name;            
            if (opendir(file.c_str()))  {                
                getFileNames(file, filenames, suffix);                
            }
            
            else {                
                if (suffix == file.substr(file.size() - suffix.size())){                    
                    filenames.push_back(file);                    
                }                
            }            
        }        
    }    
    closedir(pDir);    

    std::sort(filenames.begin(), filenames.end());
}


int LoadMapFromFile(const std::string &filename,
                    const cv::Point2d &offset, const double scale,
                    std::vector<rcll::CrossPointPtr> &cross_pts){
    std::ifstream fin(filename);
    if(!fin.is_open()){
        std::cout<<"Fail to open map file, please check!!!"<<std::endl;
        return 0;
    }
    cv::Point2d center;
    std::vector<cv::Point2d> tangents;
    cv::Point2d tangent_tmp;
    int branch_num;
    int tangent_num;
    double tangent_fitting_error;
    while(!fin.eof()){
        fin>>tangent_fitting_error;
        fin>>center.x>>center.y;
        fin>>branch_num;
        fin>>tangent_num;
        tangents.clear();
        for(int i=0; i<tangent_num; ++i){
            fin>>tangent_tmp.x>>tangent_tmp.y;
            tangents.push_back(tangent_tmp);
        }
        
        if(tangent_fitting_error<1.0 && branch_num>=3 &&branch_num<=5 &&tangent_num>=2 &&tangent_num<=4){
            rcll::CrossPointPtr cross_pt_tmp(new rcll::CrossPoint());
            center.x = (center.x + offset.x)/scale;
            center.y = (center.y + offset.y)/scale;
            
            cross_pt_tmp->ThinInit(center, branch_num, tangents, tangent_fitting_error);
            cross_pts.push_back(cross_pt_tmp);
        }
    }
}



int main(int argc, char *argv[]){   
    // 1. deal with params
    if(argc<6){
        std::cout<<"Usage: rcl_geolocalize [map_file_name] [query_img_dir] [offset_x offset_y] [result_file]..."<<std::endl;
        exit(-1);
    }
        
    std::string map_file_name(argv[1]);
    std::string query_img_dir(argv[2]);
    std::vector<std::string> requry_image_names;
    getFileNames(query_img_dir, requry_image_names, "png");   
    
    std::cout<<requry_image_names.size()<<" images found......"<<std::endl;
    
    int offset_tmp_x = atoi(argv[3]);
    int offset_tmp_y = atoi(argv[4]);
    cv::Point2d map_offset(-offset_tmp_x, -offset_tmp_y);

    cv::Mat H_map_offset = cv::Mat::eye(3,3,CV_64F);
    H_map_offset.at<double> (0,2) = offset_tmp_x;
    H_map_offset.at<double> (1,2) = offset_tmp_y + 213;
    // debug
    // std::cout<<"H_map_offset = "<<H_map_offset<<std::endl;

    std::string result_file(argv[5]);

    
        
    clock_t start, end;
    start = clock();
    std::vector<rcll::CrossPointPtr> map;
    LoadMapFromFile(map_file_name, map_offset, 1.0, map);
    end = clock();
    std::cout<<(end - start)/(CLOCKS_PER_SEC/1000.0)<<" ms passed in loading map file with "
            <<map.size()<<" cross points"<<std::endl;
   
    rcll::CrossPointFeatureParam cross_feature_detector_param;
    cross_feature_detector_param.branch_length = 20;
    cross_feature_detector_param.min_cross_point_distance = 4;
    cross_feature_detector_param.threshold = 10;
    cross_feature_detector_param.merge_angle_threshold = 20;
    
    // 2. Initilize locator
    rcll::LocatorParam locator_params;
    {
        locator_params.max_iterate_num = 300000;
        locator_params.max_requry_sample_try_num = 10;
        
        locator_params.cos_angle_distance = 0.99;                       // cos(5°)
        locator_params.inliner_rate_threshold = 0.6;
        locator_params.min_inliner_point_num = 20;

        locator_params.max_inliner_distance = 40;                       // 10 piexl
        locator_params.max_sample_distance = 2000;
        locator_params.threshold_diatance = 100;
        locator_params.min_requry_pt_num = 30;
        locator_params.max_cross_ratio_relative_error = 0.2;
        locator_params.max_tangent_error = 1.0;
        
        locator_params.location_success_confidence = 0.99;
        locator_params.query_location_confidence = 0.4;
        locator_params.query_location_success_possibility = 0.2;
        
        locator_params.query_sample_max_distance = 1500;
        locator_params.query_sample_min_diatance = 100;
        locator_params.query_sample_min_cos_angle_distance = 0.985;
    }
    

    start = clock();
    rcll::Locator locator;
    locator.Initilize(map, locator_params);
    end = clock();
    std::cout<<(end - start)/(CLOCKS_PER_SEC/1000.0)<<" ms passed in  Initilize locator"<<std::endl;
    
    
    // 3. Locate    
    std::ofstream fout(result_file);
    for(int i=0; i<requry_image_names.size(); i++){
         cv::Mat requry_img;
        std::vector<rcll::CrossPointPtr> requry_pts;
        cv::Mat H;
        std::vector<rcll::CrossPointPtr> matching_points;
        double similarity;

        // debug
        std::cout<<"\n\n******************** "<<i+1<<"/"<<requry_image_names.size()
                 <<" Start locate image "<<requry_image_names[i]
                 <<"***************************"<<std::endl;
        // fout<<"\n"<<requry_image_names[i]<<std::endl;
        
        start = clock();
        requry_img = cv::imread(requry_image_names[i]);
        rcll::CreateCrossPointFeatures(requry_img, cross_feature_detector_param, requry_pts);
        std::cout<<requry_pts.size()<<" found."<<std::endl;

        int success = locator.Locate(requry_pts, H, matching_points, similarity);
        if(requry_pts.size()<locator_params.min_requry_pt_num)
            success = -1;
        
        end = clock();
        double t = (end - start)/(CLOCKS_PER_SEC/1000.0);
        std::cout << t <<" ms passed in  localization"<<std::endl;

        // debug
        if(!H.empty())
            std::cout<<H<<std::endl;

        // 输出的H是相对于地理坐标系的
        cv::Mat H_geo;
        if(success == -1 || H.empty())
            H_geo = cv::Mat::zeros(3, 3, CV_64F);   
        else
            H_geo = H_map_offset * H;
        
        fout<<i<<"    "<<success<<"    "<<t<<"    ";
        for(int j=0; j<9; j++)
            fout<<H_geo.at<double>(j/3, j%3)<<"    ";
        fout<<std::endl;
    }
    
    return 0;
}