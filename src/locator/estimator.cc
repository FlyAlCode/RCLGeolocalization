#include "estimator.h"
#include "cross_ratio.h"
#include "matcher.h"
#include <opencv2/calib3d/calib3d.hpp>

// just for debug
#include <iostream>

namespace rcll {


/* dst_lines[i] = H^(-T)*src_lines[i]
 * src_lines[i] = H^(T)*dst_lines[i]
 * 这里的主要问题是：当对应关系多于4个的时候，由于不是每个关系都正确对应，导致会受错误匹配的影响。
 * 一种可能的解决办法是：计算得到H后，计算转换误差，如果误差大于某一值，认为不存在符合条件的H，即返回空H
 */
cv::Mat HomographyEstimator::findHomography(const std::vector<cv::Point3d> &src_lines, 
                       const std::vector<cv::Point3d> &dst_lines){
    if(src_lines.size()!=dst_lines.size() || src_lines.size()<4 ||dst_lines.size()<4)
        return cv::Mat();
    
    cv::Mat A(src_lines.size()*2, 9, CV_64F);
    cv::Point3d X, X_;
    for(int i=0; i<src_lines.size(); i++){
        X = dst_lines[i];
        X_ = src_lines[i];
        A.at<double>(i*2, 0) = 0;
        A.at<double>(i*2, 1) = 0;
        A.at<double>(i*2, 2) = 0;
        A.at<double>(i*2, 3) = -X_.z * X.x;
        A.at<double>(i*2, 4) = -X_.z * X.y;
        A.at<double>(i*2, 5) = -X_.z * X.z;
        A.at<double>(i*2, 6) = X_.y * X.x;
        A.at<double>(i*2, 7) = X_.y * X.y;
        A.at<double>(i*2, 8) = X_.y * X.z;
        
        A.at<double>(i*2+1, 0) = X_.z * X.x;
        A.at<double>(i*2+1, 1) = X_.z * X.y;
        A.at<double>(i*2+1, 2) = X_.z * X.z;
        A.at<double>(i*2+1, 3) = 0;
        A.at<double>(i*2+1, 4) = 0;
        A.at<double>(i*2+1, 5) = 0;
        A.at<double>(i*2+1, 6) = -X_.x * X.x;
        A.at<double>(i*2+1, 7) = -X_.x * X.y;
        A.at<double>(i*2+1, 8) = -X_.x * X.z;   
    }
    
    // method one (svd)
    cv::Mat h;
    cv::SVD::solveZ(A, h);
    cv::Mat H(3, 3, CV_64F, h.data);

    // method two (least eigenvalues)
//     cv::Mat ATA = A.t() * A;
//     cv::Mat eigenvalues;
//     cv::Mat eigenvectors;
//     cv::eigen(ATA, eigenvalues, eigenvectors);
//     cv::Mat h;
//     eigenvectors.rowRange(8,9).copyTo(h);
//     cv::Mat H(3, 3, CV_64F, h.data);
//     std::cout<<"eigenvalues:\n"<<eigenvalues<<std::endl;
//     std::cout<<"eigenvectors:\n"<<eigenvectors<<std::endl;    
//     std::cout<<"h:\n"<<h<<std::endl;
//     std::cout<<"H:\n"<<H<<std::endl;

    // method three (AX=b)
//     std::cout<<"A:\n"<<A<<std::endl;
//     std::cout<<"b:\n"<<b<<std::endl;
    
//     cv::Mat x;
//     cv::solve(A,b,x, cv::DECOMP_SVD);
//     cv::Mat H(3, 3, CV_64F, x.data);
//     H.at<double>(2,2)=1;
    

//     std::cout<<"x:\n"<<x<<std::endl;
//     std::cout<<"H:\n"<<H<<std::endl;
    
    // method four
//     std::vector<cv::Point2d> src_line_pts;
//     std::vector<cv::Point2d> dst_line_pts;
//     for(int i=0; i<src_lines.size();i++){
//         src_line_pts.push_back(cv::Point2d(src_lines[i].x/src_lines[i].z, src_lines[i].y/src_lines[i].z));
//     }
//     for(int i=0; i<dst_lines.size();i++){
//         dst_line_pts.push_back(cv::Point2d(dst_lines[i].x/dst_lines[i].z, dst_lines[i].y/dst_lines[i].z));
//     }
//     
//     cv::Mat H = cv::findHomography(dst_line_pts, src_line_pts, cv::RANSAC, 1);
    cv::Mat H_inv = H.t();
    
    return H_inv/H_inv.at<double>(2,2);
    
}




}   //namespace rcll