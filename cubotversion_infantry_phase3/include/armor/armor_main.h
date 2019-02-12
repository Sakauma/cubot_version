#ifndef TEST_OPENCV_MY_H
#define TEST_OPENCV_MY_H

#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/dnn.hpp>
#include <ostream>

#include"coordinante_process.h"
#include "bar_points.h"

namespace AutoHit
{

    extern int color_id ;
    extern float bar_height_H;
    extern float bar_height_L;
    extern float bar_slim_k_H;
    extern float bar_slim_k_L;

    extern float bar_angle_limit;
    extern float small_pair_bar_angle_limit;
    extern float big_pair_bar_angle_limit;

    extern float high_light_mask_k ;
    extern float proposal_bar_area_k;

    extern cv::Point2f sp_dynamic;

    extern cv::Mat img_transform;

    extern std::vector<cv::Point2f> world_pts;
    extern std::vector<cv::Point2f> new_pts;

    extern cv::Ptr<cv::ml::SVM> model_hog ;
    extern cv::HOGDescriptor *pHog ;

    extern int autohit_flag;

    int predict( cv::Ptr<cv::ml::SVM> &model, cv::HOGDescriptor *pHog, cv::Mat &transform_img );
    void transform_armor_vertex( cv::Point2f *pPts, float k );

    cv::Point2f detection( cv::Mat &img, float &dist ,float & T );
}

#endif