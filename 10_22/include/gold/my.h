//
// Created by jason on 18-6-6.
//

#ifndef TEST_OPENCV_MY_H
#define TEST_OPENCV_MY_H


#include <iostream>
#include "opencv2/opencv.hpp"
#include "opencv2/dnn.hpp"
//#include "opencv2/face.hpp"

#include "ostream"

//#include "src/pro_file.h"
#include "coordinante_process.h"
#include "bar_points.h"



namespace meng
{
    /*!!!! You need set different value according to your role*/

    extern int Color_id ;
    extern float bar_height_H;
    extern float bar_height_L;
    extern float bar_slim_k_H;
    extern float bar_slim_k_L;

    extern float bar_angle_limit;
    extern float small_pair_bar_angle_limit;
    extern float big_pair_bar_angle_limit;

    extern float high_light_mask_k ;
    extern  float proposal_bar_area_k;

    extern cv::Point2f dynamic_sp;

    extern cv::Mat transform_img;

    extern std::vector<cv::Point2f> world_pts;
    extern std::vector<cv::Point2f> new_pts;

    extern cv::Ptr<cv::ml::SVM> model_hog ;
    extern cv::HOGDescriptor * hog ;

    extern int Model_cnt;
    extern int g_flag;

    int predict(cv::Ptr<cv::ml::SVM> &model,cv::HOGDescriptor * hog ,cv::Mat &transform_img);
    void transform_armor_vertex(cv::Point2f *pts,float k);

    cv::Point2f detection(cv::Mat &img,float &dist);
}

#endif //TEST_OPENCV_MY_H
