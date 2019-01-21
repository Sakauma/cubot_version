//
// Created by snow on 18-3-3.
//

#ifndef ARMOR_DETECT_DAHUAPARAMETERS_H
#define ARMOR_DETECT_DAHUAPARAMETERS_H

#include <opencv2/opencv.hpp>
#include <iostream>

namespace cubot
{
    //该参数是1280*1024下的
    double camD0[9] = {1262.27, 0, 391.5423,
                      0, 1261.78, 315.7516,
                      0, 0, 1};
    double distCoeffD0[5] = {-0.0719, 0.1030, -0.0021, 0.0049, 0.9478};
    const cv::Mat camera_intrinsic_matrix = cv::Mat(3,3, CV_64FC1, camD0);
    const cv::Mat distortion_coefficients = cv::Mat(5,1, CV_64FC1, distCoeffD0);

    //该参数是1280*1024的图片resize到800*600下的参数
    double camD1[9] = {808.9729, 0, 388.0181,
                      0, 758.3957, 301.8837,
                      0, 0, 1};
    double distCoeffD1[5] = {-0.0671, -0.0498, -0.0010, 0.0041, 1.6923};
    const cv::Mat camera_intrinsic_matrix86 = cv::Mat(3,3, CV_64FC1, camD1);
    const cv::Mat distortion_coefficients86 = cv::Mat(5,1, CV_64FC1, distCoeffD1);

}


#endif //ARMOR_DETECT_DAHUAPARAMETERS_H
