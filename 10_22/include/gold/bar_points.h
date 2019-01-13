//
// Created by jason on 18-6-5.
//

#ifndef TEST_OPENCV_BAR_POINTS_H
#define TEST_OPENCV_BAR_POINTS_H

#include <iostream>
#include "opencv2/opencv.hpp"
#include <cstring>
#include <stdio.h>
#include <dirent.h>

#include "ostream"
#include "coordinante_process.h"

cv::Point2f getCrossPoint(cv::Point2f &p00,cv::Point2f &p01,cv::Point2f &p10,cv::Point2f &p11);

void get_expand_pts(cv::Point2f*endpoints , struct expand_pt * ex_pts,float angle1,float angle2,float distance1,float distance2,int rows,int cols);

int match_two_bars_by_expts(const struct s_bar *domain_bar ,const struct s_bar *slave_bar,float distance_limit,float angle_limit,int small_or_large);


void get_proposal_pts(cv::Point2f *endpoints,float angle,float distance_l,float distance_h,struct proposal_pt *prop_pts,int rows,int cols);
std::vector<int> match_two_bars_by_propts(struct proposal_pt prop_pts,float area_k,cv::Mat &color_mask,int rows,int cols);

/*Get   expand pts;
 *      proposal pts;
 *      large expand pts*/
void get_surround_pts(std::vector<struct s_bar> &bar_list, const cv::Mat img_src);

int find_armor_int_all_pts(cv::Mat &src_img, std::vector<struct s_bar> &valid_bar_list,cv::Mat & img_ori);

int find_armor_for_single_bar(cv::Mat &src_img, std::vector<struct s_bar> &valid_bar_list,cv::Mat & img_ori, cv::Point2f &sp);

#endif //TEST_OPENCV_BAR_POINTS_H
