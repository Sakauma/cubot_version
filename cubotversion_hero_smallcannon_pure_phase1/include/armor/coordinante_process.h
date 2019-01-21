//
// Created by jason on 18-6-5.
//

#ifndef TEST_OPENCV_COORDINANTE_PROCESS_H
#define TEST_OPENCV_COORDINANTE_PROCESS_H

//
#include <iostream>
#include "opencv2/opencv.hpp"

#include <cstring>
#include <stdio.h>
#include <dirent.h>

#include "ostream"

struct expand_pt
{
    cv::Point2f parallel_l[2];
    cv::Point2f parallel_r[2];
    cv::Point2f horizon_l[2];
    cv::Point2f horizon_r[2];
};

struct proposal_pt
{
    cv::Point2f parallel_l[4];
    cv::Point2f parallel_r[4];
};

struct serach_flag
{
    int left;
    int right;
};


struct s_bar
{
    //cv::RotatedRect r_rec;
    cv::RotatedRect ellipse;
    cv::Rect b_rec;
    int level;
    //int armor_lev;
    int is_pair;

    cv::Point2f endpoints[2];

    struct expand_pt ex_large_armor;
    struct expand_pt ex_small_armor;

    struct proposal_pt prop_small_armor;
    //struct proposal_pt prop_large_armor;

    //struct serach_flag small_search_direction;
    //struct serach_flag large_search_direction;

    cv::Point2f pair_p1;
    cv::Point2f pair_p2;

    cv::Point2f armor_vertex[4];

    int isPosePoints=0;
    cv::Point2f pose_points[4];
};




void find_newpoint(cv::Point2f start,float angle_rad,float distance,cv::Point2f & pt,int rows,int cols);

/*make sure e1 is on the top ,and e2 is on bottom*/
void check_endpoint(cv::Point2f & e1,cv::Point2f & e2);

/*make sure rect cood in image size*/
int check_rect(const cv::Rect &r,int rows,int cols);

/*check if roi in mask contain some points*/
float check_mask_area(cv::Mat &mask, cv::Rect &roi);

/*clip number in a range*/
float limit_number(float a,float hight,float low);

/*distance between 2 points*/
float cal_distance_pt(cv::Point2f a , cv::Point2f b);

/*get armotr vertexs from two pair of  bar endpoints*/
void get_armor_from_pts(cv::Point2f e1 ,cv::Point2f e2,cv::Point2f p1 ,cv::Point2f p2,cv::Point2f * armor,int rows,int cols);

/*sort by decending */
template < typename T>
std::vector< size_t>  sort_indexes(const std::vector< T>  & v , int descend_or_ascend);

/*sort bars according to their height*/
void sort_ellipse(const std::vector<struct s_bar> &ori_bars , std::vector<struct s_bar> &dst_bars,int x , int y);
void sort_bar_according_to_height(const std::vector<struct s_bar> &ori_bars , std::vector<struct s_bar> &dst_bars);

double calDistance(const std::vector<cv::Point2f> &points  ,cv::Mat &intrinsic_mat);

int limit_rect(cv::Rect &src_rect,cv::Rect & dst_rect,int rows,int cols);

#endif //TEST_OPENCV_COORDINANTE_PROCESS_H
