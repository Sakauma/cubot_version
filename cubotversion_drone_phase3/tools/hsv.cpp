#include "opencv2/opencv.hpp"
#include <iostream>
using namespace cv;
using namespace std;

Mat srcImg, hsv_img=Mat::zeros(Size(800,600), CV_8UC3);
Mat zero = Mat(60,30,CV_8U,Scalar(255));
int h_min =0,s_min = 0,v_min = 0;
int h_max = 180,s_max = 255,v_max = 46;

void onChange(int, void* param) {
    Scalar hsv_min(h_min, s_min, v_min);
    Scalar hsv_max(h_max, s_max, v_max);
    Mat dst = Mat::zeros(srcImg.size(), srcImg.type());
    inRange(hsv_img, hsv_min, hsv_max, dst);
    imshow("HSV", zero);
    imshow("dst", dst);
}


int main()
{
    srcImg = imread("/home/cubot/1.png");
    if(srcImg.empty())
        return 1;

    imshow("src", srcImg);
    cvtColor(srcImg, hsv_img, CV_BGR2HSV); //BGR转到HSV颜色空间
    namedWindow("HSV", CV_WINDOW_NORMAL);
    //创建滚动条
    createTrackbar("h_min", "HSV", &h_min, 360, onChange, 0);
    createTrackbar("s_min", "HSV", &s_min, 255, onChange, 0);
    createTrackbar("v_min", "HSV", &v_min, 255, onChange, 0);
    createTrackbar("h_max", "HSV", &h_max, 360, onChange, 0);
    createTrackbar("s_max", "HSV", &s_max, 255, onChange, 0);
    createTrackbar("v_max", "HSV", &v_max, 255, onChange, 0);
    //回调函数初始化
    onChange(h_min, 0);
    onChange(s_min, 0);
    onChange(v_min, 0);
    onChange(h_max, 0);
    onChange(s_max, 0);
    onChange(v_max, 0);

    waitKey(0);
}