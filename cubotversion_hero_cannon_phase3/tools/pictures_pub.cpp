//
// Created by snow on 18-4-7.
//

#include <opencv2/opencv.hpp>
#include <iostream>
#include"cubotcpp/imgPublish.h"
#include"cubotcpp/timer.h"

using  namespace cv;
using  namespace std;
using  namespace cubot;

int counter=0;
void timercallback()
{
    cout<<"counter "<<counter<<endl;
    counter=0;
}

void callback( Mat  &img)
{
    static  ImgPub pub("MySharedMemory",img);
    pub.pub(img);
    counter++;
}

int main()
{

    std::cout << "common camera pub img : img sending ..." << std::endl;
    string filepath = "/media/snow/Life/RoboMaster/dataset/samples/b/samples_f/";

    creatTimerCallback timer(timercallback, 1000);
    Mat frame;
    int frame_num = 0;
    while (1)
    {
        string img_name = filepath + to_string(frame_num++) +".png";

        if (frame_num>262)
            //break;
            frame_num=0;

        frame = imread(img_name);
        if (frame.empty())
            continue;

//        double time0 = getTickCount();
//        Mat grayImg, bin_img;
//        Mat img0 = frame.clone();
//        cvtColor(frame, grayImg, CV_BGR2GRAY);
//        threshold(grayImg, bin_img, 240, 255, THRESH_BINARY);
//
//        vvP2i contours;
//        findContours(bin_img, contours, RETR_EXTERNAL, CHAIN_APPROX_NONE);
//        double time1 = getTickCount();
//        double time_bias = (time1-time0)*1000/getTickFrequency();
//
//        cvtColor(frame, grayImg, CV_BGR2HSV);
//
//        cout<<"time_bias "<<time_bias<<"   "<<contours.size()<<endl;




//        imshow("bin_img",bin_img);
        callback(frame);
        cout<<"frame_cnt is "<<frame_num<<endl;
        //imshow("frame",frame);
        waitKey(1000);
    }
    return 0;
}



