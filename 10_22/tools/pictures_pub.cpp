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
    //string filepath = "/home/terminal/Files/pic/";
    string filepath = "/media/lin/cubot/724/20180724T140529/Realtime_Armor.avi";
    VideoCapture cap(filepath);
    creatTimerCallback timer(timercallback, 1000);
    Mat frame;
    int frame_num = 0;
    while (1)
    {
        cap>>frame;
        //string img_name = filepath + to_string(frame_num++) +".png";

//        if (frame_num>3751)
//            break;
//            //frame_num=10;
//
//        frame = imread(img_name);
        if (frame.empty())
            continue;

        //imwrite("/home/terminal/Files/pic/" + to_string(frame_num++) + ".png", frame);
        callback(frame);
        cout<<"frame_cnt is "<<frame_num++<<endl;
        imshow("frame",frame);
        waitKey(100);
    }
    return 0;
}



