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

Mat m = Mat(Size(1280,1024),CV_8UC3,Scalar(0,0,0));

int main()
{
    std::cout << "common camera pub img : img sending ..." << std::endl;
//    ImgPub ImgPub0("MySharedMemory", m);

    VideoCapture cap("/home/snow/Robomasters/documents/VideoTest_rock.avi");
    if(!cap.isOpened())
    {
        cout<<"video cann't open"<<endl;
        return 1;
    }

    //creatTimerCallback timer(timercallback, 1000);
    Mat frame;
    int frame_num = 0;
    int save_num = 26;
    while (1)
    {
        frame_num++;
        if(frame_num>405)
            break;

        cap>>frame;
        if (frame.empty())
            continue;

        if(frame.size()!=Size(1280,1024)||frame.channels()!=3)
        {
            cerr<<"failed ..."<<endl;
            continue;
        }

        static ImgPub ImgPub0("MySharedMemory", frame);

        cout<<"frame_cnt is "<<frame_num<<endl;
//        if(frame_num<1270)
//            continue;

        ImgPub0.pub(frame);

        namedWindow("frame",0);
        imshow("frame",frame);
        int k = waitKey(0);
        if(k==int('q'))
            waitKey();
        else if(k==int('b'))
            break;
        else if(k==int('s'))
        {
            string img_name = "../data/" + to_string(save_num) + ".png";
            imwrite(img_name,frame);
            save_num++;

            cout<<"save img "<<img_name<<endl;
        }
    }
    return 0;
}



