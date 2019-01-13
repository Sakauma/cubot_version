//
// Created by snow on 18-6-26.
//
#include <iostream>
#include "opencv2/opencv.hpp"
#include "gold/my.h"

#define ImageTest

#ifdef ImageTest
    std::string roi_root="/home/snow/Robomasters/documents/Sentry_NLG_18_5/images/";
    int main()
    {
        uchar start_num = 0;
        int readNum = 0;
        while(1)
        {
            if (readNum>100)
                break;
            std::string img_name = roi_root + std::to_string(readNum++) + ".png";
#if 0
            cv::Mat frame = cv::imread(img_name);
#else
            cv::Mat frame=cv::imread("/home/snow/Robomasters/mengyu619/infantry_sample/data/26.png");
#endif
            if (frame.empty())
                continue;

            cv::resize(frame, frame, cv::Size(1280, 1024));

            float distance = 0;
            cv::Point2f target_Point = meng::detection(frame, distance);

            start_num++;
            if (start_num==1)
                continue;
            else
                start_num=2;

            if (target_Point == cv::Point2f(0,0))
                printf("detect successfully\n");

            printf("frame: %d \n ", readNum);
            int k = cv::waitKey(0);
            if(k==int('s')) {
                cv::imwrite("/home/jason/Dataset/red_analysis/" + std::to_string(readNum-1) + ".jpg", frame);
                printf("save img successfully: ===================%d \n ", readNum-1);
            }
            else if (k == int('q'))
                cv::waitKey(0);
            else if(k == 'b')
                break;
        }

        return 0;
    }
#else
std::string save_path = "/home/snow/imgs/";
int main()
{
    int cnt=0;
    cv::Mat frame;
    cv::VideoCapture cap("/home/snow/Robomasters/documents/VideoTest_704_r.avi");

    while (cap.isOpened())
    {
        printf("frame: %d \n ", cnt++);

        cap >> frame;
        //frame = cv::imread("/home/snow/imgs/4.png");
        if (frame.empty())
            continue;
        cv::Mat frame_clone = frame.clone();

//        if(cnt<614)
//            continue;

//        if ((cnt == 127)||(cnt == 128)||(cnt == 4)||(cnt == 5)){
//            cv::imwrite(save_path + std::to_string(cnt) + ".png", frame_clone);
//            printf("save img successfully: ===================%d \n ", cnt);
//        }

        float distance;
        cv::Point2f target = meng::detection(frame,distance);
        if(target.x>0)
        {
            cv::circle(frame,target/2,10,cv::Scalar(0,255,0),3);
            std::cout<<target<<std::endl;
            cv::putText(frame,std::to_string(1),cv::Point(20,20),cv::FONT_HERSHEY_COMPLEX,0.5,cv::Scalar(0,0,255));
        }
        cv::imshow("frame",frame);
        int k = cv::waitKey(0);
        if(k == int('p')){
            cv::waitKey();
        }
        else if(k == 'b')
            break;
        else if (k == int('s')){
            cv::imwrite(save_path + std::to_string(cnt) + ".png", frame_clone);
            printf("save img successfully: ===================%d \n ", cnt);
        }
    }
}
#endif
