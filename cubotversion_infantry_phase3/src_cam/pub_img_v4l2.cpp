#include <iostream>
#include "common_cam/cam_driver_v4l2.h"
#include "cubotcpp/imgPublish.h"
#include "cubotcpp/timer.h"
#include <opencv2/opencv.hpp>

using namespace cv;
using namespace std;
using namespace cubot;

//自动曝光！
int counter=0;
void timercallback(void)
{
    cout<<"counter "<<counter<<endl;
    counter=0;
}
void callback(const cv::Mat &img)
{
    if(img.empty())
        return;

    static  ImgPub pub("MySharedMemory",img);
    pub.pub(img);
    counter++;
}

int main() {

    CamCapByV4l2 cam(callback, "/dev/video0", 640, 480);

    creatTimerCallback timer(boost::bind(timercallback),1000);
    cam.join();

    std::cout << "Hello, World!" << std::endl;
    return 0;
}
