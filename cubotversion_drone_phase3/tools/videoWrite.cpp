#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "cubotcpp/imgSubscrible.h"


using namespace cv;
using namespace cubot;

int  zp=0;
Mat  imageCopy;
double rate = 20.0;//视频的帧率
Size videoSize(800,600);
VideoWriter writer("VideoTest_719.avi", CV_FOURCC('M', 'J', 'P', 'G'), rate, videoSize);

void callback(const CubotImg &img)
{
    if (img.img.empty())
        return;

    imageCopy = img.img.clone();

    writer << imageCopy;
    imshow("video", imageCopy);
    waitKey(1);
}

int main(int argc, char *argv[])
{
    ImgSub imgsub("MySharedMemory", boost::bind(callback, _1));
    imgsub.subJoin();
    return 0;
}
