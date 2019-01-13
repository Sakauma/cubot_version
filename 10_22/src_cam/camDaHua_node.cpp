#include <opencv2/opencv.hpp>
#include <iostream>

#include "daHua_usb/StreamRetrieve.h"
#include "daHua_usb/camParaConfig.h"
#include "cubotcpp/imgPublish.h"
#include "cubotcpp/timer.h"

using namespace cv;
using namespace cubot;

daHuaPara_str daHuaPara;
void callback( Mat  &img);
std::string filename = "../cfg/daHuaCamConfig.yml";


string now()
{
    time_t t = time(0);
    char buffer[9] = {0};
    strftime(buffer, 9, "%H:%M:%S", localtime(&t));
    return string(buffer);
}

int main()
{
    cout<<"========================================= camera open at time "<<now()<<endl;
    /* get the camera parameters */
    getCamParaFromYml(filename, daHuaPara);
    if (daHuaPara.IsReadYmlSucc != true)
    {
        std::cerr<<"Failed to get the configur file "<<std::endl;
        return 0;
    }
    std::cout<<" Get the configur file successfully"<<std::endl;

    ICameraPtr cameraSptr;

    /* �����豸 */
    CSystem &systemObj = CSystem::getInstance();
    TVector<ICameraPtr> vCameraPtrList;
    bool isDiscoverySuccess = systemObj.discovery(vCameraPtrList);
    if (!isDiscoverySuccess)
    {
        printf("discovery device fail.\n");
        exit(1);
    }
    if (vCameraPtrList.size() == 0)
    {
        printf("no devices.\n");
        exit(1);
    }
    /* 2������� */
    if (!vCameraPtrList[0]->connect())
    {
        printf("connect cameral failed.\n");
        exit(1);
    }

    cameraSptr = vCameraPtrList[0];
    /* camera config  And Attention */
    int IsSetCamParaSucc = 0;
    camParaConfig(cameraSptr, daHuaPara, IsSetCamParaSucc);
    if(IsSetCamParaSucc != 0)
    {
        cerr<<"failed to set camera para"<<endl;
    }
    else
        cout<<"succeed to set camera para"<<endl;

    IStreamSourcePtr streamPtr = systemObj.createStreamSource(cameraSptr);

    if (NULL == streamPtr)
    {
        printf("create stream obj  fail.\r\n");
        exit(1);
    }

    streamPtr->stopGrabbing();
    usleep(1000);

    bool isStartGrabbingSuccess = streamPtr->startGrabbing();
    if (!isStartGrabbingSuccess)
    {
        printf("StartGrabbing  fail.\n");
    }

    Dahua::Memory::TSharedPtr<StreamRetrieve>  streamThreadSptr(new StreamRetrieve(cameraSptr, daHuaPara, streamPtr,boost::bind(callback,_1)));
    if (NULL == streamThreadSptr)
    {
        printf("create thread obj failed.\n");
        return 0;
    }
    streamThreadSptr->join();

    return 1;
}

void save_img(Mat &img)
{
    static int frame_cnt = 0;
    for(int k=0;k<500;k++){};
    string savePath = "/home/cubot/roboMaster2018/infantry4.27/img/" + to_string(frame_cnt) + ".png";
    imwrite(savePath, img);
    //imshow("img",img);
    waitKey(1);
    frame_cnt++;
}

void callback(Mat &img)
{
    if(img.empty())
        return;
    static  ImgPub pub("MySharedMemory",img);
    pub.pub(img);

   //imshow("img",img);
   //waitKey(1);
}
