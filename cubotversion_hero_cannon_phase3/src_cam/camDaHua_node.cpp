#include <opencv2/opencv.hpp>
#include <iostream>

#include "daHua_usb/StreamRetrieve.h"
#include "daHua_usb/camParaConfig.h"
#include "cubotcpp/imgPublish.h"
#include "cubotcpp/timer.h"

using namespace cv;
using namespace cubot;

int counter=0;
daHuaPara_str daHuaPara;

void timercallback(void);
void callback( Mat  &img);

std::string filename = "../cfg/daHuaCamConfig.yml";

int main()
{
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

    int BigGunCamNum=-1;
    for (int i = 0; i < vCameraPtrList.size(); i++)
    {
        cameraSptr = vCameraPtrList[i];

        CStringNode paramDeviceVersion(cameraSptr, "DeviceVersion");
        CString strDeviceVersion;
        paramDeviceVersion.getValue(strDeviceVersion);
        printf("Camera[%d] Info :\n", i);
        printf("    serial number = [%s]\n", cameraSptr->getSerialNumber());

        string serialNum = cameraSptr->getSerialNumber();
        char camFlag = serialNum.front();
        cout<<camFlag<<endl;

//        if(serialNum=="4A0480BPAK6EE9E")

        if(serialNum=="3E04D1EAAK00038")    //3E04D1EAAK00038--Cannon
            //if(serialNum=="4A0480BPAK48BD6")    //3E04D1EAAK00038--Cannon

            BigGunCamNum=i;
    }

    if(BigGunCamNum<0)
    {
        cout<<"BigGunCam doesn't exist\n";
        exit(1);
    }

    /* 2������� */
    if (!vCameraPtrList[BigGunCamNum]->connect())
    {
        printf("connect cameral failed.\n");
        exit(1);
    }

    cameraSptr = vCameraPtrList[BigGunCamNum];
    /* camera config  And Attention */
    int IsSetCamParaSucc = 0;
    camParaConfig(cameraSptr, daHuaPara, IsSetCamParaSucc);
    if(IsSetCamParaSucc != 0)
    {
        cerr<<"failed to set camera para"<<endl;
    }
    else
        cout<<"succeed to set camera para"<<endl;
    cout<<"img's original size is " << daHuaPara.imgWidth << " * " << daHuaPara.imgHeight << endl;


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

    Dahua::Memory::TSharedPtr<StreamRetrieve>  streamThreadSptr(new StreamRetrieve(streamPtr,boost::bind(callback,_1),
                                                                                   daHuaPara.imgWidth, daHuaPara.imgHeight));
    if (NULL == streamThreadSptr)
    {
        printf("create thread obj failed.\n");
        return 0;
    }

    //creatTimerCallback timer(boost::bind(timercallback),1000);
    streamThreadSptr->join();

    return 1;
}

void timercallback(void)
{
    cout<<"counter "<<counter<<endl;
    counter=0;
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
    static  ImgPub pub("CANNONIMG",img);
    pub.pub(img);
    counter++;
    //imshow("qweqwe",img);
    //waitKey(1);
    //cout<<counter<<endl;
}
