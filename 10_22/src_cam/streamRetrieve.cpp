#include "daHua_usb/StreamRetrieve.h"
#include"cubotcpp/imgPublish.h"
#include"cubotcpp/timer.h"
#include "cubotcpp/SubscribeAsync.h"

//#define dafutest
using namespace cubot;

void sub_callback(const vector<cv::Point2f> &data);
Subscrible sub_mode(sub_callback);
#ifdef dafutest
int Mode_last=0;
int Mode = 1;
#else
int Mode=2,Mode_last=0;
#endif
StreamRetrieve::StreamRetrieve(ICameraPtr &cameraSptr,daHuaPara_str &daHuaPara,IStreamSourcePtr& streamSptr,ImgCallback imgCB):
        _cameraSptr(cameraSptr),_daHuaPara(daHuaPara),m_streamSptr(streamSptr), imgCallback(imgCB), Width(daHuaPara.imgWidth), Height(daHuaPara.imgHeight)
{
    pro_thread  = boost::thread(boost::bind(&StreamRetrieve::Proc, this));
}

void StreamRetrieve::join()
{
    pro_thread.join();
}

void StreamRetrieve::Proc()
{
    std::cout<<" camera runing..."<<std::endl;
    int frameErrorNum=0;
    while (1)
    {
        if(Mode==2){
            ArmorModelParaConfig_runtime(_cameraSptr, _daHuaPara);
            Mode=0;
        } else if(Mode==1){
            RuneModelParaConfig_runtime(_cameraSptr, _daHuaPara);
            Mode=0;
        }

        CFrame frame;
        if (!m_streamSptr)
        {
            printf("m_streamPtr is NULL.\n");
            return;
        }
        bool isSuccess = m_streamSptr->getFrame(frame, 200);
        if (!isSuccess)
        {
            frameErrorNum++;
            if(frameErrorNum>2000){
                printf("USB may not be connecting\n");
                return;
            }else{
                printf("getFrame  fail.\n");
                continue;
            }
        } else
            frameErrorNum=0;

        bool isValid = frame.valid();
        if (!isValid)
        {
            frameErrorNum++;
            if(frameErrorNum>2000){
                printf("USB may not be connecting\n");
                return;
            } else{
                printf("frame is invalid!\n");
                continue;
            }
        } else
            frameErrorNum=0;
////////////////////////////////////
        if (Dahua::GenICam::gvspPixelMono8 == frame.getImagePixelFormat())
        {
            u_int8_t* pmonoFrameBuf=new(std::nothrow) u_int8_t[Width*Height*1];   //1280*1024*1

            memcpy(pmonoFrameBuf, frame.getImage(), frame.getImageSize());
            IplImage* iplImage = cvCreateImageHeader(cvSize(frame.getImageWidth(), frame.getImageHeight()), IPL_DEPTH_8U, 1);
            cvSetData(iplImage, pmonoFrameBuf, frame.getImageWidth()*1);
            _mat=cv::cvarrToMat(iplImage);
            imgCallback(_mat);

            //free(pmonoFrameBuf);  //错误的使用了free与new相配合
            delete pmonoFrameBuf;
        }
        else
        {
            uint8_t *pRGBbuffer = NULL;
            int nRgbBufferSize = 0;
            nRgbBufferSize = frame.getImageHeight() * frame.getImageWidth() * 3;
            pRGBbuffer = (uint8_t *)malloc(nRgbBufferSize);

            if (pRGBbuffer == NULL)
            {
                printf("RGBbuffer malloc failed.\n");
                free(pRGBbuffer);
                continue;
            }

            IMGCNV_SOpenParam openParam;
            openParam.width = frame.getImageWidth();
            openParam.height = frame.getImageHeight();
            openParam.paddingX = frame.getImagePadddingX();
            openParam.paddingY = frame.getImagePadddingY();
            openParam.dataSize = frame.getImageSize();
            openParam.pixelForamt = gvspPixelBayRG8;

            unsigned char * pRgbFrameBuf=new(std::nothrow)  unsigned char[Width*Height*1];

            memcpy(pRgbFrameBuf, frame.getImage(), frame.getImageSize());

            IMGCNV_EErr status = IMGCNV_ConvertToBGR24(pRgbFrameBuf, &openParam, pRGBbuffer, &nRgbBufferSize);
            if (IMGCNV_SUCCESS != status)
            {
                printf("IMGCNV_ConvertToBGR24 failed.\n");

                delete pRgbFrameBuf;
                return;
            }

            delete pRgbFrameBuf;

            IplImage* iplImage = cvCreateImageHeader(cvSize(frame.getImageWidth(), frame.getImageHeight()), IPL_DEPTH_8U, 3);
            cvSetData(iplImage, pRGBbuffer, frame.getImageWidth()*3);
            _mat=cv::cvarrToMat(iplImage);
            imgCallback(_mat);

            free(pRGBbuffer);
        }
    }
}



void sub_callback(const vector<cv::Point2f> &data)
{
    if(data.size()>0)
    {
        if(data[0].x==data[0].y){
            if(data[0].x!=Mode_last)
            {
                Mode = data[0].x;
                Mode_last = Mode;
                cout<<"Mode has change to "<<data[0]<<endl;
            }
        }
    }
}

