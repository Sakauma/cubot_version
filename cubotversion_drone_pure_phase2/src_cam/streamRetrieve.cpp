#include "daHua_usb/StreamRetrieve.h"
#include"cubotcpp/imgPublish.h"
#include"cubotcpp/timer.h"

using namespace cubot;

StreamRetrieve::StreamRetrieve(IStreamSourcePtr& streamSptr,ImgCallback imgCB, const int &img_width, const int &img_hight):
        m_streamSptr(streamSptr), imgCallback(imgCB), Width(img_width), Height(img_hight)
{
    pro_thread  = boost::thread(boost::bind(&StreamRetrieve::Proc, this));
}

void StreamRetrieve::join()
{
    pro_thread.join();
}

void StreamRetrieve::Proc()
{
    int errorNum=0;
    std::cout<<" camera runing..."<<std::endl;
    while (1)
    {
        CFrame frame;
        if (!m_streamSptr)
        {
            printf("m_streamPtr is NULL.\n");
            return;
        }
        bool isSuccess = m_streamSptr->getFrame(frame, 200);
        if (!isSuccess)
        {
            errorNum++;
            if(errorNum>2000){
                cerr<<"getFrame fail, and return..."<<endl;
                return;
            }else{
                printf("getFrame  fail.\n");
                continue;
            }
        }else
            errorNum=0;

        bool isValid = frame.valid();
        if (!isValid)
        {
            errorNum++;
            if(errorNum>2000){
                cerr<<"frame invalid, and return..."<<endl;
                return;
            }else{
                printf("frame is invalid!\n");
                continue;
            }
        } else
            errorNum=0;

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





