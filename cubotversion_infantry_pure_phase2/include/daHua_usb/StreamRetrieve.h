#ifndef _STREAMRETRIEVE_H
#define _STREAMRETRIEVE_H

#include "GenICam/StreamSource.h"
#include "GenICam/System.h"
#include "Media/VideoRender.h"
#include "Media/ImageConvert.h"
#include <opencv2/opencv.hpp>
#include "boost/bind.hpp"
#include "boost/function.hpp"
#include "boost/thread.hpp"
#include "daHua_usb/camParaConfig.h"


using namespace Dahua::GenICam;
using namespace Dahua::Infra;
using namespace cv;
typedef boost::function<void( Mat &img)> ImgCallback;

class StreamRetrieve
{

public:
	StreamRetrieve(ICameraPtr &cameraSptr,daHuaPara_str &daHuaPara, IStreamSourcePtr& streamSptr,
				   ImgCallback imgCB);
    void join();
private:
	void  threadProc();

	cv::Mat          _mat;
	IStreamSourcePtr m_streamSptr;
	ImgCallback 	 imgCallback;
    boost::thread  	 pro_thread;
	ICameraPtr 		 _cameraSptr;
	daHuaPara_str 	 _daHuaPara;
	int Height;
	int Width;

    void Proc();
};

#endif