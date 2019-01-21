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

using namespace Dahua::GenICam;
using namespace Dahua::Infra;
using namespace cv;
typedef boost::function<void( Mat &img)> ImgCallback;

class StreamRetrieve
{

public:
	StreamRetrieve(IStreamSourcePtr& streamSptr,ImgCallback imgCB, const int &img_width, const int &img_hight);
	void join();
private:
	void  threadProc();

	cv::Mat  _mat;
	IStreamSourcePtr m_streamSptr;
	ImgCallback imgCallback;
	boost::thread  pro_thread;

	int Height;
	int Width;

	void Proc();
};

#endif