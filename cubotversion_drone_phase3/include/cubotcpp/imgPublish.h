#ifndef IMG_PUBLISH_H
#define IMG_PUBLISH_H

#include <boost/date_time.hpp>
#include <boost/date_time/gregorian/gregorian.hpp>
#include <boost/timer.hpp>
#include <boost/interprocess/managed_shared_memory.hpp>
#include <boost/interprocess/sync/named_mutex.hpp>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>

typedef struct
{
    cv::Size  size;
    int       type;
    int       seq;
    boost::interprocess::managed_shared_memory::handle_t  handle;
    boost::posix_time::time_duration      timestamp;
} CubotImgPtr;

namespace cubot
{
          namespace  ip = boost::interprocess;
    using namespace  boost::posix_time;
    using namespace  std;
    using namespace  cv;

    class ImgPub
    {

    public:
        ImgPub( const char *shmname, const Mat &img);
        void pub(const Mat &captured_image);

        ~ImgPub();
    private:
        ip::managed_shared_memory *segment;
        CubotImgPtr* shared_image_header;
        void *  shared_image_data_ptr;
        ip::named_mutex  *img_mtx;
        string shm_name;
        ptime epoch;
    };

}
#endif
