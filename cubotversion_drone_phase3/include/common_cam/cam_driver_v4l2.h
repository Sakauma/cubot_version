//
// Created by snow on 18-3-16.
//

#ifndef V4L_COMMON_CAMERA_CAM_DRIVER_V4L2_H
#define V4L_COMMON_CAMERA_CAM_DRIVER_V4L2_H

#include <errno.h>
#include <fcntl.h>
#include <linux/videodev2.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <unistd.h>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <boost/bind.hpp>
#include <boost/thread.hpp>
#include <opencv2/opencv.hpp>

namespace cubot{

    typedef boost::function<void(const cv::Mat &)> mat_cb_;
    class CamCapByV4l2{

    public:
        CamCapByV4l2(mat_cb_ cb_, const char* file_video, int width, int height);
        ~CamCapByV4l2();

    public:
        int cam_init_v4l2(void);
        int cam_grab_v4l2(void);

        void save_imgs(const std::string &filepath, const int &start_num);
        void join();


    private:

        int fd;
        uchar *buffer;
        struct v4l2_buffer buf;
        int Width = 640;
        int Height= 480;
        boost::thread t0;
        mat_cb_       mat_cb;
        IplImage      *img0;
        cv::Mat       img_mat;    //BGR
        void cam_mat_v4l2(void);


        enum   v4l2_buf_type type;
        struct v4l2_streamparm setfps;
        struct v4l2_capability cap;
        struct v4l2_fmtdesc fmtdesc;
        struct v4l2_format fmt;
        struct v4l2_requestbuffers req;

    };
}

#endif //V4L_COMMON_CAMERA_CAM_DRIVER_V4L2_H
