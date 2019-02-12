#ifndef IMG_SUB_H
#define IMG_SUB_H

#include <iostream>
#include <boost/asio.hpp>
#include <boost/asio/placeholders.hpp>
#include <boost/system/error_code.hpp>
#include <boost/thread/thread.hpp>
#include <boost/bind.hpp>
#include <boost/function.hpp>
#include <boost/array.hpp>
#include <opencv2/opencv.hpp>



namespace cubot
{
using namespace boost::asio;
using namespace std;

    typedef boost::function<void(const vector<cv::Point2f> &)> socketCB;
    class Subscrible
    {
        private:
            io_service sub_io;
            ip::tcp::endpoint end_point;
            boost::system::error_code ignored_error;
            ip::tcp::socket subsocket;
            vector<vector<cv::Point2f>>  connors;
            boost::thread read_thread;
            socketCB socketcallback_;
            void read();

        public:
            Subscrible(string name,socketCB cb);
            ~Subscrible();
            void     readJoin();
    };

}

#endif
