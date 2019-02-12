#ifndef IMG_PUBLISH_H
#define IMG_PUBLISH_H

#include <iostream>
#include <boost/shared_ptr.hpp>
#include <boost/asio.hpp>
#include <boost/asio/placeholders.hpp>
#include <boost/system/error_code.hpp>
#include <boost/bind/bind.hpp>
#include <opencv2/opencv.hpp>


namespace cubot
{
using namespace std;

    class Publish
    {
        private:
            boost::asio::io_service                 pub_io;
            boost::asio::ip::tcp::acceptor          acceptor;
            boost::asio::ip::tcp::socket            pub_socket;
            boost::system::error_code               ignored_error;
            void run();

        public:
            Publish(string name);

            std::size_t  pub( vector<cv::Point2f> &points);

            ~Publish();
    };
}
#endif
