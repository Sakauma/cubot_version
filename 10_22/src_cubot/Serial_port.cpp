//
// Created by snow on 18-4-12.
//

#include "cubotcpp/Serial_port.h"

namespace cubot{

    serial_port::serial_port(sp_callback_ cb, const std::string& sp_dev, const int &baud_rate)
            :sp_callback(cb),
             sp(io_sp, sp_dev)
    {
        sp.set_option(boost::asio::serial_port::baud_rate(baud_rate));
        sp.set_option(boost::asio::serial_port::flow_control(boost::asio::serial_port::flow_control::none));
        sp.set_option(boost::asio::serial_port::parity(boost::asio::serial_port::parity::none));
        sp.set_option(boost::asio::serial_port::stop_bits(boost::asio::serial_port::stop_bits::one));
        sp.set_option(boost::asio::serial_port::character_size(8));

        t0 = boost::thread(boost::bind(&serial_port::read, this));

        io_sp.run();
    }

    serial_port::~serial_port(){
    }

    std::size_t serial_port::write_sync(uint8_t *send_buf, int send_size)
    {
        return sp.write_some(boost::asio::buffer(send_buf, send_size), error_sp);
    }

    void serial_port::spJoin()
    {
        t0.join();
    }

    void serial_port::read()
    {
        while(1)
        {
            int size_recv = 0;
            size_recv = sp.read_some(boost::asio::buffer(recv_buf, 1), error_sp);
            if (size_recv>0){
                //std::cout<<"Serial port has received data "<<size_recv<<" bytes"<<std::endl;
                sp_callback(recv_buf);
            }
            usleep(1000);
        }
    }

    int serial_port::SerialPublish(const std::vector<cv::Point2f> &connors)
    {
        if((connors[0].x==0)&&(connors[0].y==0))
            return 0;

        //connors'size() is not more than 3
        int  x = int(connors[0].x*300);
        int  y = int(connors[0].y*300);
        int  z = int(connors[1].x);


        uint8_t x_h = x>>8;
        uint8_t x_l = x;
        uint8_t y_h = y>>8;
        uint8_t y_l = y;

        uint8_t send_buf[7]={0};
        send_buf[0]=0xaa;
        send_buf[1]=x_h;
        send_buf[2]=x_l;
        send_buf[3]=y_h;
        send_buf[4]=y_l;
        send_buf[5]=z;
        send_buf[6]=0xdd;

        int isSpSucss = write_sync (send_buf,7);
        if(isSpSucss>0)
            std::cout<<" SerialPort publish "<<isSpSucss<<" bytes sucessfully."<<std::endl;
        else
            std::cerr<<"serial port connects failed...\n";
        return isSpSucss;
    }
}
