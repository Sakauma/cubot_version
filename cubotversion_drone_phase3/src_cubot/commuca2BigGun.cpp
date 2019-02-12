#include <minicom.h>
#include "cubotcpp/timer.h"
#include "cubotcpp/Serial_port.h"
#include "cubotcpp/SubscribeAsync.h"
#include <opencv2/core/core.hpp>
#include <eigen3/Eigen/Geometry>
#include <opencv2/core/eigen.hpp>
#include<math.h>


using namespace cubot;
using namespace boost::posix_time;

int  counter=0;
void timercallback();
void sub_callback(const vector<cv::Point2f> &iconnors);

Subscrible  sub(sub_callback,"127.0.0.1",1234);
serial_port_w sp("/dev/ttyUSB0", 115200);

typedef Eigen::Matrix< double , 5  , 1> Vector5d;
Eigen::Isometry3d calMatrix;
Eigen::Matrix3d cameraMatrix;
Vector5d distCoeffs;



cv::Point2f Pixel2Angle(cv::Point2f &connor)
{

    Eigen::Vector3d pixel;
    pixel<<connor.x,connor.y,1;
    Eigen::Vector3d  P_cam;//get P in camera coordinate
    P_cam = cameraMatrix.inverse()*pixel;


    Eigen::Vector3d P_hold =  calMatrix.rotation().inverse()* (P_cam-calMatrix.translation());

    cout<<P_hold<<endl;
    cv::Point2f angles;
    angles.x=atan2(P_hold(0),1)*180.0/M_PI;   //get the yaw angle
    angles.y=atan2(-P_hold(1),1)*180.0/M_PI+3;   //get the pitch angle

    return angles;

}


int main(int argc, char* argv[])
{

    FileStorage fs("../cfg/camera_params_hero.yml", FileStorage::READ);
    Mat cvCameraMatrix,  cvDistCoeffs, cvCalMatrix;
    fs["cameraMatrix"] >>  cvCameraMatrix;
    fs["distCoeffs"] >>  cvDistCoeffs;
    fs["calMatrix"] >>  cvCalMatrix;
    cout<<"Read camera params success."<<endl;

    cv::cv2eigen(cvCalMatrix, calMatrix.matrix());
    cv::cv2eigen(cvCameraMatrix, cameraMatrix);
    cv::cv2eigen(cvDistCoeffs, distCoeffs);

//    cv::Point2f a;
//    a.x=1279;
//    a.y=1023;

//        cout<<Pixel2Angle(a)<<endl;

    cout<<"Client start."<<endl;
    creatTimerCallback timer(bind(timercallback),1000);//定时器查看图片
    sub.readJoin();

    return 0;
}

void timercallback()
{
    cout << "fre : " << counter<<endl;
    counter=0;
}

void sub_callback(const vector<cv::Point2f> &connors)
{


    cv::Point2f pixel=  connors[0];
    cv::Point2f angle = Pixel2Angle(pixel);
    //connors'size() is not more than 3
    int  x = int(angle.x*100);
    int  y = int(angle.y*100);
    if((x==0)&&(y==0))
        return;

    //cout<<x<<"  "<<y<<endl;

    counter++;
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
    send_buf[5]=0;
    send_buf[6]=0xdd;

    
    cout<<sp.write_sync (send_buf,7)<<endl;
}
