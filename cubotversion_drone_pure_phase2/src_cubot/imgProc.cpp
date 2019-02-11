#include <eigen3/Eigen/Geometry>
#include <opencv2/core/eigen.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>

#include "cubotcpp/timer.h"
#include "cubotcpp/Serial_port.h"
#include "cubotcpp/imgSubscrible.h"
#include "armor/armor_main.h"

using namespace boost::posix_time;
using namespace cubot;

typedef Eigen::Matrix< double , 5  ,1 > Vector5d;

#ifdef videowriter
Size videoSize(1280, 1024);
VideoWriter writer_armor;
int frame_num_of_armor = 20;
int cnt_of_armor = 0;
#endif

Eigen::Isometry3d   calMatrix;
Eigen::Matrix3d     cameraMatrix;
Vector5d            distCoeffs;

Mat    img_backups;
float  distance0 = 0;
string now();
void   sp_callback( const uint8_t *recv_buf );
cv::Point2f Pixel2Angle( cv::Point2f &connor );
serial_port sp( sp_callback, "/dev/SerialPortCom0", 115200 );

void callback( const CubotImg &img )
{
    if ( img.img.empty() )
         return;

    double t = ( double )cv::getTickCount();
    img_backups = img.img.clone();

#ifdef videowriter
    cnt_of_armor++;
    if ( cnt_of_armor % frame_num_of_armor == 0 )
    {
         cnt_of_armor = 0;
         writer_armor << img_backups;
    }
#endif

    int   is_small_armor;
    cv::Point2f target = autohit::detection( img_backups, distance0, is_small_armor );
    //target = Point2f(400,300);
    if ( ( target.x >= 0 ) && ( target.y >= 0 ) )
    {
        vector<Point2f> targetPoints;
        targetPoints.emplace_back( Pixel2Angle( target ) );
        targetPoints.emplace_back( Point2f( distance0, is_small_armor ) );
        if ( !targetPoints.empty() )
        {
             t = ( ( double )cv::getTickCount() - t) / ( cv::getTickFrequency() * 1000 );
             cout << "successed to detect " << target << "   " << targetPoints[0]
                  << " distance  " << distance0 << "  "<< now() << "  time cost is " << t << "ms" << endl;
             sp.SerialPublish( targetPoints );
        }
    }
    else
    {
        cout << "faild to detect " << now() << endl;
    }
}

void sp_callback( const uint8_t *recv_buf )
{
    //Attention: max size is 6
    // Serial Port receive data
}

float x_bias = 15, y_bias = 0;
int main( int argc, char *argv[] )
{
    cout << "start" << endl;
    //sleep(10);

    FileStorage file_1( "../cfg/camera_params_UAV.yml", FileStorage::READ );
    Mat cvCameraMatrix,  cvDistCoeffs, cvCalMatrix;
    file_1["cameraMatrix"] >>  cvCameraMatrix;
    file_1["distCoeffs"] >>  cvDistCoeffs;
    file_1["calMatrix"] >>  cvCalMatrix;
    file_1["x_bias"] >>  x_bias;
    file_1["y_bias"] >>  y_bias;
    cout<<"Read camera params success."<<endl;

    cv::cv2eigen(cvCalMatrix, calMatrix.matrix());
    cv::cv2eigen(cvCameraMatrix, cameraMatrix);
    cv::cv2eigen(cvDistCoeffs, distCoeffs);

#ifdef videowriter
    ptime  time_now(second_clock::local_time());
    string now_iso_str(to_iso_string(time_now));
    string videodirname="../video/"+now_iso_str+"/";
    const char * videodirnamestr=videodirname.c_str();
    mkdir(videodirnamestr,S_IRUSR | S_IWUSR | S_IXUSR);
    string video_armor_name=videodirname+"Realtime_Armor.avi";
    writer_armor.open(video_armor_name,CV_FOURCC('M', 'J', 'P', 'G'), 20, Size(1280,1024));
#endif

    ImgSub imgsub("MySharedMemory", boost::bind(callback, _1));
    imgsub.subJoin();

    return 0;
}

cv::Point2f Pixel2Angle( cv::Point2f &connor )
{
    Eigen::Vector3d pixel;
    pixel << connor.x, connor.y, 1;

    Eigen::Vector3d  P_cam, P_distortion;//get P in camera coordinate
    P_cam = cameraMatrix.inverse() * pixel;

    double  r2 = pow( P_cam.x(), 2 ) + pow( P_cam.y(), 2 );

    P_distortion.x() = P_cam.x() * ( 1 + distCoeffs[0] * pow(r2, 1) + distCoeffs[1] * pow(r2, 2) + distCoeffs[2] * pow(r2, 3) )
                       + 2 * distCoeffs[3] * P_cam.x() * P_cam.y() + distCoeffs[4] * ( r2 + 2 * pow( P_cam.x(), 2 ) );

    P_distortion.y() = P_cam.y() * ( 1 + distCoeffs[0] * pow(r2, 1) + distCoeffs[1] * pow(r2, 2) + distCoeffs[2] * pow(r2, 3) )
                       + distCoeffs[3] * ( r2 + 2 * pow( P_cam.y(), 2 ) ) + 2 * distCoeffs[4] * P_cam.x() * P_cam.y();

    P_distortion.z() = 1;

    Eigen::Vector3d P_hold =  calMatrix.rotation().inverse() * ( P_distortion-calMatrix.translation() );

//    if(distance0>400){
//        x_bias=18.5;  //15
//        y_bias=18;
//    }else if(distance0>350){
//        x_bias=17.5;
//        y_bias=16.8;
//    }else if(distance0>260){
//        x_bias=17.5;
//        y_bias=15;
//    }else{
//        x_bias=17.5;
//        y_bias=13;
//    }

    int x0 = 2;
    int y0 = 0.5;
    if ( distance0 < 250 )
    {
        x_bias = 14.5 + x0;
        y_bias = 15.4 + y0;
    }
    else if ( distance0 < 300 )
    {
        x_bias = 15.5 + x0;
        y_bias = 15.4 + y0;
    }
    else if ( distance0 < 350 )
    {
        x_bias = 16.5 + x0;
        y_bias = 15.6 + y0;
    }
    else if ( distance0 < 400 )
    {
        x_bias = 17.5 + x0;
        y_bias = 15.8 + y0;
    }
    else if ( distance0 < 460 )
    {
        x_bias = 18.5 + x0;  //15
        y_bias = 16 + y0;
    }
    else
    {
        x_bias =19.5 + x0;  //15
        y_bias =16.2 + y0;
    }
//    else{
//        x_bias=17.5;
//        y_bias=13;
//    }


    cv::Point2f angles;
    angles.x =  atan2(P_hold(0),1) * 180.0 / M_PI-x_bias;   //get the yaw angle ?
    angles.y = (atan2(-P_hold(1), 1) * 180.0 / M_PI)+y_bias;    //get the pitch angle  3


//    if(distance0>950){
//        angles.y+=5.8;
//        angles.x-=0.5;
//    }
//    else if(distance0>750)
//        angles.y+=4.25; //4
//    else if(distance0>550)
//        angles.y+=3.; //4
//    else if(distance0>330)
//        angles.y+=2;
//    else
//        angles.y+=1.5;

    return angles;
}

string now()
{
    time_t t = time(0);
    char buffer[9] = {0};
    strftime(buffer, 9, "%H:%M:%S", localtime(&t));
    return string(buffer);
}
