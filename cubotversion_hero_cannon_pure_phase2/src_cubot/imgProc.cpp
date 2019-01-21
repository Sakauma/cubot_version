#include <eigen3/Eigen/Geometry>
#include <opencv2/core/eigen.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>

#include "cubotcpp/timer.h"
#include "cubotcpp/Serial_port.h"
#include "cubotcpp/imgSubscrible.h"
#include "armor/armor_main.h"

using namespace boost::posix_time;

#define Send
using namespace cubot;

typedef Eigen::Matrix< double, 5, 1> Vector5d;

Eigen::Isometry3d   calMatrix;
Eigen::Matrix3d     cameraMatrix;
Vector5d            distCoeffs;

#ifdef videowriter
Size videoSize(1280, 1024);
VideoWriter writer_armor_cannon;
int FrameNumofArmor = 50; //装甲识别模式，每15帧保存一帧图像
int CntOfArmor = 0;
#endif

Mat  imageCopy;
void timercallback();
void sp_callback(const uint8_t *recv_buf);
cv::Point2f Pixel2Angle(cv::Point2f &connor);
serial_port sp(sp_callback, "/dev/cannon_Port", 115200);
cv::Point2f tmpPoint;

void callback( const CubotImg &img )
{
    if ( img.img.empty() )
        return;
    double t0 = getTickCount();

    imageCopy = img.img.clone();

#ifdef videowriter
    CntOfArmor++;
    if(CntOfArmor%FrameNumofArmor == 0)
    {
        CntOfArmor = 0;
        writer_armor_cannon<<imageCopy;
    }
#endif

    float distance;
    cv::Point2f target = autohit::detection( imageCopy, distance );
    std::cout << "target" << target << std::endl;

    if ( ( target.x >= 0 ) && ( target.y >= 0 ) )
    {
        double t1 = getTickCount();
        cout << ( t1 - t0 ) * 1000 / getTickFrequency() << "ms" << endl;
        circle( imageCopy, target,50,Scalar(0, 0, 255), 10 );
        std::cout << "target" << target << std::endl;
        vector<Point2f> targetPoints;
        targetPoints.emplace_back( Pixel2Angle( target ) );
        std::cout << "targetPoints" << targetPoints << std::endl;
        if ( !targetPoints.empty() )
        {
            cout << "successed to detect " << target << "   " << targetPoints[0] << endl;
#ifdef Send
            sp.SerialPublish(targetPoints);
//            if((targetPoints[0].x-tmpPoint.x)>)
//            tmpPoint

#endif
        }
    }
#ifdef show
    imshow( "rev", imageCopy );
    waitKey(10);
#endif
}


void timercallback()
{
    if ( imageCopy.empty() )
        return;

}

void sp_callback( const uint8_t *recv_buf )
{
    //Attention: max size is 6
    // Serial Port receive data
}

int main( int argc, char *argv[] )
{
    sleep(3);
    cout << "start" << endl;

    FileStorage file_1( "../cfg/camera_params_cam_5.yml", FileStorage::READ );
    Mat cvCameraMatrix,  cvDistCoeffs, cvCalMatrix;
    file_1["cameraMatrix"] >> cvCameraMatrix;
    file_1["distCoeffs"] >> cvDistCoeffs;
    file_1["calMatrix"] >> cvCalMatrix;
    cout<<"Read camera params success."<<endl;

    cv::cv2eigen( cvCalMatrix, calMatrix.matrix() );
    cv::cv2eigen( cvCameraMatrix, cameraMatrix );
    cv::cv2eigen( cvDistCoeffs, distCoeffs );

#ifdef videowriter
    ptime  time_now(second_clock::local_time());
    string now_iso_str(to_iso_string(time_now));

    string videodirname = "../video/"+now_iso_str+"/";
    const char * videodirnamestr = videodirname.c_str();
    mkdir(videodirnamestr,S_IRUSR | S_IWUSR | S_IXUSR);
    string video_armor_name = videodirname+"Realtime_Armor_cannon.avi";
    writer_armor_cannon.open(video_armor_name,CV_FOURCC('M', 'J', 'P', 'G'), 20, Size(1280,1024));
#endif

    ImgSub imgsub("CANNONIMG", boost::bind(callback, _1));
    imgsub.subJoin();

    return 0;
}

cv::Point2f Pixel2Angle( cv::Point2f &connor )
{

    Eigen::Vector3d pixel;
    pixel << connor.x, connor.y, 1;

    Eigen::Vector3d  P_cam,P_distortion;
    P_cam = cameraMatrix.inverse() * pixel;

    double  r2= pow( P_cam.x(), 2 ) + pow( P_cam.y(), 2 );

    P_distortion.x() = P_cam.x() * ( 1 + distCoeffs[0] * pow( r2, 1 ) + distCoeffs[1] * pow( r2, 2 ) + distCoeffs[2] * pow( r2, 3 ) )
                       + 2 * distCoeffs[3] * P_cam.x() * P_cam.y() + distCoeffs[4] * ( r2 + 2 * pow( P_cam.x(), 2 ) );

    P_distortion.y() = P_cam.y() * ( 1 + distCoeffs[0] * pow( r2, 1 ) + distCoeffs[1] * pow( r2, 2 ) + distCoeffs[2] * pow( r2, 3 ) )
                       + distCoeffs[3] * ( r2 + 2 * pow(P_cam.y(), 2 ) ) + 2 * distCoeffs[4] * P_cam.x() * P_cam.y();
    P_distortion.z() = 1;

    Eigen::Vector3d P_hold =  calMatrix.rotation().inverse() * ( P_distortion - calMatrix.translation() );

    cv::Point2f angles;
    angles.x = ( atan2 ( P_hold(0), 1) * 180.0 / M_PI);   //get the yaw angle
    angles.y = ( atan2 (-P_hold(1), 1) * 180.0 / M_PI);   //get the pitch angle

    cout << "angles ---------------> " << angles << endl;

    return angles;

}
