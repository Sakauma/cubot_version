#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
#include <opencv2/core/eigen.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>

#include "cubotcpp/timer.h"
#include "cubotcpp/Serial_port.h"
#include "cubotcpp/imgSubscrible.h"
#include "cubotcpp/PublishAsync.h"
#include "armor/armor_main.h"
#include "../include/fordefine.h"

using namespace boost::posix_time;
using namespace cubot;

typedef Eigen::Matrix< double , 5  , 1> Vector5d;
Eigen::Isometry3d calMatrix;
Eigen::Matrix3d cameraMatrix;
Vector5d distCoeffs;

uint8_t Mode = 2;
int is_sock_pub_succ = -1;
vector<Point2f> v_pub_Mode(1);
bool is_fuwen_recv_flag = true, is_armor_recv_flag = true;

int sum0 = 0;
int sum_true = 0;

Mat  img_backups;
cv::Point2f center(0,0);//应该为全局变量
cv::Point2f send_data_center(0,0);//应该为全局变量
cv::Point2f Pixel2Angle(cv::Point2f &connor);
void pub_callback(const vector<uint8_t> &data);
void sp_callback(const uint8_t *recv_buf);

#ifndef versiontest
serial_port sp(sp_callback, "/dev/SerialPortCom0", 115200);
#endif

#ifdef videowriter
Size videoSize(1280, 1024);
VideoWriter writer_armor;
VideoWriter writer_rune;
//int frame_num_of_rune = 5;   //大符识别模式，每5帧保存一帧图像T = t/(cvGetTickFrequency()*1000);
int frame_num_of_armor = 50; //装甲识别模式，每15帧保存一帧图像
int cnt_of_rune = 0;
int cnt_of_armor = 0;
#endif

#ifdef log_output
#include <fstream>
ofstream fout;
#endif

#ifdef get_error_image
#include <sys/stat.h>
string dirname[7];
int save_num = 0;
#endif

bool calculate_sp_statePost( std::vector<cv::Point2f> adjacent_three_targets, std::vector<float> adjacent_three_times, float &sp_x_speed, float &sp_y_speed, float &sp_x_acceleration, float &sp_y_acceleration )
{
    cv::Point2f sp1(-2, -2), sp2(-2, -2), sp3(-2, -2);
    float t2, t3;
    float v_sp2_x, v_sp3_x, v_sp2_y, v_sp3_y;

    sp1 = adjacent_three_targets[0];
    sp2 = adjacent_three_targets[1];
    sp3 = adjacent_three_targets[2];
    t2  = adjacent_three_times[1];
    t3  = adjacent_three_times[2];

    v_sp2_x = ( sp2.x - sp1.x ) / t2;
    v_sp3_x = ( sp3.x - sp2.x ) / t3;
    v_sp2_y = ( sp2.y - sp1.y ) / t2;
    v_sp3_y = ( sp3.y - sp2.y ) / t3;

    sp_x_speed = ( v_sp2_x + v_sp3_x ) / 2;
    sp_y_speed = ( v_sp2_y + v_sp3_y ) / 2;
    sp_x_acceleration = ( v_sp3_x - v_sp2_x ) / ( ( t2 + t3 ) / 2 );
    sp_y_acceleration = ( v_sp3_y - v_sp2_y ) / ( ( t2 + t3 ) / 2 );
}

cv::Point2f linear_predict( cv::Point2f sp_optimize, float first_sp_x_speed, float first_sp_y_speed, float first_sp_x_acceleration, float first_sp_y_acceleration, float predict_time )
{
    cv::Point2f sp_predict(-2, -2);
    sp_predict.x = sp_optimize.x + first_sp_x_speed * predict_time + 0.5 * first_sp_x_acceleration * pow( predict_time, 2 );
    sp_predict.y = sp_optimize.y + first_sp_y_speed * predict_time + 0.5 * first_sp_y_acceleration * pow( predict_time, 2 );
    return sp_predict;
}

Publish_udp pub_mode( pub_callback );

void callback( const CubotImg &img )
{
    if ( img.img.empty() )
         return;
    img_backups = img.img.clone();

#ifdef log_output
    //输出系统时间
    ptime  time_now(second_clock::local_time());
    string now_iso_str(to_iso_string(time_now));
    fout <<"\n**********************************************"<<endl;
    fout <<now_iso_str<< endl;
#endif

    int  result_num = 0;  //范围1-9

    cv::Point2f target(-1, -1);

    cv::Point2f temp_center(-1, -1);

    bool need_to_return = false;


#ifdef dafutest
    Mode = 1;
#endif

#ifdef armortest
    Mode = 2;
#endif

    /////////////////视觉检测部分////////////////
    if ( Mode == 1 )
    {
        //Fuwen
        cout<<"===============come into FuWen mode===================="<<endl;
#ifdef log_output
        fout<<"===============come into FuWen mode===================="<<endl;
#endif


#ifdef videowriter
        cnt_of_rune++;
        if ( cnt_of_rune % frame_num_of_rune == 0 )
        {
             cnt_of_rune = 0;
             writer_rune<<imageCopy;
        }
#endif

        /* 函数功能大符检测
        * src_img：  原始图像（三通道）
        * p2f_angle：打击点的像素坐标
        * send_num： 数码管识别结果
        */
//       bool isdetected = txdetector.detect( img_backups, target, result_num );   //调用大符主函数，返回bool类型的isdetected为ture or flase

        if ( true )  //isdetected == ture
        {
             Mode = 0;

             is_fuwen_recv_flag = true;

             sum0++;

             cout<<"isdetected"<<"  "<<target<<"   "<<result_num<<"  "<<sum0<<endl;
        }
    }
        //Mode 非零
    else if ( Mode )
    {
        cout<<"=============come into AutoHit mode====================="<<endl;

#ifdef log_output
        fout<<"=============come into AutoHit mode====================="<<endl;
#endif
        float dist;
        float T;
#ifdef videowriter
        cnt_of_armor++;
        if ( cnt_of_armor % frame_num_of_armor == 0)
        {
             cnt_of_armor = 0;
             writer_armor <<imageCopy;
        }
#endif
        target = AutoHit::detection( img_backups, dist, T );   //调用装甲识别主函数，返回target

        static std::vector<cv::Point2f> adjacent_three_targets;
        static std::vector<float> adjacent_three_times;
        static cv::Point2f sp1(-2, -2), sp2(-2, -2), sp3(-2, -2);
        static float t1 = -1, t2 = -1, t3 = -1;
        static int flag = 0;
        const cv::Point2f void_sp(-2, -2);
        const float void_t = -1;
        float sp_x_speed, sp_y_speed, sp_x_acceleration, sp_y_acceleration;
        float predict_time = -1;
        cv:: Point2f predict_sp(-2, -2);

        if ( target.x < 0 )
        {
             flag = 0;
             sp1 = void_sp;
             sp2 = void_sp;
             sp3 = void_sp;
             t1 = void_t;
             t2 = void_t;
             t3 = void_t;
             adjacent_three_targets.clear();
             adjacent_three_times.clear();
        }

        if ( target.x > 0 && flag == 0 )
        {
            if ( sp1.x < 0)
            {
                 sp1 = target;
                 t1 = T;
                 adjacent_three_targets.push_back( sp1 );
                 adjacent_three_times.push_back( t1 );
            }
            else if ( sp1.x > 0 && sp2.x < 0 )
            {
                sp2 = target;
                t2 = T;
                adjacent_three_targets.push_back( sp2 );
                adjacent_three_times.push_back( t2 );
            }
            else if ( sp2.x > 0 && sp3.x < 0 )
            {
                sp3 = target;
                t3  = T;
                adjacent_three_targets.push_back(sp3);
                adjacent_three_times.push_back(t3);
            }
        }

        if ( adjacent_three_targets.size() == 3 && adjacent_three_times.size() == 3 && flag == 0 )
        {
             flag = 1;
             std::cout << "连续三个目标点:" << std::endl << adjacent_three_targets << std::endl;
             std::cout << "连续三个时间段：" << std::endl << adjacent_three_times[0] << std::endl << adjacent_three_times[1] << std::endl << adjacent_three_times[2] << std::endl;
        }
        else if ( adjacent_three_targets.size() == 3 && adjacent_three_times.size() == 3 && target.x > 0 && flag == 1 )
        {
            adjacent_three_targets[0] = adjacent_three_targets[1];
            adjacent_three_targets[1] = adjacent_three_targets[2];
            adjacent_three_targets[2] = target;

            adjacent_three_times[0] = adjacent_three_times[1];
            adjacent_three_times[1] = adjacent_three_times[2];
            adjacent_three_times[2] = T;
            std::cout << "连续三个目标点:" << std::endl << adjacent_three_targets << std::endl;
            std::cout << "连续三个时间段：" << std::endl << adjacent_three_times[0] << std::endl << adjacent_three_times[1] << std::endl << adjacent_three_times[2] << std::endl;
            if( ( adjacent_three_targets[0].x < adjacent_three_targets[1].x < adjacent_three_targets[2].x) || (adjacent_three_targets[0].x > adjacent_three_targets[1].x > adjacent_three_targets[2].x ) )
            {
                calculate_sp_statePost(adjacent_three_targets, adjacent_three_times, sp_x_speed, sp_y_speed, sp_x_acceleration, sp_y_acceleration );
                predict_time = ( adjacent_three_times[0] + adjacent_three_times[1] + adjacent_three_times[2] ) / 3;
                predict_sp = linear_predict( target, sp_x_speed, sp_y_speed, sp_x_acceleration, sp_y_acceleration, predict_time );
                std::cout<< "预测目标点坐标：" << predict_sp << std::endl;
                target = predict_sp;
            }
            else
                target = predict_sp;
        }
    }
    else
    {
        //cout<<"=============come into Mode equals to zero====================="<<endl;
    }
    /////////////////发射部分///////////////
    if ( ( target.x >= 0) && ( target.y >= 0 ) )
    {
        if ( Mode != 1 )

            resize( img_backups, img_backups, img_backups.size() * 2 );

        circle( img_backups, target, 50, Scalar(0, 0, 255), 10 );

        vector<Point2f> targetPoints;

        targetPoints.emplace_back( Pixel2Angle( target ) );

        targetPoints.emplace_back( Point2f( result_num, 0 ) );
        cout << "isdetected" << "  " << target << endl;


#ifdef log_output
        fout<<"isdetected"<<"  "<<target<<endl;
#endif

        if ( !targetPoints.empty() )
        {
#ifndef versiontest
            int isSpSucss=sp.SerialPublish(targetPoints);
            fout<<" SerialPort publish "<<isSpSucss<<" bytes sucessfully."<<std::endl;
#endif
        }
    }


#ifdef show
    namedWindow( "image", 0 );
    resize( img_backups, img_backups, Size( img_backups.size() / 3 ) );
    imshow( "image", img_backups );
    waitKey(1);
#endif
}

void sp_callback( const uint8_t *recv_buf )
{
    // Attention: max size is 6
    // Serial Port receive data
    // cout<<"serial port receive data\n";
    if ( recv_buf[0] == 1 )
    {
        // 符文模式
        if ( is_fuwen_recv_flag )
        {
             // 多次发送，但是只接收一次
             Mode = 1;
             is_fuwen_recv_flag = false;
             is_armor_recv_flag = true;

             //进程间通信，向相机节点发送信息，使得相机可以随模式切换改变参数
             v_pub_Mode[0] = cv::Point2f(1, 1);
             is_sock_pub_succ = pub_mode.pub( v_pub_Mode );
        }

        else
        {
            //cout<<"======符文模式切换======= \n";
        }
    }
    else if ( recv_buf[0] )
    {
        if( is_armor_recv_flag )
        {
            // 自动瞄准模式
            Mode = 2;
            is_fuwen_recv_flag = true;
            is_armor_recv_flag = false;

            // 进程间通信，向相机节点发送信息，使得相机可以随模式切换改变参数
            v_pub_Mode[0] = cv::Point2f(2,2);
            is_sock_pub_succ = pub_mode.pub(v_pub_Mode);
            cout << "======自动瞄准模式======= "
                 << "recv flag is " << int( recv_buf[0] )
                 << " publish " << is_sock_pub_succ << " bytes" << endl;
        }

        else
        {
            cout << "======自动瞄准模式======= \n";
        }
    }
    else
    {
        cerr << "==================receive zero======================= " << int( recv_buf[0] ) << endl;
    }
}

int main( int argc, char *argv[] )
{
    cout << "start" << endl;

    FileStorage file_1("../cfg/camera_params_cam_5.yml", FileStorage::READ );
    Mat cvCameraMatrix,  cvDistCoeffs, cvCalMatrix;
    file_1["cameraMatrix"] >>  cvCameraMatrix;
    file_1["distCoeffs"] >>  cvDistCoeffs;
    file_1["calMatrix"] >>  cvCalMatrix;
    cout << "Read camera params success." << endl;

    cv::cv2eigen( cvCalMatrix, calMatrix.matrix() );
    cv::cv2eigen( cvCameraMatrix, cameraMatrix );
    cv::cv2eigen( cvDistCoeffs, distCoeffs );

    ptime  time_now( second_clock::local_time() );
    string now_iso_str( to_iso_string( time_now ) );


#ifdef videowriter
    int armor_cnt = 0;
    string videodirname = "../video/"+now_iso_str+"/";
    const char * videodirnamestr = videodirname.c_str();
    mkdir(videodirnamestr,S_IRUSR | S_IWUSR | S_IXUSR);
    string video_rune_name = videodirname+"Realtime_Rune.avi";
    string video_armor_name = videodirname+"Realtime_Armor.avi";
    writer_rune.open( video_rune_name,CV_FOURCC('M', 'J', 'P', 'G'), 20, Size(1280,1024) );
    writer_armor.open( video_armor_name,CV_FOURCC('M', 'J', 'P', 'G'), 20, Size(1280,1024) );

    while ( !writer_rune.isOpened() )
    {
        armor_cnt++;
        writer_rune.open(video_rune_name, CV_FOURCC('M', 'J', 'P', 'G'), 20, Size(1280, 1024) );
        if ( armor_cnt >= 10 )
        {
             break;
        }
    }
    armor_cnt=0;
    while ( !writer_armor.isOpened() )
    {
        armor_cnt++;
        writer_armor.open( video_armor_name, CV_FOURCC('M', 'J', 'P', 'G'), 20, Size(1280, 1024) );
        if ( armor_cnt>=10)
        {
             break;
        }
    }

#endif

#ifdef get_error_image
    //初始化错误图片路径
    dirname[0] = "../data/"+now_iso_str+"/";
    dirname[1] = dirname[0]+"contour/";
    dirname[2] = dirname[0]+"scoreboard/";
    dirname[3] = dirname[0]+"digitron/";
    dirname[4] = dirname[0]+"smallbuff/";
    dirname[5] = dirname[0]+"bigbuff/";
    dirname[6] = dirname[0]+"armor_predict_error/";

    const char * dirnamestr = dirname[0].c_str();
    const char * dirnamestr1 = dirname[1].c_str();
    const char * dirnamestr2 = dirname[2].c_str();
    const char * dirnamestr3 = dirname[3].c_str();
    const char * dirnamestr4 = dirname[4].c_str();
    const char * dirnamestr5 = dirname[5].c_str();
    const char * dirnamestr6 = dirname[6].c_str();

    mkdir(dirnamestr,S_IRUSR | S_IWUSR | S_IXUSR);
    mkdir(dirnamestr1,S_IRUSR | S_IWUSR | S_IXUSR);
    mkdir(dirnamestr2,S_IRUSR | S_IWUSR | S_IXUSR);
    mkdir(dirnamestr3,S_IRUSR | S_IWUSR | S_IXUSR);
    mkdir(dirnamestr4,S_IRUSR | S_IWUSR | S_IXUSR);
    mkdir(dirnamestr5,S_IRUSR | S_IWUSR | S_IXUSR);
    mkdir(dirnamestr6,S_IRUSR | S_IWUSR | S_IXUSR);
#endif

#ifdef log_output
    //创建日志文件
    string logname = "../log/"+now_iso_str+".log";
    const char * lognamestr = logname.c_str();
    fout.open( lognamestr );
#endif

    ImgSub imgsub( "MySharedMemory", boost::bind( callback, _1 ) );     //

    imgsub.subJoin();

    return 0;
}

void pub_callback( const vector<uint8_t> &data )
{
    if ( data.size() > 0 )

         cout << "publish EP has received data...\n";
}

cv::Point2f Pixel2Angle( cv::Point2f &connor )
{
    Eigen::Vector3d pixel;
    pixel << connor.x, connor.y, 1;

    Eigen::Vector3d  P_cam, P_distortion;    //get P in camera coordinate
    //20180702将输入的角点值看做是实际尺寸（经过旋转和平移后），则应当先做畸变矫正，再乘相机参数矩阵的逆？
    //不对，如果是真的按照这个推导，最后一步应该是乘相机参数矩阵，而不是逆
    //还是得问问才行
    P_cam = cameraMatrix.inverse() * pixel;

    double  r2 = pow( P_cam.x(), 2 ) + pow( P_cam.y(), 2 );

    P_distortion.x() = P_cam.x() * ( 1 + distCoeffs[0] * pow( r2, 1 ) + distCoeffs[1] * pow( r2, 2 ) + distCoeffs[2] * pow( r2, 3 ) )
                       + 2 * distCoeffs[3] * P_cam.x() * P_cam.y() + distCoeffs[4] * (r2 + 2 * pow(P_cam.x(), 2 ) );

    P_distortion.y() = P_cam.y() * ( 1 + distCoeffs[0] * pow( r2, 1 ) + distCoeffs[1] * pow( r2, 2 ) + distCoeffs[2] * pow( r2, 3 ) )
                       + distCoeffs[3] * ( r2 + 2 * pow( P_cam.y(), 2 ) ) + 2 * distCoeffs[4] * P_cam.x() * P_cam.y();
    P_distortion.z() = 1;

    Eigen::Vector3d P_hold =  calMatrix.rotation().inverse() * ( P_distortion-calMatrix.translation() );

    cv::Point2f angles;

    angles.x =  (atan2(P_hold(0), 1 ) * 180.0 / M_PI ) * 0.9;  //0.5 //get the yaw angle

    angles.y = ( ( atan2(-P_hold(1), 1 ) * 180.0 / M_PI ) + 1.5 ) * 0.93; //1.2  //get the pitch angle

    //cout << "angles ---------------> " << angles << endl;

    return angles;
}

string now()
{
    time_t t = time(0);
    char buffer[9] = {0};
    strftime( buffer, 9, "%H:%M:%S", localtime(&t) );
    return string( buffer );
}