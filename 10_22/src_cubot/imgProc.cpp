#include "cubotcpp/timer.h"
#include "cubotcpp/Serial_port.h"
#include "cubotcpp/imgSubscrible.h"
#include "cubotcpp/PublishAsync.h"
#include "../Fuwen/DafuDetector2.hpp"
#include <../include/gold/my.h>
#include <../include/fordefine.h>

#include <boost/date_time/posix_time/posix_time.hpp>
using namespace boost::posix_time;
//do not include comcommation

using namespace cubot;

typedef Eigen::Matrix< double , 5  , 1> Vector5d;
Eigen::Isometry3d calMatrix;
Eigen::Matrix3d cameraMatrix;
Vector5d distCoeffs;

uint8_t Mode = 2;
int isSockPubSucc = -1;
vector<Point2f> v_pub_Mode(1);
bool isFuwenRecvFlag = true, isArmorRecvFlag = true;

int sum0 = 0;
int sum_true = 0;

Mat  imageCopy;
DafuDetector txdetector;
cv::Point2f center(0,0);//应该为全局变量
cv::Point2f send_data_center(0,0);//应该为全局变量
cv::Point2f Pixel2Angle(cv::Point2f &connor);
void pub_callback(const vector<uint8_t> &data);
void sp_callback(const uint8_t *recv_buf);

#ifndef versiontest
serial_port sp(sp_callback, "/dev/SerialPortCom0", 115200);
#endif

#ifdef videowriter
Size videoSize(1280,1024);
VideoWriter writer_armor;
VideoWriter writer_rune;
int FrameNumofRune = 5;   //大符识别模式，每5帧保存一帧图像
int FrameNumofArmor = 50; //装甲识别模式，每15帧保存一帧图像
int CntOfRune = 0;
int CntOfArmor = 0;
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


Publish_udp pub_mode(pub_callback);

void callback(const CubotImg &img)
{
    if (img.img.empty())
        return;
    imageCopy = img.img.clone();

#ifdef log_output
    //输出系统时间
    ptime  time_now(second_clock::local_time());
    string now_iso_str(to_iso_string(time_now));
    fout <<"\n**********************************************"<<endl;
    fout <<now_iso_str<< endl;
#endif

    int  result_num = 0;  //范围1-9

    cv::Point2f target(-1,-1);

    cv::Point2f temp_center(-1,-1);

    bool need_to_return = false;


#ifdef dafutest
    Mode = 1;
#endif

#ifdef armortest
    Mode = 2;
#endif

    /////////////////视觉检测部分////////////////
    if(Mode == 1)
    {
        //Fuwen
        cout<<"===============come into FuWen mode===================="<<endl;
#ifdef log_output
        fout<<"===============come into FuWen mode===================="<<endl;
#endif


#ifdef videowriter
        CntOfRune++;
        if(CntOfRune%FrameNumofRune == 0)
        {
            CntOfRune = 0;
            writer_rune<<imageCopy;
        }
#endif

        /* 函数功能大符检测
        * src_img：  原始图像（三通道）
        * p2f_angle：打击点的像素坐标
        * send_num： 数码管识别结果
        */
        bool isdetected = txdetector.detect(imageCopy,target,result_num);   //调用大符主函数，返回bool类型的isdetected为ture or flase

        if(isdetected)  //isdetected == ture
        {
            Mode = 0;

            isFuwenRecvFlag = true;

            sum0++;

            cout<<"isdetected"<<"  "<<target<<"   "<<result_num<<"  "<<sum0<<endl;
        }
    }
        //Mode 非零
    else if(Mode){

        cout<<"=============come into AutoHit mode====================="<<endl;


#ifdef log_output
        fout<<"=============come into AutoHit mode====================="<<endl;
#endif

        float dist;

#ifdef videowriter
        CntOfArmor++;
        if(CntOfArmor%FrameNumofArmor == 0)
        {
            CntOfArmor = 0;
            writer_armor <<imageCopy;
        }
#endif


        target = meng::detection(imageCopy,dist);   //调用装甲识别主函数，返回target
    }
    else
    {
        //cout<<"=============come into Mode equals to zero====================="<<endl;
    }
    /////////////////发射部分///////////////
    if((target.x >= 0)&&(target.y >= 0))
    {
        if(Mode != 1)

            resize(imageCopy,imageCopy,imageCopy.size()*2);

        circle(imageCopy,target,50,Scalar(0,0,255),10);

        vector<Point2f> targetPoints;

        targetPoints.emplace_back(Pixel2Angle(target));

        targetPoints.emplace_back(Point2f(result_num,0));
        cout<<"isdetected"<<"  "<<target<<endl;


#ifdef log_output
        fout<<"isdetected"<<"  "<<target<<endl;
#endif

        if(!targetPoints.empty())
        {
#ifndef versiontest
            int isSpSucss=sp.SerialPublish(targetPoints);
            fout<<" SerialPort publish "<<isSpSucss<<" bytes sucessfully."<<std::endl;
#endif
        }
    }


#ifdef show
    namedWindow("imageCopy",0);
    resize(imageCopy,imageCopy,Size(imageCopy.size()/3));
    imshow("imageCopy",imageCopy);
    waitKey(1);
#endif
}




void sp_callback(const uint8_t *recv_buf)
{
    // Attention: max size is 6
    // Serial Port receive data
    // cout<<"serial port receive data\n";
    if(recv_buf[0] == 1)
    {
        // 符文模式
        if(isFuwenRecvFlag)
        {
            // 多次发送，但是只接收一次
            Mode = 1;
            isFuwenRecvFlag = false;
            isArmorRecvFlag = true;

            //进程间通信，向相机节点发送信息，使得相机可以随模式切换改变参数
            v_pub_Mode[0] = cv::Point2f(1,1);
            isSockPubSucc = pub_mode.pub(v_pub_Mode);
        }

        else
        {
            //cout<<"======符文模式切换======= \n";
        }
    }
    else if(recv_buf[0])
    {
        if(isArmorRecvFlag)
        {
            // 自动瞄准模式
            Mode = 2;
            isFuwenRecvFlag = true;
            isArmorRecvFlag = false;

            // 进程间通信，向相机节点发送信息，使得相机可以随模式切换改变参数
            v_pub_Mode[0] = cv::Point2f(2,2);
            isSockPubSucc = pub_mode.pub(v_pub_Mode);
            cout<<"======自动瞄准模式======= "
                <<"recv flag is "<<int(recv_buf[0])
                <<" publish "<<isSockPubSucc<<" bytes"<<endl;
        }

        else
        {
            cout<<"======自动瞄准模式======= \n";
        }
    }
    else
    {
        cerr<<"==================receive zero======================= "<<int(recv_buf[0])<<endl;
    }
}

int main(int argc, char *argv[])
{
    cout<<"start"<<endl;

    //creatTimerCallback timer(bind(timercallback), 30);//定时器查看图片
    FileStorage fs("../cfg/camera_params_cam_5.yml", FileStorage::READ);
    Mat cvCameraMatrix,  cvDistCoeffs, cvCalMatrix;
    fs["cameraMatrix"] >>  cvCameraMatrix;
    fs["distCoeffs"] >>  cvDistCoeffs;
    fs["calMatrix"] >>  cvCalMatrix;
    cout<<"Read camera params success."<<endl;

    cv::cv2eigen(cvCalMatrix, calMatrix.matrix());
    cv::cv2eigen(cvCameraMatrix, cameraMatrix);
    cv::cv2eigen(cvDistCoeffs, distCoeffs);

    ptime  time_now(second_clock::local_time());
    string now_iso_str(to_iso_string(time_now));


#ifdef videowriter
    int armor_cnt=0;
    string videodirname = "../video/"+now_iso_str+"/";
    const char * videodirnamestr = videodirname.c_str();
    mkdir(videodirnamestr,S_IRUSR | S_IWUSR | S_IXUSR);
    string video_rune_name = videodirname+"Realtime_Rune.avi";
    string video_armor_name = videodirname+"Realtime_Armor.avi";
    writer_rune.open(video_rune_name,CV_FOURCC('M', 'J', 'P', 'G'), 20, Size(1280,1024));
    writer_armor.open(video_armor_name,CV_FOURCC('M', 'J', 'P', 'G'), 20, Size(1280,1024));

    while(!writer_rune.isOpened()){
        armor_cnt++;
        writer_rune.open(video_rune_name,CV_FOURCC('M', 'J', 'P', 'G'), 20, Size(1280,1024));
        if(armor_cnt>=10){
            break;
        }
    }
    armor_cnt=0;
    while(!writer_armor.isOpened()){
        armor_cnt++;
        writer_armor.open(video_armor_name,CV_FOURCC('M', 'J', 'P', 'G'), 20, Size(1280,1024));
        if(armor_cnt>=10){
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
    fout.open(lognamestr);
#endif

    ImgSub imgsub("MySharedMemory", boost::bind(callback, _1));     //

    imgsub.subJoin();

    return 0;
}

void pub_callback(const vector<uint8_t> &data)
{
    if(data.size()>0)

        cout<<"publish EP has received data...\n";
}


cv::Point2f Pixel2Angle(cv::Point2f &connor)
{
    Eigen::Vector3d pixel;
    pixel << connor.x, connor.y, 1;

    Eigen::Vector3d  P_cam,P_distortion;    //get P in camera coordinate
    //20180702将输入的角点值看做是实际尺寸（经过旋转和平移后），则应当先做畸变矫正，再乘相机参数矩阵的逆？
    //不对，如果是真的按照这个推导，最后一步应该是乘相机参数矩阵，而不是逆
    //还是得问问才行
    P_cam = cameraMatrix.inverse()*pixel;

    double  r2 = pow(P_cam.x(),2)+pow(P_cam.y(),2);

    P_distortion.x() = P_cam.x()*(1+distCoeffs[0]*pow(r2,1)+distCoeffs[1]*pow(r2,2)+distCoeffs[2]*pow(r2,3))
                       +2*distCoeffs[3]*P_cam.x()*P_cam.y()+distCoeffs[4]*(r2+2*pow(P_cam.x(),2));

    P_distortion.y() = P_cam.y()*(1+distCoeffs[0]*pow(r2,1)+distCoeffs[1]*pow(r2,2)+distCoeffs[2]*pow(r2,3))
                       +distCoeffs[3]*(r2+2*pow(P_cam.y(),2))+2*distCoeffs[4]*P_cam.x()*P_cam.y();
    P_distortion.z() = 1;

    Eigen::Vector3d P_hold =  calMatrix.rotation().inverse()* (P_distortion-calMatrix.translation());

    //cout<<P_hold<<endl;
    cv::Point2f angles;

    angles.x =  (atan2(P_hold(0),1) * 180.0 / M_PI)*0.9;  //0.5 //get the yaw angle

    angles.y = ((atan2(-P_hold(1), 1) * 180.0 / M_PI)+1.5)*0.93; //1.2  //get the pitch angle

    //cout << "angles ---------------> " << angles << endl;

    return angles;
}

string now()
{
    time_t t = time(0);
    char buffer[9] = {0};
    strftime(buffer, 9, "%H:%M:%S", localtime(&t));
    return string(buffer);
}