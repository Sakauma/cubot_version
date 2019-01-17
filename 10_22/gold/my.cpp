//
// Created by jason on 18-6-3.
//
#include <iostream>
#include "opencv2/opencv.hpp"

#include "gold/coordinante_process.h"
#include "gold/bar_points.h"
#include "gold/my.h"

namespace meng
{
    /*!!!! You need set different value according to your role, Hero, Infantry, or Sentry*/
    int Color_id ;
    float bar_height_H;
    float bar_height_L;
    float bar_slim_k_H;
    float bar_slim_k_L;

    float bar_angle_limit;
    float small_pair_bar_angle_limit;
    float big_pair_bar_angle_limit;

    float high_light_mask_k ;
    float proposal_bar_area_k;

    /*Global varible for nodes*/
    int Model_cnt;
    int g_flag;

    cv::Point2f dynamic_sp;

    cv::Mat transform_img;

    std::vector<cv::Point2f> world_pts(4);
    std::vector<cv::Point2f> new_pts(4);

    cv::Ptr<cv::ml::SVM> model_hog;
    cv::HOGDescriptor * hog = new cv::HOGDescriptor(cv::Size(40, 40), cv::Size(16, 16), cv::Size(8, 8), cv::Size(8, 8), 9);
    cv::Mat element = cv::getStructuringElement( cv::MORPH_CROSS, cv::Size( 2*1 + 1, 2*1+1 ), cv::Point( 1, 1 ) );
}

static void detect_init(void)
{

    cv::FileStorage fs2("../cfg/MengCfg.yml", cv::FileStorage::READ);
    if(fs2.isOpened()){

        int isSave = (int)fs2["isSave"];
        int saveStartNum  = (int)fs2["saveStartNum"];
        std::string img_savePath = (std::string)fs2["img_savePath"];
        std::string svm_model_hog_path  = (std::string)fs2["svm_model_hog_path"];
        std::string svm_model_histogram_path  = (std::string)fs2["svm_model_histogram_path"];
        meng::Color_id = (int)fs2["color2Strike"];
        meng::high_light_mask_k = (float)fs2["high_light_mask_k"];
        meng::proposal_bar_area_k=(float)fs2["proposal_bar_area_k"];
        meng::bar_height_H = (float)fs2["bar_height_H"];
        meng::bar_height_L = (float)fs2["bar_height_L"];
        meng::bar_slim_k_H = (float)fs2["bar_slim_k_H"];
        meng::bar_slim_k_L = (float)fs2["bar_slim_k_L"];
        meng::bar_angle_limit = (float)fs2["bar_angle_limit"];
        meng::small_pair_bar_angle_limit = (float)fs2["small_pair_bar_angle_limit"];
        meng::big_pair_bar_angle_limit=(float)fs2["big_pair_bar_angle_limit"];
        meng::dynamic_sp.x=(float)fs2["dynamic_sp_x"];
        meng::dynamic_sp.y=(float)fs2["dynamic_sp_y"];
        fs2.release();
        std::cout << "try to read the configurate " << std::endl;
        std::cout<<isSave<<" "<<saveStartNum<<" "<<img_savePath<<" "
                 <<svm_model_hog_path<<" " <<svm_model_histogram_path    <<" "
                 <<meng::Color_id<<" " <<meng::high_light_mask_k<<" "
                 <<meng::proposal_bar_area_k<<" " <<meng::bar_height_H <<" "
                 <<meng::bar_height_L<<" " <<meng::bar_slim_k_H<<" "<<meng::bar_slim_k_L <<" "
                 <<meng::bar_angle_limit<<" "<<meng::small_pair_bar_angle_limit<<" "
                 <<meng::big_pair_bar_angle_limit<<" "<<meng::dynamic_sp<<" "<<std::endl;
        meng::g_flag=0;
        meng::world_pts[0]=cv::Point2f(0,0);
        meng::world_pts[1]=cv::Point2f(40,0);
        meng::world_pts[2]=cv::Point2f(40,40);
        meng::world_pts[3]=cv::Point2f(0,40);
        std::cout<<std::endl<<svm_model_hog_path<<std::endl;
        meng::model_hog = cv::ml::SVM::load(svm_model_hog_path);
        if(meng::model_hog.empty())
            std::cerr<<"model_hog open failed"<<std::endl;

    }
    else{
        std::cerr<<"Meng cfg file read failed\n";
        return ;
    }
}
/*********************************
 *
 * Description: 获得加权灰度图
 * Input:
 * 		img：原始BGR图像
 * 		color_id:颜色标识：0蓝 1红
 * output:
 *      gray:相应颜色加权灰度图
 * Others：
 * 		1、判断 color_id
 * 		2、cv::split（）：图像BGR三通道分离操作
 * 		3、套用BGR图像转灰度图像公式，各通道加权后值赋给gray
 *
**********************************/
void weighted_cvt_gray(const cv::Mat &img, cv::Mat & gray, int color_id)
{
    if( color_id == 0 ){
        cv::Mat colors[3];
        cv::split(img,colors);
        gray = colors[0]*0.9+colors[1]*0.05+colors[2]*0.05;
        std::cout<<"    color to strike is blue   ";
    }else    //( color_id == 1 ) RED 红色 1
    {
        cv::Mat colors[3];
        cv::split(img,colors);
        gray = colors[0]*0.05+colors[1]*0.05+colors[2]*1.5;
        std::cout<<"    color to strike is red   ";
    }
}

/*********************************
 *
 * Description: 二值化获得灯条高亮图
 * Input:
 * 		img_input： 原始BGR图像
 * 		color_id：颜色标识：0蓝 1红
 * Output:
 *       mask： 灰度化再阈值操作后的二值化掩膜
 * Others：
 * 		1、判断color_id
 * 		2、调用 weighted_cvt_gray()函数获得对应color_id的加权灰度图
 * 		3、使用threshold()函数，对加权灰度图阈值操作后得到二值化图像，函数中的参数在此的功能是：若图像像素大于150置255白，否则置0黑
 *
*********************************/
void get_hight_light_bar(const cv::Mat & img_input, cv::Mat & mask , int color_id)
{
    cv::Mat weighted_gray;
    if(  color_id == 0){
        weighted_cvt_gray(img_input,weighted_gray,color_id);
        cv::threshold(weighted_gray,mask,150,255,cv::THRESH_BINARY);
    }
    else{   // Red

        weighted_cvt_gray(img_input,weighted_gray,color_id);

        cv::threshold(weighted_gray,mask,150,255,cv::THRESH_BINARY);

    }
}
int meng::predict(cv::Ptr<cv::ml::SVM> &model,cv::HOGDescriptor * hog ,cv::Mat &transform_img)
{
    cv::Mat t_gray,t_equ ;
    cv::cvtColor(transform_img,t_gray,cv::COLOR_BGR2GRAY);
    int r=0;
#ifdef Histogram  //步兵未使用
    cv::Mat hist = snow::get_hist(t_gray);
    r = snow::predict(hist, model_histogram);
    if (r==0){
        std::cout<<"Histogram predict to Negative Class"<<std::endl;
        return r;
    }
#endif

#ifdef HistBin  //步兵未使用
    //使用直方图的阈值进行二值化
    std::vector<float> descriptors;
    cv::threshold(t_gray, t_equ, 18, 255, cv::THRESH_BINARY);
    hog->compute(t_equ, descriptors);
    cv::Mat dst(1, int(descriptors.size()), CV_32FC1, descriptors.data());
    r = model->predict(dst);
#else

    std::vector<float> descriptors;     //HOG特征描述子
    cv::equalizeHist(t_gray,t_equ);     //直方图均值化，归一化图像亮度和增强对比度，均值化后的图像存储在t_equ

    int pxl_cnt=0, bias= 10, c_x = 20, c_y = 20;
    if(Color_id==1)
    {   ///遍历透视变换图的所有像素点///
        for (int i = 0; i < transform_img.rows; i++)
        {
            uchar *data = transform_img.ptr<uchar>(i);
            uchar *ptr_gray =  t_gray.ptr<uchar>(i);
            uchar *ptr_equ =  t_equ.ptr<uchar>(i);
            for (int j = 0; j < transform_img.cols; j++)
            {
                uchar b = data[j*3];
                uchar g = data[j*3+1];
                uchar r = data[j*3+2];
                if ((((r - b) > 50) && ((r > 100) || (g > 100))) || (ptr_gray[j] > 60))
                {
                    ptr_equ[j] = 0;
                    if((i<c_y+bias) && (i>c_y-bias) && (j<c_x+bias) && (j>c_x-bias))
                        pxl_cnt++;
                }
            }
        }
        if(pxl_cnt>2){
            std::cout<<"中心点高亮"<<pxl_cnt<<std::endl;
            return 0;
        }
    }         ////将数码管干扰像素值置为0黑////

    hog->compute(t_equ, descriptors);

    cv::Mat dst(1, int(descriptors.size()), CV_32FC1, descriptors.data());
    r = model->predict(dst);

#endif

#if 0
    cv::namedWindow("transform_img",0);
    cv::imshow("transform_img",transform_img);
    if (r==1){
        cv::namedWindow("transform_img",0);
        cv::imshow("transform_img",transform_img);

        cv::Mat bin;
        cv::threshold(t_equ,bin,0,255,cv::THRESH_OTSU); //test for threshold to t_equ
        cv::imshow("equ",t_equ);
        cv::imshow("bin",bin);
        //int k = cv::waitKey();
        //if(k=='m')
        {
            static int save_num = 0;
            std::string roi_name = "/home/snow/imgs/train/" + std::to_string(0+save_num++) + ".jpg";
            cv::imwrite(roi_name, t_gray);
            cv::waitKey(1);
            printf("save successfully\n");
        }
    }
#endif
    if (r==0)
        return r;

#if 0
    cv::Mat bin0;
    //二值化操作
    cv::threshold(t_gray,bin0,20,255,cv::THRESH_BINARY);
    cv::namedWindow("bin0",0);
    //显示二值化操作后的图像
    cv::imshow("bin0",bin0);
    {
        static int save_num = 0;
        std::string roi_name = "/home/snow/imgs/" + std::to_string(0+save_num++) + ".jpg";
        cv::imwrite(roi_name, t_gray);
        cv::waitKey(1);
        printf("save successfully\n");
    }
#endif

    if ((r != 0) && (meng::Color_id==1)) {
        int width  = bias;
        int height = bias;
        cv::Mat c3[3];
        cv::Mat diff[3];
        cv::Mat diff_mask[4];
        cv::split(transform_img,c3);
        cv::absdiff(c3[0],c3[1],diff[0]);   //B与G通道像素值之差的绝对值
        cv::absdiff(c3[1],c3[2],diff[1]);   //G与R通道像素值之差的绝对值
        cv::absdiff(c3[2],c3[0],diff[2]);   //R与B通道像素值之差的绝对值
        //inRange():检查元素的取值范围是否在另两个矩阵的元素取值之间，返回验证矩阵。结果存储在diff_mask[]中
        cv::inRange(diff[0],80,255,diff_mask[0]);   //diff[0]的像素值是否在80-255之间，在则设为白色(255),否则设为黑色(0)
        cv::inRange(diff[1],80,255,diff_mask[1]);   //diff[1]的像素值是否在80-255之间，在则设为白色(255),否则设为黑色(0)
        cv::inRange(diff[2],80,255,diff_mask[2]);   //diff[2]的像素值是否在80-255之间，在则设为白色(255),否则设为黑色(0)
        //进行逻辑或操作
        cv::bitwise_or(diff_mask[0],diff_mask[1],diff_mask[3]); //对diff_mask[0]与diff_mask[1]每个像素值做逻辑或操作，结果放入diff_mask[3]
        cv::bitwise_or(diff_mask[2],diff_mask[3],diff_mask[3]); //对diff_mask[2]与diff_mask[3]每个像素值做逻辑或操作，结果放入diff_mask[3]
        //t_r是左上点坐标为（10,10） ，边长为20 的矩形，也即是中间区域
        cv::Rect t_r(c_x - width, c_y - height, 2 * width, 2 * height);
        //尺寸约束，保证矩形在图像内部，约束后的矩形存储在t_r
        limit_rect(t_r, t_r, transform_img.rows, transform_img.cols);

        float per = check_mask_area(diff_mask[3], t_r)/t_r.area();
        if (per>0.05)
        {
            r = 0;
            std::cout<<"数码管干扰  "<<per<<std::endl;
        }
    }
    //===========


    return r;
}

///     利用大装甲板的顶点坐标计算小装甲板的顶点坐标      ///
void meng::transform_armor_vertex(cv::Point2f *pts,float k)
{
    cv::Point2f new_pts[4];

    new_pts[0]=pts[0];
    new_pts[1]=pts[1];
    new_pts[2]=pts[2];
    new_pts[3]=pts[3];

    pts[0] = new_pts[0] + ( new_pts[1]-new_pts[0] )*k;      //k = 0.25
    pts[1] = new_pts[0] + ( new_pts[1]-new_pts[0] )*(1-k);

    pts[3] = new_pts[3] + ( new_pts[2]-new_pts[3] )*k;
    pts[2] = new_pts[3] + ( new_pts[2]-new_pts[3] )*(1-k);
}

//距离估计
void estimatePose(const std::vector<cv::Point2f> corners, cv::Mat &intrinsic_matrix, cv::Mat distortion_coefficients,cv::Vec3d & rvecs, cv::Vec3d & tvecs)
{
    std::vector<cv::Point3f> odjPoints3d;
    float width = 13;
    float height= 13;
    odjPoints3d.emplace_back(-width*0.5, -height*0.5, 0.);
    odjPoints3d.emplace_back( width*0.5, -height*0.5, 0.);
    odjPoints3d.emplace_back( width*0.5,  height*0.5, 0.);
    odjPoints3d.emplace_back(-width*0.5,  height*0.5, 0.);

    assert((!odjPoints3d.empty()) || (!corners.empty()) || (odjPoints3d.size()==4) || (corners.size()==4));

//    cv::Vec3d rvecs, tvecs;
    solvePnP(odjPoints3d, corners, intrinsic_matrix, distortion_coefficients, rvecs, tvecs);//, false, SOLVEPNP_EPNP); //only 4 points

    if(tvecs[2] <= 0)
        return;
}
  /******************************装甲识别主体*******************************/
 /**********************************************************************/
/*********************************************************************/
static cv::Point2f detect_task(cv::Mat &img_src, int color_id)  //比赛时根据在场蓝红方，来调整 color_id
{   //读取相机的一帧图像，并规定此时应当识别的 color_id
    /**img_src for show and plot    //输入图片
     * img_ori for origin image save    //原始图片备份
     * color_mask for light bar to find color pair  //高亮灯条掩膜是为了寻找一对颜色灯条
     * (if blue: color mask= hsv_mask because blue hsv is stable    //HSV色彩空间 蓝色较易获取
     * if red : color mask= complex mask, because red is hard to filterd)   //红色不易过滤
     **/
    cv::Point2f sp(-1,-1);      //默认装甲板中心点（若最终结果是（-1,-1）则说明未找到）
    //resize():图像缩放操作,同时缩放长和宽.  缩放倍数0.5，即为原图像的1/4
    cv::resize(img_src,img_src,cv::Size(),0.5,0.5);
    //克隆缩放后的原图，存储在img_ori中，做备份
    cv::Mat img_ori = img_src.clone();

    /******************************* Get hight_light_bar ********************************/

    cv::Mat high_light_thre_mask;   //定义掩膜
    //get_hight_light_bar():二值化获得灯条高亮图，返回灰度化阈值操作后的二值化图像结果存储在 high_light_thre_mask 中
    get_hight_light_bar(img_src,high_light_thre_mask,color_id);

    //cv::dilate():膨胀操作，目的是来连接二值化后的不相连图像。返回结果存储在 high_light_thre_mask 中
    cv::dilate(high_light_thre_mask,high_light_thre_mask,meng::element);

    //定义轮廓和层次结构（寻找轮廓操作）
    std::vector<cv::Vec4i> hierarchy;
    std::vector< std::vector<cv::Point> > contours;
    /*********************************
    *
    * Description: findContours（）函数：寻找轮廓操作
    * Input:
    *       1. high_light_thre_mask 二值化获得的灯条高亮图
    *       2. hierarchy    层次结构
    *       3. cv::RETR_EXTERNAL：只检测最外层轮廓，对所有轮廓设置 hierarchy[i][2]=hierarchy[i][3]=-1
    *       4. cv::CHAIN_APPROX_NONE：获取每个轮廓的每个像素，保存物体边界上所有连续的轮廓点
    *       5. cv::Point(0, 0)  轮廓偏移参数，默认值
    * Output:
    *       1. contours 寻找到的轮廓放入 contours 中
    *
    **********************************/
    cv::findContours( high_light_thre_mask, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE, cv::Point(0, 0) );

    //如果寻找到轮廓，则进行下一步
    if(contours.size() > 0)
    {
        //bar_list中放置的是等同轮廓数量的灯条信息      <struct s_bar>（自定义结构体，跳转查看）
        //<struct s_bar>类型的contours.size()大小的vector，命名bar_list
        std::vector<struct s_bar> bar_list(contours.size());

        /********************1. Find hight light bar and put them in bar_list, mark them by setting level = 1   找到高亮灯条并放入bar_list中，设置级别=1来标记。
         * We have several limit condotions                                     有如下的几条约束条件
         * (1) the ellipse hight must in range of(10 ,70)                       椭圆高度在（10,70）
         * (2) the angle accoding to horizon must more than (45) degree         地平线角度大于45度
         * (3) the hight/width must in range of(1.5,10)                         长宽比在（1.5,10）
         * (4) there must be some blue/red points in the contour (k=0.2)        轮廓中有红/蓝色点
         * ...                                                                  ...
         * and finally we get end points of the bar                             最终得到灯条的端点
         *
         * The parameters above should be check according to configuration file xxx.xml         如上参数通过xxx.xml检查
         * ************************************/
#ifdef OpenTBB  //如果开启TBB（Threading Building Blocks）并行计算库，目的是提升数据并行计算的能力
        cv::Range range0(0,contours.size());
        cv::parallel_for_( range0, [&](const cv::Range& range)
        {
            cv::RotatedRect minEllipse;
            cv::Rect min_b_box;
            cv::Mat min_b_box_roi;
            cv::RotatedRect min_r_Rect;

            for (int contour_i = range.start; contour_i < range.end; contour_i++)
            {
                /*make sure you can find a ellipse*/
                if (contours[contour_i].size() > 4)
                {
                    minEllipse = fitEllipse(contours[contour_i]);
                    min_b_box = minEllipse.boundingRect();

                    float ellipse_h_w_div = minEllipse.size.height / minEllipse.size.width;
                    if (  ((minEllipse.angle < meng::bar_angle_limit) || (minEllipse.angle > (180-meng::bar_angle_limit) )) && (ellipse_h_w_div > meng::bar_slim_k_L) &&
                          (ellipse_h_w_div < meng::bar_slim_k_H) &&( minEllipse.size.height > meng::bar_height_L ) &&( minEllipse.size.height < meng::bar_height_H )
                          &&((cv::contourArea(contours[contour_i])/(minEllipse.size.area()*0.25*3.1415))>0.7) )
                    {
                        //get roi and make sure min bounding box is valid
                        if( limit_rect(min_b_box,min_b_box,img_ori.rows,img_ori.cols)==0 )
                            continue;

//                      min_b_box_roi = img_ori(min_b_box);
                        cv::Mat bbox_color_mask = color_mask(min_b_box);

                        if (!bbox_color_mask.empty())
                        {
                            /* high light bar rect must contain some blue points*/
                            float k = cv::sum(bbox_color_mask)[0] / 255.0 / (min_b_box.width*min_b_box.height);
                            if (k > meng::high_light_mask_k)
                            {
                                cv::Mat fill_roi;

                                bar_list[contour_i].ellipse = minEllipse;
                                bar_list[contour_i].b_rec = min_b_box;
                                bar_list[contour_i].level = 1;

                                /*calculate endpoint*/
                                float angle_rad = (90.0 - bar_list[contour_i].ellipse.angle) / 180.0 * 3.1415926;
                                float h = bar_list[contour_i].ellipse.size.height;
                                /*find bar endpoint*/
                                find_newpoint(bar_list[contour_i].ellipse.center, angle_rad, 0.5 * h,
                                              bar_list[contour_i].endpoints[0], img_ori.rows, img_ori.cols);
                                find_newpoint(bar_list[contour_i].ellipse.center, angle_rad, -0.5 * h,
                                              bar_list[contour_i].endpoints[1], img_ori.rows, img_ori.cols);
                                check_endpoint(bar_list[contour_i].endpoints[0], bar_list[contour_i].endpoints[1]);

                                cv::ellipse( img_src, minEllipse, cv::Scalar(255,255,255), 1, 8 );
//                                cv::putText(img_src,std::to_string((int)(minEllipse.angle)),minEllipse.center,cv::FONT_HERSHEY_COMPLEX,0.5,cv::Scalar(0,0,255));
//                                printf("minEllipse center: %f , %f\n",minEllipse.center.x , minEllipse.center.y);
                            }
                        }
                    } else
                    {
//                        printf("bar %d is not valid +++++++++\n",contour_i);
                    }
                }
            }
        });
#else
        for (int contour_i = 0; contour_i < contours.size(); contour_i++)
        {
            cv::RotatedRect minEllipse;
            cv::Rect min_b_box;
            cv::Mat min_b_box_roi;
            cv::RotatedRect min_r_Rect;

            if (contours[contour_i].size() > 4)
            {
                min_r_Rect = cv::minAreaRect(contours[contour_i]);
                minEllipse = fitEllipse(contours[contour_i]);
                min_b_box = minEllipse.boundingRect();
                float areaOfContour = cv::contourArea(contours[contour_i]);
                float ellipse_h_w_div = minEllipse.size.height / minEllipse.size.width;
                float area_persent = areaOfContour / min_r_Rect.size.area();
                if(area_persent<0.66){
                    cv::putText(img_src,"aP",minEllipse.center,cv::FONT_HERSHEY_COMPLEX,0.5,cv::Scalar(0,255,255));
                    continue;
                }

                if((ellipse_h_w_div > meng::bar_slim_k_H) || (ellipse_h_w_div < meng::bar_slim_k_L)){
                    cv::putText(img_src,"wh",minEllipse.center,cv::FONT_HERSHEY_COMPLEX,0.5,cv::Scalar(0,255,255));
                    continue;
                }

                if ((minEllipse.angle > meng::bar_angle_limit) && (minEllipse.angle < (180-meng::bar_angle_limit) )){
                    cv::putText(img_src,"al",minEllipse.center,cv::FONT_HERSHEY_COMPLEX,0.5,cv::Scalar(0,255,255));
                    continue;
                }

                if((minEllipse.size.height > meng::bar_height_H ) || (minEllipse.size.height < meng::bar_height_L)){
                    cv::putText(img_src,"hl",minEllipse.center,cv::FONT_HERSHEY_COMPLEX,0.5,cv::Scalar(0,255,255));
                    continue;
                }
                min_b_box = min_b_box + cv::Point(-3, -3) + cv::Size(6, 6);
                if( limit_rect(min_b_box,min_b_box,img_ori.rows,img_ori.cols)==0 )
                    continue;

                cv::Mat bgr_roi, hsv_Roi, colorMaskRoi;
                bgr_roi = img_ori(min_b_box);
                cv::cvtColor(bgr_roi,hsv_Roi,cv::COLOR_BGR2HSV);

                if( color_id == 0 ){    //blue
                    cv::inRange(hsv_Roi,cv::Scalar(100,100,100),cv::Scalar(124,255,255),colorMaskRoi);
                }
                else{
                    cv::Mat hsv1, hsv2;
                    cv::inRange(hsv_Roi,cv::Scalar(0,70,70),cv::Scalar(10,200,255),hsv1);
                    cv::inRange(hsv_Roi,cv::Scalar(170,70,70),cv::Scalar(180,200,255),hsv2);
                    colorMaskRoi = hsv1 + hsv2;
                }

                int correct_pxl=0;

                for(int i=0;i<colorMaskRoi.rows;i++){                   //遍历灯条二值化后的hsv图像行
                    uchar *hsv_ptr = colorMaskRoi.ptr<uchar>(i);        //灯条二值化后的hsv图像第i+1行第一个元素的指针
                    uchar *bgr_ptr = bgr_roi.ptr<uchar>(i);             //灯条bgr图像第i+1行第一个元素的指针

                    for(int j=0;j<colorMaskRoi.cols;j++){               //遍历灯条二值化后的hsv图像列
                        uchar hsv_val = hsv_ptr[j];                     //把第i+1行的第j+1元素赋值给hsv_val
                        uchar b = bgr_ptr[j*3];
                        uchar g = bgr_ptr[j*3+1];
                        uchar r = bgr_ptr[j*3+2];

                        if(color_id==0){    //Blue
                            if(hsv_val && (b>50) && (r<80) && (b-2*r>10))
                                correct_pxl++;
                        }
                        else if (color_id==1) {     //Red
                            if(hsv_val && (r>70) && (r-b>50) && (b<110))
                                correct_pxl++;
                        }
                    }
                }
                float pxl_persent = (float)correct_pxl / areaOfContour;
                if (pxl_persent < 0.1){
                    cv::putText(img_src,"pxl",minEllipse.center,cv::FONT_HERSHEY_COMPLEX,0.5,cv::Scalar(0,255,255));
                    continue;

                }

#ifdef Show
                cv::rectangle(img_src,min_b_box,cv::Scalar(0,255,255), 1);
#endif
                cv::Mat fill_roi;

                bar_list[contour_i].ellipse = minEllipse;
                bar_list[contour_i].b_rec = min_b_box;
                bar_list[contour_i].level = 1;

                /**calculate endpoint**/
                float angle_rad = (90.0 - bar_list[contour_i].ellipse.angle) / 180.0 * 3.1415926;
                float h = bar_list[contour_i].ellipse.size.height;
                /**find bar endpoint**/
                find_newpoint(bar_list[contour_i].ellipse.center, angle_rad, 0.5 * h,
                              bar_list[contour_i].endpoints[0], img_ori.rows, img_ori.cols);
                find_newpoint(bar_list[contour_i].ellipse.center, angle_rad, -0.5 * h,
                              bar_list[contour_i].endpoints[1], img_ori.rows, img_ori.cols);
                check_endpoint(bar_list[contour_i].endpoints[0], bar_list[contour_i].endpoints[1]);

#ifdef Show
                cv::ellipse( img_src, minEllipse, cv::Scalar(255,255,255), 1, 8 );
                //cv::putText(img_src,std::to_string((int)(minEllipse.angle)),minEllipse.center,cv::FONT_HERSHEY_COMPLEX,0.5,cv::Scalar(0,0,255));
                //printf("minEllipse center: %f , %f\n",minEllipse.center.x , minEllipse.center.y);
#endif
            }
        }
#endif               /******此步结束已经得到符合要求的椭圆的所有信息******/  //这些信息存储在 bar_list 中


        /********************   2. Match hight light bar pairs  ************************/
                /**sort bars according to their distance to a point***/
        std::vector<struct s_bar> valid_bar_list,tmp_valid_bar_list;
        for(int i=0;i<bar_list.size();i++) {
            if (bar_list[i].level == 1)     //对如上已经定义等级为1的bar_list  ,把bar_list[i] 信息放入 tmp_valid_bar_list 中
                tmp_valid_bar_list.push_back(bar_list[i]);
        }
        if( tmp_valid_bar_list.size()>0 )
            sort_ellipse(tmp_valid_bar_list,valid_bar_list,meng::dynamic_sp.x, meng::dynamic_sp.y);

#ifdef Show
        for(int i=0;i<valid_bar_list.size();i++)
            cv::line(img_src,valid_bar_list[i].endpoints[0],valid_bar_list[i].endpoints[1],cv::Scalar(0,255,0),2);
#endif
        if(valid_bar_list.size()>1){
            int result=-1;

            ///获取所有符合要求的椭圆小装甲扩展点,大装甲扩展点,小装甲待提点,结果已经赋值给bar结构体
            get_surround_pts(valid_bar_list,img_ori);

            ///通过获取的小装甲扩展点,大装甲扩展点,小装甲待提点,判断该灯条是否有匹配灯条,并且判断匹配对中是否存在装甲板,返回有装甲板灯条的编号
            result=find_armor_int_all_pts(img_src, valid_bar_list,img_ori);

            if( result != -1 ) {
                cv::Point2f p = getCrossPoint(valid_bar_list[result].armor_vertex[0],
                                              valid_bar_list[result].armor_vertex[2],
                                              valid_bar_list[result].armor_vertex[1],
                                              valid_bar_list[result].armor_vertex[3]);

                if (p.x != -1)
                    sp = p;
            }
        }
        if((valid_bar_list.size()==1) ||(sp.x == -1)){

            int result = 0;
            result = find_armor_for_single_bar(img_src, valid_bar_list,img_ori, sp);
        }

#ifdef Show
        cv::circle(img_src,meng::dynamic_sp,10,cv::Scalar(255,255,255),-1);
        cv::putText(img_src,std::to_string(valid_bar_list.size()),cv::Point(20,20),cv::FONT_HERSHEY_COMPLEX,0.5,cv::Scalar(0,0,255));
        if (sp.x != -1)
            cv::circle(img_src,sp,10,cv::Scalar(0,255,0),3);
#endif
    }

#ifdef Show
    cv::imshow("img_src",img_src);
    //cv::imshow("color_mask", color_mask);

    if( sp.x != -1 ) {
        cv::Mat g, e;
        cv::cvtColor(meng::transform_img, g, cv::COLOR_BGR2GRAY);
        cv::equalizeHist(g, e);

        cv::imshow("e", e);
        int key = cv::waitKey(1);
        if (key == 13){//Enter
            static int roi_cnt = 0;
            cv::imwrite( "/home/cubot/roi/"+std::to_string(523+roi_cnt++)+".jpg",e);
            printf("save roi\n");
        }
    }
    cv::waitKey(1);
#endif

    return sp;
}

/************** 装甲识别主函数  **************/
cv::Point2f meng::detection(cv::Mat &img,float & distance)
{
    cv::Point2f sp(-1,-1);
    if(meng::g_flag != 1)
    {
        detect_init();

        meng::g_flag = 1;

        printf("init=================\n");
    }
    else
    {
        double t = (double)cvGetTickCount();
        sp=detect_task(img,meng::Color_id);

        sp=sp*2;

        std::cout<<sp<<std::endl;

        t = (double)cvGetTickCount() - t;

        printf( "run time = %gms\n", t/(cvGetTickFrequency()*1000) );
    }
    return sp;
}
