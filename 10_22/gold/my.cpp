//
// Created by jason on 18-6-3.
//
#include <iostream>
#include "opencv2/opencv.hpp"

#include "gold/coordinante_process.h"
#include "gold/bar_points.h"
#include "gold/my.h"

//#define OpenTBB   //will be error
//#define Show
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
    //获取自定义内核，MORPH_CROSS：交叉形
    cv::Mat element = cv::getStructuringElement( cv::MORPH_CROSS, cv::Size( 2*1 + 1, 2*1+1 ), cv::Point( 1, 1 ) );
}

static void detect_init(void)
{
    //========get parameters from file==========
    //只读操作：从/cfg/MengCfg.yml中读取参数设置
    cv::FileStorage fs2("../cfg/MengCfg.yml", cv::FileStorage::READ);
    if(fs2.isOpened()){
        //如下操作均为读取参数设置并保存
        int isSave = (int)fs2["isSave"];
        int saveStartNum  = (int)fs2["saveStartNum"];
        std::string img_savePath = (std::string)fs2["img_savePath"];
        //使用HOG特征训练的SVM分类器
        std::string svm_model_hog_path  = (std::string)fs2["svm_model_hog_path"];
        //使用灰度直方图训练的SVM分类器
        std::string svm_model_histogram_path  = (std::string)fs2["svm_model_histogram_path"];
        //需要击打的颜色
        meng::Color_id = (int)fs2["color2Strike"];
        //确保灯条周围是否有色点
        meng::high_light_mask_k = (float)fs2["high_light_mask_k"];
        meng::proposal_bar_area_k=(float)fs2["proposal_bar_area_k"];
        //灯条尺寸限制
        meng::bar_height_H = (float)fs2["bar_height_H"];
        meng::bar_height_L = (float)fs2["bar_height_L"];
        meng::bar_slim_k_H = (float)fs2["bar_slim_k_H"];
        meng::bar_slim_k_L = (float)fs2["bar_slim_k_L"];
        //灯条角度约束
        meng::bar_angle_limit = (float)fs2["bar_angle_limit"];
        meng::small_pair_bar_angle_limit = (float)fs2["small_pair_bar_angle_limit"];
        meng::big_pair_bar_angle_limit=(float)fs2["big_pair_bar_angle_limit"];
       //偏左上角动态点   用于计算每个灯条距离进行升序排列
        meng::dynamic_sp.x=(float)fs2["dynamic_sp_x"];
        meng::dynamic_sp.y=(float)fs2["dynamic_sp_y"];
       //释放内存
        fs2.release();
        //输出信息
        std::cout << "try to read the configurate " << std::endl;
        std::cout<<isSave<<" "<<saveStartNum<<" "<<img_savePath<<" "
                 <<svm_model_hog_path<<" " <<svm_model_histogram_path    <<" "
                 <<meng::Color_id<<" " <<meng::high_light_mask_k<<" "
                 <<meng::proposal_bar_area_k<<" " <<meng::bar_height_H <<" "
                 <<meng::bar_height_L<<" " <<meng::bar_slim_k_H<<" "<<meng::bar_slim_k_L <<" "
                 <<meng::bar_angle_limit<<" "<<meng::small_pair_bar_angle_limit<<" "
                 <<meng::big_pair_bar_angle_limit<<" "<<meng::dynamic_sp<<" "<<std::endl;

        //===================
        meng::g_flag=0;
        //装甲顶点的标准坐标
        meng::world_pts[0]=cv::Point2f(0,0);
        meng::world_pts[1]=cv::Point2f(40,0);
        meng::world_pts[2]=cv::Point2f(40,40);
        meng::world_pts[3]=cv::Point2f(0,40);
        //SVM模式HOG路径
        std::cout<<std::endl<<svm_model_hog_path<<std::endl;
        meng::model_hog = cv::ml::SVM::load(svm_model_hog_path);
        //如果读取为空，提示错误信息
        if(meng::model_hog.empty())
            std::cerr<<"model_hog open failed"<<std::endl;

    }   //如果未读取成功，提示错误信息
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
        //BLUE 蓝色 0
    if( color_id == 0 ){
        //颜色三通道
        cv::Mat colors[3];
        //cv::split():多通道图像分离操作. 分离出的三通道序列保存在colors[]中
        cv::split(img,colors);
        //BGR图像转灰度图像公式:gray=B*0.9+G*0.05+R*0.05，其中blue加权系数最大
        gray = colors[0]*0.9+colors[1]*0.05+colors[2]*0.05;
        //输出“颜色是蓝色”信息
        std::cout<<"    color to strike is blue   ";
    }else    //( color_id == 1 ) RED 红色 1
    {   //颜色三通道
        cv::Mat colors[3];
        //cv::split():多通道图像分离操作. 分离后的三通道序列保存在colors[]中
        cv::split(img,colors);
        //BGR图像转灰度图像公式:gray=B*0.05+G*0.05+R*1.5，其中R通道加权系数最大,因为red不易过滤
        gray = colors[0]*0.05+colors[1]*0.05+colors[2]*1.5;
        //输出“颜色是红色”信息
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
{   //加权灰度图
    cv::Mat weighted_gray;
    // 0 蓝色
    if(  color_id == 0){
        //Blue
        //调用 weighted_cvt_gray()函数：(自定义函数，可跳转查看) 功能：获得蓝色加权灰度图
        weighted_cvt_gray(img_input,weighted_gray,color_id);

        //cv::threshold():对灰度图进行阈值操作得到二值化掩膜，其中参数cv::THRESH_BINARY :大于150置255白，否则置0黑。输出图像保存在mask
        cv::threshold(weighted_gray,mask,150,255,cv::THRESH_BINARY);
    }
    else{   //(  color_id == 1   )   1 Red
        //调用 weighted_cvt_gray()函数：(自定义函数，可跳转查看) 功能：获得红色加权灰度图
        weighted_cvt_gray(img_input,weighted_gray,color_id);

        //threshold():对灰度图进行阈值操作得到二值化掩膜。其中参数cv::THRESH_BINARY：大于150置255白，否则置0黑。结果保存在mask
        cv::threshold(weighted_gray,mask,150,255,cv::THRESH_BINARY);
        /*cv::Mat light_mask;
        cv::Mat color_list[3];
        cv::Mat r_mask,rb_mask,rg_mask;
        cv::split(img_input,color_list);
        //对单通道图像阈值操作得到二值化图像，像素值在50-255之间，则令该像素值为255,否则令其为0,输出图像保存在r_mask
        cv::inRange(color_list[2],50,255,r_mask);
        //图像相减操作，输出图像保存在rb_mask
        cv::subtract(color_list[2],color_list[0],rb_mask);
        //图像二值化操作，像素值在20-255之间，则令该像素值为255,否则令其为0,输出图像rb_mask
        cv::inRange(rb_mask,20,255,rb_mask);
        //图像相减操作，输出图像保存在rg_mask
        cv::subtract(color_list[2],color_list[1],rg_mask);
        ////图像二值化操作，像素值在20-255之间，则令该像素值为255,否则令其为0,输出图像rg_mask
        cv::inRange(rg_mask,20,255,rg_mask);
        //对图像进行与操作
        cv::bitwise_and(rb_mask,rg_mask,mask,r_mask);
        //对图像进行或操作
        cv::bitwise_or(mask,light_mask,mask);
        ////调用 weighted_cvt_gray()函数：如上有定义。
        weighted_cvt_gray(img_input,meng::weighted_gray,color_id);
        ////threshold():对灰度图进行阈值操作得到二值图像，大于150置255白，否则置0黑。结果保存在mask
        cv::threshold(meng::weighted_gray,mask,150,255,cv::THRESH_BINARY);*/
    }
}

//int meng::predict(cv::Ptr<cv::ml::SVM> &model,cv::HOGDescriptor * hog ,cv::Mat &transform_img, int &model_cnt)
//{
//    cv::Mat t_gray,t_equ ;
//    cv::cvtColor(transform_img,t_gray,cv::COLOR_BGR2GRAY);
//    cv::equalizeHist(t_gray,t_equ);
//
//    int r=0;
//    std::vector<float> descriptors;
//
//    hog->compute(t_equ, descriptors);
//    cv::Mat dst(1, int(descriptors.size()), CV_32FC1, descriptors.data());
//
//    r = model->predict(dst);
//    model_cnt++;
//
//#if 0
//    cv::namedWindow("transform_img",0);
//    cv::imshow("transform_img",transform_img);
//    if (r==1){
//        cv::namedWindow("transform_img",0);
//        cv::imshow("transform_img",transform_img);
//
//        cv::Mat bin;
//        cv::threshold(t_equ,bin,0,255,cv::THRESH_OTSU); //test for threshold to t_equ
//        cv::imshow("equ",t_equ);
//        cv::imshow("bin",bin);
//        //int k = cv::waitKey();
//        //if(k=='m')
//        {
//            static int save_num = 0;
//            std::string roi_name = "/home/snow/imgs/train/" + std::to_string(0+save_num++) + ".jpg";
//            cv::imwrite(roi_name, t_gray);
//            cv::waitKey(1);
//            printf("save successfully\n");
//        }
//    }
//#endif
//
//    if (r==0)
//        return r;
//
//    //===========
//    cv::Point2f p(20,20);
//
//    int c_x, c_y;
//    c_x = p.x;
//    c_y = p.y;
//
//
//    cv::Rect highLightRoiRect(c_x - 5, c_y - 5, 2 * 5, 2 * 5);
//    limit_rect(highLightRoiRect, highLightRoiRect, transform_img.rows, transform_img.cols);
////    cv::rectangle(transform_img,highLightRoiRect,cv::Scalar(0,255,255), 1);
////    cv::namedWindow("transform_img",0);
////    cv::imshow("transform_img",transform_img);
//    cv::Mat highLightRoi_img = transform_img(highLightRoiRect);
//
//    int pxl_cnt=0;
//    if(Color_id==0) //Blue
//    {
//        for (int i = 0; i < highLightRoi_img.rows; i++){
//            uchar *data = highLightRoi_img.ptr<uchar>(i);
//            for (int j = 0; j < highLightRoi_img.cols; j++){
//                uchar b = data[j*3];
//                uchar g = data[j*3+1];
//                uchar r = data[j*3+2];
//
//                if (((b - r) > 50) && ((b > 100) || (g > 100)))
//                    pxl_cnt++;
//            }
//        }
//    } else if(Color_id==1){
//        for (int i = 0; i < highLightRoi_img.rows; i++){
//            uchar *data = highLightRoi_img.ptr<uchar>(i);
//            for (int j = 0; j < highLightRoi_img.cols; j++){
//                uchar b = data[j*3];
//                uchar g = data[j*3+1];
//                uchar r = data[j*3+2];
//
//                if (((r - b) > 50) && ((r > 100) || (g > 100)))
//                    pxl_cnt++;
//            }
//        }
//    }
//    if(pxl_cnt>2)   //persent = pxl_cnt / 10 / 10
//    {
//        std::cout<<"中心点高亮"<<pxl_cnt<<std::endl;
//        r=0;
//    }
//
//    if(r==0)
//        return r;
//
//    //meng::Color_id为红时才需此操作
//    if ((r != 0) && (meng::Color_id==1)) {
//        int width  = 5;
//        int height = 5;
//
//        cv::Mat c3[3];
//        cv::Mat diff[3];
//        cv::Mat diff_mask[4];
//
//        cv::split(transform_img,c3);
//        cv::absdiff(c3[0],c3[1],diff[0]);
//        cv::absdiff(c3[1],c3[2],diff[1]);
//        cv::absdiff(c3[2],c3[3],diff[2]);
//
//        cv::inRange(diff[0],80,255,diff_mask[0]);
//        cv::inRange(diff[1],80,255,diff_mask[1]);
//        cv::inRange(diff[2],80,255,diff_mask[2]);
//
//        cv::bitwise_or(diff_mask[0],diff_mask[1],diff_mask[3]);
//        cv::bitwise_or(diff_mask[2],diff_mask[3],diff_mask[3]);
//
//        cv::Rect t_r(c_x - width, c_y - height, 2 * width, 2 * height);
//        limit_rect(t_r, t_r, transform_img.rows, transform_img.cols);
//
//        float per = check_mask_area(diff_mask[3], t_r)/t_r.area();
//        if (per>0.05)
//        {
//            r = 0;
//            std::cout<<"数码管干扰  "<<per<<std::endl;
//        }
//    }
//    //===========
//
//    return r;
//}

//输入SVM模型，HOG特征描述子，透视变换图
//通过SVM模型判断是不是真的装甲板，是真的返回r=1,否则返回r=0
int meng::predict(cv::Ptr<cv::ml::SVM> &model,cv::HOGDescriptor * hog ,cv::Mat &transform_img)
{
    cv::Mat t_gray,t_equ ;
    //将BGR透视变换图转换为灰度图，结果存在t_gray
    cv::cvtColor(transform_img,t_gray,cv::COLOR_BGR2GRAY);
    //SVM模型识别结果
    int r=0;
    //直方图分类
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

//###################################
    //hog特征分类
    std::vector<float> descriptors;     //HOG特征描述子
    cv::equalizeHist(t_gray,t_equ);     //直方图均值化，归一化图像亮度和增强对比度，均值化后的图像存储在t_equ

    //红色数码管干扰
    int pxl_cnt=0, bias= 10, c_x = 20, c_y = 20;
    //Red
    if(Color_id==1)
    {   ///遍历透视变换图的所有像素点///
        //遍历行
        for (int i = 0; i < transform_img.rows; i++)
        {
            //data是指向透视变换图第i+1行第一个像素的指针
            uchar *data = transform_img.ptr<uchar>(i);
            //ptr_gray是指向灰度图第i+1行第一个元素的指针
            uchar *ptr_gray =  t_gray.ptr<uchar>(i);
            //ptr_equ是指向直方图均值化图第i+1行第一个元素的指针
            uchar *ptr_equ =  t_equ.ptr<uchar>(i);
            //遍历列
            for (int j = 0; j < transform_img.cols; j++)
            {   //将透视变换图中每个像素点三通道分离
                uchar b = data[j*3];    //第i+1行第j+1列 B通道像素值
                uchar g = data[j*3+1];  //第i+1行第j+1列 G通道像素值
                uchar r = data[j*3+2];  //第i+1行第j+1列 R通道像素值

                //对三通道进行条件约束，目的是找到在中间区域内的红色像素点
                if ((((r - b) > 50) && ((r > 100) || (g > 100))) || (ptr_gray[j] > 60))
                {
                    //std::cout<<"val set to zero \n";
                    ptr_equ[j] = 0;     //如果满足约束条件，说明是数码管，应排除干扰，直方图均值化图置为0黑
                    //透视变换图尺寸为（40,40）
                    //如果该点在图像的（10-30），（10-30）即就是中心区域
                    if((i<c_y+bias) && (i>c_y-bias) && (j<c_x+bias) && (j>c_x-bias))
                        pxl_cnt++;      //pxl_cnt（红色像素点）自加1
                }
            }
        }
        //如果中间区域有超过两个红色像素点，则说明中心点高亮，则说明不是装甲，视为数码管干扰
        if(pxl_cnt>2){
            std::cout<<"中心点高亮"<<pxl_cnt<<std::endl;
            return 0;
        }
    }         ////将数码管干扰像素值置为0黑////
//    else if(Color_id==0) //Blue
//    {
//        for (int i = 0; i < highLightRoi_img.rows; i++){
//            uchar *data = highLightRoi_img.ptr<uchar>(i);
//            for (int j = 0; j < highLightRoi_img.cols; j++){
//                uchar b = data[j*3];
//                uchar g = data[j*3+1];
//                uchar r = data[j*3+2];
//
//                if (((b - r) > 50) && ((b > 100) || (g > 100)))
//                    pxl_cnt++;
//            }
//        }
//    }


//###################################
    //计算HOG特征值
    hog->compute(t_equ, descriptors);
    //将描述子展成一维，以便于送进svm训练
    cv::Mat dst(1, int(descriptors.size()), CV_32FC1, descriptors.data());
    //送入SVM模型进行识别，r存放识别的结果
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
    //说明SVM判断不是真实的装甲板
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

    //===========
    //meng::Color_id为红时才需此操作
    //SVM识别结果不为0,则说明
    if ((r != 0) && (meng::Color_id==1)) {
        //宽度为bias=10
        int width  = bias;
        //高度为bias=10
        int height = bias;
        //颜色三通道
        cv::Mat c3[3];
        //存储各通道像素值做差的绝对值
        cv::Mat diff[3];
        //
        cv::Mat diff_mask[4];

        //split():多通道图像分离操作.分离出的三通道序列保存在c3中
        cv::split(transform_img,c3);
        //absdiff（）：两个矩阵的差的绝对值。结果存储在diff[]中
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
         /*Use TBB in opencv to accelerate my code***
        It can reduce peak time from 10+ms to about 6ms*/   ////需要理解并学会使用
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
        //遍历所有轮廓
        for (int contour_i = 0; contour_i < contours.size(); contour_i++)
        {
            cv::RotatedRect minEllipse;     //最小旋转外接椭圆
            cv::Rect min_b_box;             //存放椭圆的最小外接矩形
            cv::Mat min_b_box_roi;
            cv::RotatedRect min_r_Rect;     //最小外接矩形

            /// make sure you can find a ellipse  确保能够找到椭圆 ///
            if (contours[contour_i].size() > 4) //如果每个轮廓内包含的点超过四个
            {
                //cv::minAreaRect()：求得每个轮廓包含所有点的最小外接矩形，存储在 min_r_Rect 中
                min_r_Rect = cv::minAreaRect(contours[contour_i]);

                //fitEllipse()：椭圆拟合操作：对每个找到的轮廓创建可倾斜的最小外接椭圆，存储在 minEllipse 中
                minEllipse = fitEllipse(contours[contour_i]);

                //椭圆的最小外接矩形，存储在 min_b_box 中
                min_b_box = minEllipse.boundingRect();

                //cv::contourArea():计算轮廓面积，存储在 areaOfContour 中
                float areaOfContour = cv::contourArea(contours[contour_i]);

                //计算拟合的最小外接椭圆的长/宽，比值存储在 ellipse_h_w_div 中
                float ellipse_h_w_div = minEllipse.size.height / minEllipse.size.width;

                //计算轮廓面积/最小外接矩形面积，比值存储在 area_persent 中
                float area_persent = areaOfContour / min_r_Rect.size.area();

                //面积比<0.66，在原图中的外接椭圆的中心绘制文字ap
                if(area_persent<0.66){
                    cv::putText(img_src,"aP",minEllipse.center,cv::FONT_HERSHEY_COMPLEX,0.5,cv::Scalar(0,255,255));
                    continue;   // 若约束条件符合，跳出当前循环
                }
                //拟合的最小外接椭圆长宽比大于10或小于1，绘制wh
                if((ellipse_h_w_div > meng::bar_slim_k_H) || (ellipse_h_w_div < meng::bar_slim_k_L)){
                    cv::putText(img_src,"wh",minEllipse.center,cv::FONT_HERSHEY_COMPLEX,0.5,cv::Scalar(0,255,255));
                    continue;   // 若约束条件符合，跳出当前循环
                }
                //拟合的最小外接椭圆的旋转角度大于45并且小于180-45，绘制al
                if ((minEllipse.angle > meng::bar_angle_limit) && (minEllipse.angle < (180-meng::bar_angle_limit) )){
                    cv::putText(img_src,"al",minEllipse.center,cv::FONT_HERSHEY_COMPLEX,0.5,cv::Scalar(0,255,255));
                    continue;    // 若约束条件符合，跳出当前循环
                }
                //拟合的最小外接椭圆的长度大于70或者小于10,绘制hl
                if((minEllipse.size.height > meng::bar_height_H ) || (minEllipse.size.height < meng::bar_height_L)){
                    cv::putText(img_src,"hl",minEllipse.center,cv::FONT_HERSHEY_COMPLEX,0.5,cv::Scalar(0,255,255));
                    continue;    // 若约束条件符合，跳出当前循环
                }

                // 平移：对椭圆的最小外接矩形向左向上分别平移3个像素
                // 缩放：对椭圆的最小外接矩形左上顶点不变，宽度高度分别加 6
                // 最终结果是将之前图像扩大一圈，每个边扩大6个像素
                min_b_box = min_b_box + cv::Point(-3, -3) + cv::Size(6, 6);

                //limit_rect():限定区域操作 （自定函数，跳转查看），返回范围约束后的最小外接矩形 min_b_box
                if( limit_rect(min_b_box,min_b_box,img_ori.rows,img_ori.cols)==0 )  //确保 min_b_box 在 img_ori 中
                    continue;   // 若约束条件符合，跳出当前循环

                cv::Mat bgr_roi, hsv_Roi, colorMaskRoi;

                //在未经处理的备份原图img_ori中抠出min_b_box区域，也即是我们寻找到的灯条ROI，存储在 bgr_roi 中
                bgr_roi = img_ori(min_b_box);

                //cv::cvtColor():色彩空间转换：将bgr_roi转换为HSV格式，存储在 hsv_Roi 中
                cv::cvtColor(bgr_roi,hsv_Roi,cv::COLOR_BGR2HSV);

                if( color_id == 0 ){    //blue
                    //二值化，针对HSV图像，通过阈值操作把蓝色突出，其他颜色置0,二值化后的HSV图像存储在 colorMaskRoi 中
                    cv::inRange(hsv_Roi,cv::Scalar(100,100,100),cv::Scalar(124,255,255),colorMaskRoi);
                }
                else{
                    //red   做两次二值化
                    cv::Mat hsv1, hsv2;
                    //二值化，针对HSV图像，通过阈值操作把红色突出，其他颜色置0,二值化后的HSV图像存储在 hsv1 中
                    cv::inRange(hsv_Roi,cv::Scalar(0,70,70),cv::Scalar(10,200,255),hsv1);

                    //二值化，针对HSV图像，通过阈值操作把红色突出，其他颜色置0,二值化后的HSV图像存储在 hsv2 中
                    cv::inRange(hsv_Roi,cv::Scalar(170,70,70),cv::Scalar(180,200,255),hsv2);

                    //两次二值化结果相加为最终的 colorMaskRoi    //因为红色较难过滤，需要两次阈值操作
                    colorMaskRoi = hsv1 + hsv2;
                }

                int correct_pxl=0;  //经过约束后符合要求的点的数量

                for(int i=0;i<colorMaskRoi.rows;i++){                   //遍历灯条二值化后的hsv图像行
                    uchar *hsv_ptr = colorMaskRoi.ptr<uchar>(i);        //灯条二值化后的hsv图像第i+1行第一个元素的指针
                    uchar *bgr_ptr = bgr_roi.ptr<uchar>(i);             //灯条bgr图像第i+1行第一个元素的指针

                    for(int j=0;j<colorMaskRoi.cols;j++){               //遍历灯条二值化后的hsv图像列
                        uchar hsv_val = hsv_ptr[j];                     //把第i+1行的第j+1元素赋值给hsv_val

                        //将bgr图像三通道像素值分别赋值给b,g,r
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
                //符合要求的点与轮廓面积的比值，赋给 pxl_persent
                float pxl_persent = (float)correct_pxl / areaOfContour;

                //如果比值低于百分之十，说明不符合
                if (pxl_persent < 0.1){
                    //在不符合的轮廓中绘制 px1
                    cv::putText(img_src,"pxl",minEllipse.center,cv::FONT_HERSHEY_COMPLEX,0.5,cv::Scalar(0,255,255));
                    continue;   // 若约束条件符合，跳出当前循环

                }

        ////    如上所有遍历完成，当所有的约束条件if都不满足时，剩下的就是符合要求的灯条图像     ////
#ifdef Show
                cv::rectangle(img_src,min_b_box,cv::Scalar(0,255,255), 1);
#endif
                cv::Mat fill_roi;

                bar_list[contour_i].ellipse = minEllipse;   //最小外接椭圆
                bar_list[contour_i].b_rec = min_b_box;      //最小外接矩形
                bar_list[contour_i].level = 1;              //等级为1

                /**calculate endpoint**/  //计算端点
                //将椭圆旋转角度补角值转换为弧度值
                // 90.0 - bar_list[contour_i].ellipse.angle是椭圆长轴到x方向的角度
                float angle_rad = (90.0 - bar_list[contour_i].ellipse.angle) / 180.0 * 3.1415926;

                //椭圆长轴值
                float h = bar_list[contour_i].ellipse.size.height;

                /**find bar endpoint**/
                /*find_newpoint():寻找椭圆沿长轴方向的端点坐标（自定函数，跳转查看）*/
                //bar_list[contour_i].endpoints[0]中存放椭圆上端点的坐标
                find_newpoint(bar_list[contour_i].ellipse.center, angle_rad, 0.5 * h,
                              bar_list[contour_i].endpoints[0], img_ori.rows, img_ori.cols);

                //bar_list[contour_i].endpoints[1]中存放椭圆下端点的坐标
                find_newpoint(bar_list[contour_i].ellipse.center, angle_rad, -0.5 * h,
                              bar_list[contour_i].endpoints[1], img_ori.rows, img_ori.cols);

                /*check_endpoint():确保椭圆上端点坐标在椭圆下坐标上方（自定函数，跳转查看）*/
                check_endpoint(bar_list[contour_i].endpoints[0], bar_list[contour_i].endpoints[1]);

#ifdef Show     //绘制椭圆
                cv::ellipse( img_src, minEllipse, cv::Scalar(255,255,255), 1, 8 );
                //cv::putText(img_src,std::to_string((int)(minEllipse.angle)),minEllipse.center,cv::FONT_HERSHEY_COMPLEX,0.5,cv::Scalar(0,0,255));
                //printf("minEllipse center: %f , %f\n",minEllipse.center.x , minEllipse.center.y);
#endif
            }
        }
#endif               /******此步结束已经得到符合要求的椭圆的所有信息******/  //这些信息存储在 bar_list 中


        /********************   2. Match hight light bar pairs  ************************/
                                     //匹配成对的高亮灯条//
                /**sort bars according to their distance to a point***/
        std::vector<struct s_bar> valid_bar_list,tmp_valid_bar_list;
        //遍历每个灯条
        for(int i=0;i<bar_list.size();i++) {
            if (bar_list[i].level == 1)     //对如上已经定义等级为1的bar_list  ,把bar_list[i] 信息放入 tmp_valid_bar_list 中
                tmp_valid_bar_list.push_back(bar_list[i]);
        }

        //若 tmp_valid_bar_list 不为空
        if( tmp_valid_bar_list.size()>0 )

            //sort_ellipse():升序排列灯条(自定函数，跳转查看)
            sort_ellipse(tmp_valid_bar_list,valid_bar_list,meng::dynamic_sp.x, meng::dynamic_sp.y);

#ifdef Show
        for(int i=0;i<valid_bar_list.size();i++)
            cv::line(img_src,valid_bar_list[i].endpoints[0],valid_bar_list[i].endpoints[1],cv::Scalar(0,255,0),2);
#endif
        //如果升序排列后灯条数目大于1
        if(valid_bar_list.size()>1){
            int result=-1;  //返回结果标志位

            ///获取所有符合要求的椭圆小装甲扩展点,大装甲扩展点,小装甲待提点,结果已经赋值给bar结构体
            get_surround_pts(valid_bar_list,img_ori);

            ///通过获取的小装甲扩展点,大装甲扩展点,小装甲待提点,判断该灯条是否有匹配灯条,并且判断匹配对中是否存在装甲板,返回有装甲板灯条的编号
            result=find_armor_int_all_pts(img_src, valid_bar_list,img_ori);

            /*When we have armor detection result,we need to make sure again  //当我们获得了识别装甲的结果，我们需要再确认一次
             * if there is blue or red points in the center roi of armor, this result is no valid   //如果装甲的中心有红点或者蓝点，该结果无效
             * */
            //若有装甲
            if( result != -1 ) {
                //获得装甲板的中心点坐标 p             getCrossPoint():(自定函数，跳转查看)
                cv::Point2f p = getCrossPoint(valid_bar_list[result].armor_vertex[0],
                                              valid_bar_list[result].armor_vertex[2],
                                              valid_bar_list[result].armor_vertex[1],
                                              valid_bar_list[result].armor_vertex[3]);

//                for(int i=0;i<4;i++){
//                    cv::line(img_src,valid_bar_list[result].armor_vertex[i],valid_bar_list[result].armor_vertex[(i+1)%4],
//                             cv::Scalar(0,255,255),1);
//
//                    if(valid_bar_list[result].isPosePoints)
//                        cv::line(img_src,valid_bar_list[result].pose_points[i],valid_bar_list[result].pose_points[(i+1)%4],
//                                 cv::Scalar(0,255,0),1);
//                }

                if (p.x != -1)  //若p在图像内部，将装甲板中心点坐标赋值给sp，sp初始值（-1,-1）被覆盖
                    sp = p;
            }
        }

        //如果只有单个灯条信息，或者装甲板中心点坐标为-1，我们需要利用单个灯条寻找装甲板
        if((valid_bar_list.size()==1) ||(sp.x == -1)){

            int result = 0;

            //
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

    return sp;       //返回装甲板中心点坐标
}

/************** 装甲识别主函数  **************/
cv::Point2f meng::detection(cv::Mat &img,float & distance)
{
    cv::Point2f sp(-1,-1);
    if(meng::g_flag != 1)       //如果所有参数未初始化
    {
        detect_init();          //初始化所有参数

        meng::g_flag = 1;       //flag置1

        printf("init=================\n");
    }
    else                        //已经初始化
    {
        double t = (double)cvGetTickCount();    //计算运行时间

        ///     调用detect_task，输入图像和打击颜色，输出装甲中心点坐标   ///
        sp=detect_task(img,meng::Color_id);

        sp=sp*2;                              //装甲中心点坐标扩大，因为刚才是resize 0.5

        std::cout<<sp<<std::endl;             //输出装甲中心点坐标

        t = (double)cvGetTickCount() - t;     //t是程序处理一帧图像消耗的时间

        printf( "run time = %gms\n", t/(cvGetTickFrequency()*1000) );   //打印一帧图像处理消耗时间
    }
    return sp;   //返回装甲中心点坐标
}

///函数功能解释模板
/*********************************
 *
 * Description:
 * Input:
 *
 *
 * Output：
 *
 *
 *
 * Others：
 * 		1、
 * 		2、
 * 		3、
 *
**********************************/
