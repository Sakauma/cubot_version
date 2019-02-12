#include <iostream>
#include <opencv2/opencv.hpp>

#include "armor/coordinante_process.h"
#include "armor/bar_points.h"
#include "armor/armor_main.h"

namespace AutoHit
{
    int   color_id ;
    float bar_height_H;
    float bar_height_L;
    float bar_slim_k_H;
    float bar_slim_k_L;

    float bar_angle_limit;
    float small_pair_bar_angle_limit;
    float big_pair_bar_angle_limit;

    float high_light_mask_k ;
    float proposal_bar_area_k;

    int   autohit_flag;

    cv::Point2f sp_dynamic;
    cv::Mat img_transform;

    std::vector <cv::Point2f> world_pts(4);
    std::vector <cv::Point2f> new_pts(4);

    cv::Ptr <cv::ml::SVM> model_hog;
    cv::HOGDescriptor *pHog = new cv::HOGDescriptor(cv::Size(40, 40), cv::Size(16, 16), cv::Size(8, 8), cv::Size(8, 8), 9);
    cv::Mat element = cv::getStructuringElement( cv::MORPH_CROSS, cv::Size(2 * 1 + 1, 2 * 1 + 1), cv::Point(1, 1) );
}

static void detect_init( void )
{
    //========get parameters from file==========
    cv::FileStorage file_2( "../cfg/AutoHitCfg.yml", cv::FileStorage::READ );
    if ( file_2.isOpened() )
    {
        int isSave = ( int )file_2[ "isSave" ];
        int saveStartNum  = ( int )file_2[ "saveStartNum" ];
        std::string img_savePath = ( std::string )file_2[ "img_savePath" ];

        std::string svm_model_hog_path  = ( std::string )file_2[ "svm_model_hog_path" ];

        std::string svm_model_histogram_path  = ( std::string )file_2[ "svm_model_histogram_path" ];

        AutoHit::color_id = ( int )file_2[ "color2Strike" ];

        AutoHit::high_light_mask_k = ( float )file_2[ "high_light_mask_k" ];
        AutoHit::proposal_bar_area_k=( float )file_2[ "proposal_bar_area_k" ];

        AutoHit::bar_height_H = ( float )file_2[ "bar_height_H" ];
        AutoHit::bar_height_L = ( float )file_2[ "bar_height_L" ];
        AutoHit::bar_slim_k_H = ( float )file_2[ "bar_slim_k_H" ];
        AutoHit::bar_slim_k_L = ( float )file_2[ "bar_slim_k_L" ];

        AutoHit::bar_angle_limit = ( float )file_2[ "bar_angle_limit" ];
        AutoHit::small_pair_bar_angle_limit = ( float )file_2[ "small_pair_bar_angle_limit" ];
        AutoHit::big_pair_bar_angle_limit=( float )file_2[ "big_pair_bar_angle_limit" ];

        AutoHit::sp_dynamic.x=( float )file_2[ "sp_dynamic_x" ];
        AutoHit::sp_dynamic.y=( float )file_2[ "sp_dynamic_y" ];

        file_2.release();

        std::cout << "try to read the configurate " << std::endl;
        std::cout << isSave << " " << saveStartNum << " " << img_savePath << " "
                  << svm_model_hog_path << " " << svm_model_histogram_path << " "
                  << AutoHit::color_id <<" " << AutoHit::high_light_mask_k << " "
                  << AutoHit::proposal_bar_area_k << " " << AutoHit::bar_height_H << " "
                  << AutoHit::bar_height_L <<" " << AutoHit::bar_slim_k_H << " " << AutoHit::bar_slim_k_L << " "
                  << AutoHit::bar_angle_limit << " " << AutoHit::small_pair_bar_angle_limit << " "
                  << AutoHit::big_pair_bar_angle_limit << " " << AutoHit::sp_dynamic << " " << std::endl;

        AutoHit::autohit_flag = 0;

        AutoHit::world_pts[0] = cv::Point2f(0, 0);
        AutoHit::world_pts[1] = cv::Point2f(40, 0);
        AutoHit::world_pts[2] = cv::Point2f(40, 40);
        AutoHit::world_pts[3] = cv::Point2f(0, 40);

        std::cout << std::endl << svm_model_hog_path << std::endl;
        AutoHit::model_hog = cv::ml::SVM::load( svm_model_hog_path );

        if( AutoHit::model_hog.empty() )
            std::cerr << "model_hog open failed" << std::endl;

    }
    else{
        std::cerr << "autohit cfg file read failed\n";
        return ;
    }
}

/*********************************
 *
 * Description: 获得加权灰度图
 * Input:
 * 		img_src：原始BGR图像
 * 		color_id:颜色标识：0蓝 1红
 * output:
 *      img_gray:相应颜色加权灰度图
 * Others：
 * 		1、判断 color_id
 * 		2、cv::split（）：图像BGR三通道分离操作
 * 		3、套用BGR图像转灰度图像公式，各通道加权后值赋给gray
 *
**********************************/
void weighted_cvt_gray( const cv::Mat &img_src, cv::Mat & img_gray, int color_id )
{
    if ( color_id == 0 )
    {

        cv::Mat colors[3];

        cv::split( img_src, colors );

        img_gray = colors[0] * 0.9 + colors[1] * 0.05 + colors[2] * 0.05;

        std::cout << "  color to strike is blue  ";
    }else
    {
        cv::Mat colors[3];
        cv::split( img_src, colors );
        img_gray = colors[0] * 0.05 + colors[1] * 0.05 + colors[2] * 1.5;
        std::cout << "  color to strike is red  ";
    }
}

/*********************************
 *
 * Description: 二值化获得灯条高亮图
 * Input:
 * 		img_src： 原始BGR图像
 * 		color_id：颜色标识：0蓝 1红
 * Output:
 *       mask： 灰度化再阈值操作后的二值化掩膜
 * Others：
 * 		1、判断color_id
 * 		2、调用 weighted_cvt_gray()函数获得对应color_id的加权灰度图
 * 		3、使用threshold()函数，对加权灰度图阈值操作后得到二值化图像，函数中的参数在此的功能是：若图像像素大于150置255白，否则置0黑
 *
*********************************/
void get_hight_light_bar( const cv::Mat & img_src, cv::Mat & mask, int color_id )
{
    cv::Mat img_weighted_gray;
    if (color_id == 0) {

        weighted_cvt_gray(img_src, img_weighted_gray, color_id);

        cv::threshold(img_weighted_gray, mask, 150, 255, cv::THRESH_BINARY);
    } else
    {
        weighted_cvt_gray( img_src, img_weighted_gray, color_id );

        cv::threshold( img_weighted_gray, mask, 150, 255, cv::THRESH_BINARY );
    }
}

//输入SVM模型，HOG特征描述子，透视变换图
//通过SVM模型判断是不是真的装甲板，是真的返回r=1,否则返回r=0
int AutoHit::predict( cv::Ptr<cv::ml::SVM> &model, cv::HOGDescriptor *pHog, cv::Mat &img_transform )
{
    cv::Mat trans_gray, trans_equ;
    cv::cvtColor( img_transform, trans_gray, cv::COLOR_BGR2GRAY );
    int r=0;

#ifdef HistBin
    std::vector<float> descriptors;
    cv::threshold( trans_gray, trans_equ, 18, 255, cv::THRESH_BINARY );
    hog->compute( trans_equ ,descriptors );
    cv::Mat dst( 1, int(descriptors.size()), CV_32FC1, descriptors.data() );
    r = model->predict(dst);
#else

    //hog特征分类
    std::vector<float> descriptors;
    cv::equalizeHist( trans_gray, trans_equ );

    int pxl_cnt =0, bias = 10, c_x = 20, c_y = 20;
    if ( color_id == 1 )
    {
        for ( int i = 0; i < img_transform.rows; i++ )
        {
            uchar *data = img_transform.ptr<uchar>(i);
            uchar *ptr_gray =  trans_gray.ptr<uchar>(i);
            uchar *ptr_equ =  trans_equ.ptr<uchar>(i);
            for ( int j = 0; j < img_transform.cols; j++ )
            {
                uchar b = data[j * 3];
                uchar g = data[j * 3 + 1];
                uchar r = data[j * 3 + 2];

                if ( ( ( ( r - b ) > 50 ) && ( ( r > 100 ) || ( g > 100 ) ) ) || ( ptr_gray[j] > 60 ) )
                {
                    ptr_equ[j] = 0;
                    if ( ( i < c_y + bias ) && ( i > c_y - bias ) && ( j < c_x + bias ) && ( j > c_x - bias ) )
                        pxl_cnt++;
                }
            }
        }
        if ( pxl_cnt > 2 )
        {
            std::cout << "中心点高亮" << pxl_cnt << std::endl;
            return 0;
        }
    }

    ////将数码管干扰像素值置为0黑////
//    else if(color_id==0) //Blue
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

    //计算HOG特征值
    pHog->compute(trans_equ, descriptors);
    cv::Mat dst( 1 ,int(descriptors.size() ), CV_32FC1, descriptors.data() );
    r = model->predict(dst);

#endif

//#if 0
//    cv::namedWindow("img_transform" ,0);
//    cv::imshow("img_transform" ,img_transform);
//    if (r==1){
//        cv::namedWindow("img_transform" ,0);
//        cv::imshow("img_transform" ,img_transform);
//
//        cv::Mat bin;
//        cv::threshold(trasn_equ ,bin ,0 ,255 ,cv::THRESH_OTSU);
//        cv::imshow("equ" ,trans_equ);
//        cv::imshow("bin" ,bin);
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

    //说明SVM判断不是真实的装甲板
    if ( r==0 )
        return r;

//#if 0
//    cv::Mat bin0;
//    //二值化操作
//    cv::threshold(t_gray,bin0,20,255,cv::THRESH_BINARY);
//    cv::namedWindow("bin0",0);
//    //显示二值化操作后的图像
//    cv::imshow("bin0",bin0);
//    {
//        static int save_num = 0;
//        std::string roi_name = "/home/snow/imgs/" + std::to_string(0+save_num++) + ".jpg";
//        cv::imwrite(roi_name, t_gray);
//        cv::waitKey(1);
//        printf("save successfully\n");
//    }
//#endif

    //autohit::color_id为红时才需此操作
    //SVM识别结果不为0,则说明
    if ( ( r != 0 ) && ( AutoHit::color_id==1 ) )
    {
        int width  = bias;
        int height = bias;
        cv::Mat color3[3];
        //存储各通道像素值做差的绝对值
        cv::Mat diff[3];
        cv::Mat diff_mask[4];

        cv::split( img_transform,color3 );
        cv::absdiff( color3[0], color3[1], diff[0] );
        cv::absdiff( color3[1], color3[2], diff[1] );
        cv::absdiff( color3[2], color3[0], diff[2] );

        cv::inRange( diff[0], 80, 255, diff_mask[0] );
        cv::inRange( diff[1], 80, 255, diff_mask[1] );
        cv::inRange( diff[2], 80, 255, diff_mask[2] );

        cv::bitwise_or( diff_mask[0], diff_mask[1], diff_mask[3] );
        cv::bitwise_or( diff_mask[2], diff_mask[3], diff_mask[3] );

        cv::Rect t_r( c_x - width ,c_y - height ,2 * width ,2 * height );
        limit_rect( t_r, t_r, img_transform.rows, img_transform.cols );

        float per = check_mask_area( diff_mask[3], t_r ) / t_r.area();
        if ( per>0.05 )
        {
            r = 0;
            std::cout << "数码管干扰  " << per << std::endl;
        }
    }

    return r;
}

///     利用大装甲板的顶点坐标计算小装甲板的顶点坐标      ///
void AutoHit::transform_armor_vertex( cv::Point2f *pPts, float k )
{
    cv::Point2f new_pts[4];

    new_pts[0] = pPts[0];
    new_pts[1] = pPts[1];
    new_pts[2] = pPts[2];
    new_pts[3] = pPts[3];

    pPts[0] = new_pts[0] + ( new_pts[1]-new_pts[0] ) * k;
    pPts[1] = new_pts[0] + ( new_pts[1]-new_pts[0] ) * ( 1 - k );

    pPts[3] = new_pts[3] + ( new_pts[2]-new_pts[3] ) * k;
    pPts[2] = new_pts[3] + ( new_pts[2]-new_pts[3] ) * ( 1 - k );
}

  /******************************装甲识别主体*******************************/
static cv::Point2f detect_task( cv::Mat &img_src, int color_id )
{
    cv::Point2f sp( -1, -1 );
    cv::resize(img_src ,img_src,cv::Size(),0.5 ,0.5);
    cv::Mat img_backups = img_src.clone();

    /******************************* Get hight_light_bar ********************************/

    cv::Mat high_light_mask_thre;
    get_hight_light_bar(img_src, high_light_mask_thre, color_id );

    cv::dilate( high_light_mask_thre, high_light_mask_thre, AutoHit::element );

    std::vector<cv::Vec4i> hierarchy;
    std::vector< std::vector<cv::Point> > contours;

    cv::findContours( high_light_mask_thre, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE, cv::Point(0, 0) );

    if ( contours.size() > 0 )
    {
        std::vector<struct s_bar> bar_list( contours.size() );

#ifdef OpenTBB  //如果开启TBB（Threading Building Blocks）并行计算库，目的是提升数据并行计算的能力
        cv::Range range0(0,contours.size());
        cv::parallel_for_( range0, [&](const cv::Range& range)
        {
            cv::RotatedRect minEllipse;
            cv::Rect min_b_box;
            cv::Mat min_b_box_roi;
            cv::RotatedRect min_r_Rect;

            for ( int contour_i = range.start; contour_i < range.end; contour_i++ )
            {
                /*make sure you can find a ellipse*/
                if ( contours[contour_i].size() > 4 )
                {
                    minEllipse = fitEllipse(contours[contour_i]);
                    min_b_box = minEllipse.boundingRect();

                    float ellipse_h_w_div = minEllipse.size.height / minEllipse.size.width;
                    if ( ( ( minEllipse.angle < autohit::bar_angle_limit ) || ( minEllipse.angle > ( 180-autohit::bar_angle_limit ) ) ) && ( ellipse_h_w_div > autohit::bar_slim_k_L ) &&
                          ( ellipse_h_w_div < autohit::bar_slim_k_H ) &&( minEllipse.size.height > autohit::bar_height_L ) &&( minEllipse.size.height < autohit::bar_height_H )
                          && ( ( cv::contourArea(contours[contour_i] ) / ( minEllipse.size.area()*0.25*3.1415 ) ) > 0.7 ) )
                    {
                        //get roi and make sure min bounding box is valid
                        if ( limit_rect( min_b_box, min_b_box, img_backups.rows, img_backups.cols )==0 )
                            continue;

//                      min_b_box_roi = img_backups( min_b_box );
                        cv::Mat bbox_color_mask = color_mask( min_b_box );

                        if ( !bbox_color_mask.empty() )
                        {
                            /* high light bar rect must contain some blue points*/
                            float k = cv::sum( bbox_color_mask )[0] / 255.0 / ( min_b_box.width*min_b_box.height );
                            if ( k > autohit::high_light_mask_k )
                            {
                                cv::Mat fill_roi;

                                bar_list[contour_i].ellipse = minEllipse;
                                bar_list[contour_i].b_rec = min_b_box;
                                bar_list[contour_i].level = 1;

                                /*calculate endpoint*/
                                float angle_rad = ( 90.0 - bar_list[contour_i].ellipse.angle ) / 180.0 * CV_PI;
                                float h = bar_list[contour_i].ellipse.size.height;
                                /*find bar endpoint*/
                                find_newpoint( bar_list[contour_i].ellipse.center, angle_rad, 0.5 * h,
                                               bar_list[contour_i].endpoints[0], img_backups.rows, img_backups.cols );
                                find_newpoint( bar_list[contour_i].ellipse.center, angle_rad, -0.5 * h,
                                               bar_list[contour_i].endpoints[1], img_backups.rows, img_backups.cols );
                                check_endpoint( bar_list[contour_i].endpoints[0], bar_list[contour_i].endpoints[1] );

                                cv::ellipse( img_src, minEllipse, cv::Scalar(255,255,255), 1, 8 );
//                                cv::putText( img_src,std::to_string((int)(minEllipse.angle)),minEllipse.center,cv::FONT_HERSHEY_COMPLEX,0.5,cv::Scalar(0,0,255));
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
        for ( int contour_i = 0; contour_i < contours.size(); contour_i++ )
        {
            cv::RotatedRect minEllipse;     //最小旋转外接椭圆
            cv::Rect min_b_box;             //存放椭圆的最小外接矩形
            cv::Mat min_b_box_roi;
            cv::RotatedRect min_r_Rect;     //最小外接矩形

            if ( contours[contour_i].size() > 4 ) //如果每个轮廓内包含的点超过四个
            {
                min_r_Rect = cv::minAreaRect( contours[contour_i] );

                minEllipse = fitEllipse( contours[contour_i] );

                min_b_box = minEllipse.boundingRect();

                float area_of_contour = cv::contourArea( contours[contour_i] );

                float ellipse_h_w_div = minEllipse.size.height / minEllipse.size.width;

                float area_persent = area_of_contour / min_r_Rect.size.area();

                if ( area_persent<0.66 )
                {
                    cv::putText( img_src, "aP", minEllipse.center, cv::FONT_HERSHEY_COMPLEX, 0.5, cv::Scalar(0, 255, 255) );
                    continue;
                }

                if ( ( ellipse_h_w_div > AutoHit::bar_slim_k_H ) || ( ellipse_h_w_div < AutoHit::bar_slim_k_L ) )
                {
                    cv::putText( img_src, "wh", minEllipse.center, cv::FONT_HERSHEY_COMPLEX, 0.5, cv::Scalar(0, 255, 255) );
                    continue;
                }

                if ( ( minEllipse.angle > AutoHit::bar_angle_limit ) && ( minEllipse.angle < ( 180 - AutoHit::bar_angle_limit ) ) )
                {
                    cv::putText( img_src, "al", minEllipse.center, cv::FONT_HERSHEY_COMPLEX, 0.5, cv::Scalar(0, 255, 255) );
                    continue;
                }

                if ( ( minEllipse.size.height > AutoHit::bar_height_H ) || ( minEllipse.size.height < AutoHit::bar_height_L ) )
                {
                    cv::putText( img_src, "hl", minEllipse.center, cv::FONT_HERSHEY_COMPLEX, 0.5, cv::Scalar(0 ,255 ,255) );
                    continue;
                }

                min_b_box = min_b_box + cv::Point(-3, -3) + cv::Size(6, 6);

                if ( limit_rect( min_b_box, min_b_box, img_backups.rows, img_backups.cols )==0 )
                    continue;

                cv::Mat bgr_roi, hsv_roi, color_mask_roi;

                bgr_roi = img_backups( min_b_box );

                cv::cvtColor( bgr_roi, hsv_roi, cv::COLOR_BGR2HSV );

                if( color_id == 0 )
                {
                    cv::inRange( hsv_roi, cv::Scalar(100, 100, 100), cv::Scalar(124, 255, 255), color_mask_roi );
                }
                else
                {
                    cv::Mat hsv1, hsv2;
                    cv::inRange( hsv_roi, cv::Scalar(0, 70, 70), cv::Scalar(10, 200, 255), hsv1 );
                    cv::inRange( hsv_roi, cv::Scalar(170, 70, 70), cv::Scalar(180, 200, 255), hsv2 );
                    color_mask_roi = hsv1 + hsv2;
                }

                int correct_pxl = 0;

                for ( int i = 0; i < color_mask_roi.rows; i++ )
                {
                    uchar *hsv_ptr = color_mask_roi.ptr<uchar>(i);
                    uchar *bgr_ptr = bgr_roi.ptr<uchar>(i);

                    for ( int j = 0; j < color_mask_roi.cols; j++ )
                    {
                        uchar hsv_val = hsv_ptr[j];
                        //将bgr图像三通道像素值分别赋值给b,g,r
                        uchar b = bgr_ptr[j * 3];
                        uchar g = bgr_ptr[j * 3 + 1];
                        uchar r = bgr_ptr[j * 3 + 2];

                        if ( color_id == 0 )
                        {
                            if ( hsv_val && ( b > 50 ) && ( r < 80 ) && ( b - 2 * r > 10 ) )
                                 correct_pxl++;
                        }
                        else if ( color_id == 1 )
                        {
                            if ( hsv_val && ( r > 70 ) && ( r - b > 50 ) && ( b < 110 ) )
                                 correct_pxl++;
                        }
                    }
                }
                float pxl_persent = ( float )correct_pxl / area_of_contour;

                if ( pxl_persent < 0.1 )
                {
                    cv::putText( img_src, "pxl", minEllipse.center, cv::FONT_HERSHEY_COMPLEX, 0.5, cv::Scalar(0, 255, 255) );
                    continue;

                }

#ifdef Show
                cv::rectangle( img_src, min_b_box, cv::Scalar(0, 255, 255), 1 );
#endif
                cv::Mat fill_roi;

                bar_list[contour_i].ellipse = minEllipse;
                bar_list[contour_i].b_rec = min_b_box;
                bar_list[contour_i].level = 1;

                float angle_rad = ( 90.0 - bar_list[contour_i].ellipse.angle ) / 180.0 * CV_PI;

                float h = bar_list[contour_i].ellipse.size.height;

                find_newpoint( bar_list[contour_i].ellipse.center ,angle_rad, 0.5 * h ,
                               bar_list[contour_i].endpoints[0], img_backups.rows, img_backups.cols );

                find_newpoint( bar_list[contour_i].ellipse.center, angle_rad, -0.5 * h ,
                               bar_list[contour_i].endpoints[1], img_backups.rows, img_backups.cols );

                check_endpoint( bar_list[contour_i].endpoints[0], bar_list[contour_i].endpoints[1] );

#ifdef Show
                cv::ellipse( img_src, minEllipse, cv::Scalar(255, 255, 255), 1, 8 );
                //cv::putText( img_src, std::to_string( ( int ) ( minEllipse.angle ) ), minEllipse.center, cv::FONT_HERSHEY_COMPLEX, 0.5, cv::Scalar(0, 0, 255) );
                //printf( "minEllipse center: %f ,%f\n", minEllipse.center.x, minEllipse.center.y );
#endif
            }
        }
#endif               /******此步结束已经得到符合要求的椭圆的所有信息******/  //这些信息存储在 bar_list 中


        /********************   2. Match hight light bar pairs  ************************/
                                     //匹配成对的高亮灯条//
                /**sort bars according to their distance to a point***/
        std::vector<struct s_bar> valid_bar_list, temp_valid_bar_list;
        //遍历每个灯条
        for ( int i=0; i<bar_list.size(); i++ )
        {
            if ( bar_list[i].level == 1 )
                 temp_valid_bar_list.push_back( bar_list[i] );
        }

        if ( temp_valid_bar_list.size()>0 )

             sort_ellipse( temp_valid_bar_list, valid_bar_list, AutoHit::sp_dynamic.x, AutoHit::sp_dynamic.y );

#ifdef Show
        for ( int i=0; i<valid_bar_list.size(); i++ )
            cv::line( img_src, valid_bar_list[i].endpoints[0], valid_bar_list[i].endpoints[1], cv::Scalar(0, 255, 0), 2 );
#endif
        if ( valid_bar_list.size()>1 )
        {
            int result = -1;

            get_surround_pts( valid_bar_list, img_backups );

            result = find_armor_int_all_pts( img_src,valid_bar_list, img_backups );

            //若有装甲
            if( result != -1 )
            {
                cv::Point2f p = getCrossPoint( valid_bar_list[result].armor_vertex[0]  ,
                                               valid_bar_list[result].armor_vertex[2]  ,
                                               valid_bar_list[result].armor_vertex[1]  ,
                                               valid_bar_list[result].armor_vertex[3] );
                if ( p.x != -1 )
                    sp = p;
            }
        }

        if ( ( valid_bar_list.size()==1 ) ||( sp.x == -1 ) )
        {

            int result = 0;

            //
            result = find_armor_for_single_bar( img_src, valid_bar_list, img_backups, sp );
        }

#ifdef Show
        cv::circle( img_src, autohit::sp_dynamic, 10, cv::Scalar(255, 255, 255), -1 );
        cv::putText( img_src, std::to_string( valid_bar_list.size() ), cv::Point(20,20), cv::FONT_HERSHEY_COMPLEX, 0.5, cv::Scalar(0, 0, 255) );
        if ( sp.x != -1 )
             cv::circle( img_src, sp, 10, cv::Scalar(0, 255, 0), 3 );
#endif
    }

#ifdef Show
    cv::imshow( "img_src", img_src );
    //cv::imshow( "color_mask", color_mask );

    if ( sp.x != -1 )
    {
        cv::Mat g ,e;
        cv::cvtColor( autohit::img_transform, g, cv::COLOR_BGR2GRAY );
        cv::equalizeHist(g, e);

        cv::imshow( "e", e );
        int key = cv::waitKey(1);
        if  (key == 13 )
        {//Enter
            static int roi_cnt = 0;
            cv::imwrite( "/home/cubot/roi/"+std::to_string(523+roi_cnt++)+".jpg" ,e );
            std::cout<<"save roi\n"<<std::endl;
        }
    }
    cv::waitKey(1);
#endif

    return sp;       //返回装甲板中心点坐标
}

/************** 装甲识别主函数  **************/
cv::Point2f AutoHit::detection( cv::Mat &img, float & distance ,float & T )
{
    cv::Point2f sp( -1, -1 );
    if ( AutoHit::autohit_flag != 1 )
    {
        detect_init();

        AutoHit::autohit_flag = 1;
        std::cout << "init=================" << std::endl;
    }
    else
    {
        double t =  (double )cv::getTickCount();
        sp=detect_task( img, AutoHit::color_id );
        sp=sp*2;
        std::cout<<sp<<std::endl;
        t = ( double )cv::getTickCount() - t;
        T = t/(cv::getTickFrequency()*1000);
        std::cout << "run time = %gms" << t/(cv::getTickFrequency()*1000) << std::endl;
    }
    return sp;
}
