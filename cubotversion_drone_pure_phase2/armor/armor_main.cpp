#include <iostream>
#include <opencv2/dnn.hpp>
#include <opencv2/opencv.hpp>
#include "../include/armor/coordinante_process.h"
#include "../include/armor/bar_points.h"
#include "armor/armor_main.h"

#define HogBin

float dataf[] = { 1325.4, 0, 378.7887,
                  0, 1327.5, 291.7711,
                  0, 0, 1 };
float dis[5] = { -0.0912, 0.4338, 0.00043798, -0.004, -1.1626 };

static void detect_init( void );

namespace autohit
{
    int color_id ;
    float bar_height_H;
    float bar_height_L;
    float bar_slim_k_H;
    float bar_slim_k_L;

    float bar_angle_limit;
    float small_pair_bar_angle_limit;
    float big_pair_bar_angle_limit;

    float high_light_mask_k ;
    float proposal_bar_area_k;

    int Model_cnt;
    int autohit_flag;

    cv::Point2f sp_dynamic;
    cv::Mat img_transform;

    std::vector<cv::Point2f> new_pts(4), world_pts(4);

    cv::Ptr<cv::ml::SVM> model;
    cv::Ptr<cv::ml::SVM> model_bin;
    cv::HOGDescriptor *pHog_bin = new cv::HOGDescriptor( cv::Size(30, 30), cv::Size(6, 6), cv::Size(6, 6), cv::Size(3, 3), 9 );
    cv::HOGDescriptor *pHog = new cv::HOGDescriptor( cv::Size(40, 40), cv::Size(16, 16), cv::Size(8, 8), cv::Size(8, 8), 9 );
    cv::Mat element = cv::getStructuringElement( cv::MORPH_CROSS, cv::Size( 2 * 1 + 1, 2 * 1 + 1 ), cv::Point(1, 1) );
}

static void detect_init( void )
{
    cv::FileStorage file_2( "../cfg/MengCfg.yml", cv::FileStorage::READ );

    if ( !file_2.isOpened() )
          std::cerr << "MengCfg.xml open failed...\n";

    int isSave = ( int )file_2[ "isSave" ];
    std::string img_savePath = ( std::string )file_2[ "img_savePath" ];

    std::string svm_model_path = ( std::string )file_2[ "svm_model_path" ];
    std::string svm_model_bin_path = ( std::string )file_2[ "svm_model_bin_path" ];

    autohit::color_id = ( int )file_2[ "color2Strike" ];
    autohit::high_light_mask_k = ( float )file_2[ "high_light_mask_k" ];
    autohit::proposal_bar_area_k = ( float )file_2[ "proposal_bar_area_k" ];

    autohit::bar_height_H = ( float )file_2[ "bar_height_H" ];
    autohit::bar_height_L = ( float )file_2[ "bar_height_L" ];
    autohit::bar_slim_k_H = ( float )file_2[ "bar_slim_k_H" ];
    autohit::bar_slim_k_L = ( float )file_2[ "bar_slim_k_L" ];

    autohit::bar_angle_limit = ( float )file_2[ "bar_angle_limit" ];
    autohit::small_pair_bar_angle_limit = ( float )file_2[ "small_pair_bar_angle_limit" ];
    autohit::big_pair_bar_angle_limit = ( float )file_2[ "big_pair_bar_angle_limit" ];
    autohit::sp_dynamic.x = ( float )file_2[ "sp_dynamic_x" ];
    autohit::sp_dynamic.y = ( float )file_2[ "sp_dynamic_y" ];
    file_2.release();

    std::cout << "try to read the configurate " << std::endl;
    std::cout << isSave<<" "<< img_savePath << " " << svm_model_path << " " << svm_model_bin_path << " "
              << autohit::color_id <<" " << autohit::high_light_mask_k << " " << autohit::proposal_bar_area_k << " "
              << autohit::bar_height_H << " " << autohit::bar_height_L << " " << autohit::bar_slim_k_H << " " << autohit::bar_slim_k_L << " "
              << autohit::bar_angle_limit << " " << autohit::small_pair_bar_angle_limit << " "
              << autohit::big_pair_bar_angle_limit << " "<<autohit::sp_dynamic << " " << std::endl;


    autohit::Model_cnt = 0;
    autohit::autohit_flag = 0;

    autohit::world_pts[0] = cv::Point2f(0, 0);
    autohit::world_pts[1] = cv::Point2f(40, 0);
    autohit::world_pts[2] = cv::Point2f(40, 40);
    autohit::world_pts[3] = cv::Point2f(0, 40);

    autohit::model = cv::ml::SVM::load( svm_model_path );
    autohit::model_bin = cv::ml::SVM::load( svm_model_bin_path );

    if ( autohit::model.empty())
         std::cerr << "modle file open failed..." << std::endl;
    if ( autohit::model_bin.empty())
         std::cerr << "model_bin file open failed..." << std::endl;
}

cv::Rect calcSafeRect(const cv::Rect &roi_rect, const cv::Mat &src)
{
    cv::Rect boudRect = roi_rect;

    // boudRect的左上的x和y有可能小于0
    int tl_x = boudRect.x > 0 ? boudRect.x : 0;
    int tl_y = boudRect.y > 0 ? boudRect.y : 0;

    tl_x = tl_x >= src.cols ? src.cols-1 : tl_x;
    tl_y = tl_y >= src.rows ? src.rows-1 : tl_y;

    // boudRect的右下的x和y有可能大于src的范围
    int br_x = boudRect.x + boudRect.width  < src.cols-1 ?
               boudRect.x + boudRect.width  - 1 : src.cols - 1;
    int br_y = boudRect.y + boudRect.height < src.rows-1 ?
               boudRect.y + boudRect.height - 1 : src.rows - 1;

    int roi_width  = br_x - tl_x;
    int roi_height = br_y - tl_y;
    if(roi_width<1)
        roi_width=0;
    if(roi_height<1)
        roi_height=0;

    cv::Rect  safeBoundRect = cv::Rect(tl_x, tl_y, roi_width, roi_height);

    return safeBoundRect;
}

void weighted_cvt_gray(const cv::Mat &img, cv::Mat & gray, int color_id)
{
    //BLUE
    if( color_id == 0 )
    {
        cv::Mat colors[3];
        cv::split(img,colors);
        gray = colors[0]*0.9+colors[1]*0.05+colors[2]*0.05;
    }
        //RED
    else
    {
        cv::Mat colors[3];
        cv::split(img,colors);
        gray = colors[0]*0.05+colors[1]*0.05+colors[2]*1.5;//BGR
    }
}

void get_hight_light_bar(const cv::Mat & img_input, cv::Mat & mask , int color_id)
{
    cv::Mat weighted_gray;

    if(  color_id == 0)
    {
        weighted_cvt_gray(img_input,weighted_gray,color_id);
        cv::threshold(weighted_gray,mask,150,255,cv::THRESH_BINARY);
    }
    else
    {
        weighted_cvt_gray(img_input,weighted_gray,color_id);
        cv::threshold(weighted_gray,mask,150,255,cv::THRESH_BINARY);

//        cv::Mat color_list[3];
//        cv::Mat r_mask,rb_mask,rg_mask;
//        cv::split(img_input,color_list);
//
//        cv::inRange(color_list[2],50,255,r_mask);
//
//        cv::subtract(color_list[2],color_list[0],rb_mask);
//        cv::inRange(rb_mask,20,255,rb_mask);
//
//        cv::subtract(color_list[2],color_list[1],rg_mask);
//        cv::inRange(rg_mask,20,255,rg_mask);
//
//        cv::bitwise_and(rb_mask,rg_mask,mask,r_mask);

//        cv::bitwise_or(mask,light_mask,mask);
//        weighted_cvt_gray(img_input,meng::weighted_gray,color_id);
//        cv::threshold(meng::weighted_gray,mask,150,255,cv::THRESH_BINARY);
    }
}

int bin_val = 17;
int autohit::predict( cv::Ptr<cv::ml::SVM> &model, cv::HOGDescriptor *pHog, cv::Mat &img_transform, int &model_cnt )
{
#ifdef HogBin

    cv::Mat trans_gray, trans_bin;
    cv::resize( img_transform, img_transform, cv::Size(30, 30), 0, 0, cv::INTER_NEAREST );
    cv::cvtColor( img_transform, trans_gray, cv::COLOR_BGR2GRAY );
    cv::threshold( trans_gray, trans_bin, bin_val, 255, cv::THRESH_BINARY );

    int pxl_cnt = 0, bias = img_transform.cols * 0.3, c_x = img_transform.cols * 0.5, c_y = img_transform.rows * 0.5;
    if ( color_id == 1 )
    {
        for ( int i = 0; i < img_transform.rows; i++ )
        {
              uchar *data = img_transform.ptr<uchar>(i);
              uchar *ptr_gray =  trans_gray.ptr<uchar>(i);
              uchar *ptr_equ  =  trans_bin.ptr<uchar>(i);

            for ( int j = 0; j < img_transform.cols; j++ )
            {
                  uchar b = data[ j * 3 ];
                  uchar g = data[ j * 3 + 1 ];
                  uchar r = data[ j * 3 + 2 ];

                if ( ( ( ( r - b ) > 50 ) && ( ( r > 100 ) || ( g > 100 ) ) ) || ( ptr_gray[j] > 100 ) )
                {
                    ptr_equ[j] = 0;
                    if ( ( i < c_y + bias ) && (i > c_y - bias ) && ( j < c_x + bias ) && ( j > c_x - bias ) )
                           pxl_cnt++;
                }
            }
        }
    }
    if(pxl_cnt>2){
        std::cout<<"中心点高亮"<<pxl_cnt<<std::endl;
        return 0;
    }

//    cv::namedWindow("b",0);
//    cv::namedWindow("g",0);
//    cv::imshow("b",t_bin);
//    cv::imshow("g",t_gray);
//    cv::waitKey(0);

    int r = 0;
    std::vector<float> descriptors;
    pHog_bin->compute( trans_bin, descriptors );
    cv::Mat dst( 1, int( descriptors.size() ), CV_32FC1, descriptors.data() );
    r = model_bin->predict( dst );

#if 0
    {
        static int save_num = 0;
        std::string roi_name = "/home/snow/imgs/"+std::to_string(r)+"/"+std::to_string(3600+save_num++) + ".jpg";
        cv::imwrite(roi_name, t_bin);
        //cv::waitKey(1);
        printf("save successfully\n");
    }
#endif

#else
    cv::Mat t_gray,t_equ ;
    cv::cvtColor(transform_img,t_gray,cv::COLOR_BGR2GRAY);

    int r=0;
    //hog特征分类
    std::vector<float> descriptors;
    cv::equalizeHist(t_gray,t_equ);

    //红色数码管干扰
    int pxl_cnt=0, bias= transform_img.cols*0.25, c_x = transform_img.cols*0.5, c_y = transform_img.rows*0.5;
    if(Color_id==1)
    {
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
                    //std::cout<<"val set to zero \n";
                    ptr_equ[j] = 0;

                    if((i<c_y+bias) && (i>c_y-bias) && (j<c_x+bias) && (j>c_x-bias))
                        pxl_cnt++;
                }
            }
        }
    }
    else if(Color_id==0) //Blue
    {
        for (int i = 0; i < transform_img.rows; i++){
            uchar *data = transform_img.ptr<uchar>(i);
            for (int j = 0; j < transform_img.cols; j++){
                uchar b = data[j*3];
                uchar g = data[j*3+1];
                uchar r = data[j*3+2];

                if (((b - r) > 50) && ((b > 100) || (g > 100)))
                    pxl_cnt++;
            }
        }
    }
    if(pxl_cnt>2){
        std::cout<<"中心点高亮"<<pxl_cnt<<std::endl;
        return 0;
    }

    hog->compute(t_equ, descriptors);
    cv::Mat dst(1, int(descriptors.size()), CV_32FC1, descriptors.data());
    r = model->predict(dst);

#endif

#if 1
    if ( r == 0 )
         return r;
#else
    if ( r != 1 )
         return 0;
#endif
    //===========
    //meng::Color_id为红时才需此操作
//    if (meng::Color_id==1){
//        int width  = bias;
//        int height = bias;
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
    //===========

    return r;
}

void autohit::transform_armor_vertex( cv::Point2f *pPts, float k )
{
    cv::Point2f new_pts[4];

    new_pts[0] = pPts[0];
    new_pts[1] = pPts[1];
    new_pts[2] = pPts[2];
    new_pts[3] = pPts[3];

    pPts[0] = new_pts[0] + ( new_pts[1] - new_pts[0] ) * k;
    pPts[1] = new_pts[0] + ( new_pts[1] - new_pts[0] ) * ( 1 - k );

    pPts[3] = new_pts[3] + ( new_pts[2] - new_pts[3] ) * k;
    pPts[2] = new_pts[3] + ( new_pts[2] - new_pts[3] ) * ( 1 - k );
}

float estimatePose( const std::vector<cv::Point2f> corners, cv::Mat &intrinsic_matrix, cv::Mat &distortion_coefficients, bool is_small_armor )
{
    std::vector<cv::Point3f> odjPoints3d;
    float height = 5.6, width;
    if ( is_small_armor )
         width = 13;
    else
         width  = 22.8;

    odjPoints3d.emplace_back( -width * 0.5, -height * 0.5, 0. );
    odjPoints3d.emplace_back( width * 0.5, -height * 0.5, 0. );
    odjPoints3d.emplace_back( width * 0.5,  height * 0.5, 0. );
    odjPoints3d.emplace_back( -width * 0.5,  height * 0.5, 0. );

    assert( ( odjPoints3d.size() == 4 ) && ( corners.size() == 4 ) );

    cv::Vec3d r, t;
    solvePnP( odjPoints3d, corners, intrinsic_matrix, distortion_coefficients, r, t, false, cv::SOLVEPNP_ITERATIVE );

    float distance = 0;
    if ( ( t[2] > 0 ) && ( t[2] < 1200 ) )
    {
           distance = sqrt( pow( t[0], 2 ) + pow( t[1], 2 ) + pow( t[2], 2 ) );
    }

    std::cout << "is_small_armor  " << is_small_armor << " estimatePose result ------------------> " << distance
//            <<" x "<<t[0]
//            <<" y "<<t[1]
//            <<" z "<<t[2]
              << std::endl;

    return distance;
}

static cv::Point2f detect_task( cv::Mat &img_src, int color_id, float &distance, int &is_small_armor_flag )
{
    static int roi_cnt = 0;
    cv::Point2f sp(-1, -1);
    cv::Mat img_backups, high_light_thre_mask;
    int Detected_flag = 0;

    //cv::resize(img_src,img_src,cv::Size(),0.5,0.5,cv::INTER_NEAREST);
    img_backups = img_src.clone();

    /********* Get hight_light_bar*********************/
    get_hight_light_bar( img_src, high_light_thre_mask, color_id );
    cv::dilate( high_light_thre_mask, high_light_thre_mask, autohit::element );

    std::vector<cv::Vec4i> hierarchy;
    std::vector< std::vector<cv::Point> > contours;
    cv::findContours( high_light_thre_mask, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE, cv::Point(0, 0) );
    if( contours.size() > 0 )
    {
        std::vector<struct s_bar> bar_list(contours.size());
        /********************1. Find hight light bar and put them in bar_list, mark them by setting level = 1
         * We have 3 limit condotions
         * (1) the ellipse hight must in range of(10 ,70)
         * (2) the angle accoding to horizon must more than (45) degree
         * (3) the hight/width must in range of(1.5,10)
         * (4) there must be some blue/red points in the contour (k=0.2)
         * and finally we get end points of the bar
         * ************************************/
        for ( int contour_i = 0; contour_i < contours.size(); contour_i++ )
        {
            if ( contours[contour_i].size() > 4 )
            {
                cv::RotatedRect min_ellipse = fitEllipse( contours[contour_i] );
                cv::RotatedRect min_r_rect = cv::minAreaRect( contours[contour_i] );

                float area_of_contour = cv::contourArea( contours[contour_i] );
                float area_persent = area_of_contour / min_r_rect.size.area();
                float ellipse_h_w_div = min_ellipse.size.height / min_ellipse.size.width;

                if ( area_persent < 0.66 )
                {
                     cv::putText( img_src, "aP", min_ellipse.center, cv::FONT_HERSHEY_COMPLEX, 0.5, cv::Scalar(0, 255, 255) );
                     continue;
                }
                if ( ( ellipse_h_w_div > autohit::bar_slim_k_H ) || ( ellipse_h_w_div < autohit::bar_slim_k_L ) )
                {
                       cv::putText( img_src, "wh", min_ellipse.center, cv::FONT_HERSHEY_COMPLEX, 0.5, cv::Scalar(0, 255, 255) );
                       continue;
                }
                if ( ( min_ellipse.angle > autohit::bar_angle_limit ) && ( min_ellipse.angle < ( 180 - autohit::bar_angle_limit ) ) )
                {
                       cv::putText( img_src, "al", min_ellipse.center, cv::FONT_HERSHEY_COMPLEX, 0.5, cv::Scalar(0, 255, 255) );
                       continue;
                }
                if ( ( min_ellipse.size.height > autohit::bar_height_H ) || ( min_ellipse.size.height < autohit::bar_height_L ) )
                {
                       cv::putText( img_src, "hl", min_ellipse.center, cv::FONT_HERSHEY_COMPLEX, 0.5, cv::Scalar(0, 255, 255) );
                       continue;
                }

                cv::Rect min_b_box = min_ellipse.boundingRect();
                min_b_box = calcSafeRect( min_b_box, img_backups );

                cv::Mat bgr_roi = img_backups( min_b_box );
                cv::Mat hsv_roi, bbox_color_mask;
                cv::cvtColor( bgr_roi, hsv_roi, cv::COLOR_BGR2HSV );

                if ( color_id == 0 )
                {
                     cv::inRange(hsv_roi, cv::Scalar(80, 180, 190), cv::Scalar(255, 255, 255), bbox_color_mask );
                }
                else
                {
                    cv::Mat hsv1, hsv2;
                    cv::inRange( hsv_roi, cv::Scalar(0, 175, 70), cv::Scalar(12, 255, 255), hsv1 );
                    cv::inRange( hsv_roi, cv::Scalar(170, 0, 70), cv::Scalar(180, 255, 255), hsv2 );
                    bbox_color_mask = hsv1 + hsv2;
                }

                if ( !bbox_color_mask.empty() )
                {
                    int correct_pxl = 0;
                    for( int i = 0; i < bbox_color_mask.rows; i++ )
                    {
                         uchar *hsv_ptr = bbox_color_mask.ptr<uchar>(i);
                         uchar *bgr_ptr = bgr_roi.ptr<uchar>(i);
                         for ( int j = 0; j < bbox_color_mask.cols; j++ )
                         {
                               uchar hsv_val = hsv_ptr[j];

                               uchar b = bgr_ptr[ j * 3 ];
                               uchar g = bgr_ptr[ j * 3 + 1 ];
                               uchar r = bgr_ptr[ j * 3 + 2 ];

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
                         cv::putText( img_src, "pxl", min_ellipse.center, cv::FONT_HERSHEY_COMPLEX, 0.5, cv::Scalar(0, 255, 255) );
                         continue;
                    }

                    bar_list[contour_i].ellipse = min_ellipse;
                    bar_list[contour_i].b_rec = min_b_box;
                    bar_list[contour_i].level = 1;


                    float angle_rad = ( 90.0 - bar_list[contour_i].ellipse.angle ) / 180.0 * CV_PI;
                    float h = bar_list[contour_i].ellipse.size.height;

                    find_newpoint( bar_list[contour_i].ellipse.center, angle_rad, 0.5 * h,
                                   bar_list[contour_i].endpoints[0], img_backups.rows, img_backups.cols);
                    find_newpoint( bar_list[contour_i].ellipse.center, angle_rad, -0.5 * h,
                                   bar_list[contour_i].endpoints[1], img_backups.rows, img_backups.cols);

                    check_endpoint( bar_list[contour_i].endpoints[0], bar_list[contour_i].endpoints[1]);
                }
            }
        }

        /********************2. Match hight light bar pairs************************************/

        std::vector<struct s_bar> valid_bar_list, temp_valid_bar_list;
        for ( int i = 0; i < bar_list.size(); i++ )
        {
              if ( bar_list[i].level == 1 )
                   temp_valid_bar_list.push_back( bar_list[i] );
        }
        if ( temp_valid_bar_list.size() > 0 )
             sort_ellipse( temp_valid_bar_list, valid_bar_list, autohit::sp_dynamic.x, autohit::sp_dynamic.y );

        cv::circle( img_src, autohit::sp_dynamic, 10, cv::Scalar(255, 255, 255), -1 );
        cv::putText( img_src, std::to_string( valid_bar_list.size() ), cv::Point(20, 30), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255, 255, 255), 1 );
        for ( int i = 0; i < valid_bar_list.size(); i++ )
              cv::line( img_src, valid_bar_list[i].endpoints[0], valid_bar_list[i].endpoints[1], cv::Scalar(0, 255, 0), 2 );


        if ( valid_bar_list.size() > 1 )
        {
             get_surround_pts( valid_bar_list, img_backups );

             int result = -1;
             bool is_small_armor;
             result = find_armor_int_all_pts( img_src, valid_bar_list, img_backups, is_small_armor );

            if ( result != -1 )
            {
                 std::cout << "------------------------------------------ " << is_small_armor << std::endl;
                 sp = getCrossPoint( valid_bar_list[result].armor_vertex[0],
                                     valid_bar_list[result].armor_vertex[2],
                                     valid_bar_list[result].armor_vertex[1],
                                     valid_bar_list[result].armor_vertex[3] );

                 std::vector<cv::Point2f> corners(4);
                 std::memcpy( corners.data(), valid_bar_list[result].pose_points, 4 * 8 );

                 cv::putText( img_src, "match", cv::Point(10, 50), cv::FONT_HERSHEY_COMPLEX, 1, cv::Scalar(0, 255, 255) );
                 for ( int i = 0; i < 4; i++ )
                       cv::line( img_src, corners[i], corners[ ( i + 1 ) % 4 ], cv::Scalar(0, 0, 255), 1 );

                 cv::circle( img_src, corners[0], 2, cv::Scalar(0, 255, 0), -1 );
                 cv::circle( img_src, corners[1], 2, cv::Scalar(0, 255, 255), -1 );
                 cv::circle( img_src, corners[2], 2, cv::Scalar(0, 0, 255), -1 );
                 cv::circle( img_src, corners[3], 2, cv::Scalar(255, 255, 255), -1 );

                 cv::Mat instri_matrix = cv::Mat( 3, 3, CV_32F, dataf ).clone();
                 cv::Mat distortion_coefficients = cv::Mat( 5, 1, CV_32F, dis );
                 distance = estimatePose( corners, instri_matrix, distortion_coefficients, is_small_armor );

                 if ( distance == 0 )          is_small_armor = 0; //不可信
                 else if ( is_small_armor )    is_small_armor = 1; //小装甲
                 else if ( !is_small_armor )   is_small_armor = 2; //大装甲

                 cv::putText( img_src, std::to_string( int( round(distance ) ) ), valid_bar_list[result].endpoints[0], cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 255, 255) );
            }
            else
            {
                //cv::putText(img_src,"single",cv::Point(10,50),cv::FONT_HERSHEY_SIMPLEX,1,cv::Scalar(255,255,255),1);
                //find_armor_for_single_bar(img_src, valid_bar_list,img_ori, sp);
            }
        }
        else
        {
            //单根等条不做寻找和匹配
        }

        if ( sp.x >= 0 )
             cv::circle( img_src, sp, 10, cv::Scalar(0, 255, 255), 2 );
    }

#ifdef SHOW
    if ( sp.x != -1 )
    {
         int key = cv::waitKey(3)&0xff;

         cv::Mat g, e, b;
         cv::cvtColor( autohit::img_transform, g, cv::COLOR_BGR2GRAY );
         cv::threshold( g, b, bin_val, 255, cv::THRESH_BINARY );
         cv::imshow( "b", b );

         if ( key == 13 )//Enter
         {
              cv::imwrite( "/home/snow/"+std::to_string(57+roi_cnt++)+".jpg", e );
         }
    }
    cv::namedWindow( "img_src", 0 );
    imshow( "img_src", img_src );
    cv::waitKey(1);
#endif

    return sp;
}

cv::Point2f autohit::detection( cv::Mat &img, float& dist, int &is_small_armor )
{
    cv::Point2f sp(-1, -1);
    if ( autohit::autohit_flag != 1 )
    {
         detect_init();
         autohit::autohit_flag = 1;
         std::cout << "init=================" << std::endl;
    }
    else
    {
        sp = detect_task( img, autohit::color_id,dist, is_small_armor );
        sp = sp * 2;
    }
    return sp;
}
