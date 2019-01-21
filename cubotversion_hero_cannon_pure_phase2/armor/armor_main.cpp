#include <iostream>
#include <opencv2/opencv.hpp>

#include "armor/coordinante_process.h"
#include "armor/bar_points.h"
#include "armor/armor_main.h"

namespace autohit
{
    int   color_id;
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

    std::vector<cv::Point2f> world_pts(4);
    std::vector<cv::Point2f> new_pts(4);

    cv::Ptr<cv::ml::SVM> model;
    cv::HOGDescriptor *pHog = new cv::HOGDescriptor( cv::Size(40, 40), cv::Size(16, 16), cv::Size(8, 8), cv::Size(8, 8), 9 );

    cv::Mat element = cv::getStructuringElement( cv::MORPH_CROSS, cv::Size( 2 * 1 + 1, 2 * 1 + 1 ), cv::Point( 1, 1 ) );
}

static void detect_init( void )
{
    //========get parameters from file==========
    cv::FileStorage file_2("../cfg/AutoHitCfg.yml", cv::FileStorage::READ);
    if ( file_2.isOpened() )
    {
        int isSave = ( int )file_2["isSave"];
        int saveStartNum  = ( int )file_2["saveStartNum"];
        std::string img_savePath = ( std::string )file_2["img_savePath"];

        std::string svm_model_path  = (std::string)file_2["svm_model_path"];

        autohit::color_id = ( int )file_2["color2Strike"];
        autohit::high_light_mask_k = ( float )file_2["high_light_mask_k"];
        autohit::proposal_bar_area_k = ( float )file_2["proposal_bar_area_k"];

        autohit::bar_height_H = ( float )file_2["bar_height_H"];
        autohit::bar_height_L = ( float )file_2["bar_height_L"];
        autohit::bar_slim_k_H = ( float )file_2["bar_slim_k_H"];
        autohit::bar_slim_k_L = ( float )file_2["bar_slim_k_L"];

        autohit::bar_angle_limit = (float)file_2["bar_angle_limit"];
        autohit::small_pair_bar_angle_limit = ( float )file_2["small_pair_bar_angle_limit"];
        autohit::big_pair_bar_angle_limit = ( float )file_2["big_pair_bar_angle_limit"];
        autohit::sp_dynamic.x = ( float )file_2["sp_dynamic_x"];
        autohit::sp_dynamic.y = ( float )file_2["sp_dynamic_y"];
        file_2.release();

        std::cout << "try to read the configurate " << std::endl;
        std::cout << isSave << " " << saveStartNum << " " << img_savePath << " " << svm_model_path << " "
                  << autohit::color_id <<" " << autohit::high_light_mask_k << " " << autohit::proposal_bar_area_k << " "
                  << autohit::bar_height_H << " " << autohit::bar_height_L << " " << autohit::bar_slim_k_H << " " << autohit::bar_slim_k_L << " "
                  << autohit::bar_angle_limit << " " << autohit::small_pair_bar_angle_limit << " "
                  << autohit::big_pair_bar_angle_limit << " " << autohit::sp_dynamic << " " << std::endl;

        autohit::Model_cnt = 0;
        autohit::autohit_flag = 0;

        autohit::world_pts[0] = cv::Point2f(0, 0);
        autohit::world_pts[1] = cv::Point2f(40, 0);
        autohit::world_pts[2] = cv::Point2f(40, 40);
        autohit::world_pts[3] = cv::Point2f(0, 40);

        autohit::model = cv::ml::SVM::load( svm_model_path );

        if ( autohit::model.empty() )
            std::cerr << "model open failed" << std::endl;
    }
    else
    {
        std::cerr << "AutoHit cfg file read failed" << std::endl;
        return ;
    }
}

void weighted_cvt_gray( const cv::Mat &img_src, cv::Mat &img_gray, int color_id )
{
    if ( color_id == 0 )
    {
         cv::Mat colors[3];
         cv::split( img_src, colors );
         img_gray = colors[0] * 0.9 + colors[1] * 0.05 + colors[2] * 0.05;

        std::cout<<"color to strike is blue  ";
    }
    else
    {
        cv::Mat colors[3];
        cv::split( img_src, colors );
        img_gray = colors[0] * 0.05 + colors[1] * 0.05 + colors[2] * 1.5;
        std::cout<<"color to strike is red   ";
    }
}

void get_hight_light_bar( const cv::Mat & img_src, cv::Mat & mask, int color_id )
{
    cv::Mat img_weighted_gray;

    if( color_id == 0 )
    {
        weighted_cvt_gray( img_src, img_weighted_gray, color_id );
        cv::threshold( img_weighted_gray, mask, 150, 255, cv::THRESH_BINARY );
    }
    else
    {
        //Red
        weighted_cvt_gray( img_src, img_weighted_gray, color_id );
        cv::threshold( img_weighted_gray, mask, 150, 255, cv::THRESH_BINARY );
    }
}

#if 1
int autohit::predict( cv::Ptr<cv::ml::SVM> &model, cv::HOGDescriptor *pHog, cv::Mat &img_transform, int &model_cnt )
{
    cv::Mat trans_gray, trans_equ;
    cv::cvtColor( img_transform, trans_gray, cv::COLOR_BGR2GRAY );
    cv::equalizeHist( trans_gray, trans_equ );

    int r=0;
    std::vector<float> descriptors;

    pHog->compute( trans_equ, descriptors );
    cv::Mat dst( 1, int( descriptors.size() ), CV_32FC1, descriptors.data() );

    r = model->predict(dst);
    model_cnt++;

    if ( r == 0 )
        return r;

    cv::Point2f p(20, 20);

    int c_x, c_y;
    c_x = p.x;
    c_y = p.y;


    cv::Rect high_light_roi_rect( c_x - 5, c_y - 5, 2 * 5, 2 * 5 );
    limit_rect( high_light_roi_rect, high_light_roi_rect, img_transform.rows, img_transform.cols );
    cv::Mat img_high_light_roi = img_transform(high_light_roi_rect);

    int pxl_cnt = 0;
    if ( color_id == 0 )
    {
        for ( int i = 0; i < img_high_light_roi.rows; i++)
        {
            uchar *data = img_high_light_roi.ptr<uchar>(i);
            for ( int j = 0; j < img_high_light_roi.cols; j++ )
            {
                uchar b = data[ j * 3 ];
                uchar g = data[ j * 3 + 1 ];
                uchar r = data[ j * 3 + 2 ];

                if ( ( ( b - r ) > 50) && ( ( b > 100 ) || ( g > 100 ) ) ) //暂时将绿色通道的约束加进来，但是之后会使用录色发光弹，待处理
                    pxl_cnt++;
            }
        }
    }
    else if ( color_id==1 )
    {
        for ( int i = 0; i < img_high_light_roi.rows; i++ )
        {
            uchar *data = img_high_light_roi.ptr<uchar>(i);
            for ( int j = 0; j < img_high_light_roi.cols; j++ )
            {
                uchar b = data[ j * 3 ];
                uchar g = data[ j * 3 + 1 ];
                uchar r = data[ j * 3 + 2 ];
                if ( ( ( r - b ) > 50 ) && ( ( r > 100) || ( g > 100 ) ) )//同上
                    pxl_cnt++;
            }
        }
    }
    if ( pxl_cnt > 2 )
    {
        std::cout << "中心点高亮, 异常像素值 " << pxl_cnt << std::endl;
        r = 0;
    }

    if ( r == 0 )
        return r;

    if ( ( r != 0 ) && ( autohit::color_id == 1 ) )
    {
        int width  = 5;
        int height = 10;

        cv::Mat color3[3];
        cv::Mat diff[3];
        cv::Mat diff_mask[4];

        cv::split( img_transform, color3 );
        cv::absdiff( color3[0], color3[1], diff[0] );
        cv::absdiff( color3[1], color3[2], diff[1] );
        cv::absdiff( color3[2], color3[3], diff[2] );

        cv::inRange( diff[0], 80, 255, diff_mask[0] );
        cv::inRange( diff[1], 80, 255, diff_mask[1] );
        cv::inRange( diff[2], 80, 255, diff_mask[2] );

        cv::bitwise_or( diff_mask[0], diff_mask[1], diff_mask[3] );
        cv::bitwise_or( diff_mask[2], diff_mask[3], diff_mask[3] );

        cv::Rect t_r( c_x - width, c_y - height, 2 * width, 2 * height );
        limit_rect( t_r, t_r, img_transform.rows, img_transform.cols );


        float per = check_mask_area(diff_mask[3], t_r)/t_r.area();
        if (per>0.065)
        {
            r = 0;
            std::cout<<"数码管干扰  "<<per<<std::endl;
        }
    }
#endif
    //===========
#if 0
    cv::namedWindow("t_equ",0);
    cv::imshow("t_equ",transform_img);
    cv::waitKey(1);

    if (r==0){
        int k = cv::waitKey(1);
        //if(k=='m')
        if(1)
        {
            static int save_num = 0;
            std::string roi_name = "/home/snow/imgs/train/neg/" + std::to_string(1378+save_num++) + ".jpg";
            cv::imwrite(roi_name, t_equ);
            cv::waitKey(1);
            printf("save successfully\n");
        }
    }else{
        int k = cv::waitKey(1);
        //if(k=='m')
        if(1)
        {
            static int save_num1 = 0;
            std::string roi_name = "/home/snow/imgs/train/pos/" + std::to_string(80+save_num1) + ".jpg";
            cv::imwrite(roi_name, t_equ);
            cv::waitKey(1);
            printf("save successfully, %d\n",save_num1);
            save_num1++;
        }
    }
#endif

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

static cv::Point2f detect_task( cv::Mat &img_src, int color_id )
{
    cv::Point2f sp(-1, -1);

    cv::resize( img_src, img_src, cv::Size(), 0.5, 0.5 );
    cv::Mat img_backups = img_src.clone();

    /********* Get hight_light_bar*********************/
    cv::Mat high_light_thre_mask;
    get_hight_light_bar( img_src, high_light_thre_mask, color_id );
    cv::dilate( high_light_thre_mask, high_light_thre_mask, autohit::element );

    std::vector<cv::Vec4i> hierarchy;
    std::vector< std::vector<cv::Point> > contours;
    cv::findContours( high_light_thre_mask, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE, cv::Point(0, 0) );
    if ( contours.size() > 0 )
    {
        std::vector<struct s_bar> bar_list(contours.size());

#ifdef OpenTBB
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
        for ( int contour_i = 0; contour_i < contours.size(); contour_i++ )
        {
            cv::RotatedRect min_ellipse;
            cv::Rect min_b_box;
            cv::Mat min_b_box_roi;
            cv::RotatedRect min_r_Rect;

            if ( contours[contour_i].size() > 4 )
            {
                min_r_Rect = cv::minAreaRect( contours[contour_i] );
                min_ellipse = fitEllipse( contours[contour_i] );
                min_b_box = min_ellipse.boundingRect();

                float area_of_contour = cv::contourArea(contours[contour_i]);
                float ellipse_h_w_div = min_ellipse.size.height / min_ellipse.size.width;
                float area_persent = area_of_contour / min_r_Rect.size.area();


                if ( area_persent < 0.60 )
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

                min_b_box = min_b_box + cv::Point(-3, -3) + cv::Size(6, 6);
                if ( limit_rect( min_b_box, min_b_box, img_backups.rows, img_backups.cols ) == 0 )
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
                for ( int i = 0; i < color_mask_roi.rows;i++ )
                {
                    uchar *hsv_ptr = color_mask_roi.ptr<uchar>(i);
                    uchar *bgr_ptr = bgr_roi.ptr<uchar>(i);
                    for ( int j = 0; j < color_mask_roi.cols; j++ )
                    {
                        uchar hsv_val = hsv_ptr[j];
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
                    cv::putText( img_src, "pxl", min_ellipse.center, cv::FONT_HERSHEY_COMPLEX, 0.5, cv::Scalar(0, 255, 255) );
                    continue;
                }

#ifdef Show
                cv::rectangle(img_src,min_b_box,cv::Scalar(0,255,255), 1);
#endif
                cv::Mat fill_roi;

                bar_list[contour_i].ellipse = min_ellipse;
                bar_list[contour_i].b_rec = min_b_box;
                bar_list[contour_i].level = 1;

                float angle_rad = ( 90.0 - bar_list[contour_i].ellipse.angle ) / 180.0 * CV_PI;
                float h = bar_list[contour_i].ellipse.size.height;

                find_newpoint( bar_list[contour_i].ellipse.center, angle_rad, 0.5 * h,
                               bar_list[contour_i].endpoints[0], img_backups.rows, img_backups.cols );
                find_newpoint( bar_list[contour_i].ellipse.center, angle_rad, -0.5 * h,
                               bar_list[contour_i].endpoints[1], img_backups.rows, img_backups.cols );
                check_endpoint( bar_list[contour_i].endpoints[0], bar_list[contour_i].endpoints[1] );

#ifdef Show
                //cv::putText(img_src,"lgt",minEllipse.center,cv::FONT_HERSHEY_COMPLEX,0.5,cv::Scalar(0,0,255));
#endif
            }
        }
#endif

        /********************2. Match hight light bar pairs************************************/
        /**sort bars according to their distance to a point***/
        std::vector<struct s_bar> valid_bar_list, temp_valid_bar_list;
        for ( int i = 0; i<bar_list.size(); i++)
        {
            if (bar_list[i].level == 1)
                temp_valid_bar_list.push_back(bar_list[i]);
        }

        if ( temp_valid_bar_list.size()>0 )
            sort_ellipse( temp_valid_bar_list, valid_bar_list, autohit::sp_dynamic.x, autohit::sp_dynamic.y );

#ifdef Show
        for(int i=0;i<valid_bar_list.size();i++)
            cv::line(img_src,valid_bar_list[i].endpoints[0],valid_bar_list[i].endpoints[1],cv::Scalar(0,255,0),2);
#endif
        if ( valid_bar_list.size() > 1 )
        {
            int result=-1;
            get_surround_pts( valid_bar_list, img_backups );
#ifdef Show
            cv::line(img_src,valid_bar_list[0].ex_large_armor.horizon_r[0],valid_bar_list[0].ex_large_armor.horizon_r[1],cvScalar(255,0,0),2,8);
            cv::line(img_src,valid_bar_list[0].ex_large_armor.horizon_l[0],valid_bar_list[0].ex_large_armor.horizon_l[1],cvScalar(255,0,0),2,8);
            cv::line(img_src,valid_bar_list[1].ex_large_armor.horizon_r[0],valid_bar_list[1].ex_large_armor.horizon_r[1],cvScalar(255,0,0),2,8);
            cv::line(img_src,valid_bar_list[1].ex_large_armor.horizon_l[0],valid_bar_list[1].ex_large_armor.horizon_l[1],cvScalar(255,0,0),2,8);
#endif
            result=find_armor_int_all_pts( img_src, valid_bar_list, img_backups );

            /*When we have armor detection result,we need to make sure again
             * if there is blue or red points in the center roi of armor, this result is no valid
             * */
            if ( result != -1 )
            {
                    cv::Point2f p = getCrossPoint( valid_bar_list[result].armor_vertex[0],
                                                   valid_bar_list[result].armor_vertex[2],
                                                   valid_bar_list[result].armor_vertex[1],
                                                   valid_bar_list[result].armor_vertex[3] );


//                for(int i=0;i<4;i++){
//                    cv::line(img_src,valid_bar_list[result].armor_vertex[i],valid_bar_list[result].armor_vertex[(i+1)%4],
//                             cv::Scalar(0,255,255),1);
//
//                    if(valid_bar_list[result].isPosePoints)
//                        cv::line(img_src,valid_bar_list[result].pose_points[i],valid_bar_list[result].pose_points[(i+1)%4],
//                                 cv::Scalar(0,255,0),1);
//                }

                if (p.x != -1)
                    sp = p;

                /*
                 * For guard robot, we need calculate distance using PNP algorithm,
                 * Directed by XueLiang
                 * */
                if (sp.x != -1) {

                }
            }
        }

        if ( ( valid_bar_list.size() == 1) ||( sp.x == -1 ) )
        {
            int result = 0;
            result = find_armor_for_single_bar( img_src, valid_bar_list, img_backups, sp );
        }
    }
    return sp;
}

cv::Point2f autohit::detection( cv::Mat &img, float & distance )
{
    static cv::Point2f sp;
    if (autohit::autohit_flag != 1)
    {
        detect_init();
        autohit::autohit_flag = 1;
        std::cout << "init=================" << std::endl;
    }
    else
    {
        double t = ( double )cv::getTickCount();
        sp = detect_task( img, autohit::color_id );
        sp = sp * 2;
        std::cout<<"sp"<<sp<<std::endl;
        t = ( double )cvGetTickCount() - t;
        std::cout << "run time = %gms" << t/(cv::getTickFrequency()*1000) << std::endl;
    }
    return sp;
}
