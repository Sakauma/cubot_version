#include "armor/armor_main.h"
#include "armor/coordinante_process.h"

void find_newpoint( cv::Point2f start, float angle_rad, float distance, cv::Point2f & pt, int rows, int cols )
{
    pt.x = start.x + distance * cos( angle_rad );
    if ( pt.x >= cols )
    {
         pt.x = cols-1;
    }
    if ( pt.x < 0 )
    {
         pt.x = 0;
    }

    pt.y = start.y - distance * sin( angle_rad );
    if ( pt.y >= rows )
    {
         pt.y = rows-1;
    }
    if ( pt.y < 0 )
    {
         pt.y = 0;
    }
}

void check_endpoint( cv::Point2f & e1, cv::Point2f & e2 )
{
    if ( e1.y > e2.y )
    {
         cv::Point2f temp;
         temp.x = e1.x;
         temp.y = e1.y;

         e1.x = e2.x;
         e1.y = e2.y;

         e2.x = temp.x;
         e2.y = temp.y;
    }
}

int check_rect( const cv::Rect &r, int rows, int cols )
{
    cv::Rect big_rect( 0, 0, cols, rows );

    cv::Rect and_rect;

    and_rect = big_rect & r;

    if ( and_rect.area() < 1 )
         return 0;

    return 1;
}

int  limit_rect(cv::Rect &rect_src, cv::Rect & rect_dst, int rows, int cols )
{
    if ( check_rect( rect_src, rows, cols ) == 0 )
         return 0;
    else
    {
        int x1, y1, x2, y2;

        x1 = limit_number( rect_src.x, cols - 1, 0 );
        y1 = limit_number( rect_src.y, rows - 1, 0 );
        x2 = limit_number( rect_src.x + rect_src.width, cols - 1, 0 );
        y2 = limit_number( rect_src.y + rect_src.height, rows - 1, 0 );

        if ( ( y2 <= y1 ) || ( x2 <= x1 ) )
        {
            return 0;
        }

        rect_dst.x = x1;
        rect_dst.y = y1;
        rect_dst.width = x2 - x1;
        rect_dst.height = y2 - y1;

        if ( ( rect_dst.width < 0 ) || ( rect_dst.height < 0 ) )
            return 0;
    }

    return 1;
}

float check_mask_area( cv::Mat &img_bgr, cv::Rect &roi )
{
    cv::Mat bgr_mask, color_bin_mask;
    bgr_mask = img_bgr( roi );

    float area = 0;
    if ( bgr_mask.channels() == 1 )
    {
        area = cv::sum( bgr_mask )[0] / 255.;
    }
    else if ( bgr_mask.channels() == 3 )
    {
        if ( autohit::color_id == 0 )
        {
            cv::inRange( bgr_mask, cv::Scalar(100, 100, 100), cv::Scalar(124, 255, 255), color_bin_mask );
        }
        else if ( autohit::color_id == 1 )
        {
            cv::Mat hsv1, hsv2;
            cv::inRange( bgr_mask, cv::Scalar(0, 70, 70), cv::Scalar(10, 200, 255), hsv1 );
            cv::inRange( bgr_mask, cv::Scalar(170, 70, 70), cv::Scalar(180, 200, 255), hsv2 );
            color_bin_mask = hsv1 + hsv2;
        }

        area = cv::sum( color_bin_mask )[0] / 255.;
    }

    return area;
}

float limit_number( float a, float hight, float low )
{
    if ( a > hight )
    {
         return hight;
    }

    if ( a < low )
    {
         return low;
    }
    return a;
}


float cal_distance_pt( cv::Point2f a, cv::Point2f b )
{
    return sqrt( ( a.x - b.x ) * ( a.x - b.x ) + ( a.y - b.y ) * ( a.y - b.y ) );
}

void get_armor_from_pts( cv::Point2f e1, cv::Point2f e2, cv::Point2f p1, cv::Point2f p2, cv::Point2f *pArmor, int rows, int cols )
{
    cv::Point2f temp[4];

    if ( e1.x < p1.x )
    {
        temp[0] = e1;
        temp[1] = p1;
        temp[2] = p2;
        temp[3] = e2;
    }
    else
    {
        temp[0] = p1;
        temp[1] = e1;
        temp[2] = e2;
        temp[3] = p2;
    }

    float dx, dy;
    float k = 0.5;

    dx = temp[0].x - temp[3].x;
    dy = temp[0].y - temp[3].y;
    pArmor[0].x = temp[0].x + k * dx;
    pArmor[0].y = temp[0].y + k * dy;
    pArmor[3].x = temp[3].x - k * dx;
    pArmor[3].y = temp[3].y - k * dy;

    dx = temp[1].x - temp[2].x;
    dy = temp[1].y - temp[2].y;
    pArmor[1].x = temp[1].x + k * dx;
    pArmor[1].y = temp[1].y + k * dy;
    pArmor[2].x = temp[2].x - k * dx;
    pArmor[2].y = temp[2].y - k * dy;


    pArmor[0].x = limit_number( pArmor[0].x, cols - 1, 0 );
    pArmor[1].x = limit_number( pArmor[1].x, cols - 1, 0 );
    pArmor[2].x = limit_number( pArmor[2].x, cols - 1, 0 );
    pArmor[3].x = limit_number( pArmor[3].x, cols - 1, 0 );

    pArmor[0].y=limit_number( pArmor[0].y, rows - 1, 0 );
    pArmor[1].y=limit_number( pArmor[1].y, rows - 1, 0 );
    pArmor[2].y=limit_number( pArmor[2].y, rows - 1, 0 );
    pArmor[3].y=limit_number( pArmor[3].y, rows - 1, 0 );
}

template < typename T>
std::vector< size_t>  sort_indexes( const std::vector< T>& v, int descend_or_ascend )
{
    std::vector< size_t>  idx( v.size() );
    for ( size_t i = 0; i != idx.size(); ++i ) idx[i] = i;

    if ( descend_or_ascend == 0 )
    {
        sort( idx.begin(), idx.end(),
              [& v]( size_t i1, size_t i2 ) {return v[i1] >  v[i2];} );
    }
    else
    {
        sort( idx.begin(), idx.end(),
              [& v]( size_t i1, size_t i2 ) {return v[i1] <  v[i2];} );
    }
    return idx;
}

void sort_ellipse( const std::vector<struct s_bar> &bars_src, std::vector<struct s_bar> &bars_dst, int x, int y )
{
    if ( bars_src.size() == 1 )
    {
        bars_dst.push_back( bars_src[0] );
        return ;
    }
    else
    {
        std::vector<float> heghts;
        std::vector<float> distance;
        std::vector<size_t > new_indexs;

        for ( int i = 0; i < bars_src.size(); i++ )
        {
            distance.push_back( pow( ( ori_bars[i].ellipse.center.x - x ), 2 ) + pow( ( ori_bars[i].ellipse.center.y - y ), 2 ) );
        }

        new_indexs = sort_indexes( distance, 1 );

        for ( int i = 0; i < new_indexs.size(); i++ )
        {
            bars_dst.push_back( bars_src[new_indexs[i]] );
        }
        return ;

    }
}

bool compare_bar_by_height( struct s_bar bar_0, struct s_bar bar_1 )
{
    return bar_0.ellipse.size.height > bar_1.ellipse.size.height;
}

void sort_bar_according_to_height( const std::vector<struct s_bar> &bars_src, std::vector<struct s_bar> &bars_dst )
{
#if 1
    //==========Snow============
    if ( bars_src.size() == 1 )
         bars_dst = bars_src;
    else
    {
        std::vector<struct s_bar> bars_temp = bars_src;
        std::sort( bars_temp.begin(), bars_temp.end(), compare_bar_by_height );
        bars_dst = bars_temp;
    }
#else
    //==========Meng============
    if(ori_bars.size()==1){
        dst_bars.push_back(ori_bars[0]);
    }else{
        std::vector<float> heghts;
        std::vector<size_t> new_indexs;

        for(int i=0;i<ori_bars.size();i++)
            heghts.push_back(ori_bars[i].ellipse.size.height);

        new_indexs = sort_indexes(heghts,0);
        for(int i=0;i<new_indexs.size();i++)
            dst_bars.push_back(ori_bars[new_indexs[i]]);
    }
#endif
}

double calDistance( const std::vector<cv::Point2f> &points, cv::Mat &intrinsic_mat )
{
    double   h = 0.105;//单位米
    double ux0 = ( points[0].x - intrinsic_mat.at<float>(0, 2) ) / intrinsic_mat.at<float>(0, 0);
    double uy0 = ( points[0].y - intrinsic_mat.at<float>(1, 2) ) / intrinsic_mat.at<float>(1, 1);
    double ux1 = ( points[1].x - intrinsic_mat.at<float>(0, 2) ) / intrinsic_mat.at<float>(0, 0);
    double uy1 = ( points[1].y - intrinsic_mat.at<float>(1, 2) ) / intrinsic_mat.at<float>(1, 1);

    double uh1 = std::sqrt( std::pow( ( ux1 - ux0 ), 2 ) + std::pow( ( uy1 - uy0 ), 2 ) );
    std::cout << "uh1 : " << ux0 << std::endl;
    double z=h/uh1;
    return z;
}

