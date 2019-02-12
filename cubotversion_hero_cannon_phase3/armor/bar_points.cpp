#include "armor/bar_points.h"
#include "armor/armor_main.h"

cv::Point2f getCrossPoint( cv::Point2f &p00, cv::Point2f &p01, cv::Point2f &p10, cv::Point2f &p11 )
{
    if ( ( p00.x == p01.x ) || ( p00.x == p01.x ) )
    {
        return cv::Point2f(-1, -1);
    }

    float k0 = ( p00.y - p01.y ) / ( p00.x - p01.x );
    float k1 = ( p10.y - p11.y ) / ( p10.x - p11.x );

    if( k0 == k1 )
        return cv::Point2f(-1, -1);

    cv::Point2f crossPoint;
    crossPoint.x = ( p10.y - p00.y + k0 * p00.x - k1 * p10.x ) / ( k0 - k1 );
    crossPoint.y = k0 * ( crossPoint.x - p00.x ) + p00.y;

    return crossPoint;
}

/*expand big or small points*/
void get_expand_pts( cv::Point2f *pEndpoints, struct expand_pt *pEx_pts, float sup_angle, float angle,
                         float distance1, float distance2, int rows, int cols )
{
    find_newpoint( pEndpoints[0], sup_angle, -distance1, ( *pEx_pts ).parallel_l[0], rows,cols );
    find_newpoint( pEndpoints[1], sup_angle, -distance1, ( *pEx_pts ).parallel_l[1], rows,cols );
    check_endpoint( ( *pEx_pts ).parallel_l[0], ( *pEx_pts ).parallel_l[1] );

    find_newpoint( pEndpoints[0], sup_angle, distance1, ( *pEx_pts ).parallel_r[0], rows, cols );
    find_newpoint( pEndpoints[1], sup_angle, distance1, ( *pEx_pts ).parallel_r[1], rows, cols );
    check_endpoint( ( *pEx_pts ).parallel_r[0], ( *pEx_pts ).parallel_r[1] );

    find_newpoint( pEndpoints[0], angle, -distance2, ( *pEx_pts ).horizon_l[0], rows, cols );
    find_newpoint( pEndpoints[1], angle, -distance2, ( *pEx_pts ).horizon_l[1], rows, cols );
    check_endpoint( ( *pEx_pts ).horizon_l[0], ( *pEx_pts ).horizon_l[1] );

    find_newpoint( pEndpoints[0], angle, distance2, ( *pEx_pts ).horizon_r[0], rows, cols );
    find_newpoint( pEndpoints[1], angle, distance2, ( *pEx_pts ).horizon_r[1], rows, cols );
    check_endpoint( ( *pEx_pts ).horizon_r[0], ( *pEx_pts ).horizon_r[1] );
}

int match_two_bars_by_expts(const struct s_bar *pDomain_bar ,const struct s_bar *pSlave_bar,
                            float distance_limit,float angle_limit,int small_or_large)
{
    float distance, angle_diff;

    if ( cal_distance_pt( ( *pDomain_bar ).ellipse.center, ( *pSlave_bar ).ellipse.center ) <= 8 )
    {
        std::cout << "distance less than 8" << std::endl;
        return 0;
    }

    if ( small_or_large == 0 )
    {
        distance = cal_distance_pt( ( *pSlave_bar ).ellipse.center, ( ( *pDomain_bar ).ex_small_armor.parallel_l[0] + ( *pDomain_bar ).ex_small_armor.parallel_l[1] ) / 2.0 );
        angle_diff = fabsf( ( *pDomain_bar ).ellipse.angle - ( *pSlave_bar ).ellipse.angle );

        if ( (distance < ( *pDomain_bar ).ellipse.size.height * distance_limit ) && ( ( angle_diff < angle_limit ) || ( angle_diff > ( 180 - angle_limit ) ) ) )
        {
            return 1;
        }

        distance = cal_distance_pt( ( *pSlave_bar ).ellipse.center, ( ( *pDomain_bar ).ex_small_armor.parallel_r[0] + ( *pDomain_bar ).ex_small_armor.parallel_r[1] ) / 2.0 );
        angle_diff = fabsf( ( *pDomain_bar ).ellipse.angle - ( *pSlave_bar ).ellipse.angle );
        if ( ( distance < ( *pDomain_bar ).ellipse.size.height * distance_limit ) && ( ( angle_diff < angle_limit ) || ( angle_diff > ( 180 - angle_limit ) ) ) )
        {
            return 2;
        }

        distance = cal_distance_pt( ( *pSlave_bar ).ellipse.center, ( ( *pDomain_bar ).ex_small_armor.horizon_l[0] + ( *pDomain_bar ).ex_small_armor.horizon_l[1] ) / 2.0 );
        angle_diff = fabsf( ( *pDomain_bar ).ellipse.angle - ( *pSlave_bar ).ellipse.angle );
        if ( ( distance < ( *pDomain_bar ).ellipse.size.height * distance_limit ) && ( ( angle_diff < angle_limit ) || ( angle_diff > ( 180 - angle_limit ) ) ) )
        {
            return 3;
        }

        distance = cal_distance_pt( ( *pSlave_bar ).ellipse.center, ( ( *pDomain_bar ).ex_small_armor.horizon_r[0] + ( *pDomain_bar ).ex_small_armor.horizon_r[1] ) / 2.0 );
        angle_diff = fabsf( ( *pDomain_bar ).ellipse.angle - ( *pSlave_bar ).ellipse.angle );
        if ( ( distance < ( *pDomain_bar ).ellipse.size.height * distance_limit ) && ( ( angle_diff < angle_limit ) || ( angle_diff > ( 180 - angle_limit ) ) ) )
        {
            return 4;
        }
    }

    else
    {
        /**************Hero armor***************************/
        distance = cal_distance_pt( ( *pSlave_bar ).ellipse.center, ( ( *pDomain_bar ).ex_large_armor.parallel_l[0] + ( *pDomain_bar ).ex_large_armor.parallel_l[1] ) / 2.0 );

        angle_diff = fabsf( ( *pDomain_bar ).ellipse.angle - ( *pSlave_bar ).ellipse.angle );
        if ( ( distance < ( *pDomain_bar ).ellipse.size.height*distance_limit ) && ( ( angle_diff < angle_limit ) || ( angle_diff > ( 180 - angle_limit ) ) ) )
        {
            return 5;
        }

        distance = cal_distance_pt( ( *pSlave_bar ).ellipse.center, ( ( *pDomain_bar ).ex_large_armor.parallel_r[0] + ( *pDomain_bar ).ex_large_armor.parallel_r[1] ) / 2.0 );
        angle_diff = fabsf( ( *pDomain_bar ).ellipse.angle - ( *pSlave_bar ).ellipse.angle );
        if ( ( distance < ( *pDomain_bar ).ellipse.size.height * distance_limit ) && ( ( angle_diff < angle_limit ) || ( angle_diff > ( 180 - angle_limit ) ) ) )
        {
            return 6;
        }

        distance = cal_distance_pt( ( *pSlave_bar ).ellipse.center, ( ( *pDomain_bar ).ex_large_armor.horizon_l[0] + ( *pDomain_bar ).ex_large_armor.horizon_l[1] ) / 2.0 );
        angle_diff = fabsf( ( *pDomain_bar ).ellipse.angle - ( *pSlave_bar ).ellipse.angle );
        if ( ( distance < ( *pDomain_bar ).ellipse.size.height * distance_limit ) && ( ( angle_diff<angle_limit ) || ( angle_diff > ( 180 - angle_limit ) ) ) )
        {
            return 7;
        }

        distance = cal_distance_pt( ( *pSlave_bar ).ellipse.center, ( ( *pDomain_bar ).ex_large_armor.horizon_r[0] + ( *pDomain_bar ).ex_large_armor.horizon_r[1] ) / 2.0 );
        angle_diff = fabsf( ( *pDomain_bar ).ellipse.angle - ( *pSlave_bar ).ellipse.angle );
        if ( ( distance < ( *pDomain_bar ).ellipse.size.height * distance_limit ) && ( ( angle_diff < angle_limit ) || ( angle_diff > ( 180 - angle_limit ) ) ) )
        {
            return 8;
        }
    }


    return 0;
}

/*propose big or small points*/
void get_proposal_pts(cv::Point2f *pEndpoints,float sup_angle2,float distance_l,float distance_h,struct proposal_pt *pProp_pts,int rows,int cols)
{
    find_newpoint( pEndpoints[0], sup_angle2, -distance_l, ( *pProp_pts ).parallel_l[0], rows, cols );
    find_newpoint( pEndpoints[1], sup_angle2, -distance_l, ( *pProp_pts ).parallel_l[1], rows, cols );
    find_newpoint( pEndpoints[0], sup_angle2, -distance_h, ( *pProp_pts ).parallel_l[2], rows, cols );
    find_newpoint( pEndpoints[1], sup_angle2, -distance_h, ( *pProp_pts ).parallel_l[3], rows, cols );

    find_newpoint( pEndpoints[0], sup_angle2, distance_l, ( *pProp_pts ).parallel_r[0], rows, cols );
    find_newpoint( pEndpoints[1], sup_angle2, distance_l, ( *pProp_pts ).parallel_r[1], rows, cols );
    find_newpoint( pEndpoints[0], sup_angle2, distance_h, ( *pProp_pts ).parallel_r[2], rows, cols );
    find_newpoint( pEndpoints[1], sup_angle2, distance_h, ( *pProp_pts ).parallel_r[3], rows, cols );
}

std::vector<int> match_two_bars_by_propts( struct proposal_pt prop_pts, float area_k, cv::Mat &img_src )
{
    std::vector<int> result(2);

    result[0] = 0;
    result[1] = 0;

    std::vector<cv::Point2f> proposal_vec( std::begin( prop_pts.parallel_l ), std::end( prop_pts.parallel_l ) );
    cv::Rect proposal = cv::boundingRect( proposal_vec );

    if ( check_rect( proposal, img_src.rows, img_src.cols ) )
    {
        float count_area = cv::contourArea(proposal_vec);

        if ( check_mask_area( img_src, proposal ) / count_area > area_k )
            result[0] = 1;
    }

    proposal_vec= std::vector<cv::Point2f> ( std::begin( prop_pts.parallel_r ), std::end( prop_pts.parallel_r ) );
    proposal = cv::boundingRect( proposal_vec );

    if ( check_rect( proposal, img_src.rows, img_src.cols ) )
    {
        float count_area = cv::contourArea( proposal_vec );
        if ( check_mask_area( img_src, proposal ) / count_area > area_k )
            result[1] = 1;
    }
    return result;
}

/*Get   expand pts;
 *      proposal pts;
 *      large expand pts*/
void get_surround_pts( std::vector<struct s_bar> &bar_list, const cv::Mat &img_src )
{
    for ( int i = 0; i < bar_list.size(); i++ )
    {
        float h = bar_list[i].ellipse.size.height;
        float angle = bar_list[i].ellipse.angle;
        std::cout << bar_list[i].ellipse.angle << std::endl;
        angle = ( 180.0 - angle ) / 180.0 * CV_PI;
        float stripe_length = fabsf( h / cos( angle ) );
        float largeArmor_length = 4 * stripe_length;

        get_expand_pts( bar_list[i].endpoints, &bar_list[i].ex_small_armor, angle, 0, 2 * h, 2.5 * h, img_src.rows, img_src.cols );
        get_proposal_pts( bar_list[i].endpoints, angle, 1 * h, 2.5 * h, &bar_list[i].prop_small_armor, img_src.rows, img_src.cols );
        if ( bar_list[i].ellipse.angle > 15 && bar_list[i].ellipse.angle < 165 )
        {
            largeArmor_length = 6.6 * stripe_length;
        }
        else if ( bar_list[i].ellipse.angle > 10 && bar_list[i].ellipse.angle < 170 )
        {
            largeArmor_length = 5.8 * stripe_length;
        }
        else if ( bar_list[i].ellipse.angle > 5 && bar_list[i].ellipse.angle < 175 )
        {
            largeArmor_length = 5 * stripe_length;
        }
        std::cout << largeArmor_length / h << std::endl;
        get_expand_pts( bar_list[i].endpoints, &bar_list[i].ex_large_armor, angle, 0, 4 * stripe_length, largeArmor_length, img_src.rows, img_src.cols );
    }

}

static void prepare_transform_img( cv::Mat &img_src, struct s_bar *pDomain_bar, const cv::Mat & img_backups, int small_or_big, bool isDrawLine )
{
    get_armor_from_pts( ( *pDomain_bar ).endpoints[0], ( *pDomain_bar ).endpoints[1], ( *pDomain_bar ).pair_p1, ( *pDomain_bar ).pair_p2, ( *pDomain_bar ).armor_vertex, img_backups.rows, img_backups.cols );

    if ( small_or_big == 1 )
        AutoHit::transform_armor_vertex( ( *pDomain_bar ).armor_vertex, 0.25 );

    AutoHit::new_pts[0] = ( *pDomain_bar ).armor_vertex[0];
    AutoHit::new_pts[1] = ( *pDomain_bar ).armor_vertex[1];
    AutoHit::new_pts[2] = ( *pDomain_bar ).armor_vertex[2];
    AutoHit::new_pts[3] = ( *pDomain_bar ).armor_vertex[3];

    {
        cv::line( img_src, AutoHit::new_pts[0], AutoHit::new_pts[1], cv::Scalar(0, 255, 255), 1 );
        cv::line( img_src, AutoHit::new_pts[1], AutoHit::new_pts[2], cv::Scalar(0, 255, 255), 1 );
        cv::line( img_src, AutoHit::new_pts[2], AutoHit::new_pts[3], cv::Scalar(0, 255, 255), 1 );
        cv::line( img_src, AutoHit::new_pts[3], AutoHit::new_pts[0], cv::Scalar(0, 255, 255), 1 );
    }
    cv::Mat transform = cv::getPerspectiveTransform( AutoHit::new_pts, AutoHit::world_pts );
    cv::warpPerspective( img_backups, AutoHit::img_transform, transform, cv::Size(40, 40) );
}

int find_small_armor_in_expts( cv::Mat &img_src, struct s_bar *pDomain_bar ,const struct s_bar *pSlave_bar,float distance_limit,float angle_limit, cv::Mat & img_backups)
{
    int result = 0;
    int match_result = 0;

    match_result = match_two_bars_by_expts( pDomain_bar, pSlave_bar, distance_limit, angle_limit, 0 );

    if ( match_result == 0 )
        return 0;


    ( *pDomain_bar ).pair_p1 = ( *pSlave_bar ).endpoints[0];
    ( *pDomain_bar ).pair_p2 = ( *pSlave_bar ).endpoints[1];

    pDomain_bar->isPosePoints   = 1;
    pDomain_bar->pose_points[0] = pDomain_bar->endpoints[0];
    pDomain_bar->pose_points[1] = pDomain_bar->pair_p1;
    pDomain_bar->pose_points[2] = pDomain_bar->pair_p2;
    pDomain_bar->pose_points[3] = pDomain_bar->endpoints[1];

    prepare_transform_img( img_src, pDomain_bar, img_backups, 0, false );
    result = AutoHit::predict( AutoHit::model, AutoHit::pHog, AutoHit::img_transform, AutoHit::Model_cnt );
    return result;
}

int find_small_armor_in_props( cv::Mat &img_src, struct s_bar *pDomain_bar ,float area_k, cv::Mat & img_backups )
{
    int result = 0;
    std::vector<int> check_proposal(2);
    check_proposal = match_two_bars_by_propts((*pDomain_bar).prop_small_armor, area_k, img_backups );
    //printf("small_armor match_result is: %d\n",(check_proposal[0]+check_proposal[1]));

    if( check_proposal[0]+check_proposal[1] == 0 )
        return 0;


    float h = ( *pDomain_bar ).ellipse.size.height;
    float angle = ( *pDomain_bar ).ellipse.angle;
    angle = ( 180.0 - angle ) / 180.0 * CV_PI;

    if( check_proposal[0] == 1 )
    {
        find_newpoint( ( *pDomain_bar ).endpoints[0], angle, -2 * h, ( *pDomain_bar ).pair_p1, img_backups.rows, img_backups.cols );
        find_newpoint( ( *pDomain_bar ).endpoints[1], angle, -2 * h, ( *pDomain_bar ).pair_p2, img_backups.rows, img_backups.cols );
        check_endpoint( ( *pDomain_bar ).pair_p1, ( *pDomain_bar ).pair_p2 );

        prepare_transform_img( img_src, pDomain_bar, img_backups, 0, false );
        result = AutoHit::predict( AutoHit::model, AutoHit::pHog, AutoHit::img_transform, AutoHit::Model_cnt );

        if( result != 0 )
            return result;
    }

    if( check_proposal[1] == 1 )
    {
        find_newpoint( ( *pDomain_bar ).endpoints[0], angle, 2 * h, ( *pDomain_bar ).pair_p1, img_backups.rows, img_backups.cols );
        find_newpoint( ( *pDomain_bar ).endpoints[1], angle, 2 * h, ( *pDomain_bar ).pair_p2, img_backups.rows, img_backups.cols );
        check_endpoint( ( *pDomain_bar ).pair_p1, ( *pDomain_bar ).pair_p2 );

        pDomain_bar->isPosePoints   = 1;
        pDomain_bar->pose_points[0] = pDomain_bar->endpoints[0];
        pDomain_bar->pose_points[1] = pDomain_bar->pair_p1;
        pDomain_bar->pose_points[2] = pDomain_bar->pair_p2;
        pDomain_bar->pose_points[3] = pDomain_bar->endpoints[1];

        prepare_transform_img( img_src, pDomain_bar, img_backups, 0, false );
        result = AutoHit::predict( AutoHit::model, AutoHit::pHog, AutoHit::img_transform, AutoHit::Model_cnt );

        if( result != 0 )
        {
            return result;
        }
    }

    return 0;
}

int find_large_armor_in_expts( cv::Mat &img_src, struct s_bar *pDomain_bar, const struct s_bar *pSlave_bar,float distance_limit,float angle_limit, cv::Mat & img_backups)
{
    int result = 0;
    int match_result = 0;
    if ( pDomain_bar->ellipse.size.height<25 )
    {
        distance_limit = 4;
    }

    match_result = match_two_bars_by_expts( pDomain_bar, pSlave_bar, distance_limit, angle_limit, 1 );
    if ( match_result == 0 )
         return 0;

    float h_div = pDomain_bar->ellipse.size.height / pSlave_bar->ellipse.size.height;
    if ( h_div < 0.55 )
    {
        ( *pDomain_bar ).pair_p1 = ( *pSlave_bar ).endpoints[0];
        ( *pDomain_bar ).pair_p2 = ( *pSlave_bar ).endpoints[1];

        float angle_rad = ( 90.0 - ( *pDomain_bar ).ellipse.angle ) / 180.0 * CV_PI;
        float h = ( *pDomain_bar ).ellipse.size.height;
        find_newpoint( ( *pDomain_bar ).ellipse.center, angle_rad, -h,
                       ( *pDomain_bar ).endpoints[0], img_backups.rows, img_backups.cols );
        find_newpoint( ( *pDomain_bar ).ellipse.center, angle_rad,h,
                       ( *pDomain_bar ).endpoints[1], img_backups.rows, img_backups.cols );

        ( *pDomain_bar ).pair_p1 = (*pSlave_bar).endpoints[0];
        ( *pDomain_bar ).pair_p2 = (*pSlave_bar).endpoints[1];
    }
    else if ( h_div > 1.8 )
    {
        float angle_rad = ( 90.0 - ( *pSlave_bar ).ellipse.angle ) / 180.0 * CV_PI;
        float h = ( *pSlave_bar ).ellipse.size.height;
        find_newpoint( ( *pSlave_bar ).ellipse.center, angle_rad, -h,
                       ( *pDomain_bar ).pair_p1, img_backups.rows, img_backups.cols );
        find_newpoint( ( *pSlave_bar ).ellipse.center, angle_rad, h,
                       ( *pDomain_bar ).pair_p2, img_backups.rows, img_backups.cols );
    }
    else
    {
        ( *pDomain_bar ).pair_p1 = ( *pSlave_bar ).endpoints[0];
        ( *pDomain_bar ).pair_p2 = ( *pSlave_bar ).endpoints[1];
    }

    cv::line( img_src, ( *pDomain_bar ).endpoints[0], ( *pDomain_bar ).pair_p1, cv::Scalar(0, 0, 255) );
    cv::line( img_src, ( *pDomain_bar ).pair_p2, ( *pDomain_bar ).pair_p1, cv::Scalar(0, 0, 255) );
    cv::line( img_src, ( *pDomain_bar ).endpoints[1], ( *pDomain_bar ).pair_p2,cv::Scalar(0, 0, 255) );
    cv::line( img_src, ( *pDomain_bar ).endpoints[1], ( *pDomain_bar ).endpoints[0], cv::Scalar(0, 0, 255) );

    prepare_transform_img( img_src, pDomain_bar, img_backups, 1, false );
    result = AutoHit::predict( AutoHit::model, AutoHit::pHog, AutoHit::img_transform, AutoHit::Model_cnt );
    return result;
}

int find_small_armor_in_virpts( cv::Mat &img_src, struct s_bar *pDomain_bar, float distance_k, cv::Mat & img_backups )
{
    int result = 0;
    float h = ( *pDomain_bar ).ellipse.size.height;
    float angle = ( *pDomain_bar ).ellipse.angle;
    float angle_k;
    if ( ( angle < 5 ) || ( angle > 175 ) )
        angle_k = 0.95;
    else if ( ( angle < 10 ) || ( angle > 170 ) )
        angle_k = 0.8;
    else
        angle_k = 0.7;
    angle = ( 180.0 - angle ) / 180.0 * CV_PI;

    float slope_k = std::abs( cos( angle ) ) * angle_k;

    find_newpoint( ( *pDomain_bar ).endpoints[0], angle, -distance_k * h * slope_k, ( *pDomain_bar ).pair_p1, img_backups.rows, img_backups.cols );
    find_newpoint( ( *pDomain_bar ).endpoints[1], angle, -distance_k * h * slope_k, ( *pDomain_bar ).pair_p2, img_backups.rows, img_backups.cols );
    check_endpoint( ( *pDomain_bar ).pair_p1, (*pDomain_bar ).pair_p2 );

    prepare_transform_img( img_src, pDomain_bar, img_backups, 0, false );
    result = AutoHit::predict( AutoHit::model, AutoHit::pHog, AutoHit::img_transform, AutoHit::Model_cnt );
    if( result != 0 )
        return 1;

    find_newpoint( ( *pDomain_bar).endpoints[0], angle, distance_k * h * slope_k, ( *pDomain_bar ).pair_p1, img_backups.rows, img_backups.cols );
    find_newpoint( ( *pDomain_bar).endpoints[1], angle, distance_k * h * slope_k, ( *pDomain_bar ).pair_p2, img_backups.rows, img_backups.cols );
    check_endpoint( ( *pDomain_bar).pair_p1, ( *pDomain_bar ).pair_p2 );

    prepare_transform_img( img_src, pDomain_bar, img_backups, 0, false );
    result = AutoHit::predict ( AutoHit::model, AutoHit::pHog, AutoHit::img_transform, AutoHit::Model_cnt );

    if ( result != 0 )
         return 2;

    return 0;
}

int find_large_armor_in_virpts( cv::Mat &img_src, struct s_bar *pDomain_bar, float distance_k, cv::Mat & img_backups)
{
    int result = 0;
    float h = ( *pDomain_bar ).ellipse.size.height;
    float angle = ( *pDomain_bar ).ellipse.angle;
    angle = ( 180.0 - angle ) / 180.0 * CV_PI;

    float k_bias = 0.22;
    cv::Point2f p_u = ( *pDomain_bar ).endpoints[0];
    cv::Point2f p_d = ( *pDomain_bar ).endpoints[1];
    find_newpoint( ( *pDomain_bar ).endpoints[0], angle, -( k_bias+distance_k ) * h, ( *pDomain_bar ).endpoints[0], img_backups.rows, img_backups.cols );
    find_newpoint( ( *pDomain_bar ).endpoints[1], angle, -( k_bias+distance_k ) * h, ( *pDomain_bar ).endpoints[1], img_backups.rows, img_backups.cols );

    find_newpoint( ( *pDomain_bar ).endpoints[0], angle, distance_k * h, ( *pDomain_bar ).pair_p1, img_backups.rows, img_backups.cols );
    find_newpoint( ( *pDomain_bar ).endpoints[1], angle, distance_k * h, ( *pDomain_bar ).pair_p2, img_backups.rows, img_backups.cols );

    check_endpoint( ( *pDomain_bar ).pair_p1, ( *pDomain_bar ).pair_p2 );

    prepare_transform_img( img_src, pDomain_bar, img_backups, 1, false );
    result = AutoHit::predict( AutoHit::model, AutoHit::pHog, AutoHit::img_transform, AutoHit::Model_cnt );
    if( result != 0 )
        return 1;

    ( *pDomain_bar ).endpoints[0] = p_u;
    ( *pDomain_bar ).endpoints[1] = p_d;
    find_newpoint( ( *pDomain_bar ).endpoints[0], angle, k_bias * h, ( *pDomain_bar ).endpoints[0], img_backups.rows, img_backups.cols );
    find_newpoint( ( *pDomain_bar ).endpoints[1], angle, k_bias * h, ( *pDomain_bar ).endpoints[1], img_backups.rows, img_backups.cols );

    find_newpoint( ( *pDomain_bar ).endpoints[0], angle, distance_k * h, ( *pDomain_bar ).pair_p1, img_backups.rows, img_backups.cols );
    find_newpoint( ( *pDomain_bar ).endpoints[1], angle, distance_k * h, ( *pDomain_bar ).pair_p2, img_backups.rows, img_backups.cols );

    check_endpoint( ( *pDomain_bar ).pair_p1, ( *pDomain_bar ).pair_p2 );

    prepare_transform_img( img_src, pDomain_bar, img_backups, 1, false );
    result = AutoHit::predict( AutoHit::model, AutoHit::pHog, AutoHit::img_transform, AutoHit::Model_cnt );

    ( *pDomain_bar ).endpoints[0] = p_u;
    ( *pDomain_bar ).endpoints[1] = p_d;

    if( result != 0 )
        return 2;

    return 0;
}

int find_armor_int_all_pts( cv::Mat &img_src, std::vector<struct s_bar> &valid_bar_list, cv::Mat &img_backups )
{
    int result = 0;

    for ( int i = 0; i < valid_bar_list.size(); i++ )
    {
        for (int j = 0; j < valid_bar_list.size(); j++ )
        {
            if ( i==j )
                 continue;

            result = find_small_armor_in_expts( img_src, &valid_bar_list[i], &valid_bar_list[j], 0.3, AutoHit::small_pair_bar_angle_limit, img_backups );
            if ( result != 0 )
            {
                 return i;
            }

            result = find_small_armor_in_props( img_src, &valid_bar_list[i], AutoHit::proposal_bar_area_k, img_backups );
            if ( result != 0 )
            {
                 return i;
            }
            result = find_large_armor_in_expts( img_src, &valid_bar_list[i], &valid_bar_list[j], 2.5, AutoHit::big_pair_bar_angle_limit, img_backups );
            if ( result != 0 )
            {
                 return i;
            }
        }
    };
    //sort bar list according to height
    for ( int i = 0; i < valid_bar_list.size(); i++ )
    {
        /*********Find virtual pair for single bar********************/
        if ( valid_bar_list[i].ellipse.size.height < AutoHit::bar_height_L )
        {
             continue;
        }

        result = find_small_armor_in_virpts( img_src, &valid_bar_list[i], 2.0, img_backups );
        if( result != 0 )
            return i;

        result = find_large_armor_in_virpts( img_src, &valid_bar_list[i], 4.0, img_backups );
        if ( result != 0 )
             return i;
    }
    return -1;
}

int find_armor_for_single_bar( cv::Mat &img_src, std::vector<struct s_bar> &valid_bar_list, cv::Mat & img_backups, cv::Point2f &sp )
{
    int result = 0;
    std::vector<struct s_bar> new_bar_list;
    sort_bar_according_to_height( valid_bar_list, new_bar_list );

    for ( int i = 0; i < new_bar_list.size(); i++ )
    {
        /*********Find virtual pair for single bar********************/
        if ( valid_bar_list[i].ellipse.size.height < AutoHit::bar_height_L )
        {
             std::cout << "single bar result is %f" << valid_bar_list[i].ellipse.size.height << std::endl;
             continue;
        }

        result = find_small_armor_in_virpts( img_src, &new_bar_list[i], 2.0, img_backups );
        if ( result != 0 )
        {
            cv::Point2f p = getCrossPoint( new_bar_list[i].armor_vertex[0], new_bar_list[i].armor_vertex[2],
                                           new_bar_list[i].armor_vertex[1], new_bar_list[i].armor_vertex[3] );
            if ( p.x != -1 )
                 sp = p;
                 return i;
        }

        result = find_large_armor_in_virpts( img_src, &valid_bar_list[i], 4.4, img_backups );
        if( result != 0 )
        {
            return i;
        }
    }
    sp.x = -1;
    sp.y = -1;
    return 0;
}

