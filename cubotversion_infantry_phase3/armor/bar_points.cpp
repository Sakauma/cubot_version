#include "armor/bar_points.h"
#include "armor/armor_main.h"

/*********************************
 *
 * Description: 输入两条线段的端点，返回两条线段交点的坐标
 * Input:
 *      1.  cv::Point2f &p00
 *      2.  cv::Point2f &p01
 *      3.  cv::Point2f &p10
 *      4.  cv::Point2f &p11
 * Output:
 *      1.  crossPoint
 * Others:
 * 		1、 two lines can't be vertical
 * 		2、 calculate two lines' slope
 * 		3、 calculate the crosspoint
 * Author:  jason
**********************************/
cv::Point2f getCrossPoint( cv::Point2f &p00, cv::Point2f &p01, cv::Point2f &p10, cv::Point2f &p11 )
{
    if ( p00.x==p01.x )
        return cv::Point2f(-1 ,-1);
    if ( p10.x==p11.x )
        return cv::Point2f(-1 ,-1);

    float k0 = ( p00.y-p01.y ) / ( p00.x-p01.x );
    float k1 = ( p10.y-p11.y ) / ( p10.x-p11.x );

    assert( k0 != k1 );

    if ( k0 == k1 )
        return cv::Point2f(-1,-1);

    cv::Point2f crossPoint;

    crossPoint.x = ( p10.y - p00.y + k0 * p00.x - k1 * p10.x ) / ( k0 - k1 );
    crossPoint.y = k0 * ( crossPoint.x-p00.x ) + p00.y;

    return crossPoint;
}

/*expand big or small points*/
/*********************************
*
* Description:  To get expand points
* Input :
*       1.  cv::Point2f *pEndpoints   椭圆端点
*       2.  float sup_angle           椭圆旋转角度的补角
*       3.  float angle               赋值为 0
*       4.  float distance1           椭圆长轴的倍数 2H / 4H
*       5.  float distance2           椭圆长轴的倍数 2.5H / 4H
*       6.  int rows                  原图行
*       7.  int cols                  原图列
* Output:
*       1.  struct expand_pt *pEx_pts   输出的扩展点
* Others:
* 		1.  获得左侧斜向扩展点   存储在  .parallel_l[] 中
* 		2.  获得右侧斜向扩展点   存储在  .parallel_r[] 中
* 		3.  获得左侧横向扩展点   存储在  .horizon_l[]  中
* 		4.  获得右侧横向扩展点   存储在  .horizon_r[]  中
* Author:   jason
**********************************/
void get_expand_pts( cv::Point2f *pEndpoints, struct expand_pt *pEx_pts, float sup_angle, float angle,
                         float distance1, float distance2, int rows, int cols )
{
    /************** find left parallel expand **************/

    find_newpoint( pEndpoints[0], sup_angle, -distance1, (*pEx_pts).parallel_l[0], rows,cols );
    find_newpoint( pEndpoints[1], sup_angle, -distance1, (*pEx_pts).parallel_l[1], rows,cols );
    check_endpoint( (*pEx_pts).parallel_l[0], (*pEx_pts).parallel_l[1] );


//    std::cout<<"in function endpoint1: "<<endpoints[0].x<<" "<<endpoints[0].y<<std::endl;
//    std::cout<<"in function ex_pts.parallel_l1: "<<(*ex_pts).parallel_l[0].x<<" "<<(*ex_pts).parallel_l[0].y<<std::endl;

    /************* find right parallel expand ***************/

    find_newpoint( pEndpoints[0], sup_angle,distance1, (*pEx_pts).parallel_r[0], rows,cols );
    find_newpoint( pEndpoints[1], sup_angle,distance1, (*pEx_pts).parallel_r[1], rows,cols );
    check_endpoint( (*pEx_pts).parallel_r[0], (*pEx_pts).parallel_r[1] );


    /***************** find left horizon expand *****************/

    find_newpoint( pEndpoints[0], angle, -distance2, (*pEx_pts).horizon_l[0], rows, cols );
    find_newpoint( pEndpoints[1], angle, -distance2, (*pEx_pts).horizon_l[1], rows, cols );
    check_endpoint( (*pEx_pts).horizon_l[0], (*pEx_pts).horizon_l[1] );

    /****************** find right horizon expand ****************/

    find_newpoint( pEndpoints[0], angle,distance2, (*pEx_pts).horizon_r[0], rows,cols );
    find_newpoint( pEndpoints[1], angle,distance2, (*pEx_pts).horizon_r[1], rows,cols );
    check_endpoint( (*pEx_pts).horizon_r[0], (*pEx_pts).horizon_r[1] );
}

/*********************************
 *
 * Description: 通过扩展点匹配两个灯条
 *  Input:
 *      1.  const struct s_bar *pDomain_bar  主灯条
 *      2.  const struct s_bar *pSlave_bar   从灯条
 *      3.  float distance_limit            距离限制
 *      4.  float angle_limit               角度限制
 *      5.  int small_or_large              大小装甲flag
 * Output:
 *      1.  return num                      返回数字信息
 * Others:
 * 		1、  计算从灯条中心点到主灯条各个扩展中心点的距离
 * 		2、  计算从灯条与主灯条的角度绝对值
 * 		3、  判断距离与角度是否符合约束条件
 *      4、  大装甲小装甲共八种情况，如果符合约束条件，分别返回对应的数字
 *  Athor:   jason
**********************************/
int match_two_bars_by_expts( const struct s_bar *pDomain_bar, const struct s_bar *pSlave_bar,
                             float distance_limit, float angle_limit, int small_or_large )
{
    float distance ,angle_diff;

    if( cal_distance_pt( (*pDomain_bar).ellipse.center, (*pSlave_bar).ellipse.center ) <= 8 )
    {
        std::cout << "distance less than 8" << std::endl;
        return 0;
    }

    if ( small_or_large == 0 )
    {

        distance = cal_distance_pt( ( *pSlave_bar ).ellipse.center, ( (*pDomain_bar).ex_small_armor.parallel_l[0] + ( *pDomain_bar ).ex_small_armor.parallel_l[1] ) / 2.0 );
        angle_diff = fabsf( ( *pDomain_bar ).ellipse.angle - ( *pSlave_bar ).ellipse.angle );
        if ( ( distance < ( *pDomain_bar ).ellipse.size.height*distance_limit )
               &&( ( angle_diff<angle_limit ) || ( angle_diff>( 180-angle_limit ) ) ) )
        {
               return 1;
        }

        distance = cal_distance_pt( ( *pSlave_bar ).ellipse.center, ( ( *pDomain_bar ).ex_small_armor.parallel_r[0] + ( *pDomain_bar ).ex_small_armor.parallel_r[1] ) / 2.0 );
        angle_diff = fabsf( ( *pDomain_bar ).ellipse.angle - ( *pSlave_bar ).ellipse.angle );
        if( ( distance < ( *pDomain_bar ).ellipse.size.height*distance_limit )
              &&( ( angle_diff < angle_limit )||( angle_diff > ( 180 - angle_limit) ) ) )
        {
              return 2;
        }

        distance = cal_distance_pt( ( *pSlave_bar ).ellipse.center, ( ( *pDomain_bar ).ex_small_armor.horizon_l[0] + ( *pDomain_bar ).ex_small_armor.horizon_l[1] ) / 2.0 );

        angle_diff = fabsf( ( *pDomain_bar ).ellipse.angle - ( *pSlave_bar ).ellipse.angle );

        if ( ( distance < ( *pDomain_bar ).ellipse.size.height*distance_limit )
               &&( ( angle_diff < angle_limit )||( angle_diff > ( 180-angle_limit ) ) ) )
        {
               return 3;
        }

        distance = cal_distance_pt( ( *pSlave_bar ).ellipse.center, ( ( *pDomain_bar ).ex_small_armor.horizon_r[0] + ( *pDomain_bar ).ex_small_armor.horizon_r[1] ) / 2.0 );

        angle_diff = fabsf( ( *pDomain_bar ).ellipse.angle - ( *pSlave_bar ).ellipse.angle );

        if ( ( distance < ( *pDomain_bar ).ellipse.size.height*distance_limit )
               &&( ( angle_diff < angle_limit ) || ( angle_diff > ( 180-angle_limit ) ) ) )
        {
               return 4;
        }
    }

    else    //if(small_or_large == 1) 1大装甲
    {
        /*************************** Hero Armor ***************************/

        distance = cal_distance_pt( ( *pSlave_bar ).ellipse.center, ( ( *pDomain_bar ).ex_large_armor.parallel_l[0] + ( *pDomain_bar ).ex_large_armor.parallel_l[1] ) / 2.0 );

        angle_diff = fabsf( ( *pDomain_bar ).ellipse.angle - ( *pSlave_bar ).ellipse.angle );

        if ( ( distance < ( *pDomain_bar ).ellipse.size.height * distance_limit )
               &&( ( angle_diff < angle_limit ) || ( angle_diff > ( 180-angle_limit ) ) ) )
        {
               return 5;
        }

        distance = cal_distance_pt( ( *pSlave_bar ).ellipse.center, ( ( *pDomain_bar ).ex_large_armor.parallel_r[0] + ( *pDomain_bar ).ex_large_armor.parallel_r[1] ) / 2.0 );

        angle_diff = fabsf( ( *pDomain_bar ).ellipse.angle - ( *pSlave_bar ).ellipse.angle );

        if ( ( distance < ( *pDomain_bar ).ellipse.size.height * distance_limit )
               &&( ( angle_diff < angle_limit ) || ( angle_diff > ( 180-angle_limit ) ) ) )
        {
               return 6;
        }

        distance = cal_distance_pt( ( *pSlave_bar ).ellipse.center, ( ( *pDomain_bar ).ex_large_armor.horizon_l[0] + ( *pDomain_bar ).ex_large_armor.horizon_l[1] ) / 2.0 );

        angle_diff = fabsf( ( *pDomain_bar ).ellipse.angle - ( *pSlave_bar ).ellipse.angle );

        if ( ( distance < ( *pDomain_bar ).ellipse.size.height*distance_limit )
               &&( ( angle_diff < angle_limit ) || ( angle_diff > ( 180 - angle_limit ) ) ) )
        {
               return 7;
        }

        distance = cal_distance_pt( ( *pSlave_bar ).ellipse.center, ( ( *pDomain_bar ).ex_large_armor.horizon_r[0] + ( *pDomain_bar ).ex_large_armor.horizon_r[1] ) / 2.0 );

        angle_diff = fabsf( ( *pDomain_bar ).ellipse.angle - ( *pSlave_bar ).ellipse.angle );

        if ( ( distance < ( *pDomain_bar ).ellipse.size.height*distance_limit )
               &&( ( angle_diff < angle_limit ) || ( angle_diff > ( 180 - angle_limit ) ) ) )
        {
               return 8;
        }
    }
    return 0;
}

/*propose big or small points*/
/*********************************
*
* Description:  获得待提点
* Input :
*       1.  cv::Point2f*endpoints   椭圆端点
*       2.  float angle           椭圆旋转角度的补角
*       4.  float distance_l        椭圆长轴的倍数 1h
*       5.  float distance_h        椭圆长轴的倍数 2.5h
*       6.  int rows                原图行
*       7.  int cols                原图列
* Output:
*       1.  struct proposal_pt *prop_pts   输出的待提点
* Others:
* 		1.  在左侧 distance_l 位置处获得斜向待提点
* 		2.  在左侧 distance_h 位置处获得斜向待提点
* 		3.  在右侧 distance_l 位置处获得斜向待提点
* 		4.  在右侧 distance_l 位置处获得斜向待提点
* Author:   jason
**********************************/
void get_proposal_pts( cv::Point2f *pEndpoints, float sup_angle2, float distance_l, float distance_h, struct proposal_pt *pProp_pts, int rows, int cols )
{
    find_newpoint( pEndpoints[0], sup_angle2, -distance_l, ( *pProp_pts ).parallel_l[0], rows, cols );
    find_newpoint( pEndpoints[1], sup_angle2, -distance_l, ( *pProp_pts ).parallel_l[1], rows, cols );
    find_newpoint( pEndpoints[0], sup_angle2, -distance_h, ( *pProp_pts ).parallel_l[2] ,rows, cols );
    find_newpoint( pEndpoints[1], sup_angle2, -distance_h, ( *pProp_pts ).parallel_l[3] ,rows, cols );

    find_newpoint( pEndpoints[0], sup_angle2, distance_l, ( *pProp_pts ).parallel_r[0], rows, cols );
    find_newpoint( pEndpoints[1], sup_angle2, distance_l, ( *pProp_pts ).parallel_r[1], rows, cols );
    find_newpoint( pEndpoints[0], sup_angle2, distance_h, ( *pProp_pts ).parallel_r[2], rows, cols );
    find_newpoint( pEndpoints[1], sup_angle2, distance_h, ( *pProp_pts ).parallel_r[3], rows, cols );
}

/*********************************
 *
 * Description: 通过待提点匹配两个灯条
 * Input :
 *      1.  struct proposal_pt prop_pts     输入主灯条的小装甲待提点
 *      2.  float area_k                    灯条面积约束条件
 *      3.  cv::Mat &img_src
 * Output:
 *      1.  result                          返回结果，1则匹配成功，0则匹配失败
 * Others:
 * 		1、  对四个待提点做外部矩形边界，即对主灯条左右两侧各延伸一个矩形检测框
 * 		2、  计算目标颜色占检测框面积的比例，判断是否大于area_k = 0.05，大于则返回result=1,否则返回0
 *Author :   jason
**********************************/
std::vector<int> match_two_bars_by_propts( struct proposal_pt prop_pts, float area_k, cv::Mat &img_src )      // change the float before count_area to double
{
    std::vector<int> result(2);

    result[0]=0;
    result[1]=0;

    std::vector<cv::Point2f> proposal_vec( std::begin(prop_pts.parallel_l ), std::end( prop_pts.parallel_l ) );

    cv::Rect proposal = cv::boundingRect( proposal_vec );

    if ( check_rect( proposal, img_src.rows, img_src.cols ) )
    {
        double count_area = cv::contourArea( proposal_vec );

        if( check_mask_area( img_src, proposal ) / count_area > area_k )

            result[0]=1;
    }

    proposal_vec= std::vector<cv::Point2f> ( std::begin(prop_pts.parallel_r ), std::end( prop_pts.parallel_r ) );

    proposal = cv::boundingRect( proposal_vec );

    if ( check_rect( proposal, img_src.rows, img_src.cols ) )
    {
        double count_area = cv::contourArea( proposal_vec );

        if( check_mask_area( img_src, proposal ) / count_area > area_k )

            result[1]=1;
    }
    return result;
}

/*********************************
*
* Description:  To get expand pts;
*                      proposal pts;
*                      large expand pts
* Input :
*       1.  bar_list (valid_bar_list) 输入升序后的灯条
*       2.  img_src
* Output:
*       1.  bar_list[i].ex_small_armor
*       2.  bar_list[i].prop_small_armor
*       3.  bar_list[i].ex_large_armor
* Others:
* 		1.  调用 get_expand_pts()
* 		2.  调用 get_proposal_pts()
* 		3.  调用 get_expand_pts()
* Author:   jason
**********************************/
void get_surround_pts( std::vector<struct s_bar> &bar_list, const cv::Mat img_src )
{
    for ( int i=0;i<bar_list.size();i++ )
    {
        float h = bar_list[i].ellipse.size.height;
        float angle = bar_list[i].ellipse.angle;
        angle = (180.0-angle) / 180.0 * CV_PI;

        /*find bar's samll expanded parallel and horizon expand pts*/

        get_expand_pts( bar_list[i].endpoints, &bar_list[i].ex_small_armor, angle, 0, 2 * h, 2.5 * h, img_src.rows, img_src.cols );

        get_proposal_pts( bar_list[i].endpoints, angle, 1*h, 2.5 * h, &bar_list[i].prop_small_armor, img_src.rows, img_src.cols );

        get_expand_pts( bar_list[i].endpoints, &bar_list[i].ex_large_armor, angle, 0, 4 * h, 4.0 * h, img_src.rows, img_src.cols );

    }
}

/*********************************
 *
 * Description: 得到透视变换图
 * Input :
 *      1.  cv::Mat &img_src
 *      2.  struct s_bar *pDomain_bar    //主灯条
 *      3.  const cv::Mat & img_ori
 *      4.  int small_or_big            //大小装甲flag
 *      5.  bool isDrawLine             //是否绘制直线
 * Output:
 *      1.  得到透视变换图像
 * Others:
 * 		1.  根据匹配到的一对灯条端点，计算出这对灯条对应的装甲板的四个顶点坐标
 * 		2.  如果是大装甲，通过大装甲顶点坐标计算小装甲顶点坐标，将装甲的顶点储存到autohit::new_pts
 * 		3.  获取单映矩阵，透视变换
 *Author:   jason
**********************************/
static void prepare_transform_img( cv::Mat &img_src, struct s_bar *pDomain_bar, const cv::Mat & img_backups, int small_or_big, bool isDrawLine )
{
    get_armor_from_pts( ( *pDomain_bar ).endpoints[0], ( *pDomain_bar ).endpoints[1], ( *pDomain_bar ).pair_p1, ( *pDomain_bar ).pair_p2, ( *pDomain_bar ).armor_vertex, img_backups.rows, img_backups.cols );

    if ( small_or_big == 1 )

        AutoHit::transform_armor_vertex( ( *pDomain_bar ).armor_vertex, 0.25 );

    AutoHit::new_pts[0] = ( *pDomain_bar ).armor_vertex[0];
    AutoHit::new_pts[1] = ( *pDomain_bar ).armor_vertex[1];
    AutoHit::new_pts[2] = ( *pDomain_bar ).armor_vertex[2];
    AutoHit::new_pts[3] = ( *pDomain_bar ).armor_vertex[3];

//    if (isDrawLine)
//        for(int j=0;j<4;j++)
//            cv::line(img_src,autohit::new_pts[j],autohit::new_pts[(j+1)%4],cv::Scalar(0,0,255),1);

    cv::Mat transform = cv::getPerspectiveTransform( AutoHit::new_pts, AutoHit::world_pts );

    cv::warpPerspective( img_backups, AutoHit::img_transform, transform, cv::Size(40,40) );
}

/*********************************
 *
 * Description: 在扩展点中寻找小装甲
 * Input :
 *      1.  cv::Mat &img_src
 *      2.  struct s_bar *pDomain_bar            (valid_bar_list[i]) //需要匹配的灯条，domain_bar：主灯条
 *      3.  const struct s_bar *pSlave_bar       (alid_bar_list[j])  //需要匹配的灯条，slave_bar：从灯条
 *      4.  float distance_limit                            (0.3)    //距离限制
 *      5.  float angle_limit (autohit::small_pair_bar_angle_limit)     //角度限制
 *      6.  cv::Mat & img_ori
 * Output:
 *      1.  result
 * Others:
 * 		1.  调用match_two_bars_by_expts()，对两个灯条进行两两匹配
 * 		2.  获得装甲的透视变换图像 transform_img
 * 		3.  通过SVM模型，检测透视变换图中是否有装甲，如果有装甲则result=1,如果没有装甲则result=0
 * Author:  jason
**********************************/
int find_small_armor_in_expts( cv::Mat &img_src, struct s_bar *pDomain_bar, const struct s_bar *pSlave_bar, float distance_limit, float angle_limit, cv::Mat & img_backups )
{
    int result = 0;
    int match_result = 0;

    match_result=match_two_bars_by_expts( pDomain_bar, pSlave_bar, distance_limit, angle_limit, 0 );

    if ( match_result == 0 )
         return 0;

    /*********** 如果match_result！=0,则说明从灯条与任意主灯条扩展灯条满足约束条件，直接视为匹配成对灯条 *************/

    ( *pDomain_bar ).pair_p1 = ( *pSlave_bar ).endpoints[0];

    ( *pDomain_bar ).pair_p2 = ( *pSlave_bar ).endpoints[1];

    pDomain_bar->isPosePoints = 1;

    pDomain_bar->pose_points[0] = pDomain_bar->endpoints[0];
    pDomain_bar->pose_points[1] = pDomain_bar->pair_p1;
    pDomain_bar->pose_points[2] = pDomain_bar->pair_p2;
    pDomain_bar->pose_points[3] = pDomain_bar->endpoints[1];

    prepare_transform_img( img_src, pDomain_bar, img_backups, 0, false );

    result = AutoHit::predict( AutoHit::model_hog, AutoHit::pHog, AutoHit::img_transform ); //model->predict(t_equ);

    return result;
}

/*********************************
 *
 * Description: 在待提点中寻找小装甲
 * Input:
 *      1.  cv::Mat &img_src
 *      2.  struct s_bar *pDomain_bar    //这里只用主灯条
 *      3.  float area_k
 *      4.  cv::Mat & img_ori
 * Output:
 *      1.  result
 * Others:
 * 		1.  调用match_two_bars_by_propts()函数，返回 check_proposal
 * 		2.  对check_proposal分情况讨论，绘制主灯条对应的匹配灯条
 * 		3.  送入SVM检测是否存在装甲板
 * Author:  jason
**********************************/
int find_small_armor_in_props( cv::Mat &img_src, struct s_bar *pDomain_bar, float area_k, cv::Mat & img_backups )
{
    int result = 0;

    std::vector<int> check_proposal(2);

    check_proposal = match_two_bars_by_propts( ( *pDomain_bar ).prop_small_armor, area_k, img_backups );

    //printf("small_armor match_result is: %d\n",(check_proposal[0]+check_proposal[1]));

    if ( check_proposal[0]+check_proposal[1] == 0 )
        return 0;


    float h = ( *pDomain_bar ).ellipse.size.height;
    float angle = ( *pDomain_bar ).ellipse.angle;
    angle = (180.0-angle) / 180.0 * CV_PI;

    if ( check_proposal[0] == 1 )
    {
        find_newpoint( ( *pDomain_bar ).endpoints[0], angle, -2 * h, ( *pDomain_bar ).pair_p1, img_backups.rows, img_backups.cols );
        find_newpoint( ( *pDomain_bar ).endpoints[1], angle, -2 * h, ( *pDomain_bar ).pair_p2, img_backups.rows, img_backups.cols );
        check_endpoint( ( *pDomain_bar ).pair_p1, ( *pDomain_bar ).pair_p2 );
        pDomain_bar->isPosePoints = 1;

        pDomain_bar->pose_points[0] = pDomain_bar->endpoints[0];
        pDomain_bar->pose_points[1] = pDomain_bar->pair_p1;
        pDomain_bar->pose_points[2] = pDomain_bar->pair_p2;
        pDomain_bar->pose_points[3] = pDomain_bar->endpoints[1];

        prepare_transform_img( img_src, pDomain_bar, img_backups, 0, false );

        result = AutoHit::predict( AutoHit::model_hog, AutoHit::pHog, AutoHit::img_transform );
        if ( result != 0 )
             return result;
    }

    if ( check_proposal[1] == 1 )        //若主灯条右侧待提区域满足要求
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

        result = AutoHit::predict( AutoHit::model_hog, AutoHit::pHog, AutoHit::img_transform );

        if ( result != 0 )
        {
            return result;
        }
    }

    return 0;
}

/*********************************
 *
 * Description: 在扩展点中寻找大装甲
 * Input :
 *      1.  cv::Mat &img_src
 *      2.  struct s_bar *pDomain_bar        主灯条
 *      3.  const struct s_bar *pSlave_bar   从灯条
 *      4.  float distance_limit            距离约束
 *      5.  float angle_limit               角度约束
 *      6.  cv::Mat & img_ori
 * Output:
 *      1.  result 结果为1,则有大装甲板，为0则没有大装甲板
 * Others:
 * 		1.  通过扩展点匹配两个灯条，返回结果为0,匹配不成功
 * 		2.  返回结果！=0,将从灯条作为主灯条的匹配对
 * 		3.  送入SVM分类器，判断是否有大装甲板
 * Author:  jason
**********************************/
int find_large_armor_in_expts( cv::Mat &img_src, struct s_bar *pDomain_bar, const struct s_bar *pSlave_bar, float distance_limit, float angle_limit, cv::Mat & img_backups )
{
    int result = 0;
    int match_result = 0;

    match_result = match_two_bars_by_expts( pDomain_bar, pSlave_bar, distance_limit, angle_limit, 1 );

    if ( match_result == 0 )
        return 0;

    ( *pDomain_bar ).pair_p1 = ( *pSlave_bar ).endpoints[0];
    ( *pDomain_bar ).pair_p2 = ( *pSlave_bar ).endpoints[1];

    pDomain_bar->isPosePoints   = 1;

    pDomain_bar->pose_points[0] = pDomain_bar->endpoints[0];
    pDomain_bar->pose_points[1] = pDomain_bar->pair_p1;
    pDomain_bar->pose_points[2] = pDomain_bar->pair_p2;
    pDomain_bar->pose_points[3] = pDomain_bar->endpoints[1];

    prepare_transform_img( img_src, pDomain_bar, img_backups, 1, false );

    result = AutoHit::predict( AutoHit::model_hog, AutoHit::pHog, AutoHit::img_transform );

    return result;
}

/*********************************
 *
 * Description: 在虚拟点中寻找小装甲
 * Input :
 *      1.  cv::Mat &img_src
 *      2.  struct s_bar *pDomain_bar    //只检测到单个灯条
 *      3.  float distance_k            //距离约束
 *      4.  cv::Mat & img_ori
 * Output:
 *      1.  result num      //返回的数字不为0则说明在虚拟点中找到小装甲，为0则未检测到
 * Others:
 * 		1.  对一个灯条左侧绘制虚拟匹配灯条，送入SVM，检测是否有小装甲
 * 		2.  对一个灯条右侧绘制虚拟匹配灯条，送入SVM，检测是否有小装甲
 * Author:  jason
**********************************/
int find_small_armor_in_virpts( cv::Mat &img_src, struct s_bar *pDomain_bar, float distance_k, cv::Mat & img_backups )
{
    int result=0;

    float h = ( *pDomain_bar ).ellipse.size.height;

    float angle = ( *pDomain_bar ).ellipse.angle;

    float angle_k;

    if ( ( angle < 5 ) || ( angle > 175 ) )
        angle_k = 0.95;
    else if ( ( angle < 10 ) || ( angle > 170 ) )
        angle_k = 0.8;
    else
        angle_k = 0.7;

    angle = (180.0 - angle) / 180.0 * CV_PI;

    float slope_k = std::abs( cos( angle ) ) * angle_k;

    //std::cout<<angle_k<<"  "<<slope_k<<"  "<<distance_k*h<<"  "<<distance_k*h*slope_k<<"  "<<distance_k*h*(1-slope_k)<<std::endl;

    find_newpoint( ( *pDomain_bar ).endpoints[0], angle, -distance_k * h * slope_k, ( *pDomain_bar ).pair_p1, img_backups.rows, img_backups.cols );
    find_newpoint( ( *pDomain_bar ).endpoints[1], angle, -distance_k * h * slope_k, ( *pDomain_bar ).pair_p2, img_backups.rows, img_backups.cols );
    check_endpoint( ( *pDomain_bar ).pair_p1, ( *pDomain_bar ).pair_p2 );


    prepare_transform_img( img_src, pDomain_bar, img_backups, 0, true );

    result = AutoHit::predict( AutoHit::model_hog, AutoHit::pHog, AutoHit::img_transform );
    if ( result != 0 )
        return 1;


    find_newpoint( ( *pDomain_bar ).endpoints[0], angle, distance_k * h * slope_k, ( *pDomain_bar ).pair_p1, img_backups.rows, img_backups.cols );
    find_newpoint( ( *pDomain_bar ).endpoints[1], angle, distance_k * h * slope_k, ( *pDomain_bar ).pair_p2, img_backups.rows, img_backups.cols );
    check_endpoint( ( *pDomain_bar ).pair_p1, ( *pDomain_bar ).pair_p2 );

    prepare_transform_img( img_src, pDomain_bar, img_backups, 0, true );

    result = AutoHit::predict( AutoHit::model_hog, AutoHit::pHog, AutoHit::img_transform);
    if (result != 0)
        return 2;

    return 0;
}

/*********************************
*
* Description: 在虚拟点中寻找大装甲
* Input :
*      1.  cv::Mat &img_src
*      2.  struct s_bar *pDomain_bar    //只检测到单个灯条
*      3.  float distance_k            //距离约束
*      4.  cv::Mat & img_ori
* Output:
*      1.  result num      //返回的数字不为0则说明在虚拟点中找到大装甲，为0则未检测到
* Others:
* 		1. 对一个灯条左侧绘制虚拟匹配灯条，送入SVM，检测是否有大装甲
* 		2. 对一个灯条右侧绘制虚拟匹配灯条，送入SVM，检测是否有大装甲
* Author:  jason
**********************************/
int find_large_armor_in_virpts( cv::Mat &img_src, struct s_bar *pDomain_bar, float distance_k, cv::Mat & img_backups )
{
    int result = 0;

    float h = ( *pDomain_bar ).ellipse.size.height;

    float angle = ( *pDomain_bar ).ellipse.angle;

    angle = (180.0 - angle) / 180.0 * CV_PI;

    find_newpoint( ( *pDomain_bar ).endpoints[0], angle, -distance_k * h, ( *pDomain_bar ).pair_p1, img_backups.rows, img_backups.cols );
    find_newpoint( ( *pDomain_bar ).endpoints[1], angle, -distance_k * h, ( *pDomain_bar ).pair_p2, img_backups.rows, img_backups.cols );
    check_endpoint( ( *pDomain_bar ).pair_p1, ( *pDomain_bar ).pair_p2 );

    prepare_transform_img( img_src, pDomain_bar, img_backups, 1, true );

    result = AutoHit::predict( AutoHit::model_hog, AutoHit::pHog, AutoHit::img_transform );
    if ( result != 0 )
        return 1;

    find_newpoint( ( *pDomain_bar ).endpoints[0], angle, distance_k * h , ( *pDomain_bar ).pair_p1, img_backups.rows, img_backups.cols );
    find_newpoint( ( *pDomain_bar ).endpoints[1], angle, distance_k * h , ( *pDomain_bar ).pair_p2, img_backups.rows, img_backups.cols );
    check_endpoint( ( *pDomain_bar ).pair_p1, ( *pDomain_bar ).pair_p2 );

    prepare_transform_img( img_src, pDomain_bar, img_backups, 1, true );

    result = AutoHit::predict( AutoHit::model_hog, AutoHit::pHog, AutoHit::img_transform );
    if ( result != 0 )
        return 2;

    return 0;
}

/*********************************
 *
 * Description: 在所有点中获得装甲
 * Input:
 *      1.  cv::Mat &img_src   src原始图像
 *      2.  std::vector<struct s_bar> &valid_bar_list   升序排列后的灯条
 *      3.  cv::Mat &img_ori    备份图像
 * Output：
 *      1.  result      determine whether it is armor ,if ture output result!=0 ,otherwise result=0
 * Others：
 * 		1.  通过扩展点检测这一对匹配灯条是否存在小装甲，存在装甲result！0,不存在装甲result为0
 * 		2.  通过待提点检测主灯条左右两侧待提区域是否存在小装甲，存在装甲result！0,不存在装甲result为0
 * 		3.  通过扩展点检测这一对匹配灯条是否存在大装甲，存在装甲result！0,不存在装甲result为0
 *      4.  secondly, we want to find virtual armor according to single bar（根据单个灯条去寻找虚拟虚拟装甲）
 * Author:  jason
**********************************/
int find_armor_int_all_pts( cv::Mat &img_src, std::vector<struct s_bar> &valid_bar_list, cv::Mat &img_backups )
{
    int result = 0;

    for ( int i = 0; i < valid_bar_list.size(); i++ )
    {
        for ( int j = 0; j < valid_bar_list.size(); j++ )
        {
            if( i == j )
                continue;

            result = find_small_armor_in_expts( img_src, &valid_bar_list[i], &valid_bar_list[j], 0.3, AutoHit::small_pair_bar_angle_limit, img_backups );
            if ( result != 0 )
                return i;

            result = find_small_armor_in_props( img_src, &valid_bar_list[i], AutoHit::proposal_bar_area_k, img_backups );
            if ( result != 0 )
                return i;

            result = find_large_armor_in_expts( img_src, &valid_bar_list[i], &valid_bar_list[j], 1.0, AutoHit::big_pair_bar_angle_limit, img_backups );
            if ( result != 0 )
                return i;
        }
    };

    //secondly, we want to find virtual armor accordingt to single bar  需要根据单个灯条去寻找虚拟虚拟装甲

    //sort bar list according to height

    for( int i = 0; i < valid_bar_list.size(); i++ )
    {
        /******************** Find virtual pair for single bar ********************/

        if ( valid_bar_list[i].ellipse.size.height < AutoHit::bar_height_L )
        {
            continue;
        }

        result = find_small_armor_in_virpts( img_src, &valid_bar_list[i], 2.0, img_backups );
        if ( result != 0 )
            return i;

        result = find_large_armor_in_virpts( img_src, &valid_bar_list[i], 4.0, img_backups );
        if ( result != 0 )
            return i;
    }
    return -1;
}

/*********************************
 *
 * Description: 单个灯条寻找装甲
 * Input:
 *      1.  cv::Mat &img_src
 *      2.  std::vector<struct s_bar> &valid_bar_list   //单个灯条信息
 *      3.  cv::Mat & img_ori
 *      4.  cv::Point2f &sp
 * Output：
 *      1.  result  ！0则有装甲，0则没有装甲
 * Others：
 * 		1、  调用ind_small_armor_in_virpts()，获得装甲中心点p
 * 		2、  调用find_large_armor_in_virpts()，获得装甲中心点p
 *
**********************************/
int find_armor_for_single_bar( cv::Mat &img_src, std::vector<struct s_bar> &valid_bar_list, cv::Mat & img_backups, cv::Point2f &sp )
{
    int result = 0;

    std::vector<struct s_bar> new_bar_list;

    sort_bar_according_to_height( valid_bar_list ,new_bar_list );

    for ( int i = 0;i < new_bar_list.size(); i++ )
    {

        /******************** Find virtual pair for single bar ********************/

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

            if (p.x!=-1)
                sp = p;
                return i;
        }

        result = find_large_armor_in_virpts( img_src, &valid_bar_list[i], 4.0, img_backups );
        if ( result != 0 )
        {
            return i;
        }
    }

    sp.x=-1;
    sp.y=-1;
    return 0;
}

