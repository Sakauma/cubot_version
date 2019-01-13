//
// Created by jason on 18-6-5.
//

#include "gold/bar_points.h"
#include "gold/my.h"

/*
 * p00,p01--是一条线段， p10,p11--是一条线段
 * */
/*********************************
 *
 * Description: 输入两条线段的端点，返回两条线段交点的坐标
 * Input:
 *      1.  cv::Point2f &p00     p00,p01--是一条线段
 *      2.  cv::Point2f &p01
 *      3.  cv::Point2f &p10     p10,p11--是一条线段
 *      4.  cv::Point2f &p11
 * Output：
 *      1.  crossPoint          两条线段交点的坐标
 * Others：
 * 		1、  两线段不能为竖直线段
 * 		2、  分别计算两线段的斜率
 * 		3、  根据公式计算相交点坐标
 *
**********************************/
cv::Point2f getCrossPoint(cv::Point2f &p00,cv::Point2f &p01,cv::Point2f &p10,cv::Point2f &p11)
{
    if(p00.x==p01.x)     //线段不能为竖直
        return cv::Point2f(-1,-1);
    if(p10.x==p11.x)     //线段不能为竖直
        return cv::Point2f(-1,-1);

    float k0 = (p00.y-p01.y) / (p00.x-p01.x);       //计算p00-p01线段的斜率
    float k1 = (p10.y-p11.y) / (p10.x-p11.x);       //计算p10-p11线段的斜率

    assert(k0!=k1);

    if(k0==k1)  //两线段斜率不能相同
        return cv::Point2f(-1,-1);

    cv::Point2f crossPoint; //相交点的坐标

    //利用两相交直线求交点的计算公式，带入计算即可
    crossPoint.x = (p10.y-p00.y+k0*p00.x-k1*p10.x) / (k0-k1);
    crossPoint.y = k0*(crossPoint.x-p00.x)+p00.y;

    return crossPoint;
}
/*expand big or small points*/
/*********************************
*
* Description:  获得扩展点
* Input:
*       1.  cv::Point2f*endpoints   椭圆端点
*       2.  float angle1            椭圆旋转角度的补角
*       3.  float angle2            赋值为 0
*       4.  float distance1         椭圆长轴的倍数 2H / 4H
*       5.  float distance2         椭圆长轴的倍数 2.5H / 4H
*       6.  int rows                原图行
*       7.  int cols                原图列
* Output：
*       1.  struct expand_pt * ex_pts   输出的扩展点
* Others：
* 		1.  获得左侧斜向扩展点   存储在  .parallel_l[] 中
* 		2.  获得右侧斜向扩展点   存储在  .parallel_r[] 中
* 		3.  获得左侧横向扩展点   存储在  .horizon_l[]  中
* 		4.  获得右侧横向扩展点   存储在  .horizon_r[]  中
*
**********************************/
void get_expand_pts(cv::Point2f*endpoints , struct expand_pt * ex_pts,float angle1,float angle2,
                    float distance1,float distance2,int rows,int cols)
{
    /************** find left parallel expand **************/  ///扩展点中心距源灯条中心distance1

    //获得左侧上端点斜向扩展点,存储在 (*ex_pts).parallel_l[0] 中       find_newpoint():(自定函数，跳转查看)
    find_newpoint(endpoints[0],angle1,-distance1,(*ex_pts).parallel_l[0],rows,cols);
    //获得左侧下端点斜向扩展点,存储在 (*ex_pts).parallel_l[1] 中       find_newpoint():(自定函数，跳转查看)
    find_newpoint(endpoints[1],angle1,-distance1,(*ex_pts).parallel_l[1],rows,cols);
    //确保左侧斜向扩展点0在左侧斜向扩展点1上方                           check_endpoint():(自定函数，跳转查看)
    check_endpoint((*ex_pts).parallel_l[0],(*ex_pts).parallel_l[1]);


//    std::cout<<"in function endpoint1: "<<endpoints[0].x<<" "<<endpoints[0].y<<std::endl;
//    std::cout<<"in function ex_pts.parallel_l1: "<<(*ex_pts).parallel_l[0].x<<" "<<(*ex_pts).parallel_l[0].y<<std::endl;
//

    /************* find right parallel expand ***************/  ///扩展点中心距源灯条中心distance1

    //获得右侧上端点斜向扩展点,存储在 (*ex_pts).parallel_r[0] 中       find_newpoint():(自定函数，跳转查看)
    find_newpoint(endpoints[0],angle1,distance1,(*ex_pts).parallel_r[0],rows,cols);
    //获得右侧下端点斜向扩展点,存储在 (*ex_pts).parallel_r[1] 中       find_newpoint():(自定函数，跳转查看)
    find_newpoint(endpoints[1],angle1,distance1,(*ex_pts).parallel_r[1],rows,cols);
    //确保右侧斜向扩展点0在右侧斜向扩展点1上方                          check_endpoint():(自定函数，跳转查看)
    check_endpoint((*ex_pts).parallel_r[0],(*ex_pts).parallel_r[1]);


    /***************** find left horizon expand *****************/  ///扩展点中心距源灯条中心distance2

    //获得左侧上端点横向扩展点,存储在 (*ex_pts).horizon_l[0] 中       find_newpoint():(自定函数，跳转查看)
    find_newpoint(endpoints[0],angle2,-distance2,(*ex_pts).horizon_l[0],rows,cols);
    //获得左侧下端点横向扩展点,存储在 (*ex_pts).horizon_l[1] 中       find_newpoint():(自定函数，跳转查看)
    find_newpoint(endpoints[1],angle2,-distance2,(*ex_pts).horizon_l[1],rows,cols);
    //确保左侧横向扩展点0在左侧横向扩展点1上方                         check_endpoint():(自定函数，跳转查看)
    check_endpoint((*ex_pts).horizon_l[0],(*ex_pts).horizon_l[1]);

    /****************** find right horizon expand ****************/  ///扩展点中心距源灯条中心distance2

    //获得右侧上端点横向扩展点,存储在 (*ex_pts).horizon_r[0] 中       find_newpoint():(自定函数，跳转查看)
    find_newpoint(endpoints[0],angle2,distance2,(*ex_pts).horizon_r[0],rows,cols);
    //获得右侧下端点横向扩展点,存储在 (*ex_pts).horizon_r[1] 中       find_newpoint():(自定函数，跳转查看)
    find_newpoint(endpoints[1],angle2,distance2,(*ex_pts).horizon_r[1],rows,cols);
    //确保右侧横向扩展点0在右侧横向扩展点1上方                         check_endpoint():(自定函数，跳转查看)
    check_endpoint((*ex_pts).horizon_r[0],(*ex_pts).horizon_r[1]);
}
/*********************************
 *
 * Description: 通过扩展点匹配两个灯条
 * Input:
 *      1.  const struct s_bar *domain_bar  主灯条
 *      2.  const struct s_bar *slave_bar   从灯条
 *      3.  float distance_limit            距离限制
 *      4.  float angle_limit               角度限制
 *      5.  int small_or_large              大小装甲flag
 * Output：
 *      1.  return num                      返回数字信息
 * Others：
 * 		1、  计算从灯条中心点到主灯条各个扩展中心点的距离
 * 		2、  计算从灯条与主灯条的角度绝对值
 * 		3、  判断距离与角度是否符合约束条件
 *      4、  大装甲小装甲共八种情况，如果符合约束条件，分别返回对应的数字
 *
**********************************/
int match_two_bars_by_expts(const struct s_bar *domain_bar ,const struct s_bar *slave_bar,
                            float distance_limit,float angle_limit,int small_or_large)
{
    float distance,angle_diff;

    //主从灯条的椭圆中心点距离必须大于8                         cal_distance_pt():(自定函数，跳转查看)
    if( cal_distance_pt((*domain_bar).ellipse.center,(*slave_bar).ellipse.center) <=8 )
    {
        printf("distance less than 8\n");
        return 0;
    }

    //大装甲小装甲标志位
    if( small_or_large == 0 )       //0 小装甲
    {

        //计算从灯条的中心点和主灯条的左侧斜向2h扩展灯条中心点的距离，赋给 distance            cal_distance_pt():(自定函数，跳转查看)
        distance = cal_distance_pt((*slave_bar).ellipse.center,((*domain_bar).ex_small_armor.parallel_l[0]+(*domain_bar).ex_small_armor.parallel_l[1])/2.0);

        //计算主灯条与从灯条的角度差的绝对值，赋给 angle_diff
        angle_diff = abs((*domain_bar).ellipse.angle - (*slave_bar).ellipse.angle);

        // printf("small parallel_l distance: %f , angle diff: %f \n",distance/(*domain_bar).ellipse.size.height,angle_diff);

        if( (distance < (*domain_bar).ellipse.size.height*distance_limit)
            &&((angle_diff<angle_limit) || (angle_diff>(180-angle_limit))) )            //符合约束条件,返回1
        {
//            domain_bar->pose_points[0] = domain_bar->endpoints[0];
//            domain_bar->pose_points[1] = slave_bar->endpoints[0];
//            domain_bar->pose_points[2] = slave_bar->endpoints[1];
//            domain_bar->pose_points[3] = domain_bar->endpoints[1];
            return 1;
        }

        //计算从灯条的中心点和主灯条的右侧斜向2h扩展灯条中心点的距离，赋给 distance            cal_distance_pt():(自定函数，跳转查看)
        distance = cal_distance_pt((*slave_bar).ellipse.center,((*domain_bar).ex_small_armor.parallel_r[0]+(*domain_bar).ex_small_armor.parallel_r[1])/2.0);

        //计算主灯条与从灯条的角度差的绝对值，赋给 angle_diff
        angle_diff = abs((*domain_bar).ellipse.angle - (*slave_bar).ellipse.angle);

        //printf("small parallel_r distance: %f , angle diff: %f \n",distance/(*domain_bar).ellipse.size.height,angle_diff);

        if( (distance < (*domain_bar).ellipse.size.height*distance_limit)&&
                ((angle_diff<angle_limit)||(angle_diff>(180-angle_limit))) )            //符合约束条件,返回2
        {
//            domain_bar->pose_points[0] = domain_bar->endpoints[0];
//            domain_bar->pose_points[1] = slave_bar->endpoints[0];
//            domain_bar->pose_points[2] = slave_bar->endpoints[1];
//            domain_bar->pose_points[3] = domain_bar->endpoints[1];
            return 2;
        }

        //计算从灯条的中心点和主灯条的左侧横向2.5h扩展灯条中心点的距离，赋给 distance            cal_distance_pt():(自定函数，跳转查看)
        distance = cal_distance_pt((*slave_bar).ellipse.center,((*domain_bar).ex_small_armor.horizon_l[0]+(*domain_bar).ex_small_armor.horizon_l[1])/2.0);

        //计算主灯条与从灯条的角度差的绝对值，赋给 angle_diff
        angle_diff = abs((*domain_bar).ellipse.angle - (*slave_bar).ellipse.angle);

        //printf("small horizon_l distance: %f , angle diff: %f \n",distance/(*domain_bar).ellipse.size.height,angle_diff);

        if( (distance < (*domain_bar).ellipse.size.height*distance_limit)&&
                ((angle_diff<angle_limit)||(angle_diff>(180-angle_limit))) )             //符合约束条件,返回3
        {
//            domain_bar->pose_points[0] = domain_bar->endpoints[0];
//            domain_bar->pose_points[1] = slave_bar->endpoints[0];
//            domain_bar->pose_points[2] = slave_bar->endpoints[1];
//            domain_bar->pose_points[3] = domain_bar->endpoints[1];
            return 3;
        }

        //计算从灯条的中心点和主灯条的右侧横向2.5h扩展灯条中心点的距离，赋给 distance            cal_distance_pt():(自定函数，跳转查看)
        distance = cal_distance_pt((*slave_bar).ellipse.center,((*domain_bar).ex_small_armor.horizon_r[0]+(*domain_bar).ex_small_armor.horizon_r[1])/2.0);

        //计算主灯条与从灯条的角度差的绝对值，赋给 angle_diff
        angle_diff = abs((*domain_bar).ellipse.angle - (*slave_bar).ellipse.angle);

        // printf("small horizon_r distance: %f , angle diff: %f \n",distance/(*domain_bar).ellipse.size.height,angle_diff);

        if( (distance < (*domain_bar).ellipse.size.height*distance_limit)&&
                ((angle_diff<angle_limit)||(angle_diff>(180-angle_limit))) )             //符合约束条件,返回4
        {
//            domain_bar->pose_points[0] = domain_bar->endpoints[0];
//            domain_bar->pose_points[1] = slave_bar->endpoints[0];
//            domain_bar->pose_points[2] = slave_bar->endpoints[1];
//            domain_bar->pose_points[3] = domain_bar->endpoints[1];
            return 4;
        }
    }

    else    //if(small_or_large == 1) 1大装甲
    {
        /*************************** Hero Armor ***************************/

        //计算从灯条的中心点和主灯条的左侧斜向4h扩展灯条中心点的距离，赋给 distance            cal_distance_pt():(自定函数，跳转查看)
        distance = cal_distance_pt((*slave_bar).ellipse.center,((*domain_bar).ex_large_armor.parallel_l[0]+(*domain_bar).ex_large_armor.parallel_l[1])/2.0);

        //计算主灯条与从灯条的角度差的绝对值，赋给 angle_diff
        angle_diff = abs((*domain_bar).ellipse.angle - (*slave_bar).ellipse.angle);

        //printf("big parallel_l distance: %f , angle diff: %f \n",distance/(*domain_bar).ellipse.size.height,angle_diff);

        if( (distance < (*domain_bar).ellipse.size.height*distance_limit)&&
                ((angle_diff<angle_limit)||(angle_diff>(180-angle_limit))) )             //符合约束条件,返回5
        {
//            domain_bar->pose_points[0] = domain_bar->endpoints[0];
//            domain_bar->pose_points[1] = slave_bar->endpoints[0];
//            domain_bar->pose_points[2] = slave_bar->endpoints[1];
//            domain_bar->pose_points[3] = domain_bar->endpoints[1];
            return 5;
        }

        //计算从灯条的中心点和主灯条的右侧斜向4h扩展灯条中心点的距离，赋给 distance            cal_distance_pt():(自定函数，跳转查看)
        distance = cal_distance_pt((*slave_bar).ellipse.center,((*domain_bar).ex_large_armor.parallel_r[0]+(*domain_bar).ex_large_armor.parallel_r[1])/2.0);

        //计算主灯条与从灯条的角度差的绝对值，赋给 angle_diff
        angle_diff = abs((*domain_bar).ellipse.angle - (*slave_bar).ellipse.angle);

        // printf("big parallel_r distance: %f , angle diff: %f \n",distance/(*domain_bar).ellipse.size.height,angle_diff);

        if( (distance < (*domain_bar).ellipse.size.height*distance_limit)&&
                ((angle_diff<angle_limit)||(angle_diff>(180-angle_limit))) )             //符合约束条件,返回6
        {
//            domain_bar->pose_points[0] = domain_bar->endpoints[0];
//            domain_bar->pose_points[1] = slave_bar->endpoints[0];
//            domain_bar->pose_points[2] = slave_bar->endpoints[1];
//            domain_bar->pose_points[3] = domain_bar->endpoints[1];
            return 6;
        }

        //计算从灯条的中心点和主灯条的左侧横向4h扩展灯条中心点的距离，赋给 distance            cal_distance_pt():(自定函数，跳转查看)
        distance = cal_distance_pt((*slave_bar).ellipse.center,((*domain_bar).ex_large_armor.horizon_l[0]+(*domain_bar).ex_large_armor.horizon_l[1])/2.0);

        //计算主灯条与从灯条的角度差的绝对值，赋给 angle_diff
        angle_diff = abs((*domain_bar).ellipse.angle - (*slave_bar).ellipse.angle);

        //printf("big horizon_l distance: %f , angle diff: %f \n",distance/(*domain_bar).ellipse.size.height,angle_diff);

        if( (distance < (*domain_bar).ellipse.size.height*distance_limit)&&
                ((angle_diff<angle_limit)||(angle_diff>(180-angle_limit))) )            //符合约束条件,返回7
        {
//            domain_bar->pose_points[0] = domain_bar->endpoints[0];
//            domain_bar->pose_points[1] = slave_bar->endpoints[0];
//            domain_bar->pose_points[2] = slave_bar->endpoints[1];
//            domain_bar->pose_points[3] = domain_bar->endpoints[1];
            return 7;
        }

        //计算从灯条的中心点和主灯条的右侧横向4h扩展灯条中心点的距离，赋给 distance            cal_distance_pt():(自定函数，跳转查看)
        distance = cal_distance_pt((*slave_bar).ellipse.center,((*domain_bar).ex_large_armor.horizon_r[0]+(*domain_bar).ex_large_armor.horizon_r[1])/2.0);

        //计算主灯条与从灯条的角度差的绝对值，赋给 angle_diff
        angle_diff = abs((*domain_bar).ellipse.angle - (*slave_bar).ellipse.angle);

        // printf("big horizon_r distance: %f , angle diff: %f \n",distance/(*domain_bar).ellipse.size.height,angle_diff);

        if( (distance < (*domain_bar).ellipse.size.height*distance_limit)&&
                ((angle_diff<angle_limit)||(angle_diff>(180-angle_limit))) )            //符合约束条件,返回8
        {
//            domain_bar->pose_points[0] = domain_bar->endpoints[0];
//            domain_bar->pose_points[1] = slave_bar->endpoints[0];
//            domain_bar->pose_points[2] = slave_bar->endpoints[1];
//            domain_bar->pose_points[3] = domain_bar->endpoints[1];
            return 8;
        }
    }


    return 0;
}
/*propose big or small points*/
/*********************************
*
* Description:  获得待提点
* Input:
*       1.  cv::Point2f*endpoints   椭圆端点
*       2.  float angle1            椭圆旋转角度的补角
*       4.  float distance_l        椭圆长轴的倍数 1h
*       5.  float distance_h        椭圆长轴的倍数 2.5h
*       6.  int rows                原图行
*       7.  int cols                原图列
* Output：
*       1.  struct proposal_pt *prop_pts   输出的待提点
* Others：
* 		1.  在左侧 distance_l 位置处获得斜向待提点
* 		2.  在左侧 distance_h 位置处获得斜向待提点
* 		3.  在右侧 distance_l 位置处获得斜向待提点
* 		4.  在右侧 distance_l 位置处获得斜向待提点
*
**********************************/
void get_proposal_pts(cv::Point2f *endpoints,float angle,float distance_l,float distance_h,struct proposal_pt *prop_pts,int rows,int cols)
{
    /************** find left parallel proposal **************/    ///扩展点中心距源灯条中心distance_l
    //获得椭圆上端点的左侧斜向待提点，存储在(*prop_pts).parallel_l[0]              find_newpoint():(自定函数，跳转查看)
    find_newpoint(endpoints[0],angle,-distance_l,(*prop_pts).parallel_l[0],rows,cols);
    //获得椭圆下端点的左侧斜向待提点，存储在(*prop_pts).parallel_l[1]              find_newpoint():(自定函数，跳转查看)
    find_newpoint(endpoints[1],angle,-distance_l,(*prop_pts).parallel_l[1],rows,cols);

    /************** find left parallel proposal **************/    ///扩展点中心距源灯条中心distance_h
    //获得椭圆上端点的左侧斜向待提点，存储在(*prop_pts).parallel_l[2]               find_newpoint():(自定函数，跳转查看)
    find_newpoint(endpoints[0],angle,-distance_h,(*prop_pts).parallel_l[2],rows,cols);
    //获得椭圆下端点的左侧斜向待提点，存储在(*prop_pts).parallel_l[3]              find_newpoint():(自定函数，跳转查看)
    find_newpoint(endpoints[1],angle,-distance_h,(*prop_pts).parallel_l[3],rows,cols);

    /************** find right parallel proposal **************/     ///扩展点中心距源灯条中心distance_l
    //获得椭圆上端点的右侧斜向待提点，存储在(*prop_pts).parallel_r[0]              find_newpoint():(自定函数，跳转查看)
    find_newpoint(endpoints[0],angle,distance_l,(*prop_pts).parallel_r[0],rows,cols);
    //获得椭圆上端点的右侧斜向待提点，存储在(*prop_pts).parallel_r[1]              find_newpoint():(自定函数，跳转查看)
    find_newpoint(endpoints[1],angle,distance_l,(*prop_pts).parallel_r[1],rows,cols);

    /************** find right parallel proposal **************/     ///扩展点中心距源灯条中心distance_h
    //获得椭圆上端点的右侧斜向待提点，存储在(*prop_pts).parallel_r[2]              find_newpoint():(自定函数，跳转查看)
    find_newpoint(endpoints[0],angle,distance_h,(*prop_pts).parallel_r[2],rows,cols);
    //获得椭圆上端点的右侧斜向待提点，存储在(*prop_pts).parallel_r[3]              find_newpoint():(自定函数，跳转查看)
    find_newpoint(endpoints[1],angle,distance_h,(*prop_pts).parallel_r[3],rows,cols);
}
/*********************************
 *
 * Description: 通过待提点匹配两个灯条
 * Input:
 *      1.  struct proposal_pt prop_pts     输入主灯条的小装甲待提点
 *      2.  float area_k                    灯条面积约束条件
 *      3.  cv::Mat &img_ori
 * Output：
 *      1.  result                          返回结果，1则匹配成功，0则匹配失败
 * Others：
 * 		1、  对四个待提点做外部矩形边界，即对主灯条左右两侧各延伸一个矩形检测框
 * 		2、  计算目标颜色占检测框面积的比例，判断是否大于area_k = 0.05，大于则返回result=1,否则返回0
 *
**********************************/
std::vector<int> match_two_bars_by_propts(struct proposal_pt prop_pts,float area_k,cv::Mat &img_ori)
{
    std::vector<int> result(2);

    result[0]=0;    //初始化置零
    result[1]=0;

    //proposal_vec中存储着主灯条的左侧四个待提点
    std::vector<cv::Point2f> proposal_vec(std::begin(prop_pts.parallel_l), std::end(prop_pts.parallel_l));

    //对四个待提点做外部矩形边界，即对主灯条左侧延伸一个矩形检测框，存储在 proposal 中
    cv::Rect proposal = cv::boundingRect(proposal_vec);

    //确保proposal在原图像内           check_rect():(自定函数，跳转查看)
    if(check_rect(proposal,img_ori.rows,img_ori.cols))/*check if the proposal is in image size range*/
    {
        //计算矩形检测框面积，赋给 count_area
        float count_area = cv::contourArea(proposal_vec);

        //目标颜色占检测框面积的比例，判断是否大于area_k = 0.05
        if( check_mask_area(img_ori,proposal)/count_area > area_k)

            result[0]=1;    //返回result = 1
    }

    //proposal_vec中存储着主灯条的右侧四个待提点
    proposal_vec= std::vector<cv::Point2f> (std::begin(prop_pts.parallel_r), std::end(prop_pts.parallel_r));

    //对四个待提点做外部矩形边界
    proposal = cv::boundingRect(proposal_vec);

    //确保proposal在原图像内           check_rect():(自定函数，跳转查看)
    if(check_rect(proposal,img_ori.rows,img_ori.cols))/*check if the proposal is valid*/
    {
        //计算轮廓面积，赋给 count_area
        float count_area = cv::contourArea(proposal_vec);

        //目标颜色占检测框面积的比例，判断是否大于area_k = 0.05
        if( check_mask_area(img_ori,proposal)/count_area > area_k)

            result[1]=1;    //返回result = 1
    }

    return result;  //均未进入if语句，返回result = 0
}
/*********************************
*
* Description:  Get   expand pts;                //小装甲扩展点
 *                  proposal pts;                //小装甲待提点
 *                  large expand pts             //大装甲扩装点
* Input:
*       1.  bar_list (valid_bar_list) 输入升序后的灯条
*       2.  img_ori                   原图
* Output：
*       1.  bar_list[i].ex_small_armor      //小装甲扩展点
*       2.  bar_list[i].prop_small_armor    //小装甲待提点
*       3.  bar_list[i].ex_large_armor      //大装甲扩装点
* Others：
* 		1、 调用 get_expand_pts() 函数
* 		2、 调用 get_proposal_pts() 函数
* 		3、 调用 get_expand_pts() 函数
*
**********************************/
void get_surround_pts(std::vector<struct s_bar> &bar_list, const cv::Mat img_ori)
{
    //遍历灯条
    for( int i=0;i<bar_list.size();i++ )
    {
        //灯条最小外接椭圆长轴
        float h = bar_list[i].ellipse.size.height;
        //灯条最小外接椭圆旋转角度
        float angle = bar_list[i].ellipse.angle;
        //旋转角度的补角
        angle = (180.0-angle)/180.0*3.1415926;

        /*find bar's samll expanded parallel and horizon expand pts*/

        //获得小装甲的2h斜向扩展点和2.5h的横向扩展点,结果存储在 bar_list[i].ex_small_armor 中       get_expand_pts():自定函数，跳转查看
        get_expand_pts(bar_list[i].endpoints,&bar_list[i].ex_small_armor,angle,0,2*h,2.5*h,img_ori.rows,img_ori.cols);

        //获得小装甲的1h斜向待提点和2.5h的斜向待提点,结果存储在 bar_list[i].prop_small_armor 中     get_proposal_pts():自定函数，跳转查看
        get_proposal_pts(bar_list[i].endpoints,angle,1*h,2.5*h,&bar_list[i].prop_small_armor,img_ori.rows,img_ori.cols);

        //获得大装甲的4h斜向扩展点和4h的横向扩展点,结果存储在 bar_list[i].ex_large_armor 中         get_expand_pts():自定函数，跳转查看
        get_expand_pts(bar_list[i].endpoints,&bar_list[i].ex_large_armor,angle,0,4*h,4.0*h,img_ori.rows,img_ori.cols);

        //get virtual
    }
}
/*********************************
 *
 * Description: 得到透视变换图
 * Input:
 *      1.  cv::Mat &src_img
 *      2.  struct s_bar *domain_bar    //主灯条
 *      3.  const cv::Mat & img_ori
 *      4.  int small_or_big            //大小装甲flag
 *      5.  bool isDrawLine             //是否绘制直线
 * Output：
 *      1.  得到透视变换图像
 * Others：
 * 		1、  根据匹配到的一对灯条端点，计算出这对灯条对应的装甲板的四个顶点坐标
 * 		2、  如果是大装甲，通过大装甲顶点坐标计算小装甲顶点坐标，将装甲的顶点储存到meng::new_pts
 * 		3、  获取单映矩阵，透视变换
 *
**********************************/
static void prepare_transform_img(cv::Mat &src_img, struct s_bar *domain_bar,const cv::Mat & img_ori , int small_or_big, bool isDrawLine)
{
    //根据匹配到的一对灯条端点，计算出这对灯条对应的装甲板的四个顶点坐标，存放至(*domain_bar).armor_vertex           get_armor_from_pts():(自定函数，跳转查看)
    get_armor_from_pts((*domain_bar).endpoints[0],(*domain_bar).endpoints[1],(*domain_bar).pair_p1,(*domain_bar).pair_p2,(*domain_bar).armor_vertex,img_ori.rows,img_ori.cols);

    //如果是大装甲 flag = 1
    if( small_or_big == 1 )

        //通过大装甲顶点坐标计算小装甲顶点坐标            meng::transform_armor_vertex():(自定函数，跳转查看)
        meng::transform_armor_vertex((*domain_bar).armor_vertex,0.25);

    //将装甲的顶点储存到meng::new_pts
    meng::new_pts[0]=(*domain_bar).armor_vertex[0];
    meng::new_pts[1]=(*domain_bar).armor_vertex[1];
    meng::new_pts[2]=(*domain_bar).armor_vertex[2];
    meng::new_pts[3]=(*domain_bar).armor_vertex[3];

//    if (isDrawLine)
//        for(int j=0;j<4;j++)
//            cv::line(src_img,meng::new_pts[j],meng::new_pts[(j+1)%4],cv::Scalar(0,0,255),1);

    //获取单映矩阵，存储至 transform 中
    cv::Mat transform = cv::getPerspectiveTransform(meng::new_pts,meng::world_pts);

    //透视变换,存储至 transform_img，尺寸为(40,40)
    cv::warpPerspective(img_ori,meng::transform_img,transform,cv::Size(40,40));
}
/*********************************
 *
 * Description: 在扩展点中寻找小装甲
 * Input:
 *      1.  cv::Mat &src_img
 *      2.  struct s_bar *domain_bar            (valid_bar_list[i])//需要匹配的灯条，domain_bar：主灯条
 *      3.  const struct s_bar *slave_bar       (alid_bar_list[j])//需要匹配的灯条，slave_bar：从灯条
 *      4.  float distance_limit                            (0.3)//距离限制
 *      5.  float angle_limit (meng::small_pair_bar_angle_limit)//角度限制
 *      6.  cv::Mat & img_ori
 * Output：
 *      1.  result
 * Others：
 * 		1、  调用match_two_bars_by_expts()，对两个灯条进行两两匹配
 * 		2、  获得装甲的透视变换图像 transform_img
 * 		3、  通过SVM模型，检测透视变换图中是否有装甲，如果有装甲则result=1,如果没有装甲则result=0
 *
**********************************/
int find_small_armor_in_expts(cv::Mat &src_img, struct s_bar *domain_bar ,const struct s_bar *slave_bar,float distance_limit,float angle_limit, cv::Mat & img_ori)
{
    int result = 0; //返回值flag
    int match_result=0; //匹配成功flag

    //匹配从灯条和主灯条的扩展灯条，不同情况返回不同的值           match_two_bars_by_expts（）：（自定函数，跳转查看）
    match_result=match_two_bars_by_expts(domain_bar,slave_bar,distance_limit,angle_limit,0);

    //printf("small expts match result: %d \n",match_result);
    if( match_result == 0 )
        return 0;

    /*********** 如果match_result！=0,则说明从灯条与任意主灯条扩展灯条满足约束条件，直接视为匹配成对灯条 *************/

    //将从灯条的上顶点赋给主灯条的匹配上顶点
    (*domain_bar).pair_p1 = (*slave_bar).endpoints[0];

    //将从灯条的下顶点赋给主灯条的匹配下顶点
    (*domain_bar).pair_p2 = (*slave_bar).endpoints[1];

    //将是否待提点flag置为1
    domain_bar->isPosePoints   = 1;

    //顺时针将主从灯条顶点信息放入domain_bar->pose_points[]，主灯条的顶点为pose_points[0],pose_points[3]  从灯条的端点为pose_points[1],pose_points[2]
    domain_bar->pose_points[0] = domain_bar->endpoints[0];
    domain_bar->pose_points[1] = domain_bar->pair_p1;
    domain_bar->pose_points[2] = domain_bar->pair_p2;
    domain_bar->pose_points[3] = domain_bar->endpoints[1];

    //获得装甲的透视变换图像 transform_img
    prepare_transform_img(src_img,domain_bar,img_ori,0, false);

    //输入hog模式，hog特征，装甲的透视变换图
    //通过SVM模型，检测透视变换图中是否有装甲，如果有装甲则result=1,如果没有装甲则result=0           meng::predict():(自定函数，跳转查看)
    result = meng::predict(meng::model_hog,meng::hog,meng::transform_img); //model->predict(t_equ);

    return result;
}
/*********************************
 *
 * Description: 在待提点中寻找小装甲
 * Input:
 *      1.  cv::Mat &src_img
 *      2.  struct s_bar *domain_bar    //这里只用主灯条
 *      3.  float area_k
 *      4.  cv::Mat & img_ori
 * Output：
 *      1.  result
 * Others：
 * 		1、  调用match_two_bars_by_propts()函数，返回 check_proposal
 * 		2、  对check_proposal分情况讨论，绘制主灯条对应的匹配灯条
 * 		3、  送入SVM检测是否存在装甲板
 *
**********************************/
int find_small_armor_in_props(cv::Mat &src_img, struct s_bar *domain_bar ,float area_k, cv::Mat & img_ori)
{
    int result = 0; //flag

    std::vector<int> check_proposal(2);

    //检测主灯条两侧的待提点扩展的检测区域内目标颜色占检测区域面积的比值是否大于规定阈值要求，返回result=1满足，result=0不满足          match_two_bars_by_propts():(自定函数，跳转查看)
    check_proposal = match_two_bars_by_propts((*domain_bar).prop_small_armor,area_k,img_ori);

    //printf("small_armor match_result is: %d\n",(check_proposal[0]+check_proposal[1]));

    if( check_proposal[0]+check_proposal[1] == 0 )  //主灯条的左右两侧都不满足，返回0
        return 0;


    float h = (*domain_bar).ellipse.size.height;    //主灯条外接椭圆的长轴，赋给h
    float angle = (*domain_bar).ellipse.angle;      //主灯条外接椭圆的旋转角度，赋给angle
    angle = (180.0-angle)/180.0*CV_PI;              //旋转角度的补角

    if( check_proposal[0] == 1 )        //若主灯条左侧待提区域满足要求
    {
        //主灯条右侧上端点2h处绘制一个灯条，作为与主灯条匹配的灯条上端点，存储在 (*domain_bar).pair_p1 中
        find_newpoint((*domain_bar).endpoints[0],angle,-2*h,(*domain_bar).pair_p1,img_ori.rows,img_ori.cols);
        //主灯条右侧下端点2h处绘制一个灯条，作为与主灯条匹配的灯条上端点，存储在 (*domain_bar).pair_p2 中
        find_newpoint((*domain_bar).endpoints[1],angle,-2*h,(*domain_bar).pair_p2,img_ori.rows,img_ori.cols);
        //确保上端点在下端点之上
        check_endpoint((*domain_bar).pair_p1,(*domain_bar).pair_p2);
        //将是否待提点flag置为1
        domain_bar->isPosePoints   = 1;

        //顺时针将主从灯条顶点信息放入domain_bar->pose_points[]，主灯条的顶点为pose_points[0],pose_points[3]  从灯条的端点为pose_points[1],pose_points[2]
        domain_bar->pose_points[0] = domain_bar->endpoints[0];
        domain_bar->pose_points[1] = domain_bar->pair_p1;
        domain_bar->pose_points[2] = domain_bar->pair_p2;
        domain_bar->pose_points[3] = domain_bar->endpoints[1];

        //获得装甲的透视变换图像 transform_img
        prepare_transform_img(src_img, domain_bar,img_ori,0, false);

        //输入hog模式，hog特征，装甲的透视变换图
        //通过SVM模型，检测透视变换图中是否有装甲，如果有装甲则result=1,如果没有装甲则result=0           meng::predict():(自定函数，跳转查看)
        result = meng::predict(meng::model_hog,meng::hog,meng::transform_img); //model->predict(t_equ);
        if(  result != 0)
            return result;
    }

    /*  Do not use "else" !!!!
     *  Because we want to search each side of the bar
     *
     * */
    if( check_proposal[1] == 1 )        //若主灯条右侧待提区域满足要求
    {
        //主灯条左侧上端点2h处绘制一个灯条，作为与主灯条匹配的灯条上端点，存储在 (*domain_bar).pair_p1 中
        find_newpoint((*domain_bar).endpoints[0],angle,2*h,(*domain_bar).pair_p1,img_ori.rows,img_ori.cols);
        //主灯条左侧下端点2h处绘制一个灯条，作为与主灯条匹配的灯条上端点，存储在 (*domain_bar).pair_p2 中
        find_newpoint((*domain_bar).endpoints[1],angle,2*h,(*domain_bar).pair_p2,img_ori.rows,img_ori.cols);
        //确保上端点在下端点之上
        check_endpoint((*domain_bar).pair_p1,(*domain_bar).pair_p2);

        //将是否待提点flag置为1
        domain_bar->isPosePoints   = 1;

        //顺时针将主从灯条顶点信息放入domain_bar->pose_points[]，主灯条的顶点为pose_points[0],pose_points[3]  从灯条的端点为pose_points[1],pose_points[2]
        domain_bar->pose_points[0] = domain_bar->endpoints[0];
        domain_bar->pose_points[1] = domain_bar->pair_p1;
        domain_bar->pose_points[2] = domain_bar->pair_p2;
        domain_bar->pose_points[3] = domain_bar->endpoints[1];

        //获得装甲的透视变换图像 transform_img
        prepare_transform_img(src_img, domain_bar,img_ori,0,false);

        //输入hog模式，hog特征，装甲的透视变换图
        //通过SVM模型，检测透视变换图中是否有装甲，如果有装甲则result=1,如果没有装甲则result=0           meng::predict():(自定函数，跳转查看)
        result = meng::predict(meng::model_hog,meng::hog,meng::transform_img); //model->predict(t_equ);

        if(  result != 0)
        {
            return result;
        }
    }

    return 0;
}
/*********************************
 *
 * Description: 在扩展点中寻找大装甲
 * Input:
 *      1.  cv::Mat &src_img
 *      2.  struct s_bar *domain_bar        主灯条
 *      3.  const struct s_bar *slave_bar   从灯条
 *      4.  float distance_limit            距离约束
 *      5.  float angle_limit               角度约束
 *      6.  cv::Mat & img_ori
 * Output：
 *      1.  result 结果为1,则有大装甲板，为0则没有大装甲板
 * Others：
 * 		1、  通过扩展点匹配两个灯条，返回结果为0,匹配不成功
 * 		2、  返回结果！=0,将从灯条作为主灯条的匹配对
 * 		3、  送入SVM分类器，判断是否有大装甲板
 *
**********************************/
int find_large_armor_in_expts(cv::Mat &src_img, struct s_bar *domain_bar ,const struct s_bar *slave_bar,float distance_limit,float angle_limit, cv::Mat & img_ori)
{
    int result = 0;
    int match_result=0;

    //通过扩展点匹配两个灯条，结果存储在match_result 中           match_two_bars_by_expts():(自定函数，跳转查看)
    match_result=match_two_bars_by_expts(domain_bar,slave_bar,distance_limit,angle_limit,1);

    //printf("large_armor match_result is: %d \n" ,match_result );

    if( match_result == 0 )         //未匹配成功，返回0
        return 0;

    //从灯条成为主灯条的匹配对
    (*domain_bar).pair_p1 = (*slave_bar).endpoints[0];
    (*domain_bar).pair_p2 = (*slave_bar).endpoints[1];

    //将是否待提点flag置为1
    domain_bar->isPosePoints   = 1;

    //顺时针将主从灯条顶点信息放入domain_bar->pose_points[]，主灯条的顶点为pose_points[0],pose_points[3]  从灯条的端点为pose_points[1],pose_points[2]
    domain_bar->pose_points[0] = domain_bar->endpoints[0];
    domain_bar->pose_points[1] = domain_bar->pair_p1;
    domain_bar->pose_points[2] = domain_bar->pair_p2;
    domain_bar->pose_points[3] = domain_bar->endpoints[1];

    //获得装甲的透视变换图像 transform_img
    prepare_transform_img(src_img, domain_bar,img_ori,1,false);

    //输入hog模式，hog特征，装甲的透视变换图
    //通过SVM模型，检测透视变换图中是否有装甲，如果有装甲则result=1,如果没有装甲则result=0           meng::predict():(自定函数，跳转查看)
    result = meng::predict(meng::model_hog,meng::hog,meng::transform_img); //model->predict(t_equ);

    return result;
}
/*********************************
 *
 * Description: 在虚拟点中寻找小装甲
 * Input:
 *      1.  cv::Mat &src_img
 *      2.  struct s_bar *domain_bar    //只检测到单个灯条
 *      3.  float distance_k            //距离约束
 *      4.  cv::Mat & img_ori
 * Output：
 *      1.  result num      //返回的数字不为0则说明在虚拟点中找到小装甲，为0则未检测到
 * Others：
 * 		1、  对一个灯条左侧绘制虚拟匹配灯条，送入SVM，检测是否有小装甲
 * 		2、  对一个灯条右侧绘制虚拟匹配灯条，送入SVM，检测是否有小装甲
 *
**********************************/
int find_small_armor_in_virpts(cv::Mat &src_img, struct s_bar *domain_bar,float distance_k, cv::Mat & img_ori)
{
    int result=0;

    float h = (*domain_bar).ellipse.size.height;     //灯条最小外接椭圆长轴

    float angle = (*domain_bar).ellipse.angle;       //灯条最小外接椭圆旋转角度

    float angle_k;      //角度系数

    if ((angle < 5) || (angle >175))
        angle_k = 0.95;                      //灯条很直
    else if ((angle < 10) || (angle >170))
        angle_k = 0.8;                       //灯条比较直
    else
        angle_k = 0.7;                       //灯条不太直

    angle = (180.0-angle)/180.0*CV_PI;       //旋转角度的补角

    float slope_k = std::abs(cos(angle))*angle_k;   //距离系数

    //std::cout<<angle_k<<"  "<<slope_k<<"  "<<distance_k*h<<"  "<<distance_k*h*slope_k<<"  "<<distance_k*h*(1-slope_k)<<std::endl;

    //左侧 2h×slope_k 绘制虚拟匹配灯条
    find_newpoint((*domain_bar).endpoints[0],angle,-distance_k*h*slope_k,(*domain_bar).pair_p1,img_ori.rows,img_ori.cols);
    find_newpoint((*domain_bar).endpoints[1],angle,-distance_k*h*slope_k,(*domain_bar).pair_p2,img_ori.rows,img_ori.cols);
    check_endpoint((*domain_bar).pair_p1,(*domain_bar).pair_p2);

    //透视变换
    prepare_transform_img(src_img, domain_bar,img_ori,0, true);
    //送入SVM，检测是否有装甲
    result = meng::predict(meng::model_hog,meng::hog,meng::transform_img); //model->predict(t_equ);
    if(  result != 0)    //左侧虚拟匹配灯条有装甲则返回1
        return 1;

    //右侧 2h×slope_k 绘制虚拟匹配灯条
    find_newpoint((*domain_bar).endpoints[0],angle,distance_k*h*slope_k,(*domain_bar).pair_p1,img_ori.rows,img_ori.cols);
    find_newpoint((*domain_bar).endpoints[1],angle,distance_k*h*slope_k,(*domain_bar).pair_p2,img_ori.rows,img_ori.cols);
    check_endpoint((*domain_bar).pair_p1,(*domain_bar).pair_p2);

    //透视变换
    prepare_transform_img(src_img,domain_bar,img_ori,0, true);
    //送入SVM，检测是否有装甲
    result = meng::predict(meng::model_hog,meng::hog,meng::transform_img); //model->predict(t_equ);
    if(result != 0)     //右侧虚拟匹配灯条有装甲则返回2
        return 2;

    return 0;
}
/*********************************
*
* Description: 在虚拟点中寻找大装甲
* Input:
*      1.  cv::Mat &src_img
*      2.  struct s_bar *domain_bar    //只检测到单个灯条
*      3.  float distance_k            //距离约束
*      4.  cv::Mat & img_ori
* Output：
*      1.  result num      //返回的数字不为0则说明在虚拟点中找到大装甲，为0则未检测到
* Others：
* 		1、  对一个灯条左侧绘制虚拟匹配灯条，送入SVM，检测是否有大装甲
* 		2、  对一个灯条右侧绘制虚拟匹配灯条，送入SVM，检测是否有大装甲
*
**********************************/
int find_large_armor_in_virpts(cv::Mat &src_img, struct s_bar *domain_bar,float distance_k, cv::Mat & img_ori)
{
    int result=0;

    float h = (*domain_bar).ellipse.size.height;        //灯条最小外接椭圆长轴

    float angle = (*domain_bar).ellipse.angle;          //灯条最小外接椭圆旋转角度

    angle = (180.0-angle)/180.0*3.1415926;               //旋转角度的补角

    //左侧 4h 绘制虚拟匹配灯条
    find_newpoint((*domain_bar).endpoints[0],angle,-distance_k*h,(*domain_bar).pair_p1,img_ori.rows,img_ori.cols);
    find_newpoint((*domain_bar).endpoints[1],angle,-distance_k*h,(*domain_bar).pair_p2,img_ori.rows,img_ori.cols);
    check_endpoint((*domain_bar).pair_p1,(*domain_bar).pair_p2);

    //透视变换
    prepare_transform_img(src_img, domain_bar,img_ori,1, true);
    //送入SVM，检测是否有装甲
    result = meng::predict(meng::model_hog,meng::hog,meng::transform_img); //model->predict(t_equ);
    if(  result != 0)           //左侧虚拟匹配灯条有装甲则返回1
        return 1;

    //右侧 4h 绘制虚拟匹配灯条
    find_newpoint((*domain_bar).endpoints[0],angle,distance_k*h,(*domain_bar).pair_p1,img_ori.rows,img_ori.cols);
    find_newpoint((*domain_bar).endpoints[1],angle,distance_k*h,(*domain_bar).pair_p2,img_ori.rows,img_ori.cols);
    check_endpoint((*domain_bar).pair_p1,(*domain_bar).pair_p2);

    //透视变换
    prepare_transform_img(src_img, domain_bar,img_ori,1,true);
    //送入SVM，检测是否有装甲
    result = meng::predict(meng::model_hog,meng::hog,meng::transform_img); //model->predict(t_equ);
    if(  result != 0)           //右侧虚拟匹配灯条有装甲则返回2
        return 2;

    return 0;
}
/*********************************
 *
 * Description: 在所有点中获得装甲
 * Input:
 *      1.  cv::Mat &src_img    src原始图像
 *      2.  std::vector<struct s_bar> &valid_bar_list   升序排列后的灯条
 *      3.  cv::Mat &img_ori    备份图像
 * Output：
 *      1.  result      determine whether it is armor ,if ture output result!=0 ,otherwise result=0
 * Others：
 * 		1、  通过扩展点检测这一对匹配灯条是否存在小装甲，存在装甲result！0,不存在装甲result为0
 * 		2、  通过待提点检测主灯条左右两侧待提区域是否存在小装甲，存在装甲result！0,不存在装甲result为0
 * 		3、  通过扩展点检测这一对匹配灯条是否存在大装甲，存在装甲result！0,不存在装甲result为0
 *      4、  secondly, we want to find virtual armor according to single bar（根据单个灯条去寻找虚拟虚拟装甲）
 *
**********************************/
int find_armor_int_all_pts(cv::Mat &src_img, std::vector<struct s_bar> &valid_bar_list,cv::Mat &img_ori)
{
    int result = 0; //flag

    /*
     * Firstly we go through the bar list, and find bar pairs for small armor   首先遍历所有灯条，寻找成对小装甲
     * */
    for( int i=0;i<valid_bar_list.size();i++ )
    {
        for(int j=0;j<valid_bar_list.size();j++)
        {
            if( i==j)
                continue;

            //通过扩展点检测这一对匹配灯条是否存在小装甲，存在装甲result！0,不存在装甲result为0
            result = find_small_armor_in_expts(src_img, &valid_bar_list[i],&valid_bar_list[j],0.3,meng::small_pair_bar_angle_limit, img_ori);
            //printf("small expts: %d \n" ,result );
            if( result != 0 )
                return i;

            //通过待提点检测主灯条左右两侧待提区域是否存在小装甲，存在装甲result！0,不存在装甲result为0
            result = find_small_armor_in_props(src_img, &valid_bar_list[i],meng::proposal_bar_area_k, img_ori);
            //printf("small props: %d \n" ,result );
            if( result != 0 )
                return i;

            //通过扩展点检测这一对匹配灯条是否存在大装甲，存在装甲result！0,不存在装甲result为0
            result = find_large_armor_in_expts(src_img, &valid_bar_list[i],&valid_bar_list[j],1.0,meng::big_pair_bar_angle_limit, img_ori);
            //printf("large expts: %d \n" ,result );
            if( result != 0 )
                return i;
        }
    };

    /*secondly, we want to find virtual armor accordingt to single bar  需要根据单个灯条去寻找虚拟虚拟装甲
     * #Meng:
     * Attention!!!!
     * For guard robot, you should be very careful to start the code bellow, for more virtual armor may lead to more
     * disdurbution for our detection algorithm...
     *
     * #Snow:
     * it's necessary for all robots to do below, because there are more than 2 lights, and must exist one real armor,
     * but we failed to detect it, so just do it
     * */

    //sort bar list according to height

    for( int i=0;i<valid_bar_list.size();i++ )
    {
        /******************** Find virtual pair for single bar ********************/

        //若灯条的最小椭圆长轴小于meng::bar_height_L=8,则跳出此循环，不进行下一步操作
        if( valid_bar_list[i].ellipse.size.height < meng::bar_height_L ){
            //printf("bar height is %f\n", valid_bar_list[i].ellipse.size.height);
            continue;
        }

        //在虚拟点寻找小装甲         find_small_armor_in_virpts():(自定函数，跳转查看)
        result = find_small_armor_in_virpts(src_img, &valid_bar_list[i],2.0,img_ori);
        if( result != 0 )
            return i;

        //在虚拟点寻找大装甲         find_small_armor_in_virpts():(自定函数，跳转查看)
        result = find_large_armor_in_virpts(src_img, &valid_bar_list[i],4.0,img_ori);
        if( result != 0 )
            return i;
    }
    return -1;
}
/*********************************
 *
 * Description: 单个灯条寻找装甲
 * Input:
 *      1.  cv::Mat &src_img
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
int find_armor_for_single_bar(cv::Mat &src_img, std::vector<struct s_bar> &valid_bar_list,cv::Mat & img_ori, cv::Point2f &sp)
{
    int result = 0;

    std::vector<struct s_bar> new_bar_list;

    //根据灯条外接椭圆长轴，降序排列灯条存储在 new_bar_list 中   单个灯条，直接返回单个灯条
    sort_bar_according_to_height(valid_bar_list,new_bar_list);

    for( int i=0;i<new_bar_list.size();i++ )
    {

        /******************** Find virtual pair for single bar ********************/

        //若灯条的最小椭圆长轴小于meng::bar_height_L=8,则跳出此循环，不进行下一步操作
        if( valid_bar_list[i].ellipse.size.height < meng::bar_height_L ){
            printf("single bar result is %f\n",valid_bar_list[i].ellipse.size.height);
            continue;
        }

        //result=1则表示左侧虚拟匹配有小装甲，result=2表示右侧虚拟匹配有小装甲          find_small_armor_in_virpts():(自定函数，跳转查看)
        result = find_small_armor_in_virpts(src_img, &new_bar_list[i],2.0,img_ori);

        //虚拟匹配有小装甲
        if( result != 0 )
        {
            //获得装甲中心点p           getCrossPoint():(自定函数，跳转查看)
            cv::Point2f p = getCrossPoint(new_bar_list[i].armor_vertex[0],new_bar_list[i].armor_vertex[2],
                                          new_bar_list[i].armor_vertex[1],new_bar_list[i].armor_vertex[3]);

            if(p.x!=-1)      //若p在图像内部，将装甲板中心点坐标赋值给sp，sp初始值（-1,-1）被覆盖
             sp = p;
            return i;
        }

        //result=1则表示左侧虚拟匹配有大装甲，result=2表示右侧虚拟匹配有大装甲          find_large_armor_in_virpts():(自定函数，跳转查看)
        result = find_large_armor_in_virpts(src_img, &valid_bar_list[i],4.0,img_ori);
        //虚拟匹配有大装甲
        if( result != 0 )
        {
            return i;
        }
    }
    //没有虚拟装甲则sp（-1,-1）
    sp.x=-1;
    sp.y=-1;
    return 0;
}

