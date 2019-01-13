//
// Created by jason on 18-6-5.
//

#include <gold/my.h>
#include "gold/coordinante_process.h"
/*********************************
 *
 * Description: 寻找椭圆沿长轴的端点坐标pt
 * Input:
 *      1. start 椭圆中心点
 *      2. angle_rad  旋转弧度
 *      3. distance 椭圆长轴的倍数
 *      4. int rows,int cols src图像尺寸
 * Output:
 *      1. cv::Point2f & pt 计算后的椭圆端点
 * Others：
 * 		1.  pt.x是端点x坐标+长半轴长度*cos得出的椭圆沿长轴的端点x坐标
 * 		2.  pt.y是端点y坐标+长半轴长度*sin得出的椭圆沿长轴的端点y坐标
 *      3.  若超出原图尺寸，坐标强制设为原图大小
**********************************/
void find_newpoint(cv::Point2f start,float angle_rad,float distance,cv::Point2f & pt,int rows,int cols)
{
    pt.x = start.x+distance*cos(angle_rad); // pt.x是椭圆端点的x坐标
    if( pt.x >=cols)                        // 如果超出原图尺寸，变为尺寸值
    {
        pt.x = cols-1;
    }
    if( pt.x < 0 )
    {
        pt.x = 0;
    }

    pt.y = start.y-distance*sin(angle_rad); // pt.y是椭圆端点的y坐标
    if( pt.y >=rows)                        // 如果超出原图尺寸，变为尺寸值
    {
        pt.y = rows-1;
    }
    if( pt.y < 0 )
    {
        pt.y = 0;
    }
}


/*********************************
 *
 * Description:make sure e1 is on the top ,and e2 is on bottom
 * Input:
 *      1. e1
 *      2. e2
 * Output:
 *      1. e1
 *      2. e2
 * Others：
 * 		1、如果e1在e2下方
 * 		2、将e1 e2的 x和y坐标互换
 * 		3、确保e1在e2上方
 *
**********************************/
void check_endpoint(cv::Point2f & e1,cv::Point2f & e2)
{
    if ( e1.y > e2.y )
    {
        cv::Point2f tmp;
        tmp.x=e1.x;
        tmp.y=e1.y;

        e1.x=e2.x;
        e1.y=e2.y;

        e2.x=tmp.x;
        e2.y=tmp.y;
    }
}

/**  make sure rect cood in image size   **/
int check_rect(const cv::Rect &r,int rows,int cols)
{
    // 定义整个图像矩形：左上角坐标(0,0), width为cols，height为rows
    cv::Rect big_rect(0,0,cols,rows);

    // 两矩形的交集矩形
    cv::Rect and_rect;

    // 两矩形求交集，交集矩形赋给 and_rect
    and_rect = big_rect & r;    //r为src_rect的索引

    // 如果 and_rect 面积小于 1 ，也即是检测矩形是否在图像矩形范围内，返回 0
    if( and_rect.area() < 1 )
        return 0;

    return 1;
}

/*********************************
 *
 * Description: 限制矩形在图像范围内
 * Input:
 *      1. src_rect 输入的矩形
 *      2. rows
 *      3. cols
 * Output：
 *      1. dst_rect 输出的矩形
 * Others：
 * 		1、调用check_rect()函数，首先判断 src_rect 是否在图像尺寸范围内
 * 		2、若在图像尺寸范围内，调用limit_number()函数，进行尺寸约束
 * 		3、保证输入的最小外接矩形在图像范围内
 *
**********************************/
int  limit_rect(cv::Rect &src_rect,cv::Rect & dst_rect,int rows,int cols)
{
    // check_rect():判断矩形是否在图像尺寸范围内 （自定函数，跳转查看）
    if( check_rect( src_rect,rows,cols ) == 0 )
        return 0;

    else{   //确保矩形在图片尺寸内
        int x1,y1,x2,y2;

        //limit_number():尺寸约束(自定函数，跳转查看)
        //（x1,y1）为矩形左上角点坐标，（x2,y2）为矩形右下角点坐标
        x1 = limit_number(src_rect.x,cols-1,0);
        y1 = limit_number(src_rect.y,rows-1,0);
        x2 = limit_number(src_rect.x+src_rect.width,cols-1,0);
        y2 = limit_number(src_rect.y+src_rect.height,rows-1,0);

        if( (y2<=y1)||(x2<=x1) )
        {
            return 0;
        }

        dst_rect.x = x1;                //左上角点x坐标
        dst_rect.y = y1;                //左上角点y坐标
        dst_rect.width = x2-x1;         //矩形宽度
        dst_rect.height = y2-y1;        //矩形高度

        if ((dst_rect.width < 0) || (dst_rect.height < 0))
            return 0;
    }

    return 1;
}


/** check if roi in mask contain some points **/
float check_mask_area(cv::Mat &bgr_img, cv::Rect &roi)
{
    cv::Mat bgr_mask, color_bin_mask;

    bgr_mask = bgr_img(roi);             //将bgr_img的roi区域图赋给bgr_mask
    float area = 0;                      //色彩值初始化

    if(bgr_mask.channels()==1){          //单通道
        //cv::sum():计算各通道所有像素总和
        area = cv::sum(bgr_mask)[0]/255.;//计算色彩值，赋给area
    }else{
            //蓝色
        if(meng::Color_id==0){
            //二值化操作
            cv::inRange(bgr_mask,cv::Scalar(100,100,100),cv::Scalar(124,255,255),color_bin_mask);

        }else if(meng::Color_id==1){    //红色
            //二值化操作
            cv::Mat hsv1, hsv2;
            cv::inRange(bgr_mask,cv::Scalar(0,70,70),cv::Scalar(10,200,255),hsv1);
            cv::inRange(bgr_mask,cv::Scalar(170,70,70),cv::Scalar(180,200,255),hsv2);
            color_bin_mask = hsv1 + hsv2;
        }

        area = cv::sum(color_bin_mask)[0]/255.;//计算色彩值，赋给area
    }

    return area;
}

/**  clip number in a range  **/
/// 尺寸约束:如果low < a < hight ,返回a , a<low的返回low , a>hight的返回hight ///
float limit_number(float a,float hight,float low)
{
    if (a > hight)
    {
        return hight;
    }

    if(  a<low )
    {
        return low;
    }
    return a;
}

/**  distance between 2 points  **/
float cal_distance_pt(cv::Point2f a , cv::Point2f b)
{
    return sqrt( (a.x-b.x)*(a.x-b.x) + (a.y-b.y)*(a.y-b.y) );
}

/*get armot vertexs from two pair of  bar endpoints*/
/*********************************
 *
 * Description: 从一对灯条的端点获取装甲的顶点
 * Input:
 *      1.  cv::Point2f e1      ((*domain_bar).endpoints[0])
 *      2.  cv::Point2f e2      ((*domain_bar).endpoints[1])
 *      3.  cv::Point2f p1      ((*domain_bar).pair_p1)
 *      4.  cv::Point2f p2      ((*domain_bar).pair_p2)
 *      5.  int rows            (img_ori.rows)
 *      6.  int cols            (img_ori.cols)
 * Output：
 *      1.  cv::Point2f * armor ((*domain_bar).armor_vertex)
 * Others：
 * 		1、  将输入的灯条端点坐标顺时针编号0,1,2,3
 * 		2、  计算装甲板四个点坐标
 * 		3、  将装甲坐标约束在图像尺寸内，超出图像，则为图像边界
 *
**********************************/
void get_armor_from_pts(cv::Point2f e1 ,cv::Point2f e2,cv::Point2f p1 ,cv::Point2f p2,cv::Point2f * armor,int rows,int cols)
{
    cv::Point2f tmp[4];

    if( e1.x < p1.x )       //左上为0顺时针编号0,1,2,3
    {
        tmp[0]=e1;
        tmp[1]=p1;
        tmp[2]=p2;
        tmp[3]=e2;
    } else
    {
        tmp[0]=p1;
        tmp[1]=e1;
        tmp[2]=e2;
        tmp[3]=p2;
    }

    float dx,dy;
    float k = 0.5;

    ///     计算装甲板四个点坐标，左上角为0       ///
    dx=tmp[0].x-tmp[3].x;
    dy=tmp[0].y-tmp[3].y;

    armor[0].x =  tmp[0].x + k*dx;  //左上角
    armor[0].y =  tmp[0].y + k*dy;
    armor[3].x =  tmp[3].x - k*dx;  //右下角
    armor[3].y =  tmp[3].y - k*dy;

    dx=tmp[1].x-tmp[2].x;
    dy=tmp[1].y-tmp[2].y;

    armor[1].x =  tmp[1].x + k*dx;  //右上角
    armor[1].y =  tmp[1].y + k*dy;
    armor[2].x =  tmp[2].x - k*dx;  //左下角
    armor[2].y =  tmp[2].y - k*dy;

    //将装甲坐标约束在图像尺寸内，超出图像，则为图像边界             limit_number():(自定函数，跳转查看)
    armor[0].x=limit_number(armor[0].x,cols-1,0);
    armor[1].x=limit_number(armor[1].x,cols-1,0);
    armor[2].x=limit_number(armor[2].x,cols-1,0);
    armor[3].x=limit_number(armor[3].x,cols-1,0);

    armor[0].y=limit_number(armor[0].y,rows-1,0);
    armor[1].y=limit_number(armor[1].y,rows-1,0);
    armor[2].y=limit_number(armor[2].y,rows-1,0);
    armor[3].y=limit_number(armor[3].y,rows-1,0);


}

/*sort by decending */
template < typename T>
/*********************************
 *
 * Description: 升序降序排序
 * Input:
 *      1.  vector< T>  & v     距离
 *      2.  descend_or_ascend   升序或降序flag
 * Output：
 *      1.  排序后的索引值
 * Others：
 * 		1、初始化索引 idx
 * 		2、根据灯条椭圆中心点到指定动态点距离的平方排序,根据升降序flag，确定升序还是降序
 * 		3、返回排序后的 idx
 *
**********************************/
std::vector< size_t>  sort_indexes(const std::vector< T>  & v , int descend_or_ascend)
{
    // initialize original index locations
    std::vector< size_t>  idx(v.size());
    for (size_t i = 0; i != idx.size(); ++i)
        idx[i] = i;

    // sort indexes based on comparing values in v

    if( descend_or_ascend == 0 )    //flag = 0 降序
    {
        /***
        * sort():对给定区间所有元素进行排序,头文件是#include <algorithm>
        * sort函数有三个参数：
        *（1）第一个是要排序的数组的起始地址;
        *（2）第二个是结束的地址（最后一位要排序的地址的下一地址）;
        *（3）第三个参数是排序的方法，可以是从大到小也可是从小到大，还可以不写第三个参数，此时默认的排序方法是从小到大排序。
        ***/
        sort(idx.begin(), idx.end(),
             [& v](size_t i1, size_t i2) {return v[i1] >  v[i2];});         // '>' descending sort  ; '<' ascending sort
    }
    else                            //flag = 1 升序
    {
       /***
       * sort():对给定区间所有元素进行排序,头文件是#include <algorithm>
       * sort函数有三个参数：
       *（1）第一个是要排序的数组的起始地址;
       *（2）第二个是结束的地址（最后一位要排序的地址的下一地址）;
       *（3）第三个参数是排序的方法，可以是从大到小也可是从小到大，还可以不写第三个参数，此时默认的排序方法是从小到大排序。
       ***/
        sort(idx.begin(), idx.end(),
             [& v](size_t i1, size_t i2) {return v[i1] <  v[i2];});// '>' descending sort  ; '<' ascending sort
    }
    return idx;
}

/*********************************
 *
 * Description: sort bars according to their height
 * Input:
 *      1.  ori_bars（tmp_valid_bar_list）  灯条信息
 *      2.  x（meng::dynamic_sp.x）  虚拟的目标点x坐标
 *      3.  y（meng::dynamic_sp.y）  虚拟的目标点y坐标
 * Output：
 *      1.  dst_bars（valid_bar_list）  升序排序后的灯条信息
 * Others：
 * 		1、判断灯条个数，如果只有一个灯条，直接放入dst_bars，返回
 * 		2、如果不止一个灯条，根据距离升序排序索引
 * 		3、把索引信息放入 dst_bars（valid_bar_list） 中
 *
**********************************/
void sort_ellipse(const std::vector<struct s_bar> &ori_bars , std::vector<struct s_bar> &dst_bars,int x , int y)
{
    if(  ori_bars.size()==1)    //如果只有一个灯条
    {
        dst_bars.push_back(ori_bars[0]);    //这一个灯条信息直接放入 dst_bars 中
        return ;
    }
    else
    {
        std::vector<float> heghts;           //拟合最小外接椭圆长轴
        std::vector<float> distance;         //灯条椭圆中心点到目标点的距离的平方
        std::vector<size_t > new_indexs;

        for(int i=0;i<ori_bars.size();i++)
        {
//          heghts.push_back(ori_bars[i].ellipse.size.height);
            //pow():求幂次方函数：第一个参数是大小，第二个参数是幂次方大小     这里是将灯条的椭圆中心点到目标点的距离的平方，放入 distance 中
            distance.push_back( pow((ori_bars[i].ellipse.center.x-x),2)+pow((ori_bars[i].ellipse.center.y-y),2) );
        }

        //sort_indexes():排序操作(自定函数，跳转查看)
        new_indexs = sort_indexes(distance,1);

        for(int i=0;i<new_indexs.size();i++)
        {
            dst_bars.push_back(ori_bars[new_indexs[i]]);
        }
        return ;

    }
}

/*
 * 根据灯条的高度来进行降序排列
 * */
bool compare_bar_by_height(struct s_bar bar_0, struct s_bar bar_1)
{
    return bar_0.ellipse.size.height > bar_1.ellipse.size.height;
}

//根据灯条外接椭圆长轴排列灯条
//输入bar,输出bar
void sort_bar_according_to_height(const std::vector<struct s_bar> &ori_bars , std::vector<struct s_bar> &dst_bars)
{
#if 1
    //==========Snow============
    if(ori_bars.size()==1)      //单个灯条
        dst_bars = ori_bars;    //直接输出灯条
    else{   //多个灯条
        std::vector<struct s_bar> bars_tmp = ori_bars;  //将灯条暂时储存在bars_tmp
        std::sort(bars_tmp.begin(),bars_tmp.end(),compare_bar_by_height);    //根据灯条外接椭圆长轴降序排列灯条
        dst_bars = bars_tmp;    //输出降序排列后的灯条
    }
#else
    //==========Meng============
    if(ori_bars.size()==1){                  //单个灯条
        dst_bars.push_back(ori_bars[0]);     //直接输出灯条
    }else{
        std::vector<float> heghts;
        std::vector<size_t> new_indexs;

        //遍历每个灯条
        for(int i=0;i<ori_bars.size();i++)
            heghts.push_back(ori_bars[i].ellipse.size.height);      //将所以灯条的外接椭圆长轴存储在 heghts 中

         //降序排列
        new_indexs = sort_indexes(heghts,0);

        //按顺序输出降序排列的灯条
        for(int i=0;i<new_indexs.size();i++)
            dst_bars.push_back(ori_bars[new_indexs[i]]);
    }
#endif
}

//
double calDistance(const std::vector<cv::Point2f> &points  ,cv::Mat &intrinsic_mat)
{
    double  h=0.105;//单位米
    double ux0 = (points[0].x - intrinsic_mat.at<float>(0,2)) / intrinsic_mat.at<float>(0,0);//归一化平面坐标
    double uy0 = (points[0].y - intrinsic_mat.at<float>(1,2)) / intrinsic_mat.at<float>(1,1);
    double ux1 = (points[1].x - intrinsic_mat.at<float>(0,2)) / intrinsic_mat.at<float>(0,0);
    double uy1 = (points[1].y - intrinsic_mat.at<float>(1,2)) / intrinsic_mat.at<float>(1,1);
//    double ux2 = (points[2].x - intrinsic_mat.at<float>(0,2)) / intrinsic_mat.at<float>(0,0);
//    double uy2 = (points[2].y - intrinsic_mat.at<float>(1,2)) / intrinsic_mat.at<float>(1,1);

    double uh1= std::sqrt(std::pow((ux1-ux0),2)+std::pow( (uy1-uy0),2));//这里二维码可能不是平行于相机平面，所以这里取形变程序小的距离
//    double uh2= sqrt(pow((ux2-ux1),2)+pow( (uy2-uy1),2));
std::cout<<"uh1 : "<<ux0<<std::endl;
//    double uh=  std::max(uh1,uh2);
    double z=h/uh1;
    return z;
}

