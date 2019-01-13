#ifndef DAFU_DETECTOR_H_
#define DAFU_DETECTOR_H_

#include <boost/asio.hpp>
#include <boost/array.hpp>
#include <boost/thread.hpp>
#include <opencv2/opencv.hpp>
#include <iostream>

#include <eigen3/Eigen/Geometry>
#include <opencv2/core/eigen.hpp>

#include "cubotcpp/timer.h"
#include "cubotcpp/Serial_port.h"
#include "cubotcpp/imgSubscrible.h"

#include "ParaReader.h"
#include "NumClass.hpp"
#include "DigitronDetector(svm_tube).hpp"
#include "../include/fordefine.h"
#include <boost/date_time/posix_time/posix_time.hpp>

using namespace boost::posix_time;
using namespace cv;
using namespace std;

typedef Eigen::Matrix<double, 5, 1> Vector5d;

class DafuDetector
{
private:
    Mat cvCameraMatrix, cvDistCoeffs, cvCalMatrix,cvDistCoeffs2;  // 相机参数,两个畸变参数排布不同。分别用于发射角度计算及深度计算
    Eigen::Isometry3d calMatrix;
    Eigen::Matrix3d cameraMatrix;
    Vector5d distCoeffs,distCoeffs2;
    // 色彩空间：绿、黄、天蓝、紫
    Scalar color[4] = {Scalar(0, 255, 0), Scalar(0, 255, 255), Scalar(255, 255, 0), Scalar(255, 0, 255)};//绿、黄、天蓝、紫

    vector<Mat> dst_squares;            // 透视变换图中九宫格中心点坐标，在initData()中初始化，数据存在squares中

    double classProb[9]= {0.0};         // CNN手写数字分类概率
    int classId[9]= {0};                // CNN手写数字分类结果
    int classIdLast[9] = {0};           // 上一次分类结果

    int digitalTube[5] = {0};           // 数码管识别结果
    int digitalTubeLast[5] = {0};       // 上一次数码管分类识别结果

    bool isDigitalTubeChanged = false;  // 数码管是否改变
    bool isDafuChanged = false;         // 九宫格是否改变

    int orderOfTubeNum = 0;             // 数码管数字序号(12345)，标志当前需要打第几个数码管中数字对应的九宫格

    NumClass nc;                        // CNN手写数字分类
    digitronDetector hb_detector;       // 数码管检测

    vector<Point2f> dstPts;             // findHomography参数
    vector<Point2f> srcPts;
    ParaReader pr;                      // 大符角点标准数据

    vector<Rect> minRect;               // 九宫格Rect

    Mat element;                        // 膨胀参数
    TermCriteria termcrit;              // 亚像素交点精确终止条件参数

    ////////// private functions //////////

    bool dafuNumAnalysis(int * classId, double * classProb);

    void correct_scoreboard(int condition,vector<int> & contours_idx,const vector<vector<Point2f> > & contours,int src_cols);

    int get_vector_idx(const vector<int> & v, int k);

    int idx_vector_analysis(const vector<int> & v);

    void sortPoints(vector<Point>& p_);

    int calc_len(const Point & p1, const Point & p2);

    float calc_len(const Point2f & p1, const Point2f & p2);

    bool ptInRange(const Point2f & p1, const Point2f & p2, const float r);

public:

    // 完成CNN模型加载
    DafuDetector() ;
    ~DafuDetector()
    {
        element.release();
    };

    bool detect(Mat src_img, Point2f & p2f_angle, int & send_num);
};


//// 函数作用：载入CNN模型和九宫格实际尺寸对应的像素坐标
DafuDetector::DafuDetector():nc("../Fuwen/0725_final.pb"), pr("../Fuwen/corner.txt")
{
    // 找到九宫格ROI之后，待提取的九个格子的像素尺寸
    //ROI:OpenCV感兴趣区域，指定矩形的坐标，并且规定好长宽。
    //emplace_back能就地通过参数构造对象，不需要拷贝或者移动内存,使容器插入元素的性能得到进一步提升。
    // 是透视变换之后的九宫格ROI
    //制定好的透视变换后的九宫格ROI放入九宫格minRect中，（已经定义过的：vector<Rect> minRect）
    minRect.emplace_back(Rect(79, 55, 42, 40));
    minRect.emplace_back(Rect(172, 55, 42, 40));
    minRect.emplace_back(Rect(264, 55, 42, 40));
    minRect.emplace_back(Rect(79, 110, 42, 40));
    minRect.emplace_back(Rect(172, 110, 42, 40));
    minRect.emplace_back(Rect(264, 110, 42, 40));
    minRect.emplace_back(Rect(79, 165, 42, 40));
    minRect.emplace_back(Rect(172, 165, 42, 40));
    minRect.emplace_back(Rect(264, 165, 42, 40));

    // 透视变换图中九宫格中心点坐标（x,y,1）
    // 坐标定义为三维(方便进行矩阵运算？)
    float squares[9][3] = {
            {400.0f,  299.7f, 1.0f},
            {770.0f,  299.7f, 1.0f},
            {1140.0f, 299.7f, 1.0f},
            {400.0f,  515.2f, 1.0f},
            {770.0f,  515.2f, 1.0f},
            {1140.0f, 515.2f, 1.0f},
            {400.0f,  739.7f, 1.0f},
            {770.0f,  739.7f, 1.0f},
            {1140.0f, 739.7f, 1.0f}
    };

    // 九宫格中心点坐标缩放至原来的1/4
    //遍历squares[i][0],squares[i][1]操作
    //size_t ：放置尺寸大小的值，类型为整数。
    for (size_t i = 0; i < 9; i++)
    {
       // 将（定义类一个三行一列的内容为float类型的矩阵，中心点x坐标×0.25倍,中心点y坐标×0.25倍，1 三个数赋给矩阵）赋给 dst_p
        Mat dst_p = (Mat_<float>(3, 1) << squares[i][0] * 0.25f, squares[i][1] * 0.25f, 1.0f);
        // 将缩放后的九宫格中心点坐标 dst_p 放置dst_squares中(已经定义过的：vector<Mat> dst_squares)
        dst_squares.push_back(dst_p);
    }
    ////这样九次循环后，dst_squares里储存了3×1的矩阵:(x,y,1):九宫格中心点坐标信息


    // 自定义内核大小，第一个参数表示内核的形状:矩形，第二个参数表示内核的尺寸：2×2
    element = getStructuringElement(MORPH_RECT, Size(2, 2));
    //TermCriteria模板类是用来作为迭代算法的终止条件。该类变量需要3个参数,第一个参数是类型:EPS:(当算法的精确度小于参数double epsilon指定的精确度时，停止算法)，第二个参数是迭代的最大次数:20，最后一个是所要求的精度:0.03
    termcrit = TermCriteria(TermCriteria::COUNT | TermCriteria::EPS, 20, 0.03);


    // 读取相机参数,from:“../cfg/camera_config.xml”,
    FileStorage fs("../cfg/camera_config.xml", FileStorage::READ);
    fs["cameraMatrix"] >> cvCameraMatrix;       //相机内参数矩阵
    fs["distCoeffs"] >> cvDistCoeffs;           //相机畸变系数1矩阵
    fs["distCoeffs2"] >> cvDistCoeffs2;         //相机畸变系数2矩阵
    fs["calMatrix"] >> cvCalMatrix;             //相机外参参数矩阵T
    //关闭文件并释放缓存
    fs.release();


    // 变量类型转换
    // 将CV格式转换为eigen格式，便于矩阵运算
    cv::cv2eigen(cvCalMatrix, calMatrix.matrix());
    cv::cv2eigen(cvCameraMatrix, cameraMatrix);
    cv::cv2eigen(cvDistCoeffs, distCoeffs);
    cv::cv2eigen(cvDistCoeffs2, distCoeffs2);


    // 记录实时时间
    ptime  time_now(second_clock::local_time());
    string now_iso_str(to_iso_string(time_now));
}

/* 函数功能大符检测
 * src_img：  原始图像（三通道）
 * p2f_angle：打击点的像素坐标
 * send_num： 数码管识别结果
 */
bool DafuDetector::detect(Mat src_img, Point2f &p2f_angle, int & send_num)
{
    // getTickCount()返回从操作系统启动到当前所经过的毫秒数,赋值给 start
    double start = static_cast<double>(getTickCount());

    // 读入图像失败，立即返回 false
    if (src_img.empty())
    {
        //输出程序错误信息：“DafuDetector : input image is empty”
        std::cerr << "DafuDetector : input image is empty" << std::endl;
        return false;
    }

    /////图像预处理/////
    Mat src,gray_mat,canny_mat;
    // 缩放图片，提高处理速度，缩放为原图的0.6倍
    //resize（）函数：改变图像的大小
    resize(src_img, src, Size(), 0.6, 0.6);
    // 转换为灰度图
    //cvtColor（）函数：颜色空间转换
    cvtColor(src, gray_mat, COLOR_BGR2GRAY);
    // 将输入图像备份
    //克隆输入图像src给src2
    Mat src2 = src.clone();
    //// （Canny之前先使用3×3内核blur降噪？？）//blur(gray_mat,canny_mat,Size(3,3));
    //边缘检测
    //Canny()函数：边缘检测,这个函数阈值1和阈值2两者较小的值用于边缘连接，而较大的值用来控制强边缘的初始段，推荐的高低阈值比在2：1，到3：1之间。
    Canny(gray_mat, canny_mat, 50, 120);
    // 膨胀处理，使断开的边缘连接起来
    //dilate（）函数：膨胀操作，（element之前已经自定义好：element = getStructuringElement(MORPH_RECT, Size(2, 2));）
    dilate(canny_mat, canny_mat, element);

    //显示预处理后的图像
#ifdef show
    imshow("canny", canny_mat);
#endif

    //// 对canny边缘图提取所有轮廓,通过面积筛选轮廓，放入contours_mat/////
    //contours一个向量，并且是一个双重向量，向量内每个元素保存了一组由连续的Point点构成的点的集合的向量，每一组Point点集就是一个轮廓。有多少轮廓，向量contours就有多少元素。
    vector<vector<Point> > contours;

    // 查找轮廓，检测到的轮廓、函数调用后的运算结果放在contours
    //RETR_TREE：检测所有轮廓，所有轮廓建立一个等级树结构。外层轮廓包含内层轮廓，内层轮廓还可以继续包含内嵌轮廓。
    //CHAIN_APPROX_SIMPLE：仅保存轮廓的拐点信息，把所有轮廓拐点处的点保存入contours向量内，拐点与拐点之间直线段上的信息点不予保留，例如一个矩形轮廓只需要4个点来保存轮廓信息。
    findContours(canny_mat, contours, RETR_TREE, CHAIN_APPROX_SIMPLE);

    // 绘制第一次面积约束得到的图像轮廓
    //定义一个单通道的空白图片的contours_mat,用来存放轮廓
    Mat contours_mat(src.rows, src.cols, CV_8UC1, Scalar(0));
    // 遍历所有轮廓
    // contours.size()：轮廓个数     // size():先列后行
    for (int j = 0; j < contours.size(); j++)
    {
        //每次遍历的轮廓计算面积后赋值给area
        //contourArea()函数：主要用于计算图像轮廓的面积。
        double area = contourArea(contours[j]);

        // 面积约束范围: 70<area<2000
        if (area > 70 && area < 2000)
        {
            // 绘制轮廓
            //drawContours():绘制轮廓，用于绘制找到的图像轮廓
            //contours_mat:要绘制轮廓的图像
            //contours:所有输入的轮廓，每个轮廓被保存成一个point向量
            //j: 指定要绘制轮廓的编号
            //Scalar(255)：绘制轮廓所用的颜色:白色
            drawContours(contours_mat, contours, j, Scalar(255));
        }
    }
    //到目前已经完成了第一次面积约束后的轮廓
    /// 目前已将检测到所有轮廓并且将符合面积要求的轮廓绘制出来储存在contours_mat中///
#ifdef show //将第一次绘制的轮廓显示出来
    imshow("cc_1", contours_mat);
#endif

    //// 对contours_mat提取所有 "外部轮廓",经过面积、长宽比、四边形情况进行筛选，放入 contours2中 ////
    vector<vector<Point> > contours1;      //轮廓点向量contours1
    vector<vector<Point2f> > contours2;    // 满足第二次约束的所有轮廓，每个轮廓用四个点表示（四边形），四个顶点经过亚像素精确，对应contours_idx3

    // 第二次轮廓检测，提取外部轮廓，将轮廓信息放置contours1
    //RETR_EXTERNAL：只检测最外层的轮廓
    findContours(contours_mat, contours1, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
#ifdef show
    //单通道的白色空白图片contours_mat1，用来存放第二次面积约束的轮廓
    Mat contours_mat1(src.rows, src.cols, CV_8UC1, Scalar(0));               // 绘制第二次约束得到的轮廓图像
#endif
    // 遍历所有第二次检测的轮廓
    for (int j = 0; j < contours1.size(); j++)
    {
        // 轮廓面积
        //contourArea()函数：主要用于计算图像轮廓的面积。
        double area = contourArea(contours1[j]);

        // 最小外接矩形面积，用于大致计算长宽比
        // minAreaRect（）：包含点集的最小外接矩形
        RotatedRect rr = minAreaRect(contours1[j]);

        // 最小外接矩形的长宽比
        float aspect_ratio = rr.size.height / rr.size.width;

        // 将长宽比变为大于1的值
        //A ？ B ： C
        aspect_ratio = aspect_ratio < 1.0f ? 1.0f / aspect_ratio : aspect_ratio;

        // 多边形拟合得到的轮廓角点（size等于多边形的边的数量：四个点即四个边）
        vector<Point> tempContour1;//轮廓角点
        float rect_height = MIN(rr.size.height, rr.size.width);//找出二者之中小的一个赋给rect_height，多边形拟合参数

        // 多边形拟合，第三个参数需要根据轮廓的大小（面积）进行调整
        // 第一个参数：输入的二维点集，在此为<vector>型的contours1，第二个参数：输出的二维点集，多边形逼近的结果，在此为tempContour1，第三个参数：逼近的精度，也即是原始曲线与近似曲线之间的最大距离，第四个参数：表示输出的多边形是否封闭
        approxPolyDP(contours1[j], tempContour1, double(rect_height / 2.0f), true);

        //设置标志位，若为true则表示筛选出来合适的四边形
        bool point_order = false;

        // 筛选四边形，tempContour1存放的是轮廓角点点集，判断轮廓的角点数即可筛选四边形
        if (tempContour1.size() == 4)
        {
            // 如果是四边形，则对四个顶点排序：左上、右上、右下、左下（顺时针）,对应次序依次为0,1,2,3
            // sortPoints（）：对点序进行顺时针排序
            sortPoints(tempContour1);

            // 求四边形边长平方，计算两点间距离的平方
            //calc_len（）：计算两点距离的平方，目的是保证距离大于零（自定义的函数）
            float tw1 = calc_len(tempContour1[0], tempContour1[1]);     // 上边
            float tw2 = calc_len(tempContour1[3], tempContour1[2]);     // 下边
            float th1 = calc_len(tempContour1[1], tempContour1[2]);     // 右边
            float th2 = calc_len(tempContour1[0], tempContour1[3]);     // 左边

            // 上边 / 下边，将比例变为大于1
            //sqrt（）：开平方根
            float rw = sqrt(tw1 / tw2);
            //A ？ B ： C
            rw = rw > 1.0f ? rw : 1.0f / rw;
            ///此时 rw >1

            // 右边 / 左边，将比例变为大于1
            //sqrt（）：开平方根
            float rh = sqrt(th1 / th2);
            //A ？ B ： C
            rh = rh > 1.0f ? rh : 1.0f / rh;
            ///此时 rh >1

            // 上边 / 右边
            //sqrt（）：开平方根
            float rrr = sqrt(tw1 / th1);

            // 轮廓面积/最小外接矩形面积，越接近1表示越像四边形
            // 轮廓面积：double area = contourArea(contours1[j])
            // 最小外接矩形面积：RotatedRect rr = minAreaRect(contours1[j])
            float ra = float(area) / (rr.size.height * rr.size.width);

            // 四边形检测情况，通过上下边比例/左右边比例/宽高比/轮廓面积/最小外接矩形面积比例判定是否可用
            if (rw < 1.5f && rh < 1.5f && rrr < 3.0f && rrr > 1.0f && ra > 0.8f)
                // 标志位，表示筛选出合适的四边形
                point_order = true;
        }

        // 是否满足面积、长宽比、四边形情况
        // 轮廓面积，外接四边形面积筛选
        if (area > 70 && area < 2000 && aspect_ratio > 1.2f && aspect_ratio < 3.0f && tempContour1.size() == 4 && point_order)
        {
#ifdef show
            // 绘制出符合要求的四边形轮廓，存入contours_mat1
            //-1：全部绘制
            //Scalar(255)：线条颜色为白色
            drawContours(contours_mat1, contours1, -1, Scalar(255));
#endif
            // 满足条件，则存入tempContour2中（注意tempContour1和tempContour2中变量类型有区别，由于下面处理需要）
            vector<Point2f> tempContour2;   //二维点集向量tempContour2
            for (const auto &c : tempContour1)  //c是一个常量引用，绑定到tempContour1
                //将Point2f(c.x, c.y)放入tempContour2中
                tempContour2.emplace_back(Point2f(c.x, c.y));

            // 亚像素级精确度的角点，输入图像必须是单通道，参数二为输入角点的初始坐标以及精准化后的坐标用于输出，其类型必须是vector<Point2f>
            //termcrit：迭代过程的终止条件，之前已定义过（可返回查看）设定值为0.03
            cornerSubPix(gray_mat, tempContour2, Size(5, 5), Size(-1, -1), termcrit);
            //将tempContour2放入contours2中
            contours2.push_back(tempContour2);
#ifdef show
            for (int k = 0; k < 4; k++)
            {
                //绘制直线，起点为tempContour2[k]，终点为tempContour2[(k + 1) % 4]，用红色线条绘制
                line(src, tempContour2[k], tempContour2[(k + 1) % 4], Scalar(0, 0, 255), 1, LINE_AA);
                //绘制直线，起点为 tempContour2[k] - Point2f(2, 0)，终点为tempContour2[k] + Point2f(2, 0)，线条颜色用定义过的color[k] //坐标十字标定左右
                line(src, tempContour2[k] - Point2f(2, 0), tempContour2[k] + Point2f(2, 0), color[k]);
                //绘制直线，起点为 tempContour2[k] - Point2f(0, 2)，终点为tempContour2[k] + Point2f(0, 2)，线条颜色用定义过的color[k] //坐标十字标定上下
                line(src, tempContour2[k] - Point2f(0, 2), tempContour2[k] + Point2f(0, 2), color[k]);
            }
#endif
        }
    }
    /// 以上完成多次四边形筛选，将角点信息放入contours2



    //如果contours2为空，（显示符合要求的四边形轮廓contours_mat1,显示缩放过的图片src）？忽略当前帧
    if (contours2.empty())
    {
#ifdef show
        imshow("contours_mat1",contours_mat1);
        imshow("src0",src);
        waitKey(1);
#endif
        return false;
    }

    //// 对contours2中的轮廓进行位置约束，使用contours_idx3对所有轮廓进行标记，如不可以，则标记为-1或0 ////
    // 元素数量等于contours2，每个元素表示对应轮廓的分布情况，0或-1表示不是记分板
    vector<int> contours_idx3;  //整型数据集contours_idx3
    //size_t ：放置尺寸大小的值，类型为整数。
    //遍历contours2
    for (size_t j = 0; j < contours2.size(); j++)
    {
        //现将所有标记置为0，保证contours_idx3在contours.size范围内全都标记置0
        contours_idx3.push_back(0);
    }
    //累加变量，赋初值为0
    int contour_idx_count = 0;
    //临时使用的变量，赋初值为1
    int _multiple = 1;
    //再次遍历contours2
    for (size_t j = 0; j < contours2.size(); j++)
    {
        // 左边的斜率
        //是否计算上边的斜率,判断上边的斜率值是否贴近于0？？//float bb = (contours2[j][0].y - contours2[j][1].y) / (contours2[j][0].x - contours2[j][1].x);
        float aa = (contours2[j][0].y - contours2[j][3].y) / (contours2[j][0].x - contours2[j][3].x);

        // 轮廓左侧边的斜率约束，不符合的编号设置为-1
        //如果是正方形或长方形，其左边的斜率aa应该非常大，贴近y轴，所以如果斜率-2<aa<2（自己设置）,视作不符合要求的轮廓
        if (aa > -2 && aa < 2)
        {   //给不符合要求的轮廓编号为-1
            contours_idx3[j] = -1;
            //不中断，继续下一步
            continue;
        }

        // 对于某个四边形轮廓，取其左上角点，根据实际比例计算出其上方记分板左上角点位置p2f
        Point2f p2f;    //点集p2f

        // 从矩形轮廓左下角点向上延伸一定长度（矩形轮廓左边长的2.87倍，这一长度是按照实际尺寸得来）
        // p2f为根据左下角点推算出的上一个四边形轮廓左上角点坐标
        //x坐标为不变，因为contours2[j][3].x与contours2[j][0].x相同，相减为0
        p2f.x = contours2[j][3].x - (contours2[j][3].x - contours2[j][0].x) * 2.87f;
        //y坐标即可推算出上一个四边形左上角的y坐标
        p2f.y = contours2[j][3].y - (contours2[j][3].y - contours2[j][0].y) * 2.87f;

        // 判断p2f周围是否存在轮廓点（记分板）
        // 1.当检测到某一四边形未标号，同时标号其自身与延展过去的第一个四边形
        // 2.后续理论上检测到的都是有编号的，使用contour_idx_count保存
        // 3.直到某一列四边形检测完，开始检测另一列四边形
        // 最后一次遍历contours2，只为将记分板按照规定进行编号成功
        for (size_t k = j + 1; k < contours2.size(); k++)
        {
            // 满足某矩形左上顶点上方存在矩形角点时，for循环只执行一次
            // 自定义函数：ptInRange（）：判断点p1是否在以p2为中心，2r为边长的正方形区域内（不含边界）
            //如果找到contours2[k][0]即第k个轮廓的左上角点在p2f为中心，12为边长的正方形区域内，则将此轮廓视作向上延展到的四边形
            if (ptInRange(contours2[k][0], p2f, 6))
            {
                // 对于每一列矩形，该if语句只满足一次，即每一列都是同一数量级
                ////不符合的四边形编号为-1,则这里判断编号是否为0，以确定是对筛选后的长方形进行编号，一次遍历只进入一次if()语句，这里为找到的第一个第二个轮廓编号1和2,_multiple自乘10,以便为下一次遍历做区别编号
                if (contours_idx3[j] == 0) {
                    contours_idx3[j] = _multiple;
                    contours_idx3[k] = _multiple + 1;
                    _multiple *= 10;
                }
                ////第二次遍历时，contours_idx3[j]实际已经经过如上方法实现了编号，即不进入如上if()语句，执行如下的语句：
                //将 contours_idx3[j]的值赋给contour_idx_count
                contour_idx_count = contours_idx3[j];
                //将contour_idx_count + 1的值赋给contours_idx3[k]，实现之后的轮廓编号操作
                contours_idx3[k] = contour_idx_count + 1;
                break;
                ////此编号结束后则break此次编号操作，返回至之前一次的遍历操作中，千万注意每次遍历的范围
            }
        }
#ifdef show
        //画圆：承载的图像，圆心，半径，颜色，粗细，线型
        circle(src, p2f, 6, Scalar(20, 200, 240), 1, LINE_AA);
#endif
    }
////此次共经过三次遍历操作，实现对轮廓的长方形筛选，以及筛选后进行遍历编号，由此结束后，完成了记分板的所有约束性寻找，人为定义了规则的编号排序，以便下面的分类情况讨论使用


    // 分析contours_idx3，分4种情况,1,2,3进入correct_scoreboard函数进行处理
    // 0. 1~5，10~14都没有
    // 1. 有1~5，10~14，说明提取到了10个记分板，此时要判断哪个在左、哪个在右。如果1~5在右，需要调换位置
    // 2. 只有1~5，说明只提取了一侧的记分板，需要判断记分板是左侧还是右侧的（在画面左半边就是左侧，右半边就是右侧）
    // 3. 只有10~14，与上面相似
    // 对于condition等于2或3，只有一边的记分板，上一步根据其图像的左侧或右侧来判断，不够准确
    // 为保证透视变换精度，这里按照实际比例关系搜索其另一列相应位置是否符合条件的记分板矩形，如有，则放入容器contours2中
    // 供下一步透视变换使用


    // 函数说明：idx_vector_analysis（），自定义函数，可查看功能
    // 函数作用：分析当前检测到的计分板结果属于哪种情况
    // 输入参数：计分板编号容器
    // return：检测结果所属情况编号
    int condition = idx_vector_analysis(contours_idx3);

    //如果condition=0,说明记分板编号失败，忽略当前帧
    if (condition == 0)
    {
#ifdef show
        imshow("src0",src);
        waitKey(1);
#endif
        return false;
    }


    // 对计分板进行位置修正等操作，根据不同的condition，分情况处理，以保证1-5,10-14编号的记分板在正确的位置上
    //correct_scoreboard（）：自定义函数，功能可跳转查询
    correct_scoreboard(condition, contours_idx3, contours2, src.cols);
    ////位置修正后的记分板对应编号对应位置，以便之后处理使用////

    // 求单应矩阵，透视变换
    srcPts.clear(); // srcPts容器清空操作
    dstPts.clear(); // dstPts容器清空操作

    // 将检测到的计分板坐标放入图像坐标srcPts中
    //遍历contours_idx3
    for (size_t j = 0; j < contours_idx3.size(); j++)
    {
        //contours_idx3[j]为编号，大于零且小于二十时，存入角点坐标
        if (contours_idx3[j] > 0 && contours_idx3[j] < 20)
        {
            // 每个矩形角点坐标顺序均为：顺时针（左上、右上、右下、左下）
            srcPts.push_back(contours2[j][0]);
            srcPts.push_back(contours2[j][1]);
            srcPts.push_back(contours2[j][2]);
            srcPts.push_back(contours2[j][3]);
            ////此步即完成了实际记分板坐标放入图像坐标srcPts中


            vector<Point2f> pp; //角点坐标存入pp
            //根据编号对应每个记分板的角点信息，传给pp
            switch (contours_idx3[j])
            {
                case 1:
                    pr.getData("left5", pp);
                    break;
                case 2:
                    pr.getData("left4", pp);
                    break;
                case 3:
                    pr.getData("left3", pp);
                    break;
                case 4:
                    pr.getData("left2", pp);
                    break;
                case 5:
                    pr.getData("left1", pp);
                    break;
                case 10:
                    pr.getData("right5", pp);
                    break;
                case 11:
                    pr.getData("right4", pp);
                    break;
                case 12:
                    pr.getData("right3", pp);
                    break;
                case 13:
                    pr.getData("right2", pp);
                    break;
                case 14:
                    pr.getData("right1", pp);
                    break;
                default:
                    break;
            }

            // 映射图像中包含上方的数码管，但是检测数据中不包含。因此要对其中的点做坐标平移，即截取含有计分板而不含有数码管的区域
            //将实际角点的z型定义转换为顺时针的角点坐标定义，将角点坐标下移Point2f(0, 174.7f)) * 0.25f)（根据实际计算出来）
            dstPts.push_back((pp[0] + Point2f(0, 174.7f)) * 0.25f);
            dstPts.push_back((pp[1] + Point2f(0, 174.7f)) * 0.25f);
            dstPts.push_back((pp[3] + Point2f(0, 174.7f)) * 0.25f);
            dstPts.push_back((pp[2] + Point2f(0, 174.7f)) * 0.25f);
            ////此步即完成了将坐标平移后的标准角点坐标按照顺时针顺序存入dstPts中
        }
    }

    // 求单应矩阵        0表示使用所有点的常规方法;CV_RANSAC 基于RANSAC鲁棒性的方法；CV_LMEDS 最小中值鲁棒性方法
    Mat H = findHomography(srcPts, dstPts, 0);
    //单应矩阵的作用：实际坐标srcPts×单映矩阵H=标准坐标dstPts，现在已有标准九宫格中心点坐标dst_squares，用单应矩阵逆×标准坐标dst_squares，反求得实际坐标src_p
    // 矩阵求逆
    Mat H_inv = H.inv();

    // CV_64F(64位数据)，则进行类型转换
    if (H_inv.type() == 6)
        //.convertTo（）：类型转换为CV_32F（32位数据）
        H_inv.convertTo(H_inv, CV_32F);

    // 根据单应矩阵计算九宫格九个中心点在实际图像中的像素坐标
    vector<Point2f> classid_t;
    for (size_t j = 0; j < 9; j++)
    {   //利用单应矩阵和标准九宫格中心点坐标，求得实际九宫格中心点坐标
        Mat src_p = H_inv * dst_squares[j];

        // 对src_p进行读写，得到(x, y, z)
        float src_arr[3] = {src_p.at<float>(0), src_p.at<float>(1), src_p.at<float>(2)};

        // (x, y, z) ---> (x/z, y/z, 1),放入classid_t中
        classid_t.emplace_back(Point2f(src_arr[0] / src_arr[2], src_arr[1] / src_arr[2]));
    }
    ////此次遍历结束之后，classid_t存入了计算出来的实际九宫格九个中心点坐标
    Mat dst;

    /* 透视变换
     * src2：原始图像
     * dst ：矫正图像
     * H：   单应矩阵
     * Size(385, 216)：透视变换到较小的尺寸（正好包含数码管和九宫格区域）(x轴y轴同时进行缩放)
     */
    warpPerspective(src2, dst, H, Size(385, 216));

#ifdef show
    //显示经透视变换得到矫正后的图像dst
    imshow("dst", dst);
#endif

    // 提取数码管ROI,应为整体的数码管ROI
    //rect_digital_tube(): Rect(x坐标，y坐标，width,height)
    // cvRound(): 返回跟参数最接近的整数值
    Rect rect_digital_tube(cvRound(dst.cols * 0.3f), 0, cvRound(dst.cols * 0.3975f), cvRound(dst.rows * 0.185f));

    // 数码管识别结果放入digital_tube_v中
    vector<int> digital_tube_v;

    // 数码管数字识别，(使用SVM分类方法:在DigitronDetector(svm_tube).hpp中),返回到digital_tube_v
    hb_detector.process(dst(rect_digital_tube).clone(), digital_tube_v);
    ////此步完成了数码管的数字识别，并将结果(五个数字)放入 digital_tube_v

#ifdef show
    imshow("digital_tube", dst(rect_digital_tube));
    imshow("hb_detector", hb_detector.warp_image);

    int xxx = 100;
    // 显示数码管检测结果
    //遍历过程中获得容器里的每一个元素
    for (auto d : digital_tube_v)
    {   //putText()：在图像上绘制文字,注第三个参数：Point(xxx, 50):文本框的左下角。其他参数详情查找资料
        //（100,50），（150,50），（200,50），（250,50），（300,50）
        putText(src, to_string(d), Point(xxx, 50), CV_FONT_HERSHEY_DUPLEX, 1.0, color[0], 2, LINE_AA);
        xxx += 50;
    }
#endif

    // 检测结果应当包括5个，如数量不正确，则直接忽略
    if (digital_tube_v.size() == 5)
    {   //遍历操作
        for (int j=0; j<5; j++)
            //依次放入digitalTube[]中（数码管识别结果）
            digitalTube[j] =  digital_tube_v[j];
    }

    // 未检测到数码管，忽略当前帧
    else
    {
#ifdef show
        imshow("src0",src);
        waitKey(1);
#endif
        return false;
    }


    // 提取并保存九宫格Mat
    vector<Mat> dafu_mat;
    for (auto &r : minRect) //r是一个常量引用，绑定到minRect
        //提取出dst中的九宫格放入dafu_mat
        dafu_mat.push_back(dst(r));

    // CNN九宫格手写数字分类
    nc.predict(dafu_mat, classId, classProb);

    // 分类结果为18类（小符9类 + 大符9类），小符1-9，大符10-18
    for (size_t i = 0; i < 9; i++)
    {   //分出小符1-9
        classId[i] = classId[i] > 9 ? classId[i] - 9 : classId[i];
    }
#ifdef show
    // 显示大符ROI
    //dafu_mat_t(_rows,_cols,type,scalar);
    Mat dafu_mat_t(120, 126, CV_8UC3, Scalar(0, 0, 0));
    //将minRect的坐标复制到dafu_mat_t中，目的是方便观察手写体数字
    dst(minRect[0]).copyTo(dafu_mat_t(Rect(0, 0, 42, 40)));
    dst(minRect[1]).copyTo(dafu_mat_t(Rect(42, 0, 42, 40)));
    dst(minRect[2]).copyTo(dafu_mat_t(Rect(84, 0, 42, 40)));
    dst(minRect[3]).copyTo(dafu_mat_t(Rect(0, 40, 42, 40)));
    dst(minRect[4]).copyTo(dafu_mat_t(Rect(42, 40, 42, 40)));
    dst(minRect[5]).copyTo(dafu_mat_t(Rect(84, 40, 42, 40)));
    dst(minRect[6]).copyTo(dafu_mat_t(Rect(0, 80, 42, 40)));
    dst(minRect[7]).copyTo(dafu_mat_t(Rect(42, 80, 42, 40)));
    dst(minRect[8]).copyTo(dafu_mat_t(Rect(84, 80, 42, 40)));
    //显示dafu_mat_t
    imshow("dafu", dafu_mat_t);
#endif

    // 对检测得到的内容进行分析：
    // 如1-9各出现一次，则认为检测成功，返回true
    // 如9个数字中某一数字出现2次，则根据置信度判断哪一数字为真，哪一数字去填未出现数字的坑，之后返回true
    // 其他情况返回false

    //如果dafuNumAnalysis()为false，忽略当前帧图片
    if (!dafuNumAnalysis(classId, classProb))
    {
#ifdef show
        for(int i=0;i<9;i++)
            //显示九宫格检测结果
            putText(src, to_string(classId[i]), classid_t[i], CV_FONT_HERSHEY_DUPLEX, 1.0, color[0], 2, LINE_AA);
        imshow("src0",src);
        waitKey(1);
#endif
        return false;
    }

#ifdef show
    for(int i=0;i<9;i++)
        //显示九宫格检测结果
        //putText()：在图像上绘制文字,注第三个参数：classid_t[i]:文本框的左下角。
        putText(src, to_string(classId[i]), classid_t[i], CV_FONT_HERSHEY_DUPLEX, 1.0, color[0], 2, LINE_AA);

    // 绘制最后符合条件的矩形框（绿）
    for (size_t j = 0; j < contours2.size(); j++)
    {
        if (contours_idx3[j] > 0)
        {
            for (size_t k = 0; k < 4; k++)
            {
                //画直线，直线的起点为contours2[j][k]，直线的终点为contours2[j][(k + 1) % 4]，也即遍历画出所有符合要求的矩形框
                line(src, contours2[j][k], contours2[j][(k + 1) % 4], Scalar(0, 255, 0), 1, LINE_AA);
            }
        }
    }
#endif


    /////////////////// Shot /////////////////////
    /**大小符打击逻辑：
     * 按照数码管显示的数字顺序依次打击九宫格出现的手写体数字
     * 如果打击成功，数码管数字不变，九宫格数字切换，记分板顺序亮
     * 如果打击不成功，则数码管，九宫格立即同时切换
     * 如果中途1.5秒时间内没有进行打击，则数码管九宫格同时切换，重新从第一位打击，记分板记分消失
     * 如果中途击打失败，则数码管切换，重新从第一位打击，记分板之前记分消失
     * **/

    // 和前一帧相比，数码管数字变化的数量
    int tubeChangedCount = 0;
    for(int m = 0;m < 5; m++)
        if(digitalTubeLast[m] != digitalTube[m])
            tubeChangedCount++;

    // 和前一帧相比，九宫格数字变化的数量
    int dafuChangedCount = 0;
    for (int j = 0; j < 9; ++j)
        if (classId[j] != classIdLast[j])
            dafuChangedCount++;
    //数码管数字变化量>2,不是1的原因是可能会出现误识别的情况 5/2=2
    isDigitalTubeChanged = tubeChangedCount > 2;
    //九宫格数字变化量>4,不是1的原因是可能会出现误识别的情况 9/2=4
    isDafuChanged = dafuChangedCount > 4;

    // 分情况分别讨论

    // 1.数码管数字变化
    if (isDigitalTubeChanged)
    {
        // 重新打击数码管第一位
        orderOfTubeNum = 0;

        // 1.1大符画面切换
        if (isDafuChanged)
        {
            for (int j = 0; j < 9; ++j)
            {
                // 发送和当前要打击数码管数字相同的大符数字像素坐标（原图坐标，即正视图的反透视变换坐标）
                //手写体结果和数码管第一位结果相同
                if (classId[j] == digitalTube[orderOfTubeNum])
                {
                    // src图片在一开始resize得到0.6,所以这里要除以0.6获得实际像素位置
                    Point2f tp(classid_t[j].x / 0.6f, classid_t[j].y / 0.6f);
                    //把实际像素坐标赋给p2f_angle（实际打击点像素坐标）
                    p2f_angle = tp;
                    //数码管识别结果
                    send_num = digital_tube_v[orderOfTubeNum];
                    cout << "target point pix -------------------> " << tp << endl;
                    cout << "orderOfTubeNum -------------------> " << orderOfTubeNum << endl;
                    break;
                }
            }
            // 更新当前大符识别结果为上一帧数据（用来和下一次处理结果作比较）
            for (int k = 0; k < 9; ++k)
            {
                classIdLast[k] = classId[k];
            }

            // 更新当前数码管数字为上一帧数据（用来和下一次处理结果作比较）
            for (int k = 0; k < 5; ++k)
            {
                digitalTubeLast[k] = digitalTube[k];
            }
        }
        // 1.2大符画面未切换
        //此时数码管改变，但检测到的大符未变，说明此时九宫格未更新，不能打
        else
        {

#ifdef show
            imshow("src0",src);
            imshow("dst",dst);
            waitKey(1);
#endif
            cout<<"数码管改变，但检测到的大符未变"<<endl;
            return false;
        }
    }

    // 2.数码管数字未变化
    else
    {
        // 2.1大符画面切换，说明打击成功
        if (isDafuChanged)
        {
            // 继续发送下一位数字
            orderOfTubeNum++;
            // 发送和当前要打击数码管数字相同的大符数字像素坐标（原图坐标，即正视图的反透视变换坐标）
            //手写体结果和数码管下一位结果相同
            for (int j = 0; j < 9; ++j)
            {
                if (classId[j] == digitalTube[orderOfTubeNum])
                {
                    // src图片在一开始resize得到0.6,所以这里要除以0.6获得实际像素位置
                    Point2f tp(classid_t[j].x / 0.6f, classid_t[j].y / 0.6f);
                   //实际打击点像素坐标
                    p2f_angle = tp;
                    //数码管识别结果
                    send_num = digital_tube_v[orderOfTubeNum];
                    cout << "target point pix -------------------> " << tp << endl;
                    cout << "orderOfTubeNum -------------------> " << orderOfTubeNum << endl;
                    break;
                }
            }
            // 更新当前大符识别结果为上一帧数据（用来和下一次处理结果作比较）
            for (int k = 0; k < 9; ++k)
            {
                classIdLast[k] = classId[k];
            }
            //数码管五位数字遍历顺利完成，打击成功，退出程序
            if (orderOfTubeNum >= 4)
            {
                cout<<"这一轮数码管打击完毕"<<endl;
#ifdef show
                imshow("src0",src);
                imshow("dst",dst);
                waitKey(1);
#endif
                return false;
            }
        }
        // 2.2大符画面未切换
        // 数码管数字未变化，大符画面未变化，说明此时九宫格未更新，不能打
        else
        {

#ifdef show
            imshow("src0",src);
            imshow("dst",dst);
            waitKey(1);
#endif
            return false;
        }
    }
#ifdef show
    imshow("src0",src);
    imshow("dst",dst);
    waitKey(1);
#endif
    return true;
}


/* 函数作用：CNN识别结果中如果有且仅有一组数字识别重复，则根据准确率判别并输出正确结果，
 *          有两组及以上两个数字重复或者同一个数字出现三次以上则舍弃当前识别结果
 * classId：分类结果
 * classProb：每种类别对应的概率
 */

bool DafuDetector::dafuNumAnalysis(int * classId, double * classProb)
{
    // 保存每个数字出现次数的数组
    int  idx_count[9] = {0};

    // 同一个数字出现两次的次数
    int pairs = 0;

    // 重复的数字
    int same_num = 0;

    // 统计每个数字出现的次数
    for (int i = 0; i < 9; ++i) {
        //classId[i]-1：定义idx_count[]数组上限最大到8
        idx_count[classId[i] - 1]++;
    }
    for (int i = 0; i < 9; ++i)
    {
        // 某个数字出现超过三次，直接返回
        if (idx_count[i] >= 3)
        {
            return false;
        }

        // 某个数字出现两次
        else if (idx_count[i] == 2)
        {
            //确定重复的数字
            same_num = i+1;
            //同一个数字出现两次的次数
            pairs++;
        }
        // 循环结束时判别上述条件，前面的循环不进入此条件判断
        else if (i == 8 && idx_count[i] == 1 && pairs == 0)
            return true;
    }

    // 用来保存重复的两个数字索引值的数组
    int idx[2] = {-1};

    // 缺失的数字
    int miss_num = 0;

    // 同一个数字重复的情况出现超过两次，直接返回false
    if (pairs > 1)
        return false;
    else
    {
        for (int i = 0; i < 9; ++i)
        {
            // 查找出现0次的数字
            if (idx_count[i] == 0)
                miss_num = i+1;

            // 记录重复的两个数字的索引值,第一次给idx[0]赋值为i后，idx[0]！= -1，进入else if
            if (classId[i] == same_num && idx[0] == -1)
                idx[0] = i;
            else if (classId[i] == same_num && idx[0] != -1)
                idx[1] = i;
        }
    }

    // 根据概率判别两个重复的数字的正确值
    if (classProb[idx[0]] > classProb[idx[1]])
        classId[idx[1]] =  miss_num;
    else
        classId[idx[0]] =  miss_num;
    return true;
}


// 分析contours_idx3，分4种情况,1,2,3进入correct_scoreboard函数进行处理
// 0. 1~5，10~14都没有
// 1. 检测到1~5，10~14，说明提取到了10个记分板，此时要判断哪个在左、哪个在右？如果1~5在右，需要调换位置
// 2. 只检测到1~5，说明只提取了一侧的记分板，需要判断记分板是左侧还是右侧的（在画面左半边就是左侧，右半边就是右侧）
// 3. 只检测到10~14，与上面相似

/* 参数说明：
 * condition：   计分板检测结果所属类别（对应上述0,1,2,3）
 * contours_idx：计分板轮廓对应的编号（正常情况：左侧1~5，右侧10~14）
 * contours：    计分板轮廓
 * src_cols：    整幅图像宽度
 */
void DafuDetector::correct_scoreboard(int condition,vector<int> & contours_idx,const vector<vector<Point2f> > & contours,int src_cols)
{
    if (condition == 1)     //即左右两边记分板全部检测到
    {
        // 比较计分板1和10的左上角点横坐标大小。如差值大于0，说明1-5计分板横坐标大，在右侧，需要调换两列计分板编号
        if ((contours[get_vector_idx(contours_idx, 1)][0].x - contours[get_vector_idx(contours_idx, 10)][0].x) > 0)
        {
            //遍历以调换两列计分板编号
            for (int k = 0; k < contours_idx.size(); k++)
            {
                if (contours_idx[k] == 1)
                    contours_idx[k] = 10;
                else if (contours_idx[k] == 2)
                    contours_idx[k] = 11;
                else if (contours_idx[k] == 3)
                    contours_idx[k] = 12;
                else if (contours_idx[k] == 4)
                    contours_idx[k] = 13;
                else if (contours_idx[k] == 5)
                    contours_idx[k] = 14;
                else if (contours_idx[k] == 10)
                    contours_idx[k] = 1;
                else if (contours_idx[k] == 11)
                    contours_idx[k] = 2;
                else if (contours_idx[k] == 12)
                    contours_idx[k] = 3;
                else if (contours_idx[k] == 13)
                    contours_idx[k] = 4;
                else if (contours_idx[k] == 14)
                    contours_idx[k] = 5;
            }
        }
    }

    else if (condition == 2)    //即只检测到1-5编号的记分板
    {
        // 判断计分板横坐标与图像中心的关系，记分板1的左上角右上角横坐标之和除2可得到记分板1的中心点横坐标，与整幅图像的宽度的一般做比较，来判断左右
        // 若检测到1号计分板位于整幅图像的右侧，则将这一列计分板改为10-14,将其余编号置零
        if ((contours[get_vector_idx(contours_idx, 1)][0].x + contours[get_vector_idx(contours_idx, 1)][1].x) / 2 > src_cols / 2)
        {
            //遍历操作以调整记分板编号
            for (int k = 0; k < contours_idx.size(); k++)
            {
                if (contours_idx[k] == 1)
                    contours_idx[k] = 10;
                else if (contours_idx[k] == 2)
                    contours_idx[k] = 11;
                else if (contours_idx[k] == 3)
                    contours_idx[k] = 12;
                else if (contours_idx[k] == 4)
                    contours_idx[k] = 13;
                else if (contours_idx[k] == 5)
                    contours_idx[k] = 14;
                else if (contours_idx[k] == 10)
                    contours_idx[k] = 0;
                else if (contours_idx[k] == 11)
                    contours_idx[k] = 0;
                else if (contours_idx[k] == 12)
                    contours_idx[k] = 0;
                else if (contours_idx[k] == 13)
                    contours_idx[k] = 0;
                else if (contours_idx[k] == 14)
                    contours_idx[k] = 0;
            }
        }

        // 若1号计分板位于整幅图像的左侧，则将10-14号（不足5个）编号置为0
        else {
            for (int k = 0; k < contours_idx.size(); k++) {
                if (contours_idx[k] == 10)
                    contours_idx[k] = 0;
                else if (contours_idx[k] == 11)
                    contours_idx[k] = 0;
                else if (contours_idx[k] == 12)
                    contours_idx[k] = 0;
                else if (contours_idx[k] == 13)
                    contours_idx[k] = 0;
                else if (contours_idx[k] == 14)
                    contours_idx[k] = 0;
            }
        }
    }

    // 同上
    else if (condition == 3)    //即只检测到编号10-14的记分板
    {       // 若检测到10号计分板位于整幅图像的左侧，则将这一列计分板改为1-5，其余编号置零
        if ((contours[get_vector_idx(contours_idx, 10)][0].x + contours[get_vector_idx(contours_idx, 10)][1].x) / 2 < src_cols / 2)
        {    //遍历操作以调整记分板编号
            for (int k = 0; k < contours_idx.size(); k++)
            {
                if (contours_idx[k] == 1)
                    contours_idx[k] = 0;
                else if (contours_idx[k] == 2)
                    contours_idx[k] = 0;
                else if (contours_idx[k] == 3)
                    contours_idx[k] = 0;
                else if (contours_idx[k] == 4)
                    contours_idx[k] = 0;
                else if (contours_idx[k] == 5)
                    contours_idx[k] = 0;
                else if (contours_idx[k] == 10)
                    contours_idx[k] = 1;
                else if (contours_idx[k] == 11)
                    contours_idx[k] = 2;
                else if (contours_idx[k] == 12)
                    contours_idx[k] = 3;
                else if (contours_idx[k] == 13)
                    contours_idx[k] = 4;
                else if (contours_idx[k] == 14)
                    contours_idx[k] = 5;
            }
        }
        else
            {   // 若10号计分板位于整幅图像的右侧，则将1-5（不足5个）编号置为0
                for (int k = 0; k < contours_idx.size(); k++)
                {
                    if (contours_idx[k] == 1)
                        contours_idx[k] = 0;
                    else if (contours_idx[k] == 2)
                        contours_idx[k] = 0;
                    else if (contours_idx[k] == 3)
                        contours_idx[k] = 0;
                    else if (contours_idx[k] == 4)
                        contours_idx[k] = 0;
                    else if (contours_idx[k] == 5)
                        contours_idx[k] = 0;
                }
            }
    }


    // 针对检测到完整的一列计分板，同时另一列只检测到部分（不足5个）的情况，为了提高最后的分割精度，
    // 选择使用6（5 + 1）个计分板来进行计算单映矩阵，但是需要确定第6个计分板的编号，下面这部分程序就是这个功能


    // 思路：依次对完整一列的计分板计算上边所在直线关系式，判断在另一列是否存在单一的计分板，满足其左上顶点坐标在直线附近的条件
    //      如满足，则可以推断出该计分板的编号


    // 再次获取condition,这一状态下即为修正之后的值，各个计分板严格对应其位置
    int condition2 = idx_vector_analysis(contours_idx);

    // 存在1-5（即左侧一列）计分板
    if (condition2 == 2)
    {
        bool isOK = false;
        for (int j = 0; j < 5; j++)
        {
            // 获取对应编号计分板轮廓的索引值，找到1-5记分板的索引值
            int contour_idx_t = get_vector_idx(contours_idx, j + 1);

            // 计分板左上、右上角点
            Point2f p2f_t1 = contours[contour_idx_t][0];
            Point2f p2f_t2 = contours[contour_idx_t][1];

            /*
             * 公式推导：两点法取直线后,已知x的值，求对应的y值
             * (y-y1)/(x-x1)=(y2-y1)/(x2-x1) ---斜率相等
             * y-y1=(x-x1)*(y2-y1)/(x2-x1)
             * y=(x-x1)*(y2-y1)/(x2-x1)+y1
             * 获取计算得到的y值后，再与实际图像中的值进行比较，判断是否符合条件
            */

            // 确定检测范围，即矩形高的一半：左下角y坐标与左上角y坐标之差除以2（二分之一这个值应该是测试过的，之前放到2/3会导致检测错误）
            float dis1 = (contours[contour_idx_t][3].y - contours[contour_idx_t][0].y) / 2;

            for (int k = 0; k < contours.size(); k++)
            {
                // 先判断是否在另一列（根据前面的程序可知：在condition=2下，已将记分板另一列所有编号置0，这里根据是否置0判断是否在另一列）
                //（这里的contours是另一列的矩形，要与之前编号1-5的记分板区分开来）
                if (contours_idx[k] == 0)
                {
                    // 保证在图像的右侧（左上角x坐标大于总宽度的一半）
                    if (contours[k][0].x > src_cols / 2)
                    {
                        // (p2f_t2.y - p2f_t1.y) / (p2f_t2.x - p2f_t1.x)为斜率
                        // 公式法带入：y=(x-x1)*(y2-y1)/(x2-x1)+y1
                        float y_t = (contours[k][0].x - p2f_t1.x) * (p2f_t2.y - p2f_t1.y) / (p2f_t2.x - p2f_t1.x) + p2f_t1.y;

                        float slopeCol    = (float)(p2f_t2.y - p2f_t1.y) / (p2f_t2.x - p2f_t1.x);
                        cout<<"计分板斜率： "<<slopeCol<<endl;

                        float slopeSingle = (float)(contours[k][1].y - contours[k][0].y) / (contours[k][1].x - contours[k][0].x);
                        cout<<"矩形斜率： "<<slopeSingle<<endl;

                        // 判断单一计分板左上角点坐标是否满足在直线的小范围内
                        if (abs(y_t - contours[k][0].y) < dis1)
                        {
                            // 判断斜率是否相近（忘记最后是否使用，以NUC中最终版代码为准）
                            if(abs(slopeCol - slopeSingle) < 0.04)
                            {
                                // 所有条件都满足，则推断出该计分板编号
                                contours_idx[k] = j + 10;
                                isOK = true;
                                break;
                            }
                        }
                    }
                }
            }
            if (isOK) break;
        }
    }

    // 存在10-14（即右侧一列）计分板
    // 同上
    else if (condition2 == 3)
    {
        bool isOK = false;
        for (int j = 0; j < 5; j++)
        {
            int contour_idx_t = get_vector_idx(contours_idx, j + 10);
            Point2f p2f_t1 = contours[contour_idx_t][0];
            Point2f p2f_t2 = contours[contour_idx_t][1];
            float dis1 = (contours[contour_idx_t][3].y - contours[contour_idx_t][0].y) / 2;
            for (int k = 0; k < contours.size(); k++)
            {
                if (contours_idx[k] == 0)
                {
                    if(contours[k][0].x < src_cols / 2)
                    {
                        float y_t = (contours[k][0].x - p2f_t1.x) * (p2f_t2.y - p2f_t1.y) / (p2f_t2.x - p2f_t1.x) + p2f_t1.y;

                        float slopeCol    = (float)(p2f_t2.y - p2f_t1.y) / (p2f_t2.x - p2f_t1.x);
                        cout<<"计分板斜率： "<<slopeCol<<endl;

                        float slopeSingle = (float)(contours[k][1].y - contours[k][0].y) / (contours[k][1].x - contours[k][0].x);
                        cout<<"矩形斜率： "<<slopeSingle<<endl;

                        if (abs(y_t - contours[k][0].y) < dis1)
                        {

                            if(abs(slopeCol - slopeSingle) < 0.04)
                            {
                                contours_idx[k] = j + 1;
                                isOK = true;
                                break;
                            }
                        }
                    }
                }
            }
            if (isOK) break;
        }
    }
}


// 函数作用：查找v中对应数值为k的索引
// 在程序中表现形式为在所有轮廓中查找编号的轮廓中对应的第一个编号的索引
int DafuDetector::get_vector_idx(const vector<int> & v, int k)
{
    for (int i = 0; i < v.size(); i++)
    {
        if (v[i] == k)
            return i;
    }

    return -1;
}

// 函数作用：分析当前检测到的计分板所属情况
// v：矩形框（候选计分板）编号
int DafuDetector::idx_vector_analysis(const vector<int> & v)
{
    int a[10];
    for (int i = 0; i < 10; i++)
        a[i] = 0;
    //在遍历过程中获得容器v里的每一个元素
    //循环的目的是数字给a[]里的元素进行赋值
    for (auto i : v)
    {
        //在v中，也即contours_idx3中，只有十个数字，分别为左右记分板的编号，一旦遍历寻找到其中的数字，将1赋给数组a[]，立即退出进行下一次遍历寻找
        switch (i)
        {
            case 1:
                a[0] = 1;
                break;
            case 2:
                a[1] = 1;
                break;
            case 3:
                a[2] = 1;
                break;
            case 4:
                a[3] = 1;
                break;
            case 5:
                a[4] = 1;
                break;
            case 10:
                a[5] = 1;
                break;
            case 11:
                a[6] = 1;
                break;
            case 12:
                a[7] = 1;
                break;
            case 13:
                a[8] = 1;
                break;
            case 14:
                a[9] = 1;
                break;
        }
    }
    //定义几个整型临时变量
    int a1 = 0, a2 = 0, a3 = 0;
    //循环五次
    for (int i = 0; i < 5; i++)
    {
        //累加左边记分板提取个数
        a1 += a[i];
        //累加右边记分板提取个数
        a2 += a[i + 5];
    }
    //统计提取到的记分板的总数
    a3 = a1 + a2;
    if (a3 == 10)
        return 1;		// 1.存在1~5，10~14，十个记分板都提取到
    else if (a1 == 5 && a2 != 5)
        return 2;		// 2.存在1~5，其他不确定
    else if (a1 != 5 && a2 == 5)
        return 3;		// 3.存在10~14，其他不确定
    else
        return 0;       // 0.
}

// 矩形框定点排序  sortPoints()
// p_：含有矩形框四个顶点的容器
void DafuDetector::sortPoints(vector<Point>& p_)
{
    for (int i = 0; i < 3; i++)
        for (int j = i + 1; j < 4; j++)
            if (p_[i].y > p_[j].y)
                swap(p_[i], p_[j]);

    if (p_[0].x > p_[1].x)
        swap(p_[0], p_[1]);
    if (p_[2].x < p_[3].x)
        swap(p_[2], p_[3]);
}

// 计算两点间距离的平方
int DafuDetector::calc_len(const Point & p1, const Point & p2)
{
    return (p1.x - p2.x) * (p1.x - p2.x) + (p1.y - p2.y) * (p1.y - p2.y);
}

// 计算两点间距离的平方
float DafuDetector::calc_len(const Point2f & p1, const Point2f & p2)
{
    return (p1.x - p2.x) * (p1.x - p2.x) + (p1.y - p2.y) * (p1.y - p2.y);
}

// 判断点p1是否在以p2为中心，2r为边长的正方形区域内（不含边界）
bool DafuDetector::ptInRange(const Point2f & p1, const Point2f & p2, const float r)
{
    if (p1.x > (p2.x - r) && p1.x < (p2.x + r) && (p1.y >(p2.y - r) && p1.y < (p2.y + r)))
        return true;
    else
        return false;
}

#endif