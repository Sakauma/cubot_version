#ifndef DIGITRONDETECTOR_H_FILE
#define DIGITRONDETECTOR_H_FILE

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <opencv2/core/core.hpp>  
#include <iterator>

using namespace cv;
using namespace std;

/* 大符数码管识别 */
class digitronDetector
{
public:
    //输入数码管原图像,返回数码管数字结果
	void process(const Mat& singleNumberImage, vector<int>& result);
	//bgr_image：三通道彩色图像
	//hsv_image：利用hsv约束后的图像
	//and_image：与运算后的图像
	//warp_image：透视变换后的图像
    Mat bgr_image,hsv_image,and_image,warp_image;

private://各个函数的定义,如下对每个函数都有详细的描述

	int blueThreshold=255, redThreshold=170;
    //再次进行透视变换
	int perspectiveK = 6.5;//6.5 透视变换的系数 勿动

	void BubbleSort(int  *p, int length, int * ind_diff);//带索引的排序

	bool isExist(vector<int>& src, int t);//检查t是否已经存在于in中

	void warpTrans(Mat& src, Mat& dst);//预变换 不适用
	void correctImage(Mat& src, Mat&dst);

	int getNumber(Mat& singleNumberImage);//单个数字识别
    //加载训练好的SVM模型
	Ptr<cv::ml::SVM> svm0 = cv::ml::SVM::load("../cfg/digital_svm_model_7_9.xml");
	/*****************************
	 * HOGDescriptor（）五个参数详解：对图像img->窗口window->块block->细胞单元cell进行向量统计
	 * winSize（24,30）：窗口大小
	 * blockSize（6, 6）：块大小
	 * blockStride（6, 6）：块滑动增量
	 * cellSize（3, 3）：胞元大小
	 * nbins：梯度方向数  例如nBins=9时,在一个胞元内统计9个方向的梯度直方图,每个方向为180/9=20度.
	 * 每一个cell有9个向量,对每一个block有2×2个cell,对于window而言,计算block个数的方法是：对两个方向计算 (window_size - block_size)/block_stride + 1,算得共有4*5个block,共有9×4×20=720个向量
	 ****************************/
	HOGDescriptor * hog_digital = new HOGDescriptor(Size(24,30), Size(6, 6), Size(6, 6), Size(3, 3), 9);

	int predictDigital(Mat &m);

};

/*********************************
 *
 * Description: 带索引的冒泡排序
 * Input:
 * 		p: 待排序的数组
 * 		length： 数组长度
 * 		ind_diff： 排序后的元素在原数组中的索引(升序)
 * Output：
 * 		无
 * Example：
 * 		p：[5,1,2,6,8]，length：5
 * 		ind_diff： [1,2,0,3,4]
 *
**********************************/

void digitronDetector::BubbleSort(int *p, int length, int * ind_diff)
{
	for (int m = 0; m < length; m++) //初始化索引数组
	{   //按顺序初始化索引
		ind_diff[m] = m;
	}
    //冒泡排序法：遍历和交换,个人理解冒泡排序只需要遍历 length-1 次即可,最后一次不必要遍历
	for (int i = 0; i < length; i++)    //for (int i = 0; i < length - 1 ; i++)
	{
		for (int j = 0; j < length - i - 1; j++)
		{
			if (p[j] > p[j + 1])
			{   //冒泡排序,交换数字位置,从小到大正序排序
				float temp = p[j];
				p[j] = p[j + 1];
				p[j + 1] = temp;
                //记录索引,交换索引值位置,对应正序排序,存储在ind_diff
				int ind_temp = ind_diff[j];
				ind_diff[j] = ind_diff[j + 1];
				ind_diff[j + 1] = ind_temp;
			}
		}
	}
}

/*********************************
 *
 * Description: 判断元素是否存在于容器中
 * Input:
 * 		src： 待查找的容器
 * 		t： 目标元素
 * Output：
 * 		bool: 是否存在于容器中
 * Example：
 * 		Input: src：[5,1,2,6,8], t：5
 * 		Output： true
 *
**********************************/

bool digitronDetector::isExist(vector<int>& src, int t)
{
	for (size_t i = 0; i < src.size()-1; i++)
	{
		if (t == src[i])
			return true;
	}
	return false;
}

/*********************************
 *
 * Description: SVM识别单个数码管数字
 * Input:
 * 		m: 经预处理后的单个数码管图片
 * Output：
 * 		识别结果0-9
 *
**********************************/
int digitronDetector::predictDigital(Mat &m)
{
	Mat n;
	resize(m, n, Size(24, 30)); //将图像resize到24x30，以便于计算hog特征

	vector<float> descriptors; //存储描述子
	hog_digital->compute(n, descriptors); //计算hog特征     //hog_digital封装在 class digitronDetector中
    //创建一个一行n列的向量,n=descriptors.size()=特征的个数,dst用来存放当前要预测的图片的特征
	Mat dst(1, int(descriptors.size()), CV_32FC1, descriptors.data()); //将描述子展成一维，以便于送进svm训练
    //预测类别为int类型
	int result = svm0->predict(dst); //预测类别

	return result; //返回结果
}
/*********************************
 *
 * Description: 穿线法识单个数码管数字（比赛中不使用，用predictDigital代替）
 * Input:
 * 		image： 包含单个数码管的图片
 * Output：
 * 		int: 图片中数码管对应的数字
 * Others：
 * 		穿线法识别数码管，对预处理要求较高，比赛时不使用
 * 		穿线法算法流程请参考相关论文
 *
**********************************/
int digitronDetector::getNumber(Mat& image)
{
	Mat temp = image.clone();
	Mat lie = temp.colRange(int(temp.cols*1.0 / 2.0), int(temp.cols*1.0 / 2.0) + 1);
	Mat hang1 = temp.rowRange(int(temp.rows*1.0 / 3.0), int(temp.rows*1.0 / 3.0) + 1);
	Mat hang2 = temp.rowRange(int(temp.rows*2.0 / 3.0), int(temp.rows *2.0 / 3.0) + 1);
	int lie_count = 0, hang1_count = 0, hang2_count = 0;

	for (int i = 0; i < lie.rows - 1; i++) //29
	{
		uchar* pixelPtr0_lie = lie.ptr<uchar>(i);
		uchar* pixelPtr1_lie = lie.ptr<uchar>(i + 1);
		if (pixelPtr0_lie[0] == 255 && pixelPtr0_lie[0] != pixelPtr1_lie[0])
		{
			lie_count++;
		}
	}

	uchar* pixelPtr0_hang1 = hang1.ptr<uchar>(0);
	for (int i = 0; i < hang1.cols - 1; i++)
	{
		if (pixelPtr0_hang1[i] == 255 && pixelPtr0_hang1[i] != pixelPtr0_hang1[i + 1])
		{
			hang1_count++;
		}
	}

	uchar* pixelPtr0_hang2 = hang2.ptr<uchar>(0);
	for (int i = 0; i < hang2.cols - 1; i++)
	{
		if (pixelPtr0_hang2[i] == 255 && pixelPtr0_hang2[i] != pixelPtr0_hang2[i + 1])
		{
			hang2_count++;
		}
	}

	//cout << lie_count << " . " << hang1_count << " . " << hang2_count << endl;

	if (lie_count == 2 && hang1_count == 4 && hang2_count == 2)
	{
		return 4;
	}
	else if (lie_count == 6 && hang1_count == 2 && hang2_count == 4)
	{
		return 6;
	}
	else if (lie_count == 2 && hang1_count == 2 && hang2_count == 2)
	{
		return 7;
	}
	else if (lie_count == 6 && hang1_count == 4 && hang2_count == 4)
	{
		return 8;
	}
	else if (lie_count == 6 && hang1_count == 4 && hang2_count == 2)
	{
		return 9;
	}
	else if (lie_count == 6 && hang1_count == 2 && hang2_count == 2)
	{
		uchar* pixelPtr2 = hang2.ptr<uchar>(0);
		for (int i = 0; i < hang2.cols - 1; i++)
		{
			if (pixelPtr2[i] == 255 && pixelPtr2[i] != pixelPtr2[i + 1])
			{
				if (i<int(hang2.cols / 2))
				{
					return 2;
				}
				else
				{
					uchar* pixelPtr3 = hang1.ptr<uchar>(0);
					for (int j = 0; j < hang1.cols - 1; j++)
					{
						if (pixelPtr3[j] == 255 && pixelPtr3[j] != pixelPtr3[j + 1])
						{
							if (j<int(hang1.cols / 2))
								return 5;
							else
								return 3;
						}
					}
				}
			}
		}
	}
	else
	{
		return 10;//识别错误
	}

}

/*********************************
 *
 * Description: 对原始数码管图像进行预矫正，原始数码管为未分割的图像
 * Input:
 * 		src： 原始数码管图像（二值图像）,单通道图像
 * 		dst： 矫正后的数码管图像，五个数码管数字大体上竖直排列
 * Output：
 * 		void
 * Others：
 * 		1、找到数码管的最小包围矩形
 * 		2、根据最小包围矩形的旋转角度计算整幅图像的旋转矩阵
 * 		3、根据旋转矩阵对图像进行仿射变换
 *
**********************************/

void digitronDetector::correctImage(Mat& src, Mat& dst)
{   //src原图克隆
	Mat temp = src.clone();
    //膨胀操作图像
	Mat toDilate,dilateTo;
    //copyMakeBorder处理后的图像（给边原图像填充界像素）
	Mat image_pad;//padding
    //寻找轮廓操作后的轮廓点集
	vector<vector<Point>> contours;
	//hierarchy参数,四个参数
	vector<Vec4i> hierarchy;
    //遍历所有轮廓的每个点的点向量
	vector<Point> allPoints;
    //copyMakeBorder的参数,边界宽度
	int padding_n_ori = 15;
    //外接矩形面积
	RotatedRect minArea;
	//四个顶点坐标
	Point2f vertex[4];

	Point2f center;//旋转中心
    //调整图像大小操作,调整大小后的图像存入toDilate中
	resize(src,toDilate,Size(src.cols*3,src.rows*3)); //先将图像增大，便于形态学滤波
	//自定义内核大小,MORPH_RECT：内核形状为矩形    Size(3,3)：内核的尺寸为3×3
	Mat element = getStructuringElement(MORPH_RECT,Size(3,3));
	//膨胀操作，膨胀后的图片存入dilateTo中
	dilate(toDilate,dilateTo,element); //膨胀
	//调整图像大小操作，调整大小后的图像存入temp中
	resize(dilateTo,temp,Size(src.cols,src.rows)); //将尺寸缩放回原来的尺寸

	//copyMakeBorder():可以复制图像并制作边界,将特定图像轻微变大,然后以各种方式自动填充图像边界.
    //Bordertype=IPL_BORDER_CONSTANT时,有一个像素宽的黑色边界.
    //填充后的图像存入image_pad     之所以要padding之后的图像，是以免数码管紧挨着图像边缘导致后面的轮廓检测失效
	copyMakeBorder(temp, image_pad, padding_n_ori, padding_n_ori, padding_n_ori, padding_n_ori, BORDER_CONSTANT, Scalar(0));

	//查找数码管的轮廓，只检测最外层轮廓，只保存角点
	findContours(image_pad, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);

	//遍历数码管轮廓上的点并保存进allPoints
	//contours.size()：所有轮廓
	for (size_t i = 0; i < contours.size(); i++)
	{   //contours[i].size()：每个轮廓的每一个点
		for (size_t j = 0; j < contours[i].size(); j++)
		{   //轮廓的每个点存入allPoints中
			allPoints.push_back(contours[i][j]);
		}
	}
    //allPoints的个数大于50
	if (allPoints.size()>50)//如果图像中有数码管
	{   //minAreaRect():最小外接矩形操作
		minArea = minAreaRect(allPoints); //理解为整个数码管的最小包围矩形
		//
		minArea.points(vertex); //获取矩形四个顶点的坐标,顺时针,0号顶点在下方

		//图像的中心点坐标,也就是后面的旋转中心
		center.x = float((image_pad.cols*1.0) / 2.0 + 0.5);
		center.y = float((image_pad.rows*1.0) / 2.0 + 0.5);
        //旋转矩阵
		Mat rotationM;

		//判断vertex中每个坐标的顺序
		if (vertex[0].x < vertex[2].x)//vertex[0]在左下
		{	//getRotationMatrix2D()：图像围绕旋转中心的旋转矩阵
			rotationM = getRotationMatrix2D(center, minArea.angle, 1.0);// 计算旋转矩阵
		}
		else //vertex[0]在右下
		{	//旋转角度增加90度,使vertex[0]在左下
			rotationM = getRotationMatrix2D(center, 90.0 + minArea.angle, 1.0);// 计算旋转矩阵
		}
		//根据旋转矩阵进行仿射变换,仿射变换后的图像即是正的，运算结果存在dst中
		warpAffine(image_pad, dst, rotationM,Size(image_pad.size().width, image_pad.size().height));
	}
	else //若图片中无数码管,则返回空白图像
	{
		dst = Mat(src.size(), CV_8UC1, Scalar(0));
	}
}


/*********************************
 *
 * Description: 数码管识别主体函数,最终外部调用的接口
 * Input:
 * 		singleNumberImage： 原始数码管图像
 * 		result： 包含五个识别结果的容器，若返回-1，则表示识别失败
 * Output：
 * 		void
 * Others：
 * 		输入图片情况： 数码管不全
 * 					不包含数码管
 * 					数码管全但是有的不完整
 * 					正常数码管
 *
 * 		所用容器：	vector<int>& result                  形参     已清理    必须
 *					vector<vector<Point>> contours;      轮廓
 *					vector<Vec4i> hierarchy;             轮廓
 *					vector<Point2f> pts_src;             透视变换 已清理    非
 *					vector<Point2f> pts_dst;             透视变换 已清理    非
 *
**********************************/
void digitronDetector::process(const Mat& singleNumberImage, vector<int>& result)
{
	/*
		测试:
	*/

	Mat sliceImage(singleNumberImage.size(), CV_8UC1, Scalar(0)); //切片约束后的图像

	Mat hsv_splite;//利用hsv约束后的图像

	Mat andImage; //与运算后的图像

	Mat binary_r;//共用图片接口
	//轮廓点集
	vector<vector<Point>> contours;

	Rect singleNumber;//单个数字的包围框

	//单个图像的仿射变换
	vector<Point2f> pts_src;	//透视变换要用的实际坐标点集
	vector<Point2f> pts_dst;	//透视变换要用的标准坐标点集
	Mat transM;
	Mat transedImage;//变换后的数码管
	//偏差
	int bias = 0;

	int isOne = -1; //判断是否为1的flag

	//temp_finalPoints[5][5]对应存储的是5个数字最小外界矩形左上角点坐标，右下角点坐标，数码管是否为1信息
	//finalPoints[5]对应temp_finalPoints中存储的每个轮廓的外接矩形的左上角x坐标
	//intd[5]对应排序后的索引
	int temp_finalPoints[5][5], finalPoints[5], intd[5];//intd 排序后的索引

	Mat number,number_pad, canny_image;

	Mat hsv_temp;

	//这里我们同时进行两种处理，1、在HSV空间进行约束 2、直接在RGB空间进行约束，最后将两次的处理结果进行与运算获得最终的结果

	//将彩色图转换到HSV空间，转换后的图像存储在hsv_temp中
	//HSV：色相（H）：用度数来描述色相，其中红色对应0度，绿色对应120度，蓝色对应240度。
	// 饱和度（S）：Saturation即饱和度，色彩的深浅度(0-100%)
	// 明度（V）：Value即色调，纯度，色彩的亮度(0-100%)
	cvtColor(singleNumberImage, hsv_temp, COLOR_BGR2HSV);

	//这里约束了v通道，不需要调节，宽范围
	//输出的hsv_splite是一幅二值化之后的图像
	inRange(hsv_temp, Scalar(0, 0, 200), Scalar(180, 255, 255), hsv_splite);

	hsv_image = hsv_splite;//将处理后的图片保存到hsv_image，以便于调试

	//这里对RGB空间中的红色通道进行约束，redThreshold正常情况下不需要调节
	//输出的sliceImage是一幅二值化之后的图像
	inRange(singleNumberImage, Scalar(0, 0, redThreshold), Scalar(255, 255, 255), sliceImage);//--

	bgr_image = sliceImage; //将处理后的图片保存到bgr_image，以便于调试

	bitwise_and(hsv_splite, sliceImage, andImage); //与运算，获得两种处理方式的共同部分，存储入andImage中

	Mat erodeImage;

	//此时的图像为二值图像，在进行预矫正，将图像中的数码管的角度旋转至竖直方向
	//binary_r为输出后的较正图像
	correctImage(andImage, binary_r);

	and_image = andImage; //将处理后的图片保存到and_image，以便于调试

	//由于正常情况下，数码管是倾斜的，所以我们还需要将他们变换为垂直的，才能更加准确分割出单个数码管进行识别
	//实际
	pts_src.clear();//将透视变换的两组坐标容器清空
	//标准
	pts_dst.clear();

	//构造像素坐标系下的四个坐标点
	pts_src.push_back(Point2f(int(singleNumberImage.rows*1.0 / perspectiveK), 0));
	pts_src.push_back(Point2f(singleNumberImage.cols, 0));
	pts_src.push_back(Point2f(int(singleNumberImage.cols - singleNumberImage.rows*1.0 / perspectiveK), singleNumberImage.rows));
	pts_src.push_back(Point2f(0, singleNumberImage.rows));

	//构造目标坐标系下的坐标
	pts_dst.push_back(Point2f(0, 0));
	pts_dst.push_back(Point2f(int(singleNumberImage.cols - singleNumberImage.rows*1.0 / perspectiveK), 0));
	pts_dst.push_back(Point2f(int(singleNumberImage.cols - singleNumberImage.rows*1.0 / perspectiveK), singleNumberImage.rows));
	pts_dst.push_back(Point2f(0, singleNumberImage.rows));

	//生成变换矩阵，计算过程详见“坐标系变换相关资料”
	//getPerspectiveTransform():求解变换公式的函数。输入图像的四边形顶点坐标,输出图像的相应的四边形顶点坐标.
	transM = getPerspectiveTransform(pts_src, pts_dst);

	//对binary_r进行透视变换,变换后的图像存储在transedImage中
	warpPerspective(binary_r, transedImage, transM, binary_r.size(), INTER_LINEAR);

	warp_image = transedImage;//将处理后的图片保存到warp_image，以便于调试
	//透视变化后寻找轮廓操作：查找到的是每个数字的轮廓，存储在 contours中
	findContours(transedImage, contours,CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);
    //临时轮廓点集
	vector<vector<Point> > contours_temp;

	//若检测到的轮廓数大于5，则先进行一轮筛选
	if (contours.size() > 5)
    {   //c常量引用
		for (auto & c : contours)
        {
			double area = contourArea(c);
			if (area < 50) //面积太小
				continue;  //不执行下一步操作，直接进入下次循环
            //boundingRect返回矩形的x，y，w，h：x，y是矩阵左上点的坐标，w，h是矩阵的宽和高
			Rect r = boundingRect(c);
			//计算长宽比
			float h_w = float(r.height) / float(r.width);
			if (h_w < 1.0f) //长宽比不符合要求
				continue;
            //符合约束要求的放入contours_temp中
			contours_temp.push_back(c);
		}
		//清空保存contours的容器
        contours.clear();
        contours = contours_temp;   //这个轮廓是数字的轮廓
	}
	////此时contours中的轮廓应为图像中的数码管个数，接下来对轮廓个数判断，进一步识别数码管数字

	result.clear();//清空保存结果的容器

	if (contours.size() != 5)//如果检测到的数码管的个数不是5个，则说明数码管不全或者检测失败
	{
		result.push_back(-1);
	}
	else
    {
    	//当检测到的数码管是5时，则进行分割
        for (int i = 0; i < 5; i++)
        {   //boundingRect返回矩形的x，y，w，h：x，y是矩阵左上点的坐标，w，h是矩阵的宽和高
            singleNumber = boundingRect(contours[i]);//找到最大包围矩形

			//数字1特征明显，直接通过长宽比即可判断
            if ((singleNumber.height * 1.0) / (singleNumber.width * 1.0) > 2.2)
            {
                isOne = 1;
            }
            else
            {
                isOne = -1;
            }

            //此时的五个轮廓是乱序的，不严格对应图像中数码管的位置
			//下面保存每个轮廓的外接矩形的参数，左上角顶点坐标，右下角坐标顶点，该数码管是否为1
            //左上角顶点x坐标
			temp_finalPoints[i][0] = singleNumber.x;
            //左上角顶点y坐标
            temp_finalPoints[i][1] = singleNumber.y;
            //右下角顶点x坐标
            temp_finalPoints[i][2] = singleNumber.width + singleNumber.x;
            //右下角顶点y坐标
            temp_finalPoints[i][3] = singleNumber.height + singleNumber.y;
            //数码管是1
            temp_finalPoints[i][4] = isOne;
            //把每个轮廓的外接矩形的左上角顶点x坐标赋给finalPoints[i]
            finalPoints[i] = temp_finalPoints[i][0];//用于排序
        }

        //此处的排序将轮廓按照其外接矩形的左上角顶点坐标升序排列，则temp_finalPoints中的intd[0]位代表第一个数码管，intd[1]位代表第二个数码管
        //带索引的冒泡排序
        BubbleSort(finalPoints, 5, intd);//排序，算法流程详见实现
        //fiveNumberImage.clear();//观察单独五章图片时开启

		//下面就是将五个数码管单独分割出来，并送进分类器进行识别
        for (int i = 0; i < 5; i++)
        {
        	//这里的number图像就是按照temp_finalPoints中储存的外接矩形进行分割
            number = transedImage(Range((temp_finalPoints[intd[i]][1] - bias) < 0 ? temp_finalPoints[intd[i]][1] : (
                    temp_finalPoints[intd[i]][1] - bias), \
(temp_finalPoints[intd[i]][3] + bias) > transedImage.rows ? temp_finalPoints[intd[i]][3] : ((
                    temp_finalPoints[intd[i]][3] + bias))), \
Range((temp_finalPoints[intd[i]][0] - bias) < 0 ? temp_finalPoints[intd[i]][0] : (temp_finalPoints[intd[i]][0] - bias), \
(temp_finalPoints[intd[i]][2] + bias) > transedImage.cols ? temp_finalPoints[intd[i]][2] : (
                    temp_finalPoints[intd[i]][2] + bias)));

            //如果当前位已确定为数字1，则不需要进行识别
            if (temp_finalPoints[intd[i]][4] == 1)
            {
                result.push_back(1);
            }
            else
            {
            	//调用predictDigital进行识别并将结果保存进result中
                result.push_back(predictDigital(number));

                //调用isExist判断元素是否相同，若当前的检测值与已检测到的重复则识别出错
                if (isExist(result, result[i]))
                {
                    result.clear();
                    result.push_back(-1);
                    return;
                }
            }
        }
    }
}

#endif // DIGITRONDETECTOR_H_FILE
