#pragma once
#ifndef ROBOMASTER_NUMCLASS_H
#define ROBOMASTER_NUMCLASS_H

#include <opencv2/dnn.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <iostream>

using namespace cv;
using namespace cv::dnn;

class NumClass {
private:
	dnn::Net net;

	String _modelFile;
    String _imageFile;
    String _inBlobName;
    String _outBlobName;

	int _img_color = 1;	// 0-gray, 1-bgr

	Size _imgSize;

	void getMaxClass_n(const Mat &probBlob, int *classId, double *classProb);

public:
    NumClass();

	NumClass(String modelFileName);

	NumClass(String modelFileName, String inputBlobName, String outputBlobName, Size imgSize);

	bool predict(const std::vector<Mat> & imgs, int *classId, double *classProb_n);
};
//构造函数，唯一参数DNN路径
NumClass::NumClass(String modelFileName) {
	_modelFile = modelFileName;             //DNN路径
	_inBlobName = "img";                    //输入层名称
	_outBlobName = "model_1/pre";           //输出层名称
	_imgSize = Size(32, 32);                //图像尺寸
    //读取DNN模型
	net = readNetFromTensorflow(_modelFile);
	if (net.empty()) {
		std::cerr << "Can't load network by using the mode file: " << _modelFile << std::endl;
	}
}
//构造函数，参数为DNN路径，输入层名称，输出层名称，图像尺寸
NumClass::NumClass(String modelFileName, String inputBlobName, String outputBlobName, Size imgSize) {
	_modelFile = modelFileName;
	_inBlobName = inputBlobName;
	_outBlobName = outputBlobName;
	_imgSize = imgSize;
    //DNN加载模型
	net = readNetFromTensorflow(_modelFile);
	if (net.empty()) {
		std::cerr << "Can't load network by using the mode file: " << _modelFile << std::endl;
	}
}
//输入九宫格图像，将识别的结果存放classId，每个结果对应的概率存放classProb
bool NumClass::predict(const std::vector<Mat> & imgs, int *classId, double *classProb) {
	//先判断是否为九宫格
	if (imgs.size() != 9) {
		std::cerr << "Need 9 images! " << std::endl;
		return false;
	}

	Mat blobs;
	for (size_t i = 0; i < 9; i++) {
	/****************************
    Mat cv::dnn::blobFromImage (InputArray 	image,						//输入图像
                                double 	scalefactor = 1.0,				//这个参数很重要的，如果训练时，是归一化到0-1之间，那么这个参数就应该为0.00390625f （1/256），否则为1.0
                                const Size & 	size = Size(),			//输出图像尺寸，应该与训练时的输入图像尺寸保持一致
                                const Scalar & 	mean = Scalar(),		//如果输出图像为BGR，且swapRB为真则输出图像三通道顺序为RGB
                                bool 	swapRB = false,					//是否交换图像第1个通道和最后一个通道的顺序
                                bool 	crop = false,					//如果为true，就是裁剪图像，如果为false，就是等比例放缩图像
                                int 	ddepth = CV_32F					//输出的blob深度
                             )
        ***************************/
	    //转换成批量图像
		Mat inputBlob = blobFromImage(imgs[i], 1.0f, _imgSize, Scalar(), false, false);   //Convert Mat to batch of images
		//用blobs来保存inputBlob
		blobs.push_back(inputBlob);
	}
    //设置网络输入参数，_inBlobName是输入层的名字
	net.setInput(blobs, _inBlobName);
    //运行DNN模型，输出层名字为_outBlobName
	Mat result = net.forward(_outBlobName);
	getMaxClass_n(result, classId, classProb);

	return true;
}

void NumClass::getMaxClass_n(const Mat &probBlob, int *classId, double *classProb) {
	for (size_t i = 0; i < 9; i++) {
        //将blob重新转变为1×10000的矩阵，由于probBlob为三通道，而minMaxLoc只能处理单通道，所以将图像变为单通道
        //reshape（int cn, int rows=0 const），cn：目标通道数，如果是0则保持和原通道数一致；rows：目标行数，同上是0则保持不变；
	    Mat probMat = probBlob.row(i).reshape(1, 1); //reshape the blob to 1x1000 matrix
		Point classNumber;

		double _classProb;
		/****************************
        void cv::minMaxLoc( InputArray 	src,					//输入单通道图像
                            double * 	minVal,	     			//返回最小值，如果不需要，则使用NULL。
                            double * 	maxVal = 0,				//返回最大值
                             Point * 	minLoc = 0,				//返回最小值的位置
                             Point * 	maxLoc = 0,				//返回最大值的位置
                            InputArray 	mask = noArray()		//掩膜
                           )
     ******************************/
        //返回的最大值就是该数字的概率_classProb，返回的点坐标classNumber
		minMaxLoc(probMat, NULL, &_classProb, NULL, &classNumber);
		classId[i] = classNumber.x + 1;         //识别到的数字的编号
		classProb[i] = _classProb;              //对应数字的概率
	}
}

#endif
