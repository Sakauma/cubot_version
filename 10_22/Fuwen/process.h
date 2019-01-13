#pragma once
#include <opencv2/opencv.hpp>
#include <iostream>
#include "ParaReader.h"
#include "NumClass.hpp"
#include "DigitronDetector.hpp"
#include <eigen3/Eigen/Geometry>
#include <opencv2/core/eigen.hpp>

using namespace cv;
using namespace std;


void readSerial();
void sendMsg( const cv::Point2f &pixel,uchar num, float depth);

bool dafuNumAnalysis(int * classId, double * classProb) {
	int idx_count[9] = {0};
	int pairs = 0;
	int same_num = 0;

	for (int i = 0; i < 9; ++i)
		idx_count[classId[i]-1]++;	// classId : 1~9

	for (int i = 0; i < 9; ++i) {
		if (idx_count[i] >= 3) {
			return false;
		}
		else if (idx_count[i] == 2) {
			same_num = i+1;
			pairs++;
		}
		else if (i == 8 && idx_count[i] == 1 && pairs == 0)
			return true;
	}

	int idx[2] = {-1};
	int miss_num = 0;
	if (pairs > 1)
		return false;
	else {
		for (int i = 0; i < 9; ++i) {
			if (idx_count[i] == 0)
				miss_num = i+1;

			if (classId[i] == same_num && idx[0] == -1)
				idx[0] = i;
			else if (classId[i] == same_num && idx[0] != -1)
				idx[1] = i;
		}
	}

	if (classProb[idx[0]] > classProb[idx[1]])
		classId[idx[1]] =  miss_num;
	else
		classId[idx[0]] =  miss_num;
	return true;
}

bool dafuNumAnalysis2(int * classId, double * classProb, int sum) {
	//数字i保存在容器第i个位置
	vector<vector<int> > boxOfCharacter;
	for(int t = 0; t < 9; t++)
	{
		boxOfCharacter[classId[t]].push_back(classId[t]);
	}

	for(int t = 0; t < 9; t++)
	{
		int timesOfRepetition = 0;
		//每个数字只出现一次
		if(boxOfCharacter[t].size() == 1)
			continue;

			//同一个数字出现两次
		else if(boxOfCharacter[t].size() == 2)
		{
			//该情况出现次数
			timesOfRepetition++;
		}

			//同一个数字出现三次，舍弃
		else
			return false;

		if(timesOfRepetition == 1)
		{
			//查找重复数字编号
			vector<int> index;
			for(int m = 1; m < 10; m++)
			{
				if(classId[m] == t)
					index.push_back(m);
			}

			if(classProb[index[0]] > classProb[index[1]])
				classId[index[1]] = t  + 45 - sum;
			else
				classId[index[0]] = t  + 45 - sum;
		}
			//同一个数字出现两次的情况大于一次，舍弃
		else
			return false;
	}

	return true;
}


int get_vector_idx(const vector<int> & v, int k) {
	/*vector<int>::iterator it = find(v.begin(), v.end(), k);
	int index = it - v.begin();*/

	for (int i = 0; i < v.size(); i++) {
		if (v[i] == k)
			return i;
	}

	return -1;
}

int idx_vector_analysis(const vector<int> & v) {
	int a[10];
	for (int i = 0; i < 10; i++)
		a[i] = 0;

	for (auto i : v) {
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

	int a1 = 0, a2 = 0, a3 = 0;
	for (int i = 0; i < 5; i++) {
		a1 += a[i];
		a2 += a[i + 5];
	}
	a3 = a1 + a2;
	if (a3 == 10)
		return 1;		// 存在1~5，10~14，十个记分板都提取到
	else if (a1 == 5 && a2 != 5)
		return 2;		// 存在1~5，其他不确定
	else if (a1 != 5 && a2 == 5)
		return 3;		// 存在10~14，其他不确定
	else
		return 0;
}

void sortPoints(vector<Point>& p_)
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
int calc_len(const Point & p1, const Point & p2) {
	return (p1.x - p2.x) * (p1.x - p2.x) + (p1.y - p2.y) * (p1.y - p2.y);
}

// 计算两点间距离的平方
float calc_len(const Point2f & p1, const Point2f & p2) {
	return (p1.x - p2.x) * (p1.x - p2.x) + (p1.y - p2.y) * (p1.y - p2.y);
}

// 判断点p1是否在一个（）矩形范围内
bool ptInRange(const Point2f & p1, const Point2f & p2, const float r) {
	if (p1.x > (p2.x - r) && p1.x < (p2.x + r) && (p1.y >(p2.y - r) && p1.y < (p2.y + r)))
		return true;
	else
		return false;
}

// 输入两组点集（每组四个点），将两组点的位置统一，即左上角的点移动至Point(0,0)，然后计算两组点的相似度
int calc_dist(const vector<Point> & pts1, const vector<Point> & pts2) {
	int dist = 0;
	for (int i = 1; i < 4; i++) {
		Point p1 = pts1[i] - pts1[0];
		Point p2 = pts2[i] - pts2[0];
		p1 -= p2;
		dist += abs(p1.x) + abs(p1.y);
	}

	return dist;
}

double get_point_angle(cv::Point2f pointO, cv::Point2f pointA) {
	double angle = 0;
	cv::Point2f point;
	double temp;

	point = cv::Point2f((pointA.x - pointO.x), (pointA.y - pointO.y));

	if ((0 == point.x) && (0 == point.y))
	{
		return 0;
	}

	if (0 == point.x)
	{
		angle = 90;
		return angle;
	}

	if (0 == point.y)
	{
		angle = 0;
		return angle;
	}

	temp = fabsf(float(point.y) / float(point.x));
	temp = atan(temp);
	temp = temp * 180 / CV_PI;

	if ((0<point.x) && (0<point.y))
	{
		angle = temp;
		return angle;
	}

	if ((0>point.x) && (0<point.y))
	{
		angle = (180 - temp);
		return angle;
	}

	if ((0<point.x) && (0>point.y))
	{
		angle = -temp;
		return angle;
	}

	if ((0>point.x) && (0>point.y))
	{
		angle = -(180 - temp);
		return angle;
	}

	printf("sceneDrawing :: getAngle error!");
	return -1;
}
