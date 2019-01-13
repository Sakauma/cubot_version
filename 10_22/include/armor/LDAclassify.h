#ifndef _LDA_CLASSIFY_H_
#define _LDA_CLASSIFY_H_


#include "opencv2/highgui/highgui.hpp"
#include  <opencv2/opencv.hpp>
#include  <Eigen/Core>
#include  <Eigen/Dense>
#include  <opencv2/core/eigen.hpp>
#include  "opencv2/imgproc/imgproc.hpp"
#include  <opencv2/core/core.hpp>
#include  <iostream>
#include  <map>
#include  <fstream>
#include  "math.h"
#include  <iostream>

using namespace Eigen;
using namespace std;
using namespace cv;

class LAD
{
public:
    LAD(String invSname,String muname);
    int LDAclassify(const Mat &num);
private:
          MatrixXf readCSV(std::string file, int rows, int cols) ;
          MatrixXf conv;
          MatrixXf mu;
          VectorXf logc;
          VectorXf B;

};

#endif
