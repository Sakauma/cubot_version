//
// Created by snow on 18-7-13.
//

#ifndef SENTRY_HISTOGRAM_H
#define SENTRY_HISTOGRAM_H


#include "opencv2/opencv.hpp"

namespace snow{

    cv::Mat get_hist(const cv::Mat &gray_in)
    {
        cv::MatND hist;       // 在cv中用CvHistogram *hist = cvCreateHist
        int dims = 1;
        float hranges[] = {0, 255};
        const float *ranges[] = {hranges};   // 这里需要为const类型
        int size = 256;
        int channels = 0;
        // 计算图像的直方图
        cv::Mat gray;
        cv::medianBlur(gray_in,gray,3);
        cv::calcHist(&gray, 1, &channels, cv::Mat(), hist, dims, &size, ranges);    // cv 中是cvCalcHist
        int scale = 1;
        //Mat imageShow(size * scale, size, CV_8U, Scalar(0));
        // 获取最大值和最小值
        double minVal = 0;
        double maxVal = 0;
        cv::minMaxLoc(hist,&minVal, &maxVal, 0, 0);  //  cv中用的是cvGetMinMaxHistValue

        return hist;
    }


//Ptr<SVM> svm = SVM::load(model_name);
//int result = predict(imgs[i],svm);
    int predict(cv::Mat &img , cv::Ptr<cv::ml::SVM> &svm)
    {
        int result =0;

        cv::Mat m = get_hist(img);
        cv::Mat dst(1, m.cols*m.rows*m.channels(), CV_32FC1);
        for(int i=0; i<m.rows; i++)
            *(dst.ptr<float>(0)+i) = *m.ptr<float>(i);

        result = svm->predict(dst);
        return result;
    }

    void smooth(cv::MatND &hist_in, cv::MatND &hist_out, int size, int step) {
        int start = step - step/2;
        int end = size - step/2 ;
        double temp = 0;

        for (int i = start; i < end; i++) {
            temp = 0;
            for (int j = 0-step/2; j < step/2; j++) {
                temp += hist_in.at<float>(i + j);
            }
            temp /= step;
            hist_out.at<float>(i) = (int)temp;
        }
    }
}























#endif //SENTRY_HISTOGRAM_H
