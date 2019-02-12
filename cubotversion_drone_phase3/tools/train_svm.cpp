//
// Created by jason on 18-6-9.
//
#include "opencv2/opencv.hpp"
#include "../include/gold/pro_file.h"

using namespace cv;
using namespace ml;
using namespace std;

//#define ModelForInfantry


int predict(Mat img , Ptr<SVM> svm)
{
    int result =0;
    Mat gray0,gray1;

    vector<float> descriptors;
    HOGDescriptor * hog = new HOGDescriptor(Size(40, 40), Size(16, 16), Size(8, 8), Size(8, 8), 9);
//    cvtColor(img,gray0,CV_BGR2GRAY);
//    equalizeHist(gray0,gray1);
    descriptors.clear();

    hog->compute(img, descriptors);
    Mat dst(1, int(descriptors.size()), CV_32FC1, descriptors.data());
    result = svm->predict(dst);

    return result;
}

void get_img_vec(std::string img_root , std::vector<cv::Mat> &imgs , std::vector<int> &labeles ,int pos_or_neg,int flag)
{
    Mat m,gray0,gray1;
    std::vector<std::string> file_list;

    get_file_list(img_root,file_list,".jpg");
    for( int i=0;i<file_list.size();i++)
    {
        std::cout<<file_list[i]<<std::endl;

        Mat m;
        m = imread(file_list[i]);

        if( flag == 1 )
        {
            cvtColor(m,gray0,CV_BGR2GRAY);
            equalizeHist(gray0,gray1);
        }
        else
        {
            gray1 = m;
        }
        imgs.push_back(gray1);

        labeles.push_back(pos_or_neg);
    }
}


#ifdef ModelForInfantry
    std::string model_name = "/home/snow/Robomasters/mengyu619/infantry_sample/cfg/svm_hog_infantry.xml";
#else
    std::string model_name = "/home/snow/svm_hog_sentry.xml";
#endif
void hog_svm_train_2(std::vector<cv::Mat> &imgs , std::vector<int> &labeles)
{
    Mat trainingDataMat;
    Mat labelsMat;

    HOGDescriptor * hog = new HOGDescriptor(Size(40, 40), Size(16, 16), Size(8, 8), Size(8, 8), 9);
    vector<float> descriptors;

    for(int i=0;i<imgs.size();i++)
    {
        descriptors.clear();
        hog->compute(imgs[i], descriptors);
        Mat dst(1, int(descriptors.size()), CV_32FC1, descriptors.data());
        trainingDataMat.push_back(dst);
        labelsMat.push_back(labeles[i]);
    }

    Ptr<SVM> svm = SVM::create();
    svm->setType(SVM::Types::C_SVC);
    //svm->setKernel(SVM::KernelTypes::INTER);   //INTER LINEAR
    svm->setKernel(SVM::KernelTypes::INTER);
    svm->setTermCriteria(TermCriteria(CV_TERMCRIT_ITER, 10000, 1e-6));
    svm->setC(1.0);
    Ptr<TrainData>Inputdata = TrainData::create(trainingDataMat,ROW_SAMPLE,labelsMat);
    svm->trainAuto(Inputdata);

    svm->save(model_name);
}

int main()
{
    std::string train_path = "/home/snow/Robomasters/mengyu619/train/";
    std::vector<cv::Mat> imgs ;
    std::vector<int> labeles;

#ifdef ModelForInfantry
    //circle postive
    get_img_vec(train_path + "train5/pos/",imgs,labeles,1,1);
    get_img_vec(train_path + "train4/pos/",imgs,labeles,1,1);

    // number postive
    get_img_vec(train_path + "train6/roi_pos/1/",imgs,labeles,1,0);
    get_img_vec(train_path + "train6/roi_pos/2/",imgs,labeles,1,0);
    get_img_vec(train_path + "train6/roi_pos/3/",imgs,labeles,1,0);
    get_img_vec(train_path + "train6/roi_pos/4/",imgs,labeles,1,0);
    get_img_vec(train_path + "train6/roi_pos/5/",imgs,labeles,1,0);
    get_img_vec(train_path + "train8/pos/",imgs,labeles,1,0);

    //circle negtive
    get_img_vec(train_path + "train4/neg/",imgs,labeles,0,1);
    get_img_vec(train_path + "train5/neg/",imgs,labeles,0,1);

    //number negtive
    get_img_vec(train_path + "train6/roi_neg/",imgs,labeles,0,0);
    get_img_vec(train_path + "train7/neg/",imgs,labeles,0,0);
    get_img_vec(train_path + "train8/neg/",imgs,labeles,0,0);
    get_img_vec(train_path + "train9/neg/",imgs,labeles,0,0);
#else
    /*For sentry, only the NUMs is postive*/
    // number postive
    get_img_vec(train_path + "train6/roi_pos/1/",imgs,labeles,1,0);
    get_img_vec(train_path + "train6/roi_pos/2/",imgs,labeles,1,0);
    get_img_vec(train_path + "train6/roi_pos/3/",imgs,labeles,1,0);
    get_img_vec(train_path + "train6/roi_pos/4/",imgs,labeles,1,0);
    get_img_vec(train_path + "train6/roi_pos/5/",imgs,labeles,1,0);
    get_img_vec(train_path + "train8/pos/",imgs,labeles,1,0);

    //circle negative
    get_img_vec(train_path + "train5/pos/",imgs,labeles,0,1);
    get_img_vec(train_path + "train4/pos/",imgs,labeles,0,1);

    //circle negtive
    get_img_vec(train_path + "train4/neg/",imgs,labeles,0,1);
    get_img_vec(train_path + "train5/neg/",imgs,labeles,0,1);

    //number negtive
    get_img_vec(train_path + "train6/roi_neg/",imgs,labeles,0,0);

    get_img_vec(train_path + "train7/neg/",imgs,labeles,0,0);

    get_img_vec(train_path + "train8/neg/",imgs,labeles,0,0);
    get_img_vec(train_path + "train9/neg/",imgs,labeles,0,0);
#endif

    hog_svm_train_2(imgs,labeles);
    Ptr<SVM> svm = SVM::load(model_name);

    for( int i=0;i<imgs.size();i++)
    {
        int result = predict(imgs[i],svm);
        printf(" %d" , result);

        if(i%50 == 0)
            printf("\n");
    }

    cout<<"\nmodle over"<<endl;
    return 0;
}
