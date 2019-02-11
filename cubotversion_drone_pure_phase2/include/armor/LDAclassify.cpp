#include "LDAclassify.h"

LAD::LAD(String invSname="invS.csv",String muname="mu.csv"):logc(5),B(5)
{

    conv = readCSV(invSname,400,400);
    mu   = readCSV(muname,5,400);

    logc<<log(0.259),log(0.25933),log(0.111),log(0.232),log(0.13867);

    MatrixXf B1 = mu.row(0)* conv *mu.row(0).transpose();
    MatrixXf B2 = mu.row(1)* conv *mu.row(1).transpose();
    MatrixXf B3 = mu.row(2)* conv *mu.row(2).transpose();
    MatrixXf B4 = mu.row(3)* conv *mu.row(3).transpose();
    MatrixXf B5 = mu.row(4)* conv *mu.row(4).transpose();
    B<<B1,B2,B3,B4,B5;

}

int LAD::LDAclassify(const Mat &num)
{
    Mat numimg;
    num.copyTo(numimg);
    resize(numimg,numimg,Size(20,20));

    Eigen::Matrix<float ,Dynamic,Dynamic> eigen_img;

    cv::cv2eigen(numimg,eigen_img);
    eigen_img.transpose();

    VectorXf imgserials(Eigen::Map<VectorXf>(eigen_img.data(), eigen_img.cols()*eigen_img.rows()));

    VectorXf A = imgserials.transpose()*conv*mu.transpose();

    VectorXf result=A-0.5*B+logc;
    std::map<float,int> ldares;
    ldares.insert(make_pair(result[0],1));
    ldares.insert(make_pair(result[1],3));
    ldares.insert(make_pair(result[2],5));
    ldares.insert(make_pair(result[3],9));
    ldares.insert(make_pair(result[4],-1));

    return (prev(ldares.end(), 1)->second);

}


Eigen::MatrixXf LAD::readCSV(std::string file, int rows, int cols)
{

    std::ifstream in(file);

    std::string line;

    int row = 0;
    int col = 0;

    Eigen::MatrixXf res = Eigen::MatrixXf(rows, cols);

    if (in.is_open())
    {
        while (std::getline(in, line)) {

            char *ptr = (char *) line.c_str();
            int len = line.length();

            col = 0;

            char *start = ptr;
            for (int i = 0; i < len; i++) {

                if (ptr[i] == ',') {
                    res(row, col++) = atof(start);
                    start = ptr + i + 1;
                }
            }
            res(row, col) = atof(start);

            row++;
        }

        in.close();
    }
    return res;
}


