#pragma once

#include "opencv2/opencv.hpp"
#include <iostream>



using namespace cv;

using std::cout;
using std::endl;
using std::vector;

Vec4d computeBilinearWeights(Point2f q);
Mat1d computeGaussianWeights(Size winSize, float sigma);
Matx22d invertMatrix2x2(const Matx22d& A);

class OpticalFlowLK
{

public:
    OpticalFlowLK(Size winSize_ ,
                               float _epsilon, int _iterations) :
        winSize(winSize_),epsilon(_epsilon),iterations(_iterations)
    {
    }


    void compute(Mat1b _prevImg, Mat1b _nextImg,
                       std::vector<Point2f> &_prevPts,     std::vector<Point2f> &_nextPts,
                       std::vector<uchar> &_status);

private:
    Size winSize;
    float epsilon;
    int iterations;
};


