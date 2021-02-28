#pragma once


#include "share.h"

#define DEBUG_VIEWS

class Disparity
{
public:

    // Patch Radius. The total patch size is   (2*patchRadius+1) x (2*patchRadius+1)
    int patchRadius =  4;

    // Number of random guesses in initialization
    int initIterations = 3;

    // Number of spatial propagations
    int propagateIterations = 2;

    // Number of random guesses in optimization step of spatial propagation
    int optimizeIterations = 2;

    void computeDisparity(const Mat1b& img1, const Mat1b& img2, Mat1f &disp, float minDisp, float maxDisp);


    float cost(const Mat1b& img1, const Mat1b& img2, int x, int y, float d);

    void initRandom(const Mat1b& img1, const Mat1b& img2, Mat1f &disp, Mat1f &scoreF, float minDisp, float maxDisp);
    void propagate(const Mat1b& img1, const Mat1b& img2, Mat1f &disp, Mat1f &scoreF, float minDisp, float maxDisp, bool forward = true);
    void randomSearch(const Mat1b& img1, const Mat1b& img2, Mat1f &disp, Mat1f &scoreF, float minDisp, float maxDisp);
};
