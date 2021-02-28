#pragma once

#include "share.h"
#include "disparity.h"

class StereoVision
{
public:

    float maxDisp = 90;
    Disparity disparity;

    Intrinsics intrinsics1;
    Intrinsics intrinsics2;
    StereoExtrinsics extrinsics;

    StereoVision();


    void run();

    Mat R1,R2 ,P1 ,P2 ,Q;

    Mat map11, map12;
    Mat map21, map22;
};
