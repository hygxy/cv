#pragma once

#include "share.h"

class StereoCalibration
{
public:
    double SquareDistanceMeters = 3.2 / 100; //3.2cm
    StereoCalibration();
    void calibStereoVideo();

private:
    std::vector<Point2f> findPattern(Mat image);

    Intrinsics calibrateIntrinsics(vector<vector<Point2f>>& corners, int w, int h);
    StereoExtrinsics calibrateStereo(Intrinsics intrinsics1, Intrinsics intrinsics2, vector<vector<Point2f>>& corners1,vector<vector<Point2f>>& corners2);

    Size patternsize;
    vector<Point3f> objPoints;


    //For visualization
    vector<Point2f> allPoints1,allPoints2;
};
