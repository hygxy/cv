#pragma once

// system includes
#include <stdio.h>
#include <iostream>
#include <fstream>

// library includes
#include <opencv2/opencv.hpp>
#include "opencv2/core/core.hpp"
//#include "opencv2/features2d/features2d.hpp"

#define SOLUTION 10

using namespace std;
using namespace cv;

struct Intrinsics
{
    int w, h;
    Mat K, dist;

    void writeToFile(std::string file);
    void readFromFile(std::string file);
};

struct StereoExtrinsics
{
    Mat R,t,E,F;

    void writeToFile(std::string file);
    void readFromFile(std::string file);
};


void openCameras(VideoCapture &capture1, VideoCapture &capture2);
