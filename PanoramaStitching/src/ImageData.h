#pragma once

#include <stdio.h>
#include <iostream>
#include <random>
#include <chrono>

#include <opencv2/opencv.hpp>
#include "opencv2/core/core.hpp"
#include "opencv2/features2d/features2d.hpp"

using namespace cv;
using namespace std;


struct ImageData
{
    string file;
    int id ;
    Mat img;

    std::vector<KeyPoint> keypoints;
    Mat1b descriptors;

    // Init with identity matrix
    Matx33d HtoReference = Matx33d::eye();
    Matx33d HtoPrev = Matx33d::eye();
    Matx33d HtoNext = Matx33d::eye();

    void computeFeatures()
    {
        static thread_local Ptr<ORB> detector =  cv::ORB::create(2000);
        detector->detectAndCompute(img,noArray(), keypoints, descriptors);
        cout << "Found " << keypoints.size() << " ORB features on image " << id << endl;
    }
};
