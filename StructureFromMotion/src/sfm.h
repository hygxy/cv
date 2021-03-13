#include <stdio.h>
#include <iostream>
#include <fstream>

#include <opencv2/opencv.hpp>
#include "opencv2/core/core.hpp"
#include "opencv2/features2d/features2d.hpp"
#include <opencv2/viz.hpp>

using namespace std;
using namespace cv;


Matx33d computeFRANSAC(std::vector<cv::Point2f> points1, std::vector<cv::Point2f> points2, std::vector<int>& inliers);

Matx34d relativeTransformation(Matx33d E, std::vector<cv::Point2f> points1, std::vector<cv::Point2f> points2, Matx33d K);


Point3f triangulate(Matx34d P1, Matx34d P2, Point2f p1, Point2f p2);
vector<Point3f> triangulate(Matx34d View1, Matx34d View2, Matx33d K, std::vector<cv::Point2f> points1, std::vector<cv::Point2f> points2);
