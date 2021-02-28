#pragma once

#include "ImageData.h"
Matx33d computeHomography(std::vector<Point2f> points1, std::vector<Point2f> points2);
Matx33d computeHomographyRansac(ImageData& img1, ImageData& img2, vector<DMatch>& matches, int iterations, double threshold);
