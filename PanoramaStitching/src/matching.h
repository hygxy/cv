#pragma once

#include "ImageData.h"


vector<DMatch> computeMatches(ImageData &img1, ImageData &img2);



inline Mat createMatchImage(ImageData& img1, ImageData& img2, vector<DMatch>& matches)
{
    Mat img_matches;
    drawMatches( img1.img, img1.keypoints, img2.img, img2.keypoints,
                 matches, img_matches, Scalar(0,255,0), Scalar(0,255,0),
                 vector<char>(), DrawMatchesFlags::DEFAULT );
    return img_matches;
}
