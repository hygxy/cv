#include "homography.h"
#include <opencv2/core.hpp>
using namespace cv;


int numInliers(std::vector<Point2f> points1, std::vector<Point2f> points2, Matx33d H, double threshold )
{
    int inlierCount  = 0;

    // TODO 4.1
    // Compute the number of inliers for the given homography
    // - Project the image points from image 1 to image 2
    // - A point is an inlier if the distance between the projected point and
    //      the point in image 2 is smaller than threshold.
    //
    // Hint: Construct a Homogeneous point of type 'Vec3d' before applying H.
        //int NumberOfPoints =sizeof(points1)/sizeof(Point2f);
        //for (int i =0; i<NumberOfPoints;i++)
        for (int i =0; i < (int)points1.size();i++)
	{
	Vec3d p1(points1[i].x, points1[i].y,1);
	Vec3d p1Projected =H*p1;
	Point2f p1dehomo(p1Projected[0]/p1Projected[2], p1Projected[1]/p1Projected[2]);
	if (norm(Mat(p1dehomo), Mat(points2[i]),NORM_L2)<threshold)
	inlierCount++;
	}
	//printf("numbers =%d\n", inlierCount);
    return inlierCount;
	
}

Matx33d computeHomographyRansac(ImageData& img1, ImageData& img2, vector<DMatch>& matches, int iterations, double threshold)
{
    std::vector<Point2f> points1;
    std::vector<Point2f> points2;
    for( int i = 0; i < (int)matches.size(); i++ )
    {
        points1.push_back( img1.keypoints[ matches[i].queryIdx ].pt );
        points2.push_back( img2.keypoints[ matches[i].trainIdx ].pt );
    }


    // Use modern c++ random numbers
    std::uniform_int_distribution<int> dis{0,(int)points1.size()-1};
    static thread_local std::mt19937 gen(std::chrono::system_clock::to_time_t(std::chrono::system_clock::now()));
   

    // The best homography and the number of inlier for this H.
    Matx33d bestH;
    int bestInlierCount = 0;

    Matx33d curH;
    int curInlierCount;

    for(int k = 0; k < iterations; ++k)
    {
        std::vector<Point2f> subset1;
        std::vector<Point2f> subset2;

        // TODO 4.2
        // - Construct the subsets by randomly choosing 4 matches.
        // - Compute the homography for this subset
        // - Compute the number of inliers
        // - Keep track of the best homography (use the variables bestH and bestInlierCount)
	for (int i =0; i<4;i++)
	{
	 int randomNumber =dis(gen);
         subset1.push_back( points1[randomNumber] );
         subset2.push_back( points2[randomNumber]);
	}
	
        curH =computeHomography(subset1, subset2);
        curInlierCount =numInliers(points1, points2, curH, threshold);

        if (curInlierCount > bestInlierCount)
        {
            bestH = curH;
            bestInlierCount = curInlierCount;
        }
    }

    cout << "(" << img1.id << "," << img2.id << ") found " << bestInlierCount << " RANSAC inliers." << endl;
    return bestH;
}
