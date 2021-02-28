#include "sfm.h"
#include <chrono>
#include <random>
#include <math.h>
#include <cmath>


Matx33d computeF(std::vector<cv::Point2f>& points1, std::vector<cv::Point2f>& points2)
{
    assert(points1.size() == 8);
    assert(points2.size() == 8);

    Mat1d A(8, 9,0.0);


    for(int i = 0 ; i < 8 ; ++i)
    {
        //TODO 2.2
       // Step 1
        // - Fill the matrix A.
//        printf ("points1[%d].x is %f\n", i,points1[i].x);
//        printf ("points2[%d].x is %f\n", i,points2[i].x);
//        printf ("points1[%d].y is %f\n", i,points1[i].y);
//        printf ("points2[%d].y is %f\n", i,points2[i].y);
    A.at<double>(i,0) =points1[i].x*points2[i].x;
    A.at<double>(i,1) =points1[i].x*points2[i].y;
    A.at<double>(i,2) =points1[i].x;
    A.at<double>(i,3) =points1[i].y*points2[i].x;
    A.at<double>(i,4) =points1[i].y*points2[i].y;
    A.at<double>(i,5) =points1[i].y;
    A.at<double>(i,6) =points2[i].x;
    A.at<double>(i,7) =points2[i].y;
    A.at<double>(i,8) =1;
    }

//    for(int i =0;i<8;i++)
//    {
//        for (int j =0; j<9;j++)
//        {
//            printf ("A(%d,%d) =%f   ",i,j,A.at<double>(i,j));
//        }
//        printf("\n");
//    }


    // Step 2S
    cv::SVD svdA(A,SVD::FULL_UV);
    Mat1d V = svdA.vt.t();
    Matx33d F;

//    printf("first element is %f\n",svdA.w.at<float>(0,0));
//    printf("second element is %f\n",svdA.w.at<float>(1,0));
//    printf("second element is %f\n",svdA.w.at<float>(2,0));

    F(0,0) = V(0,8);
    F(0,1) = V(3,8);
    F(0,2) = V(6,8);

    F(1,0) = V(1,8);
    F(1,1) = V(4,8);
    F(1,2) = V(7,8);

    F(2,0) = V(2,8);
    F(2,1) = V(5,8);
    F(2,2) = V(8,8);


    //TODO 2.2
    // Step 3
    // - Enforce Rank(F) = 2

    cv::SVD svdF(F, SVD::FULL_UV);

//        printf("first element is %f\n",svdF.w.at<double>(0,0));
//        printf("second element is %f\n",svdF.w.at<double>(1,0));
//        printf("second element is %f\n",svdF.w.at<double>(2,0));


    Matx33d W(svdF.w.at<double>(0,0),0,0,0,svdF.w.at<double>(1,0),0,0,0,svdF.w.at<double>(2,0));
//    printf ("W(0,0) is %f", W(0,0));printf ("   W(0,1) is %f", W(0,1));printf ("   W(0,2) is %f\n", W(0,2));
//    printf ("W(1,0) is %f", W(1,0));printf ("   W(1,1) is %f", W(1,1));printf ("   W(1,2) is %f\n", W(1,2));
//    printf ("W(2,0) is %f", W(2,0));printf ("   W(2,1) is %f", W(2,1));printf ("   W(2,2) is %f\n", W(2,2));


    Matx33d U =svdF.u;
//    printf ("----------\n");
//    printf ("U (0,0) is %f",U(0,0));printf ("   U(0,1) is %f", U(0,1));printf ("   U(0,2) is %f\n", U(0,2));
//    printf ("U(1,0) is %f", U(1,0));printf ("   U(1,1) is %f", U(1,1));printf ("   U(1,2) is %f\n", U(1,2));
//    printf ("U(2,0) is %f", U(2,0));printf ("   U(2,1) is %f", U(2,1));printf ("   U(2,2) is %f\n", U(2,2));


    Matx33d VF=Matx33d(svdF.vt).t();
//    printf ("----------\n");
//    printf ("VF (0,0) is %f",VF(0,0));printf ("   VF(0,1) is %f", VF(0,1));printf ("   VF(0,2) is %f\n", VF(0,2));
//    printf ("VF(1,0) is %f", VF(1,0));printf ("   VF(1,1) is %f", VF(1,1));printf ("   VF(1,2) is %f\n", VF(1,2));
//    printf ("VF(2,0) is %f", VF(2,0));printf ("   VF(2,1) is %f", VF(2,1));printf ("   VF(2,2) is %f\n", VF(2,2));

//    printf ("height of W is %d\nwidth of W is %d\n",W.rows,W.cols);
//    printf ("height of U is %d\nwidth of U is %d\n",U.rows,U.cols);
//    printf ("height of VF is %d\nwidth of VF is %d\n",VF.rows,VF.cols);


    W(2,2) =0; // W.row(2).setTo(Scalar(0));
//    printf ("----------\n");
//    printf ("W(0,0) is %f", W(0,0));printf ("   W(0,1) is %f", W(0,1));printf ("   W(0,2) is %f\n", W(0,2));
//    printf ("W(1,0) is %f", W(1,0));printf ("   W(1,1) is %f", W(1,1));printf ("   W(1,2) is %f\n", W(1,2));
//    printf ("W(2,0) is %f", W(2,0));printf ("   W(2,1) is %f", W(2,1));printf ("   W(2,2) is %f\n", W(2,2));

   F =U*W*VF.t();





   // Step 4
    F = F * (1.0 / F(2,2));
    return F;
}


bool epipolarConstraint(Point2f p1, Point2f p2, Matx33d F, double t)
{
    Vec3d p1h(p1.x,p1.y,1);
    Vec3d p2h(p2.x,p2.y,1);

    // TODO 2.1
    // - Compute the normalized epipolar line
    // - Compute the distance to the epipolar line
    // - Check if the distance is smaller than t
    Vec3d epipolar =F*p1h;
    Vec3d normalized_epipolar =epipolar/ sqrt(epipolar[0]*epipolar[0]+ epipolar[1]*epipolar[1]);
    double distance =p2h[0]*normalized_epipolar[0]+ p2h[1]*normalized_epipolar[1]+ normalized_epipolar[2];
    if (abs(distance)<t)
       return true;
    else 
       return false;

}

std::vector<int> numInliers(std::vector<Point2f> points1, std::vector<Point2f> points2, Matx33d F, double threshold )
{
    std::vector<int> inliers;
    for( int i = 0; i < (int)points1.size(); i++ )
    {
        if(epipolarConstraint(points1[i],points2[i],F,threshold))
            inliers.push_back(i);
    }

    return inliers;
}


Matx33d computeFRANSAC(std::vector<cv::Point2f> points1, std::vector<cv::Point2f> points2, std::vector<int> &inliers)
{
    // Use modern c++ random numbers
    std::uniform_int_distribution<int> dis{0,(int)points1.size()-1};
    static thread_local std::mt19937 gen(std::chrono::system_clock::to_time_t(std::chrono::system_clock::now()));

    // The best fundamental matrix and the number of inlier for this F.
    Matx33d bestF;
    std::vector<int> bestInliers;
    int bestInlierCount = 0;
    double threshold = 4;

    int iterations = 10000 ;
    for( int k = 0; k < iterations; ++k)
    {
        std::vector<Point2f> subset1;
        std::vector<Point2f> subset2;
        for(int i = 0; i < 8; ++i)
        {
            int x = dis(gen);
            subset1.push_back(points1[x]);
            subset2.push_back(points2[x]);
        }
        Matx33d F = computeF(subset1,subset2);
        std::vector<int> num = numInliers(points1,points2,F,threshold);
        if( (int)num.size() > bestInlierCount)
        {
            bestF = F;
            bestInlierCount = num.size();
            bestInliers = num;
        }
    }
    inliers = bestInliers;
    return bestF;
}


void testFundamentalMat()
{
    std::vector<Point2f> points1 =
    {
        {1,1},{3,7},{2,-5},{10,11},{11,2},{-3,14},{236,-514},{-5,1}
    };
    std::vector<Point2f> points2 =
    {
        {25,156},{51,-83},{-144,5},{345,15},   {215,-156},{151,83},{1544,15},{451,-55}
    };
    auto F = computeF(points1,points2);
    cout << "Testing Fundamental Matrix..." << endl << "Your result:" << endl;
    cout << F << endl;

    Matx33d Href = {
        0.001260822171230067, 0.0001614643951166923, -0.001447955678643285,
         -0.002080014358205309, -0.002981504896782918, 0.004626528742122177,
         -0.8665185546662642, -0.1168790312603214, 1
    };
    cout << "Reference: " << endl << Href << endl;

    auto error = Href - F;
    double e = norm(error);
    cout << "Error: " << e << endl;
    if(e < 1e-10)
        cout << "Test: SUCCESS!" << endl;
    else
        cout << "Test: FAIL!" << endl;
    cout << "============================" << endl;
}
