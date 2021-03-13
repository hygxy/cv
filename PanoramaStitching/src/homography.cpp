#include "homography.h"


//compute a homography matrix from 4 point matches
Matx33d computeHomography(std::vector<Point2f> points1, std::vector<Point2f> points2)
{
    // Solution with OpenCV calls (not allowed!!!).
    assert(points1.size() == 4);
    assert(points2.size() == 4);

    Mat1d A(8, 9, 0.0);
    // TODO 3
    // Construct the 8x9 matrix A.
    // Use the formula from the exercise sheet.
    // Note that every match contributes to exactly two rows of the matrix.
    A.at<double>(0,0) =-points1[0].x; A.at<double>(0,1)=-points1[0].y;A.at<double>(0,2)=-1; A.at<double>(0,6)=points1[0].x*points2[0].x; A.at<double>(0,7)=points1[0].y*points2[0].x;A.at<double>(0,8)=points2[0].x;

    A.at<double>(1,3) =-points1[0].x; A.at<double>(1,4)=-points1[0].y;A.at<double>(1,5)=-1; A.at<double>(1,6)=points1[0].x*points2[0].y; A.at<double>(1,7)=points1[0].y*points2[0].y;A.at<double>(1,8)=points2[0].y;

    A.at<double>(2,0) =-points1[1].x; A.at<double>(2,1)=-points1[1].y;A.at<double>(2,2)=-1; A.at<double>(2,6)=points1[1].x*points2[1].x; A.at<double>(2,7)=points1[1].y*points2[1].x;A.at<double>(2,8)=points2[1].x;

    A.at<double>(3,3) =-points1[1].x; A.at<double>(3,4)=-points1[1].y;A.at<double>(3,5)=-1; A.at<double>(3,6)=points1[1].x*points2[1].y; A.at<double>(3,7)=points1[1].y*points2[1].y;A.at<double>(3,8)=points2[1].y;

    A.at<double>(4,0) =-points1[2].x; A.at<double>(4,1)=-points1[2].y;A.at<double>(4,2)=-1; A.at<double>(4,6)=points1[2].x*points2[2].x; A.at<double>(4,7)=points1[2].y*points2[2].x;A.at<double>(4,8)=points2[2].x;

    A.at<double>(5,3) =-points1[2].x; A.at<double>(5,4)=-points1[2].y;A.at<double>(5,5)=-1; A.at<double>(5,6)=points1[2].x*points2[2].y; A.at<double>(5,7)=points1[2].y*points2[2].y;A.at<double>(5,8)=points2[2].y;

    A.at<double>(6,0) =-points1[3].x; A.at<double>(6,1)=-points1[3].y;A.at<double>(6,2)=-1; A.at<double>(6,6)=points1[3].x*points2[3].x; A.at<double>(6,7)=points1[3].y*points2[3].x;A.at<double>(6,8)=points2[3].x;

    A.at<double>(7,3) =-points1[3].x; A.at<double>(7,4)=-points1[3].y;A.at<double>(7,5)=-1; A.at<double>(7,6)=points1[3].x*points2[3].y; A.at<double>(7,7)=points1[3].y*points2[3].y;A.at<double>(7,8)=points2[3].y;


    cv::SVD svd(A,SVD::FULL_UV);
    Mat1d V = svd.vt.t();


    Matx33d H(3,3);
    // TODO 3
    // - Extract the homogeneous solution of Ah=0 as the rightmost column vector of V.
    // - Store the result in H.
    // - Normalize H
	H(0,0) =V.at<double>(0,8)*1/V.at<double>(8,8);
	H(0,1) =V.at<double>(1,8)*1/V.at<double>(8,8);
	H(0,2) =V.at<double>(2,8)*1/V.at<double>(8,8);
	H(1,0) =V.at<double>(3,8)*1/V.at<double>(8,8);
	H(1,1) =V.at<double>(4,8)*1/V.at<double>(8,8);
	H(1,2) =V.at<double>(5,8)*1/V.at<double>(8,8);
	H(2,0) =V.at<double>(6,8)*1/V.at<double>(8,8);
	H(2,1) =V.at<double>(7,8)*1/V.at<double>(8,8);
	H(2,2) =V.at<double>(8,8)*1/V.at<double>(8,8);
	
    return H;
}

void testHomography()
{
    std::vector<Point2f> points1 =
    {
        {1,1},{3,7},{2,-5},{10,11}
    };
    std::vector<Point2f> points2 =
    {
        {25,156},{51,-83},{-144,5},{345,15}
    };
    auto H = computeHomography(points1,points2);
    cout << "Testing Homography..." << endl << "Your result:" << endl;
    cout << H << endl;

    Matx33d Href = {
        -151.2372466105457, 36.67990057507507, 130.7447340624461,
        -27.31264543681857, 10.22762978292494, 118.0943169422209,
        -0.04233528054472634, -0.3101691983762523, 1
    };
    cout << "Reference: " << endl << Href << endl;

    auto error = Href - H;
    float e = norm(error);
    cout << "Error: " << e << endl;
    if(e < 1e-10)
        cout << "Test: SUCCESS!" << endl;
    else
        cout << "Test: FAIL!" << endl;
    cout << "============================" << endl;
}
