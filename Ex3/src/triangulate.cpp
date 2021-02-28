#include "sfm.h"


Point3f triangulate(Matx34d P1, Matx34d P2, Point2f p1, Point2f p2)
{
    Matx44d A;//= Matx44d::eye();

    // TODO 3
    // - Construct the matrix A.
//    A.row(1) =p1.y*P1.row(2)-P1.row(1);
//    A.row(2) =p2.x*P2.row(2)-P2.row(0);
//    A.row(3) =p2.y*P2.row(2)-P2.row(1);
//    otherwise A is all zeros

    Matx14d r0 =p1.x*P1.row(2)-P1.row(0);
    Matx14d r1 =p1.y*P1.row(2)-P1.row(1);
    Matx14d r2 =p2.x*P2.row(2)-P2.row(0);
    Matx14d r3 =p2.y*P2.row(2)-P2.row(1);

    A(0, 0) = r0(0,0);
    A(0, 1) = r0(0,1);
    A(0, 2) = r0(0,2);
    A(0, 3) = r0(0,3);

    A(1, 0) = r1(0,0);
    A(1, 1) = r1(0,1);
    A(1, 2) = r1(0,2);
    A(1, 3) = r1(0,3);

    A(2, 0) = r2(0,0);
    A(2, 1) = r2(0,1);
    A(2, 2) = r2(0,2);
    A(2, 3) = r2(0,3);

    A(3, 0) = r3(0,0);
    A(3, 1) = r3(0,1);
    A(3, 2) = r3(0,2);
    A(3, 3) = r3(0,3);


    std::cout << A << std::endl;

    cv::SVD svd(A,SVD::FULL_UV);
    Mat1d V = svd.vt.t();

    std::cout << V << std::endl;


    // TODO 3
    // - Extract the solution and project it back to 3D (from homogeneous space)
    Point3f x(V(0,3)/V(3,3), V(1,3)/V(3,3),V(2,3)/V(3,3));
    return x;
}


vector<Point3f> triangulate(Matx34d View1, Matx34d View2, Matx33d K, std::vector<cv::Point2f> points1, std::vector<cv::Point2f> points2)
{
    vector<Point3f> wps;

    Matx34d P1, P2;
    P1 = K * View1;
    P2 = K * View2;

    for(int i = 0; i < (int)points1.size(); ++i)
    {
        Point3f wp = triangulate(P1,P2,points1[i],points2[i]);

        //check if this point is in front of both cameras
        Vec4d ptest(wp.x,wp.y,wp.z,1);
        Vec3d p1 = K * View1 * ptest;
        Vec3d p2 = K * View2 * ptest;

        if(p1[2] < 0 || p2[2] < 0)
        {
            //point is behind one of the cameras
        }else
        {
            wps.push_back(wp);
        }
    }
    return wps;
}



void testTriangulate()
{
    Matx34d P1 = {
        1,2,3,6,
        4,5,6,37,
        7,8,9,15
    };
    Matx34d P2 = {
        51,12,53,73,
        74,15,-6,-166,
        714,-8,95,16
    };

    auto F = triangulate(P1,P2,Point2f(14,267),Point2f(626,67));
    cout << "Testing Triangulation..." << endl << "Your result:" << endl;
    cout << F << endl;

    Point3f wpref = {
       0.782409, 3.89115, -5.72358
    };
    cout << "Reference: " << endl << wpref << endl;

    auto error = wpref - F;
    double e = norm(error);
    cout << "Error: " << e << endl;
    if(e < 1e-5)
        cout << "Test: SUCCESS!" << endl;
    else
        cout << "Test: FAIL!" << endl;
    cout << "============================" << endl;
}
