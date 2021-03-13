#include "sfm.h"



vector<DMatch> ratioTest(std::vector< std::vector<DMatch> > knnmatches, float ratio_threshold)
{
    std::vector<DMatch> matches;


    for( int i = 0; i < (int)knnmatches.size(); i++ )
    {
        DMatch first = knnmatches[i][0];
        DMatch second = knnmatches[i][1];

        if(first.distance < ratio_threshold * second.distance)
        {
            matches.push_back(first);
        }
    }
    return matches;
}

void testFundamentalMat();
void testTriangulate();

int main(int argc, char** argv )
{
    testFundamentalMat();
    testTriangulate();

    Mat3b img1 = imread("data/img1.jpg");
    Mat3b img2 = imread("data/img2.jpg");

    assert(img1.data);
    assert(img2.data);

    Matx33d K = {
        2890, 0, 1440,
        0, 2890, 960,
        0, 0, 1
    };

    /***********************Feature Matching**************************/

    std::vector<cv::KeyPoint> keypoints1;
    std::vector<cv::KeyPoint> keypoints2;
    Mat descriptors1;
    Mat descriptors2;
    static thread_local Ptr<ORB> detector =  cv::ORB::create(20000);
    detector->detectAndCompute(img1,noArray(), keypoints1, descriptors1);
    detector->detectAndCompute(img2,noArray(), keypoints2, descriptors2);

    cout << "Num Featues: " << keypoints1.size() << " " << keypoints2.size() << endl;

    FlannBasedMatcher matcher =  cv::FlannBasedMatcher(cv::makePtr<cv::flann::LshIndexParams>(12, 20, 2));;
    std::vector< std::vector<DMatch> > knnmatches;
    matcher.knnMatch(descriptors1, descriptors2, knnmatches,2 );

    std::vector<cv::DMatch> matches;
    matches= ratioTest(knnmatches,0.8);

    {
        Mat img_matches;
        drawMatches( img1, keypoints1, img2, keypoints2,
                     matches, img_matches, Scalar(0,255,0), Scalar(0,255,0),
                     vector<char>(), DrawMatchesFlags::DEFAULT );
        resize(img_matches,img_matches,Size(),0.2,0.2);
        imshow("All Matches" , img_matches );
    }

    cout << "Matches after ratio test: " << matches.size() << endl;



    /************** Compute F and E **************************/


    std::vector<cv::Point2f> points1, points2;
    for (auto m : matches) {
        points1.push_back(keypoints1[m.queryIdx].pt);
        points2.push_back(keypoints2[m.trainIdx].pt);
    }

    std::vector<int> inliers;
    Matx33d F = computeFRANSAC(points1,points2,inliers);
     cout << "RANSAC inliers " << inliers.size() << endl;

    // Compute E and normalize
    Matx33d E = K.t() * F * K;
    E *= 1.0 / E(2,2);



    std::vector<cv::DMatch> inlier_matches;
    std::vector<cv::Point2f> inlier_points1, inlier_points2;
    for(int i : inliers)
    {
        inlier_matches.push_back(matches[i]);
        inlier_points1.push_back(points1[i]);
        inlier_points2.push_back(points2[i]);
    }

    if(inlier_matches.size() == 0)
        return 0;

    {
        //draw filtered matches (ransac inliers)
        Mat img_matches;
        drawMatches( img1, keypoints1, img2, keypoints2, inlier_matches, img_matches , Scalar(0,255,0), Scalar(0,255,0),
                     vector<char>(), DrawMatchesFlags::DEFAULT );
        resize(img_matches, img_matches,Size(),0.2,0.2);
        imshow("RANSAC inlier matches", img_matches );
    }


    /************** Compute relative transformation **************************/

    Matx34d View1 = Matx34d::eye();
    Matx34d View2 = relativeTransformation(E,inlier_points1,inlier_points2,K);


    /************** Triangulate inlier matches **************************/

    vector<Point3f> wps = triangulate(View1,View2,K,inlier_points1,inlier_points2);

    //small sanity check to remove low angle triangulated points and some more outliers
    wps.erase(std::remove_if(wps.begin(),wps.end(),[](Point3f p){ return p.z > 10 || p.y > 1;}),wps.end());



    /************** Rendering **************************/


    vector<Vec3b> colors(wps.size(),Vec3b(255,0,0));

    for(int i =0; i < (int)wps.size(); ++i)
    {
        colors[i] = img1(inlier_points1[i].y,inlier_points1[i].x);
    }

    cv::viz::WCloud cloud( wps,colors );
    cloud.setRenderingProperty(cv::viz::POINT_SIZE,4);

    cv::viz::WCoordinateSystem cys(0.2);

    cv::viz::WCameraPosition cp1(K,img1,0.5);
    cv::viz::WCameraPosition cp2(K,img2,0.5);

//    cv::viz::WCameraPosition cp1(K,0.5);
//    cv::viz::WCameraPosition cp2(K,0.5);

    Matx44d View1a = Matx44d::eye(), View2a= Matx44d::eye();
    for(int i = 0; i < 3; ++i)
        for(int j = 0; j < 4; ++j)
        {
            View1a(i,j) = View1(i,j);
            View2a(i,j) = View2(i,j);
        }


    cp1.applyTransform(View1a.inv());
    cp2.applyTransform(View2a.inv());

    cv::viz::Viz3d viewer("Two-View Reconstruction");
    viewer.setWindowSize(Size(1200,800));
    viewer.setBackgroundColor();
    viewer.showWidget( "cys", cys );
    viewer.showWidget( "Cloud", cloud );
    viewer.showWidget( "cp1", cp1 );
    viewer.showWidget( "cp2", cp2 );

    cv::waitKey(0);
    viewer.spin();
    exit(1);
}


