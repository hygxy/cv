#include "stereoVision.h"
#include <random>
#include <chrono>
#include <thread>
#include "omp.h"

StereoVision::StereoVision()
{
    intrinsics1.readFromFile("intr1.yml");
    intrinsics2.readFromFile("intr2.yml");
    extrinsics.readFromFile("extr.yml");

    cv::stereoRectify(intrinsics1.K,intrinsics1.dist,intrinsics2.K,intrinsics2.dist,Size(intrinsics1.w,intrinsics1.h),extrinsics.R,extrinsics.t,R1,R2,P1,P2,Q);

    cv::initUndistortRectifyMap(intrinsics1.K,intrinsics1.dist,R1,P1,Size(intrinsics1.w,intrinsics1.h),CV_32FC1,map11,map12);
    cv::initUndistortRectifyMap(intrinsics2.K,intrinsics2.dist,R2,P2,Size(intrinsics2.w,intrinsics2.h),CV_32FC1,map21,map22);


    // use low quality so it works in real time
    disparity.initIterations = 1;
    disparity.optimizeIterations = 1;
    disparity.patchRadius =  2;
    disparity.propagateIterations = 1;
}

void StereoVision::run()
{
    cout << "Starting StereoVision with 2 USB-Cameras..." << endl;
//    int w,h;

    VideoCapture capture1, capture2;
    openCameras(capture1,capture2);

//    w = capture1.get(CV_CAP_PROP_FRAME_WIDTH);
//    h = capture1.get(CV_CAP_PROP_FRAME_HEIGHT);





    Mat frame1,gray1;
    Mat frame2,gray2;

    Mat1b frame1_rect, frame2_rect;

    Mat disp;
    Mat1f dispFL,dispFR, dispF;


    cout << "Press ESC to stop" << endl;

    while(true)
    {


        capture1.grab();
        capture2.grab();

        capture1.retrieve(frame1);
        capture2.retrieve(frame2);

        cvtColor(frame1, gray1, CV_RGB2GRAY);
        cvtColor(frame2, gray2, CV_RGB2GRAY);

        cv::remap(gray1,frame1_rect,map11,map12,INTER_LINEAR);
        cv::remap(gray2,frame2_rect,map21,map22,INTER_LINEAR);

        {
            imshow("Left", frame1_rect);
            imshow("Right", frame2_rect);
            cv::moveWindow("Left",20,20);
            cv::moveWindow("Right",20+gray1.cols,20);
        }


        std::thread t1([&](){disparity.computeDisparity(frame1_rect,frame2_rect,dispFL,0,maxDisp);});

        std::thread t2([&](){disparity.computeDisparity(frame2_rect,frame1_rect,dispFR,-maxDisp,0);});

        t1.join();
        t2.join();


        imshow("disp L", (dispFL)/maxDisp);
        imshow("disp R", (dispFR)/-maxDisp);


        cv::moveWindow("disp L",20,60+gray1.rows);
        cv::moveWindow("disp R",20+gray1.cols,60+gray1.rows);





        int keyPressed = waitKey(10)%256;
        if( keyPressed == 27 )
        {
            cout << "done." << endl;
            break;
        }

    }
}
