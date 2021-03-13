#include "stereoCalibration.h"
#include <thread>

StereoCalibration::StereoCalibration()
{

    patternsize = Size(13,6);
    //    patternsize = Size(29,16);
    for( int i = 0; i < patternsize.height; i++ )
    {
        for( int j = 0; j < patternsize.width; j++ )
        {
            objPoints.push_back(Point3f(j*SquareDistanceMeters,
                                        i*SquareDistanceMeters, 0));
        }
    }
}

std::vector<Point2f> StereoCalibration::findPattern(Mat image)
{
    vector<Point2f> corners;

    bool found = findChessboardCorners( image, patternsize, corners);
    //        cout << "found " << found << endl;

    if ( found)                // If done with success,
    {
        // improve the found corners' coordinate accuracy for chessboard
        Mat viewGray;
        cvtColor(image, viewGray, CV_BGR2GRAY);
        cornerSubPix( viewGray, corners, Size(11,11),
                      Size(-1,-1), TermCriteria( CV_TERMCRIT_EPS+CV_TERMCRIT_ITER, 30, 0.1 ));
    }
    return corners;
}



Intrinsics StereoCalibration::calibrateIntrinsics(vector<vector<Point2f>>& corners, int w, int h)
{
    Intrinsics intrinsics;
    if(corners.empty()) return intrinsics;
    vector<vector<Point3f>> objPointss(corners.size(),objPoints);


    intrinsics.w = w;
    intrinsics.h = h;
    Mat rvecs,tvecs;
    auto error = calibrateCamera(objPointss,corners,Size(w,h),intrinsics.K,intrinsics.dist,rvecs,tvecs);

    cout << "calibrateCamera error: " << error << endl;

    return intrinsics;
}

StereoExtrinsics StereoCalibration::calibrateStereo(Intrinsics intrinsics1, Intrinsics intrinsics2, vector<vector<Point2f> > &corners1, vector<vector<Point2f> > &corners2)
{
    assert(corners1.size() == corners2.size());
    StereoExtrinsics extr;
    if(corners1.empty()) return extr;
    vector<vector<Point3f>> objPointss(corners1.size(),objPoints);


    auto error = cv::stereoCalibrate(objPointss,corners1,corners2,intrinsics1.K,intrinsics1.dist,intrinsics2.K,intrinsics2.dist,Size(intrinsics1.w,intrinsics1.h),extr.R,extr.t,extr.E,extr.F,CALIB_FIX_INTRINSIC);

    cout << "stereoCalibrate error: " << error << endl;

    return extr;
}




void StereoCalibration::calibStereoVideo()
{

    int w,h;

    VideoCapture capture1, capture2;
    openCameras(capture1,capture2);

    w = capture1.get(CV_CAP_PROP_FRAME_WIDTH);
    h = capture1.get(CV_CAP_PROP_FRAME_HEIGHT);



    namedWindow("Left",1);
    moveWindow("Left", 150, 150);

    namedWindow("Right",1);
    moveWindow("Right", 150+w, 150);

    Mat frame1;
    Mat frame2;

    vector<vector<Point2f>> all_corners1, corners1;
    vector<vector<Point2f>> all_corners2, corners2;

    cout << "========= Controls ===========" << endl;

    cout << "[space]      Capture Image for both cameras. Used for intrinsic and extrinsic calibration." << endl;
    cout << "[r][l]       Capture Image for right/left camera. Can only be used for intrinsic calibration." << endl;
    cout << "[i]          Write intrinsics to file." << endl;
    cout << "[e]          Write extrinsics to file." << endl;
    cout << "[x]          Remove last frame." << endl;
    cout << "[esc]        Quit." << endl;


    int space = 32;
    int r = 114;
    int l = 108;
    int e = 101;
    int x = 120;
    int i = 105;
    int esc = 27;

    Intrinsics i1,i2;
    StereoExtrinsics extr;

    while(true)
    {
        std::thread t1([&](){
            capture1.grab();
            capture1.retrieve(frame1);
        });
        capture2.grab();
        capture2.retrieve(frame2);
        t1.join();

        {

            Mat vis1 = frame1.clone();
            Mat vis2 = frame2.clone();

            for(auto& v : all_corners1)
                for(auto& p : v)
                    cv::circle(vis1,p,2,cv::Scalar(0,255,0));

            for(auto& v : all_corners2)
                for(auto& p : v)
                    cv::circle(vis2,p,2,cv::Scalar(0,255,0));

            cv::putText(vis1,std::to_string(corners1.size()) + "/" + std::to_string(all_corners1.size()),Point2f(0,h-6),0,0.5,cv::Scalar(255,255,255));
            cv::putText(vis2,std::to_string(corners2.size()) + "/" + std::to_string(all_corners2.size()),Point2f(0,h-6),0,0.5,cv::Scalar(255,255,255));


            imshow("Left", vis1);
            imshow("Right", vis2);
        }

        bool changed = false;

        int keyPressed = waitKey(1)%256;
        //        if(keyPressed!=-1) cout << keyPressed << endl;

        vector<Point2f> pattern1, pattern2;


        if(keyPressed == l || keyPressed == space  )
        {
            pattern1 = findPattern(frame1);
            if(pattern1.size() > 0)
            {
                all_corners1.push_back(pattern1);
                //                cout << "Found pattern 1" << endl;
                allPoints1.insert(allPoints1.end(),pattern1.begin(),pattern1.end());
                changed = true;
            }
        }

        if(keyPressed == r || keyPressed == space  )
        {
            pattern2 = findPattern(frame2);
            if(pattern2.size() > 0)
            {
                all_corners2.push_back(pattern2);
                //                cout << "Found pattern 2" << endl;
                allPoints2.insert(allPoints2.end(),pattern2.begin(),pattern2.end());
                changed = true;
            }

        }

        if(pattern1.size() > 0  && pattern2.size() >0)
        {
            corners1.push_back(pattern1);
            corners2.push_back(pattern2);
            changed = true;

        }

        if(keyPressed == x)
        {
            corners1.pop_back();
            corners2.pop_back();
            all_corners1.pop_back();
            all_corners2.pop_back();
            changed = true;
        }

        if(changed)
        {
            cout << "Reprojection Error:" << endl;
            cout << "[Camera 1] ";
            i1 = calibrateIntrinsics(all_corners1,w,h);
            cout << "[Camera 2] ";
            i2 = calibrateIntrinsics(all_corners2,w,h);
            cout << "[ Stereo ] ";
            extr = this->calibrateStereo(i1,i2,corners1,corners2);
        }

        if(keyPressed == e)
        {
            if(corners1.size() < 5)
            {
                cout << "Need at lesat 5 images for stereo calibration." << endl;
            }else{
                cout << "Saving extrinsics" << endl;
                extr.writeToFile("extr.yml");

            }
        }

        if(keyPressed == i)
        {
            if(all_corners1.size() < 5)
            {
                cout << "Need at least 5 images for intrinsic calibration of camera 1." << endl;
            }else{
                cout << "Saving intrinsics 1" << endl;
                i1.writeToFile("intr1.yml");
            }
            if(all_corners1.size() < 5)
            {
                cout << "Need at least 5 images for intrinsic calibration of camera 2." << endl;
            }else{
                cout << "Saving intrinsics 2" << endl;
                i2.writeToFile("intr2.yml");
            }
        }

        if( keyPressed == esc )
        {
            cout << "done." << endl;
            break;
        }

    }




    return;




}

