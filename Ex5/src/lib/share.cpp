#include "share.h"

void Intrinsics::writeToFile(string file)
{
    cv::FileStorage fs(file,FileStorage::WRITE);

    fs << "w" << w;
    fs << "h" << h;
    fs << "K" << K;
    fs << "dist" << dist;
}

void Intrinsics::readFromFile(string file)
{
       cv::FileStorage fs(file,FileStorage::READ);
       fs["w"] >> w;
       fs["h"] >> h;
       fs["K"] >> K;
       fs["dist"] >> dist;
}
void StereoExtrinsics::writeToFile(string file)
{
    cv::FileStorage fs(file,FileStorage::WRITE);

    fs << "R" << R;
    fs << "t" << t;
    fs << "F" << F;
    fs << "E" << E;
}

void StereoExtrinsics::readFromFile(string file)
{
    cv::FileStorage fs(file,FileStorage::READ);
    fs["R"] >> R;
    fs["t"] >> t;
    fs["F"] >> F;
    fs["E"] >> E;
}

void openCameras(VideoCapture& capture1, VideoCapture& capture2)
{
    int w = 640;
    int h = 480;

    double exposure = 0;
    int fps = 10;


    int camIdLeft = 0;
    int camIdRight = 1;
    capture1.open(camIdLeft);

    if(!capture1.isOpened())
    {
        cout << "Could not open camera " << camIdLeft << endl;
        return;
    }

    capture2.open(camIdRight);
    if(!capture2.isOpened())
    {
        cout << "Could not open camera " << camIdRight << endl;
        return;
    }


    capture1.set(CV_CAP_PROP_FRAME_WIDTH, w);
    capture1.set(CV_CAP_PROP_FRAME_HEIGHT, h);
    capture2.set(CV_CAP_PROP_FRAME_WIDTH, w);
    capture2.set(CV_CAP_PROP_FRAME_HEIGHT, h);

    capture1.set(CV_CAP_PROP_FPS, fps);
    capture2.set(CV_CAP_PROP_FPS, fps);


//    capture1.set(CV_CAP_PROP_AUTO_EXPOSURE, 1);
//    capture1.set(CAP_PROP_BRIGHTNESS, 0.25);
//    capture1.set(CV_CAP_PROP_EXPOSURE, exposure);
//    capture1.set(CV_CAP_PROP_GAIN, 0);

//    capture2.set(CV_CAP_PROP_AUTO_EXPOSURE, 1);
//    capture2.set(CAP_PROP_BRIGHTNESS, 0.25);
//    capture2.set(CV_CAP_PROP_EXPOSURE, exposure);
//    capture2.set(CV_CAP_PROP_GAIN, 0);




    cout << "Both cameras openend." << endl;
    cout << "Input size: " << w << "x" << h << endl;


}
