#include "opticalFlowLK.h"

#include <thread>
#include <condition_variable>
#include <mutex>


void test()
{
    {
        // Tests for bilinear weights
        Point2f p(0.125,0.82656);
        Vec4d weights(0.15176, 0.02168, 0.72324, 0.10332);
        auto err = computeBilinearWeights(p) - weights;
        auto e = norm(err);
        cout << "computeBilinearWeights error: " << e << endl;
        if (e < 1e-6) cout << "Test: SUCCESS!" << endl;
        else cout << "Test: FAIL" << endl;
    }

    {
        // Test gaussian kernel
        Mat1d kernel(5, 5);
        kernel << 0.1690133273455962, 0.3291930023422986,
                0.4111123050281957, 0.3291930023422986, 0.1690133273455962,
                0.3291930023422986, 0.6411803997536073,
                0.8007374099875735, 0.6411803997536073, 0.3291930023422986,
                0.4111123050281957, 0.8007374099875735, 1,
                0.8007374099875735, 0.4111123050281957,
                0.3291930023422986, 0.6411803997536073,
                0.8007374099875735, 0.6411803997536073, 0.3291930023422986,
                0.1690133273455962, 0.3291930023422986,
                0.4111123050281957, 0.3291930023422986, 0.1690133273455962;
        auto err = computeGaussianWeights(Size(5, 5), .3) - kernel;
        auto e = norm(err);
        cout << endl << "computeGaussianWeights error: " << e << endl;
        if (e < 1e-10) cout << "Test: SUCCESS!" << endl;
        else cout << "Test: FAIL" << endl;
    }

    {

        // Tests for matrix inversion and gaussian weights
        Matx22d A(12, 4, 4, 8);
        Matx22d Ainv(0.1, -0.05, -0.05, 0.15);
        auto err = invertMatrix2x2(A) - Ainv;
        auto e = norm(err);
        cout << endl << "invertMatrix2x2 error: " << e << endl;
        if (e < 1e-10) cout << "Test: SUCCESS!" << endl;
        else cout << "Test: FAIL" << endl;
    }


    cout << "============================" << endl;

}

struct BufferStage1
{
    volatile bool valid = false;
    std::shared_ptr<Mat3b> image;
};

struct BufferStage2
{
    volatile bool valid = false;
    std::shared_ptr<Mat3b> image;
    std::shared_ptr<Mat1b> grey;
    vector<Point2f> keypoints;
};


int main( int argc, char** argv )
{
    test();

    VideoCapture cap;
    cap.open(0);
    if( !cap.isOpened() )
    {
        cout << "Could not find a camera." << endl;
        return 0;
    }

    int w = 640;
    int h = 480;
    cap.set(cv::CAP_PROP_FRAME_WIDTH,w);
    cap.set(cv::CAP_PROP_FRAME_HEIGHT,h);

    std::string windowName = "CV Exercise 4";
    namedWindow(windowName , 1 );



    BufferStage1 buffer1;
    BufferStage2 buffer2;
    volatile bool running = true;
    std::mutex lock1, lock2;
    std::condition_variable cv1, cv2;

    std::thread pullThread(
                [&](){
        Mat3b  frame;
        while(running)
        {
            cap >> frame;
            if( frame.empty() )
            {
                running = false;
                break;
            }
            BufferStage1 output;
            output.valid = true;
            output.image = std::make_shared<Mat3b>(h,w);
            frame.copyTo(*output.image);
            {
                std::unique_lock<std::mutex> l(lock1);
                buffer1 = output;
                cv1.notify_one();
            }
        }
    });

    std::thread preprocessThread(
                [&](){
        while(running)
        {
            BufferStage1 input;
            {
                std::unique_lock<std::mutex> l(lock1);
                cv1.wait(l, [&](){return buffer1.valid;}); //wait until the buffer is valid
                input = buffer1;
                buffer1.valid = false;
            }
            assert(input.image);

            BufferStage2 output;
            output.valid = true;
            output.image = input.image;
            output.grey = std::make_shared<Mat1b>(h,w);
            cvtColor(*input.image, *output.grey, COLOR_BGR2GRAY);



            output.keypoints.clear();
#if 1
            TermCriteria termcrit(TermCriteria::COUNT|TermCriteria::EPS,20,0.03);
            Size subPixWinSize(10,10);
            goodFeaturesToTrack(*output.grey, output.keypoints, 1000, 0.01, 10, Mat(),  3, false, 0.04);
            cornerSubPix(*output.grey, output.keypoints, subPixWinSize, Size(-1,-1), termcrit);
#else

            int wsize = 5;
            for (int i = wsize+1; i < output.image->rows - wsize-1; i+=20)
            {
                for (int j = wsize+1; j < output.image->cols - wsize-1; j+= 20)
                {
                    output.keypoints.push_back(cv::Point2f(j,i));
                }
            }
#endif

            {
                std::unique_lock<std::mutex> l(lock2);
                buffer2 = output;
                cv2.notify_one();
            }
        }
        return;

    });


    vector<uchar> status;
    vector<Point2f> prevpoints;
    Size winSize(31,31);
    std::shared_ptr<Mat1b> prevGray;
    std::shared_ptr<Mat3b> prevImg;

    int frame = 0;
    while(running)
    {

        BufferStage2 input;
        {
            std::unique_lock<std::mutex> l(lock2);
            cv2.wait(l, [&](){return buffer2.valid;}); //wait until the buffer is valid
            input = buffer2;
            buffer2.valid = false;
        }
        assert(input.grey);
        assert(input.image);


        vector<Point2f> points;
        int flowPoints= 0;
        if( !input.keypoints.empty() && prevGray )
        {

            static OpticalFlowLK of(winSize,0.03,20);
            of.compute(*prevGray, *input.grey, prevpoints, points, status);


            for( int i  = 0; i < (int)points.size(); i++ )
            {
                if( !status[i] )
                    continue;

                auto diff = points[i] - prevpoints[i];
                float distance = norm(diff);

                if(distance > 15 || distance < 1)
                    continue;

                auto otherP = prevpoints[i] + diff * 2;
                flowPoints++;
                Scalar color(0, 255, 0);
                circle(*prevImg,prevpoints[i], 1,color);
                line(*prevImg, prevpoints[i], otherP,
                     color);
            }
            imshow(windowName, *prevImg);
        }

        cout << "[Frame] " << frame << "   [Keypoints] moving/total: " << flowPoints << "/" << input.keypoints.size() << endl;


        char c = (char)waitKey(1);
        if( c != -1 ) running = false;
        prevGray = input.grey;
        prevpoints = input.keypoints;
        prevImg= input.image;

        frame++;
    }

    pullThread.join();
    preprocessThread.join();

    return 0;
}

