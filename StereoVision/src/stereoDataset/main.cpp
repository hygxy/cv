#include "disparity.h"
#include <thread>

void testScore(Disparity& disparity)
{
    auto oldPatchRadius = disparity.patchRadius;
    disparity.patchRadius = 1;

    Mat1b img1(5, 5);
    img1 <<
            15,26,77,15,72,
            72,161,37,15,94,
            15,5,6,16,72,
            1,2,3,4,5,
            73,1,6,27,71;
    Mat1b img2(5, 5);
    img2 <<
            1,2,3,4,5,
            7,2,2,7,2,
            27,15,15,77,88,
            76,76,41,99,0,
            16,17,723,14,1;


    float disps[5] = { -1,0,2,1,1};

    Mat1f costs(5,5);
    for(int i = 0; i < 5; ++i)
        for(int j = 0; j < 5; ++j)
            costs(i,j) = disparity.cost(img1,img2,j,i,disps[j]);

    Mat1f costsref(5,5);
    costsref <<
                10000000, 10000000, 10000000, 10000000, 10000000,
            10000000, 44.666668, 10000000, 33.888889, 10000000,
            10000000, 53, 10000000, 39.333332, 10000000,
            10000000, 55.111111, 10000000, 52.333332, 10000000,
            10000000, 10000000, 10000000, 10000000, 10000000;


    auto err =  costs - costsref;
    auto e = norm(err);
    cout << endl << "cost error: " << e << endl;
    if (e < 1e-4) cout << "Test: SUCCESS!" << endl;
    else
    {
        cout << "Reference " << endl << costsref << endl;
        cout << "Your result " << endl << costs << endl;
        cout << "Test: FAIL" << endl;
    }
    cout << endl;

    disparity.patchRadius = oldPatchRadius;
}

int main(int argc, char** argv )
{
    Disparity disparity;
    testScore(disparity);

    cout << "Running disparity matcher from tsukuba dataset..." << endl;

    Mat left, right;


    left = cv::imread("data/tsukuba/img1.ppm");
    right = cv::imread("data/tsukuba/img2.ppm");


    Mat1b leftGray, rightGray;
    cvtColor(left, leftGray, CV_RGB2GRAY);
    cvtColor(right, rightGray, CV_RGB2GRAY);


    cv::imshow("Left",left);
    cv::imshow("Right",right);

    cv::moveWindow("Left",20,20);
    cv::moveWindow("Right",20+left.cols,20);



    auto maxDisp = 14;

    Mat1f dispFR, dispFL;

    auto t = cv::getTickCount();
    disparity.computeDisparity(leftGray,rightGray,dispFL,0,maxDisp);

    //    disparity.computeDisparity(rightGray,leftGray,dispFR,-maxDisp,0);
    auto t2 = cv::getTickCount();


    imshow("Final Disparity Left", (dispFL)/maxDisp);
    //    imshow("Final Disparity Right", (-dispFR)/maxDisp);


    //    Mat1b out;
    //    dispFL.convertTo(out,out.type(),1.0/maxDisp*255);
    //    imwrite("output.png",out);

    cv::moveWindow("Final Disparity Left",20,60+left.rows);
    //    cv::moveWindow("Final Disparity Right",20+left.cols,60+left.rows);


    cout << "Total time for both disparity maps: " << (t2-t)/cv::getTickFrequency()*1000.0 << " ms" << endl;

    cout << "Press any key to exit..." << endl;
    cv::waitKey();

    return 0;

}
