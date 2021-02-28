#include <math.h>
#include "opticalFlowLK.h"

using namespace std;
void OpticalFlowLK::compute(Mat1b prevImg, Mat1b nextImg,
                            std::vector<Point2f>& prevPts, std::vector<Point2f>& nextPts,
                            std::vector<uchar>& status)
{
    assert(prevImg.data && nextImg.data);
    assert(prevImg.rows == nextImg.rows);
    assert(prevImg.cols == nextImg.cols);

    int N = prevPts.size();

    nextPts.resize(N);
    status.resize(N);

    for(int i = 0; i < N; ++i)
    {
        status[i] = true;
        nextPts[i] = prevPts[i];
    }


    // TODO 2.4
    // - Compute the spacial derivatives of prev using the Scharr function
    // - Make sure to use the normalized Scharr filter!!
    Mat1d prevDerivx;
    Mat1d prevDerivy;

    Scharr(prevImg, prevDerivx,CV_64F,1,0,1./32,0,BORDER_DEFAULT);
    Scharr(prevImg, prevDerivy,CV_64F,0,1,1./32,0,BORDER_DEFAULT);



    Point2f halfWin((winSize.width-1)*0.5f, (winSize.height-1)*0.5f);
    Mat1d weights = computeGaussianWeights(winSize,0.3);

#pragma omp parallel for
    for( int ptidx = 0; ptidx < N; ptidx++ )
    {
        Point2f u0 = prevPts[ptidx];
        u0 -= halfWin;

        Point2f u = u0;
        Point2i iu0;
        iu0.x = cvFloor(u0.x);
        iu0.y = cvFloor(u0.y);

        if( iu0.x < 0 || iu0.x + winSize.width >= prevImg.cols ||
                iu0.y < 0 || iu0.y + winSize.height >= prevImg.rows )
        {
            status[ptidx] = 0;
            continue;
        }

        // Bilinear weights
        Vec4d bw = computeBilinearWeights(u0);


        Mat1d bprev(winSize.width * winSize.height,1);
        Mat1d A(winSize.width * winSize.height, 2);
        Matx22d AtWA = Matx22d::zeros();
        Matx22d invAtWA;

        Mat1d W(winSize.width * winSize.height,winSize.width * winSize.height,0.0);  //  Mat_ (int _rows, int _cols, const _Tp &value)

//    cout <<W.cols<<endl;
//    cout <<W.rows<<endl;




        for (int i =0;i<winSize.height;i++)
        {
            for (int j =0;j<winSize.width;j++)
            {
                W(i*winSize.width+j,i*winSize.width+j)=weights(i,j);

            }
        }

//   std::cout <<W  <<std::endl;


        for(int y = 0; y < winSize.height; y++)
        {
            for(int x = 0 ; x < winSize.width; x++)
            {
                int gx = iu0.x + x;
                int gy = iu0.y + y;

                // TODO 3.1
                // Compute the following parts of step 2.
                //   bprev      (w*h) x 1 Matrix
                //   A          (w*h) x 2 Matrix
                //   AtWA           2 x 2 Matrix
                // Use the bilinear weights bw!
                // W is stored in 'weights', but not as a diagonal matrix!!!
                // Keine Schleife notwendig
                bprev(y*winSize.width+x,0)= bw[0]*prevImg(gy,gx)+bw[2]*prevImg(gy+1,gx)+bw[1]*prevImg(gy,gx+1)+bw[3]*prevImg(gy+1,gx+1);
                A(y*winSize.width+x,0)= bw[0]*prevDerivx(gy,gx)+bw[2]*prevDerivx(gy+1,gx)+bw[1]*prevDerivx(gy,gx+1)+bw[3]*prevDerivx(gy+1,gx+1);
                A(y*winSize.width+x,1)= bw[0]*prevDerivy(gy,gx)+bw[2]*prevDerivy(gy+1,gx)+bw[1]*prevDerivy(gy,gx+1)+bw[3]*prevDerivy(gy+1,gx+1);
       //Note that Mat::at(int y,int x) and Mat_::operator()(int y,int x) do absolutely the same and run at the same speed, but the latter is certainly shorter

            }
        }

        AtWA =Mat(A.t()*(W*A));



        // TODO 3.2
        // Compute invAtWA
        // Use the function invertMatrix2x2!
        invAtWA =invertMatrix2x2(AtWA);

        // Estimate the target point with the previous point
        u = u0;

        // Iterative solver
        for(int j = 0; j < iterations; j++ )
        {
            Point2i iu;
            iu.x = cvFloor(u.x);
            iu.y = cvFloor(u.y);

            if( iu.x < 0 || iu.x + winSize.width >= prevImg.cols ||
                    iu.y < 0 || iu.y + winSize.height >= prevImg.rows )
            {
                status[ptidx] = 0;
                break;
            }

            Vec4d bw = computeBilinearWeights(u);
            Mat1d bnext(winSize.width * winSize.height,1);

            Vec2d AtWbnbp(0,0);
            for(int y = 0; y < winSize.height; y++ )
            {
                for(int x = 0 ; x < winSize.width; x++)
                {
                    int gx = iu.x + x;
                    int gy = iu.y + y;

                    // TODO 3.2
                    // Compute the following parts of step 2
                    //   AtWbnbp      2 x 1 Vector

                        bnext(y*winSize.width+x,0)=
                                bw[0]*nextImg(gy,gx)+
                                bw[2]*nextImg(gy+1,gx)+
                                bw[1]*nextImg(gy,gx+1)+
                                bw[3]*nextImg(gy+1,gx+1);


                }
            }
            AtWbnbp =Mat(A.t()*(W*(bnext-bprev)));

            // TODO 3.2
            // - Solve the linear system for deltaU: At * W * A * deltaU = - At * W * (bnext - bprev)
            // - Add deltaU to u
            // - Implement the early termination condition (Step 4)
            Point2d deltaU(0, 0);
            deltaU =-invAtWA*AtWbnbp;
            u.x+=deltaU.x;
            u.y+=deltaU.y;
            if ((pow(deltaU.x,2.0)+pow(deltaU.y,2.0))<epsilon)
                break;

        }

                nextPts[ptidx] = u + halfWin;

    }

}


