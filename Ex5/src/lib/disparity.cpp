#include "disparity.h"
#include <random>
#include <chrono>
#include <thread>
#include <math.h>
#include "omp.h"


void Disparity::computeDisparity(const Mat1b &img1, const Mat1b &img2, Mat1f& dispF, float minDisp, float maxDisp)
{
    dispF.create(img1.rows,img1.cols);
    Mat1f scoreF(img1.rows,img1.cols);


    initRandom(img1,img2,dispF,scoreF,minDisp,maxDisp);
    for(int i = 0; i < propagateIterations; ++i)
    {
        propagate(img1,img2,dispF,scoreF,minDisp,maxDisp,true);
        randomSearch(img1,img2,dispF,scoreF,minDisp,maxDisp);

        propagate(img1,img2,dispF,scoreF,minDisp,maxDisp,false);
        randomSearch(img1,img2,dispF,scoreF,minDisp,maxDisp);
    }

}



float Disparity::cost(const Mat1b &img1, const Mat1b &img2, int x, int y, float d)
{
    if(y + patchRadius >= img1.rows ||
            y - patchRadius < 0 ||
            x - patchRadius < 0 ||
            x + patchRadius >= img1.cols ||
            x -d - patchRadius < 0 ||
            x - d + patchRadius >= img1.cols)
    {
        // return a really high cost
        return 10000000;
    }


    //   *(ceil(j-d)-(j-d))
    //   *((j-d)-floor(j-d))


    // TODO 2.1
    // Compute the SAD score.
    // - Iterate over the patch in img1
    // - use the variable "patchRadius". Note the total patch size is (patchRadius*2+1)*(patchRadius*2+1)
    // - sample the second image with linear interpolation
    float c = 0.f;
    for(int i =y-patchRadius;i<=y+patchRadius;i++)    // note the upper border of the loop
    {
        for(int j =x-patchRadius;j<=x+patchRadius;j++)
        {
            float alpha =((j-d)-floor(j-d));
       //    float belta =(ceil(j-d)-(j-d))/(ceil(j-d)-floor(j-d));
           c+=abs(img1(i,j)-(img2(i,floor(j-d))*(1-alpha)+img2(i,ceil(j-d))*alpha));
            //if the d is an interger,we can use the following line
        //    c+=abs(img1(i,j)-(img2(i,floor(j-d))+img2(i,ceil(j-d)))/2);
        }
    }

    c /=(patchRadius*2+1)*(patchRadius*2+1);
    return c;

}

void Disparity::initRandom(const Mat1b &img1, const Mat1b &img2, Mat1f &disp, Mat1f &scoreF, float minDisp, float maxDisp)
{
    static std::mt19937 gen(std::chrono::system_clock::to_time_t(std::chrono::system_clock::now())*omp_get_thread_num());
    std::uniform_real_distribution<float> dis{minDisp,maxDisp};


    // TODO 2.2
    // Random initialization of the disparity map.
    // - For each pixel compute "initIterations" random disparity values in the range [minDisp,maxDisp]
    // - Choose the best disparity according to the cost function and store it in disp.
    // - Store the best cost in scoreF
    for (int i =0; i<disp.rows;i++)
    {

        for( int j =0; j<disp.cols;j++)
        {

            scoreF(i,j) =10000000.f;
            //disp(i,j) =0.0;
            for (int it =0; it<initIterations;it++)
            {
                float d =dis(gen);
                float curcost=cost(img1, img2,j,i,d);
                if (curcost<scoreF(i,j))
                {
                     scoreF(i,j) =curcost;
                     disp(i,j) =d;
                }

            }

        }
    }


#ifdef DEBUG_VIEWS
    Mat vis = disp.clone() / (maxDisp+minDisp);
    imshow("initRandom", vis);
    cv::moveWindow("initRandom",20,60+img1.rows*2);
#endif

}

void Disparity::propagate(const Mat1b &img1, const Mat1b &img2, Mat1f &disp, Mat1f &scoreF, float minDisp, float maxDisp, bool forward)
{
    // TODO 2.2
    // Spatial propagation
    // - In the "forward" mode, iterate over the disparity image and sample the left and upper disparity values.
    // - Compute the cost and choose the disparity producing the lowest cost (don't forget to compare to the current cost of that pixel)
    // - In backward mode (forward=false) this propagation is done in opposite direction
    if (forward)
    {
        for(int i =1+patchRadius; i<=disp.rows-patchRadius; i++)
        {
            for( int j =1+patchRadius; j <=disp.cols-patchRadius; j++)
            {
                float curcost =cost(img1, img2,j,i,disp(i,j));
                float leftcost =cost(img1, img2, j-1, i, disp(i,j-1));
                float uppercost =cost(img1, img2, j, i-1, disp(i-1,j));
                scoreF(i,j)= (curcost>leftcost?(leftcost>uppercost? uppercost:leftcost):(curcost>uppercost?uppercost:curcost));
                if (scoreF(i,j) == curcost)
                    disp(i,j) =disp(i,j);
                else if (scoreF(i,j)== leftcost) {
                    disp(i,j)=disp(i,j-1);
                }
                else
                    disp(i,j)=disp(i-1,j);
            }
        }

    }
    else
    {
        for(int i =1+patchRadius; i<disp.rows-1-patchRadius; i++)
        {
            for( int j =1+patchRadius; j <disp.cols-1-patchRadius; j++)
            {
                float curcost =cost(img1, img2,j,i,disp(i,j));
                float rightcost =cost(img1, img2, j+1, i, disp(i,j+1));
                float lowercost =cost(img1, img2, j, i, disp(i+1,j));
                scoreF(i,j)= (curcost>rightcost?(rightcost>lowercost? lowercost:rightcost):(curcost>lowercost?lowercost:curcost));
                if (scoreF(i,j) == curcost)
                    disp(i,j) =disp(i,j);
                else if (scoreF(i,j)== rightcost) {
                    disp(i,j)=disp(i,j+1);
                }
                else
                    disp(i,j)=disp(i+1,j);
            }
        }

    }





#ifdef DEBUG_VIEWS
    Mat vis = disp.clone() / (maxDisp+minDisp);
    imshow("propagate", vis);
    cv::moveWindow("propagate",20+img1.cols,60+img1.rows*2);
#endif
}

void Disparity::randomSearch(const Mat1b &img1, const Mat1b &img2, Mat1f &disp, Mat1f &scoreF, float minDisp, float maxDisp)
{
    static std::mt19937 gen(std::chrono::system_clock::to_time_t(std::chrono::system_clock::now())*omp_get_thread_num());
    std::uniform_real_distribution<float> uniformDis{-1.0f,1.0f};
    float R0 = (maxDisp - minDisp) * 0.25;

    // TODO 2.2
    // Local search
    // - for each pixel sample "optimizeIterations" new disparity values at decrasing radius
    // - Choose the best disparity according to the cost (don't forget to include the current value in the comparison)
    for(int i =0;i<disp.rows; i++)
    {
        for(int j =0; j <disp.cols;j++)
        {
            scoreF(i,j) =cost(img1,img2,j,i,disp(i,j));
            for (int it =0; it <optimizeIterations;it++)
            {
                float a =uniformDis(gen);
                float offset =a*R0/pow(2.0,it);
                float disparity=disp(i,j)+offset;
                float curcost =cost(img1,img2, j,i,disparity);
                if(curcost<scoreF(i,j))
                {
                    scoreF(i,j)=curcost;
                    disp(i,j)=disparity;
                }
            }
        }
    }

}



