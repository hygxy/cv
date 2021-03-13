#include <stdio.h>
#include <iostream>
#include <opencv2/opencv.hpp>
#include "opencv2/imgproc.hpp"
#include "opencv2/core/types.hpp"

using namespace std;
using namespace cv;

int windowStartX = 10;
int windowStartY = 50;

int windowXoffset = 5;
int windowYoffset = 40;


void show(string name, Mat img, int x, int y)
{
    int w = img.cols + windowXoffset;
    int h = img.rows + windowYoffset;

    cv::namedWindow(name);
    cv::moveWindow(name,windowStartX + w * x,windowStartY + h * y);
    cv::imshow(name, img);
}

cv::Mat harrisResponseImage(cv::Mat img)
{

    show("Input",img,0,0);


    // TODO 1.1
    // Compute the spatial derivatives in x and y
    // direction with the sobel filter: [-1,0,1]
    Mat dIdx(img.rows,img.cols,img.type(),Scalar(0));
    Mat dIdy(img.rows,img.cols,img.type(),Scalar(0));

    //Sobel(img, dIdx,-1, 1,0,3,1,0,BORDER_DEFAULT);
	//Sobel(img, dIdy,-1, 0,1,3,1,0,BORDER_DEFAULT);
	Mat xKernel =(Mat_<float>(1,3) <<-1,0,1);
	Mat yKernel =xKernel.t();
	filter2D(img, dIdx,-1,xKernel);
	filter2D(img, dIdy,-1,yKernel);



    show("dI/dx",abs(dIdx),0,1);
    show("dI/dy",abs(dIdy),1,1);


    // TODO 1.2
    // Compute Ixx, Iyy, and Ixy with
    // Ixx = (dI/dx) * (dI/dx),
    // Iyy = (dI/dy) * (dI/dy),
    // Ixy = (dI/dx) * (dI/dy).
    // Note: The multiplication between the images is element-wise (not a matrix multiplication)!!
    Mat Ixx(img.rows,img.cols,img.type(),Scalar(0));
    Mat Iyy(img.rows,img.cols,img.type(),Scalar(0));
    Mat Ixy(img.rows,img.cols,img.type(),Scalar(0));

	Ixx =dIdx.mul(dIdx);
	Iyy =dIdy.mul(dIdy);
	Ixy =dIdx.mul(dIdy); //https://stackoverflow.com/questions/10936099/matrix-multiplication-in-opencv



    // TODO 1.3
    // Compute the images A,B, and C by blurring the
    // images Ixx, Iyy, and Ixy with a
    // Gaussian filter of size 3x3 and standard deviation of 1.
    Mat A(img.rows,img.cols,img.type(),Scalar(0));
    Mat B(img.rows,img.cols,img.type(),Scalar(0));
    Mat C(img.rows,img.cols,img.type(),Scalar(0));
    Size kernelSize = Size(3,3);
    float sdev = 1;

        GaussianBlur(Ixx, A, kernelSize,sdev,BORDER_DEFAULT);
        GaussianBlur(Iyy, B, kernelSize,sdev,BORDER_DEFAULT);
        GaussianBlur(Ixy, C, kernelSize,sdev,BORDER_DEFAULT);




    show("A",abs(A)*5,0,2);
    show("B",abs(B)*5,1,2);
    show("C",abs(C)*5,2,2);

    // TODO 1.4
    // Compute the harris response with the following formula:
    // R = Det - k * Trace*Trace
    // Det = A * B - C * C
    // Trace = A + B
    Mat response(img.rows,img.cols,img.type(),Scalar(0));
    float k = 0.06;
	for(int i =0; i<img.size().height;i++)
	{
		for(int j =0; j<img.size().width;j++)
		{
			float det =A.at<float>(i,j)*B.at<float>(i,j)-C.at<float>(i,j)*C.at<float>(i,j);
			float tra =A.at<float>(i,j) +B.at<float>(i,j);
			response.at<float>(i,j)=det -k*tra*tra;
                        // we we this form because not all edges are horizontal or vertical
		}
	}


    // Normalize
    double min, max;
    cv::minMaxLoc(response, &min, &max);
    response *= 1.0f / max;
    Mat dbg;
    normalize( response, dbg, 0, 1, NORM_MINMAX, CV_32FC1 );
    show("Harris Response",dbg,2,1);


    return response;
}

std::vector<KeyPoint> harrisKeypoints(Mat response, float threshold = 0.1)
{
    // TODO 2.1
    // Generate a keypoint for a pixel,
    // if the response is larger than the threshold
    // and it is a local maximum.
    //
    // Don't generate keypoints at the image border.
    // images from real world will crash my codes, because pixels at the image border are not considered
    std::vector<KeyPoint> points;
	for(int i =1; i<response.size().height-1;i++)
	{
		for (int j =1; j<response.size().width-1;j++)
		{
                        if((response.at<float>(i,j)>threshold) &&(response.at<float>(i,j)>response.at<float>(i-1,j-1))
                                &&(response.at<float>(i,j)>response.at<float>(i-1,j))&&(response.at<float>(i,j)>response.at<float>(i-1,j+1))&&
                                (response.at<float>(i,j)>response.at<float>(i,j-1))&&(response.at<float>(i,j)>response.at<float>(i,j+1))&&(response.at<float>(i,j)
                                                                                                                                           >response.at<float>(i+1,j-1))&&(response.at<float>
                                                                                                                                                                           (i,j)>response.at<float>(i+1,j))&&
                                (response.at<float>(i,j)>response.at<float>(i+1,j+1)))
			{	
                                KeyPoint kp(j,i,1);
			    points.push_back(kp);
			}
		}
	}

    return points;
}

cv::Mat harrisEdges(Mat input, Mat response, float edgeThreshold = -0.01)
{
    // TODO 2.2
    // Set edge pixels to red.
    //
    // A pixel belongs to an edge, if the response is smaller than a threshold
    // and it is a minimum in x or y direction.
    //
    // Don't generate edges at the image border.

    Mat res = input.clone();
    for(int i =1; i<response.size().height-1;i++)
    {
        for(int j=1; j<response.size().width-1;j++)
        {
            if(response.at<float>(i,j)<edgeThreshold)
                /*
                if (((response.at<float>(i,j)< response.at<float>(i-1,j)) &&
                     (response.at<float>(i,j)< response.at<float>(i+1,j))) ||
                    ((response.at<float>(i,j)< response.at<float>(i,j-1)) &&
                     (response.at<float>(i,j)< response.at<float>(i,j+1))))
                */
                if ((response.at<float>(i,j)< response.at<float>(i-1,j) &&
                     response.at<float>(i,j)< response.at<float>(i+1,j)) ||
                    (response.at<float>(i,j)< response.at<float>(i,j-1) &&
                     response.at<float>(i,j)< response.at<float>(i,j+1)))
                     {
                       res.at<Vec3b>(i,j) =Vec3b(255,0,0);
                     }
        }
    }

    return res;
}

int main(int argc, char** argv )
{
    Mat input_img = imread("blox.jpg");
    //convert to floating point grayscale in the range [0,1]
    Mat input_gray;
    cvtColor( input_img, input_gray, CV_BGR2GRAY );
    Mat img;
    input_gray.convertTo(img,CV_32F);
    img *= (1.0f/255.0f);

    //compute corners and edges
    Mat response = harrisResponseImage(img);
    std::vector<cv::KeyPoint> keyPoints = harrisKeypoints(response);
    Mat edges = harrisEdges(input_img,response);


    show("Harris Edges",edges,1,0);
    Mat imgKeypoints1;
    drawKeypoints( input_img, keyPoints, imgKeypoints1, Scalar(0,255,0), DrawMatchesFlags::DEFAULT );
    show("Harris Keypoints",imgKeypoints1,2,0);


    cv::waitKey();
    return 0;
}
