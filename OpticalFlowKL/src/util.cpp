#include <math.h>
#include "opticalFlowLK.h"

Vec4d computeBilinearWeights(Point2f q)
{
    // TODO 2.1
    // - Compute bilinear weights for point q
    // - Entry 0 weight for pixel (x, y)
    // - Entry 1 weight for pixel (x + 1, y)
    // - Entry 2 weight for pixel (x, y + 1)
    // - Entry 3 weight for pixel (x + 1, y + 1)
    Vec4d weights;
    float a =q.x -floor(q.x);
    float b =q.y-floor(q.y);
    weights(0) = (1-a)*(1-b);
    weights(1) = a*(1-b);
    weights(2) = (1-a)*b;
    weights(3) = a*b;
    return weights;
}

Mat1d computeGaussianWeights(Size winSize, float sigma)
{
	// TODO 2.2
    // - Fill matrix with gaussian weights
    // - Note, the center is ((winSize.width - 1) / 2,winSize.height - 1) / 2)
    // - Don't use pow() to calculate the square, will lose precision
        Mat1d weights = Mat1d::ones(winSize);
        for (int x =0; x<winSize.width;x++)
        {
            for(int y =0; y<winSize.height;y++)
            {
             weights.at<double>(x,y) =exp(-(((double)(x-(winSize.width-1)/2)/winSize.width)*((double)(x-(winSize.width-1)/2)/winSize.width)+((double)(y-(winSize.width-1)/2)/winSize.height)*((double)(y-(winSize.width-1)/2)/winSize.height))/(2*sigma*sigma));
            }
        }

	return weights;
}

Matx22d invertMatrix2x2(const Matx22d& A)
{
	// TODO 2.3
    // - Compute the inverse of the 2x2 Matrix A

    double determinant =A(0,0)*A(1,1)-A(0,1)*A(1,0);
     Matx22d invA(A(1,1)/determinant, -A(0,1)/determinant, -A(1,0)/determinant, A(0,0)/determinant);
    return invA;
}
