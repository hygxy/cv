#include "ImageData.h"


void computeHtoref(vector<ImageData>& ids, int center)
{
    for(int i = center - 1; i >= 0; --i)
    {
        ImageData& c = ids[i];
        ImageData& next = ids[i+1];
        c.HtoReference = next.HtoReference * c.HtoNext;
    }

    for(int i = center + 1; i < (int)ids.size(); ++i)
    {
        ImageData& c = ids[i];
        ImageData& prev = ids[i-1];
        c.HtoReference = prev.HtoReference * c.HtoPrev;
    }
}

Mat createStichedImage(vector<ImageData>& ids)
{
    cout << "Stiching with " << ids.size() << " images..." << endl;
    int center = ids.size() / 2;


    ImageData& ref = ids[center];
    computeHtoref(ids,center);

    cout << "Reference Image: " << center << " - " << ref.file << endl;

    float minx = 2353535;
    float maxx =-2353535;
    float miny = 2353535;
    float maxy = -2353545345;

    // Compute global bounding box by warping the image corners to the reference
    for(int i = 0; i < (int)ids.size(); ++i)
    {
        ImageData& img2 = ids[i];
        std::vector<Point2f> corners2(4);
        corners2[0] = Point2f(0,0);
        corners2[1] = Point2f( img2.img.cols, 0 );
        corners2[2] = Point2f( img2.img.cols, img2.img.rows );
        corners2[3] = Point2f( 0, img2.img.rows );
        std::vector<Point2f> corners2_in_1(4);
        perspectiveTransform( corners2, corners2_in_1, img2.HtoReference);
        for(auto p : corners2_in_1)
        {
            minx = min(minx,p.x);
            maxx = max(maxx,p.x);
            miny = min(miny,p.y);
            maxy = max(maxy,p.y);
        }
    }
    Rect roi(floor(minx),floor(miny), ceil(maxx) - floor(minx), ceil(maxy) - floor(miny));
    cout << "ROI " << roi << endl;

    // Translate everything so the top left corner is at (0,0)
    // Note: This can be simply done by adding the negavite offset to the homopgrahy
    int offsetX = floor(minx);
    int offsetY = floor(miny);
    ref.HtoReference(0,2) = -offsetX;
    ref.HtoReference(1,2) = -offsetY;
    computeHtoref(ids,center);

    cv::namedWindow("Panorama");
    cv::moveWindow("Panorama",0,500);


    // Init big output image
    Mat stichedImage(roi.height,roi.width,ref.img.type(),Scalar(0));
    for(int k = 0; k < (int)ids.size()+1; ++k)
    {
        // Instead of adding the images left to right, start at the center
        // and go outwards
        int i = center + (k%2==0?1:-1) * ( (k+1)/2);
        if(i<0 || i >= (int)ids.size())
            continue;

        // Project the image onto the reference image plane
        ImageData& img2 = ids[i];
        Mat tmp(roi.height,roi.width,ref.img.type(),Scalar(0));
        warpPerspective(img2.img,tmp,img2.HtoReference,cv::Size(tmp.cols,tmp.rows),INTER_LINEAR,BORDER_TRANSPARENT);

        // Added it to the output image
        for(int y = 0; y < stichedImage.rows; ++y)
        {
            for(int x = 0; x < stichedImage.cols; ++x)
            {
                if(x < stichedImage.cols && y <stichedImage.rows &&
                        stichedImage.at<Vec3b>(y,x) == Vec3b(0,0,0))
                {
                    stichedImage.at<Vec3b>(y,x) = tmp.at<Vec3b>(y,x);
                }
            }
        }

        cout << "Added image " << i << " - " << img2.file << "." << endl;
        imshow("Panorama",stichedImage);
        waitKey(0);
    }

    return stichedImage;
}

