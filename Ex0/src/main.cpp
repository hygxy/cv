#include <iostream>
#include <opencv2/opencv.hpp>
#include <csignal>

using namespace std;
using namespace cv;

void workaround(int x)
{
    cout <<"this is just a work around" <<endl;
}

int main(int argc, char** argv )
{

//    cout <<getBuildInformation() <<endl;
    signal(SIGABRT, workaround);
    Mat src =imread(argv[1]);
    cin.ignore();
    if(src.empty())
    {
        cout <<"Please input the name of the image file as command line argument"<<endl;
    }

    // TODO 3.1
    // - Load Image
    // - Show it on screen until a key is pressed
/*    String file = "A.jpg";
    Mat image;
    image =imread(file,CV_LOAD_IMAGE_COLOR);
    imshow("Display Window",image);
    vector<int> params;
    params.push_back(9);
    imwrite("A1.jpg", image,params );
    waitKey(0);

/*
    // TODO 3.2
    // - Resize Image by a factor of 0.5
    // - Show it on screen
    // - save as small.jpg
    Mat smallImage;
    resize(image, smallImage,Size(),0.5,0.5,INTER_LINEAR);
    imshow("smallImage", smallImage);
    imwrite("small.png",smallImage);
    waitKey(0);



    // TODO 3.3
    // - Create 3 Mats of the same size and type as the image.
    // - Split the image into its RGB channels.
    // - Display the channels on screen.
    

    Mat imageR, imageG, imageB;
    imageR.create(image.size().height,image.size().width,image.type());
    imageG.create(image.size().height,image.size().width,image.type());
    imageB.create(image.size().height,image.size().width,image.type());
    for (int i =0;i<image.size().height;i++)
    {
	for (int j =0;j<image.size().width;j++)
		{
		Vec3b e =image.at<Vec3b>(i,j);
		e[0]=0;
		e[1]=0;
		imageR.at<Vec3b>(i,j)=e;
		}
     }

    for (int i =0;i<image.size().height;i++)
    {
	for (int j =0;j<image.size().width;j++)
		{
		Vec3b e =image.at<Vec3b>(i,j);
		e[1]=0;
		e[2]=0;
		imageB.at<Vec3b>(i,j)=e;
		}
     }

    for (int i =0;i<image.size().height;i++)
    {
	for (int j =0;j<image.size().width;j++)
		{
		Vec3b e =image.at<Vec3b>(i,j);
		e[0]=0;
		e[2]=0;
		imageG.at<Vec3b>(i,j)=e;
		}
     }

  imshow("imageR", imageR);
  imshow("imageG", imageG);
  imshow("imageB", imageB);

   waitKey(0);
*/
    return 0;
}

