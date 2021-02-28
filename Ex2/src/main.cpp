#include "ImageData.h"
#include "homography.h"
#include "matching.h"

Mat createStichedImage(vector<ImageData>& ids);
void testHomography();

int main( int argc, char** argv )
{

    testHomography();


    // =========== Loading ===========

    vector<string> images = {
        "data/7.jpg",
        "data/8.jpg",
        "data/9.jpg",
        "data/10.jpg",
        "data/11.jpg",
        "data/12.jpg",
        "data/13.jpg",
        "data/14.jpg",
        "data/15.jpg",
    };

    vector<ImageData> ids;
    int i = 0;
    for(auto str : images)
    {
        ImageData id;
        id.file = str;
        id.img = imread(str);
        // Resize because monitors in Huber-CIP are so small :(
        resize(id.img,id.img,Size(),0.7,0.7,INTER_AREA);
        id.id = i++;
        assert(id.img.data);
        ids.push_back(id);
        cout << "Loaded Image " << id.id << " " << id.file << " " << id.img.cols << "x" << id.img.rows << endl;
    }

    // =========== Feature Detection ===========

    for(ImageData& id : ids)
    {
        id.computeFeatures();
    }

    // =========== Pairwise Feature Matching ===========

    for(int i = 1; i < (int)ids.size(); ++i)
    {
        vector<DMatch> matches = computeMatches(ids[i-1],ids[i]);

        {
            // Debug output
            Mat matchImg = createMatchImage(ids[i-1],ids[i],matches);
            int h = 200;
            int w = (float(matchImg.cols)/matchImg.rows) * h;
            resize(matchImg,matchImg,Size(w,h));
            auto name = "Matches (" + std::to_string(i-1) + "," + std::to_string(i) +") " + ids[i-1].file + " - " + ids[i].file;
            cv::namedWindow(name);
            cv::moveWindow(name,10+ w * ((i-1)%2),10 + (h+30) * ((i-1)/2));
            imshow(name , matchImg );

            //        imwrite("match.png",matchImg);
        }

        auto H = computeHomographyRansac(ids[i-1],ids[i],matches,1000,2.0);
        ids[i].HtoPrev = H.inv();
        ids[i-1].HtoNext = H;
    }

    // =========== Stiching ===========

    Mat simg = createStichedImage(ids);
    imwrite("output.png",simg);


    return 0;
}


