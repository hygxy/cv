#include "matching.h"


std::vector< std::vector<DMatch> > matchknn2(Mat1b& descriptors1, Mat1b& descriptors2)
{  
    // Initialize N x 2 array of matches.
    std::vector< std::vector<DMatch> > knnmatches(descriptors1.rows,std::vector<DMatch>(2,DMatch(-1,-1,3453454)));
    
    // TODO 2.1
    // Find the two nearest neighbors for every descriptor in image 1.
    //
    // For a given descriptor i in image 1:
    // Store the best match (smallest distance) in knnmatches[i][0]
    // Store the second best match in knnmatches[i][1]
    //
    // Hint: Compute the distance between the descriptors yourself or use the templated function normL2Sqr<uchar,float>.

	for(int i =0; i<descriptors1.rows; i++)
		{
			std::vector<DMatch> matches;
			for (int j =0; j <descriptors2.rows; j++)
			{
				 double distance =norm(descriptors1.row(i), descriptors2.row(j),NORM_HAMMING);
				 DMatch dm(i, j,distance);
				 matches.push_back(dm);
			}

			knnmatches[i][0] =matches[0];
			knnmatches[i][1] =matches[1];


			for(int k =2; k<descriptors2.rows; k++)
			{
				if(matches[k].distance<knnmatches[i][0].distance)
				{
					knnmatches[i][1] =knnmatches[i][0];
					knnmatches[i][0]=matches[k];
				}
				else if (matches[k].distance<knnmatches[i][1].distance)
				{
				knnmatches[i][1] =matches[k];
				}
			}

		}
	
    return knnmatches;
}


vector<DMatch> ratioTest(std::vector< std::vector<DMatch> > knnmatches, float ratio_threshold)
{
    std::vector<DMatch> matches;

    // TODO 2.2
    // Compute the ratio between the nearest and second nearest neighbor.
    // Add the nearest neighbor to the output matches if the ratio is smaller than ratio_threshold.
	for (std::vector< std::vector<DMatch> >::iterator it =knnmatches.begin(); it!=knnmatches.end(); it++)
	{
		if(((*it)[0].distance/(*it)[1].distance)<ratio_threshold)
		{
			matches.push_back((*it)[0]);
		}
	}

    return matches;
}

vector<DMatch> computeMatches(ImageData &img1, ImageData &img2)
{
    auto knnmatches = matchknn2(img1.descriptors,img2.descriptors);
    auto matches = ratioTest(knnmatches,0.7);
    cout << "(" << img1.id << "," << img2.id << ") found " << matches.size() << " matches." << endl;
    return matches;
}
