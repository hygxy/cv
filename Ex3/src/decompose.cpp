#include "sfm.h"
#include <math.h>

void decompose(const Matx33d& E, Matx33d& R1, Matx33d& R2, Matx31d& t1, Matx31d& t2)
{
    // TODO 4.1
    // Decompose the essential matrix E into R1,R2,t1,t2.
    // Note: R1,R2,t1 and t2 are output variables.
    cv::SVD svd(E,SVD::FULL_UV);
    Matx33d V = Matx33d(svd.vt);//.t();  already transposed by opencv
    Matx33d U =svd.u;
    Matx33d W(0,-1,0,1,0,0,0,0,1);
    R1 =U*W*V;
    R2 =U*W.t()*V;

    double norm =sqrt(U(0,2)*U(0,2) +U(1,2)*U(1,2) +U(2,2)*U(2,2));
    Matx31d s(U(0,2)/norm ,U(1,2)/norm, U(2,2)/norm);
    t1 =s;
    t2 =-s;

}



Matx34d relativeTransformation(Matx33d E, std::vector<cv::Point2f> points1, std::vector<cv::Point2f> points2, Matx33d K)
{
    Matx33d R1, R2;
    Matx31d t1, t2;

    decompose(E,R1,R2,t1,t2);

    // Note: There are actually 4 possible R's, but two of them contain a reflection
    // and therefore are not "real" rotation matrices.
    if(cv::determinant(R1) < 0){
        //scaling the essential matrix by -1 is allowed
        E = -E;
        decompose(E,R1,R2,t1,t2);
    }


    int bestCount = 0;


    Matx34d V;

    for(int dR = 0; dR < 2; ++dR)
    {
        Matx33d cR = dR == 0 ? R1 : R2;

        //        Matx33d cR = float(dR*2-1) * R;
        for(int dt = 0; dt < 2; ++dt)
        {
            Matx31d ct = dt == 0 ? t1 : t2;


            Matx34d View1 = Matx34d::eye();
            Matx34d View2;
            for(int i = 0; i < 3; ++i)
                for(int j = 0; j < 3; ++j)
                    View2(i,j) = cR(i,j);
            for(int i = 0; i < 3; ++i)
                View2(i,3) = ct(i,0);

            int count = triangulate(View1,View2,K,points1,points2).size();

            if(count > bestCount)
            {
                V = View2;
                count = bestCount;
            }
        }
        cR = cR.t();
    }

    return V;


}



