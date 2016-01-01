#include<opencv2/core/core.hpp>
#include<opencv2/highgui/highgui.hpp>
#include<opencv2/calib3d/calib3d.hpp>
#include<opencv2/imgproc/imgproc.hpp>
#include<iostream>

using namespace cv;
using namespace std;

const int BLOCK_SIZE=9;
const int ndisp=16*10;
const double doffs=28.797;
const double baseline=237.604;

double q[4][4]={{1,0,0,254.151},{0,1,0,240.842},{0,0,0,980.198},{0,0,1/baseline,doffs/baseline}};
Mat Q(4,4,CV_64F,q);

int main(int argc, char* argv[])
{
    if(argc!=3)
    {
        cout<<"Usage: cvDepth LeftImage RightImage";
        return -1;
    }

    Mat imgL=imread(argv[1],0);
    Mat imgR=imread(argv[2],0);

    Mat disp16=Mat(imgL.rows,imgL.cols,CV_16S);
    Mat disp8=Mat(imgL.rows,imgL.cols,CV_8UC1);

    Ptr<StereoBM> stereo=StereoBM::create(ndisp,BLOCK_SIZE);
    stereo->compute(imgL,imgR,disp16);

    double minD,maxD;
    minMaxLoc(disp16,&minD,&maxD);
    
    disp16.convertTo(disp8,CV_8UC1,255/(maxD-minD));

    Mat depth3D,depth8,depth3DN;
    reprojectImageTo3D(disp8,depth3D,Q);

    for(int i=300;i<305;i++)
    {
        for(int j=300;j<305;j++)
        {
            for(int k=0;k<3;k++)
                cout<<depth3D.at<Vec3f>(i,j)[k]<<",";
            cout<<" ";
        }
        cout<<endl;
    }

    namedWindow("Disp",WINDOW_NORMAL);
    imshow("Disp",disp8);

    waitKey(0);
    return 0;
}
