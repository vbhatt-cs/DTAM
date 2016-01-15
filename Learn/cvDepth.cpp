#include<opencv2/core/core.hpp>
#include<opencv2/highgui/highgui.hpp>
#include<opencv2/calib3d/calib3d.hpp>
#include<opencv2/imgproc/imgproc.hpp>
#include<iostream>
#include<cmath>

using namespace cv;
using namespace std;

const int BLOCK_SIZE=5;
const int ndisp=16*5;
const double doffs=32.778;
const double baseline=193.001;
const double f=999.421;

void costFunction(double x,const Mat &z,const Mat &imL,const Mat &imR, const Mat &imGrX, double &cost,double &grad)
{
    cost=0;
    grad=0;
    
    for(int i=0;i<imL.rows;i++)
        for(int j=ndisp;j<imL.cols;j++)
        {
            int d=x*f/z.at<double>(i,j)-doffs;
            if(d<ndisp && d>=0)
            {
                double diff=imR.at<uchar>(i,j-d)-imL.at<uchar>(i,j);
                cost+=diff*diff;
                grad+=2*diff*imGrX.at<double>(i,j-d);
            }
        }
    cost/=1000000;
    grad/=1000000;
}

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
    Mat dmin=Mat(imgL.rows,imgL.cols,CV_64FC1);

    Ptr<StereoBM> stereo=StereoBM::create(ndisp,BLOCK_SIZE);
    stereo->compute(imgL,imgR,disp16);

    double minD,maxD;
    minMaxLoc(disp16,&minD,&maxD);

    disp16.convertTo(dmin,CV_64FC1);
    dmin=dmin/16;
    Mat z=f*baseline/(dmin+doffs);

    //Pose estimation
    double initialX=120;
    double x=initialX;
    double cost,grad;

    //Mat cost=Mat::zeros(300,1,CV_64FC1);
    //Mat grad=Mat::zeros(300,1,CV_64FC1);
    Mat imGrX;

    Scharr(imgR,imGrX,CV_64FC1,1,0);

    costFunction(initialX,z,imgL,imgR,imGrX,cost,grad);
    int numIter=300;
    Mat Jhist=Mat::zeros(numIter,1,CV_64FC1);
    Jhist.at<double>(0,1)=cost;
    x+=10*sqrt(cost)/grad;
    //cout<<cost<<" "<<grad<<" "<<x<<endl;

    costFunction(x,z,imgL,imgR,imGrX,cost,grad);
    for(int i=1;i<numIter;i++)
    {
        costFunction(x,z,imgL,imgR,imGrX,cost,grad);
        x+=10*sqrt(cost)/grad;
        Jhist.at<double>(i,1)=cost;
    }

    cout<<x;
    //cout<<Jhist;
    /*for(int i=0;i<300;i++)
        costFunction(i,z,imgL,imgR,imGrX,cost.at<double>(i,1),grad.at<double>(i,1));

    cout<<grad;*/
    
    disp16.convertTo(disp8,CV_8UC1,255/(maxD-minD));  

    namedWindow("Disp",WINDOW_NORMAL);
    imshow("Disp",disp8);

    waitKey(0);
    return 0;
}
