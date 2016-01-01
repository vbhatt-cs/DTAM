#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include <fstream>
#include <string>
#include <sstream>
#include <algorithm>
#include <time.h>

using namespace cv;
using namespace std;

const int neighbourhood=3;
const int COST_THRESHOLD=10000;

int main(int argc, char** argv)
{
    //Timing
    clock_t begin, end;
    double time_spent;

    begin = clock();

    //Input
    if(argc != 4)
    {
	    cout <<" Usage: imread LeftImage RightImage RequiredDepthImage" << endl;
	    return -1;
    }

    Mat imageL,imageR;
    imageL = imread(argv[1],0);   // Read the file
    imageR = imread(argv[2],0);

    if(!imageL.data || !imageR.data)    // Check for invalid input
    {
        cout <<  "Could not open or find the image" << endl ;
        return -1;
    }
    //Input end
    
    //calib.txt values. Will be replaced by reading pose and projecting the corners of voxel cube for max depth.
    int ndisp=190;  //pixels
    int width=722;
    int height=480;
    //calib end
    
    //Cost computation
    Mat dmin(height,width,CV_8UC1);
    Mat costMin(height,width,CV_64FC1);
    Mat costM(height,width,CV_64FC1);

    const int COST_MAX=256*256*(2*neighbourhood+1)*(2*neighbourhood+1)*3;

    for(int i=0;i<height;i++)
        for(int j=0;j<width;j++)
            costMin.at<double>(i,j)=COST_MAX;

    /*for(int d=1;d<ndisp;d++)
    {
        for(int i=0;i<height;i++)
        {
            for(int j=0;j<width;j++)
            {
                int startX=max(0,j-neighbourhood);
                int startY=max(0,i-neighbourhood);
                int endX=min(j+neighbourhood,width-1);
                int endY=min(i+neighbourhood,height-1);
                double cost=0;
                for(int y=startY;y<=endY;y++)
                {
                    for(int x=startX;x<=endX;x++)
                    {
                        Mat right=(Mat_<double>(3,1)<<x,y,1);
                        Mat worldR;
                        worldR=(camR.inv()*right)*d;
                        worldR.push_back(1.0);
                        Mat worldL=Trel*worldR;
                        worldL.pop_back();
                        Mat left=(camL*worldL)/d;
                        
                        double term=imageR.at<uchar>(x,y)-imageL.at<uchar>(left.at<int>(0),left.at<int>(1));
                        cost+=term*term;
                    }
                }

                if(cost<costMin.at<double>(i,j))
                {
                    costMin.at<double>(i,j)=cost;
                    dmin.at<uchar>(i,j)=d;
                }
            }
        }
    }*/

    for(int d=0;d<ndisp;d++)
    {
        cout<<d<<endl;
        for(int i=0;i<height;i++)
            for(int j=0;j<(width-d);j++)
            {
               double term=imageR.at<uchar>(i,j)-imageL.at<uchar>(i,j+d);
               costM.at<double>(i,j)=term*term;
            }


        for(int i=neighbourhood;i<height-neighbourhood-d;i++)
        {
            for(int j=neighbourhood;j<width-neighbourhood-d;j++)
            {
                int startX=max(0,j-neighbourhood);
                int startY=max(0,i-neighbourhood);
                int endX=min(j+neighbourhood,width-1);
                int endY=min(i+neighbourhood,height-1);
                
                double cost=0;
                for(int y=startY;y<=endY;y++)
                    for(int x=startX;x<=endX;x++)
                        cost+=costM.at<double>(y,x);

                if(cost<costMin.at<double>(i,j))
                {
                    costMin.at<double>(i,j)=cost;
                    dmin.at<uchar>(i,j)=d;
                }
            }
        }
    }

    /*for(int i=0;i<height;i++)
        for(int j=0;j<width;j++)
            if(costMin.at<double>(i,j)>COST_THRESHOLD)
                dmin.at<uchar>(i,j)=0;*/

    dmin=dmin*255/ndisp;
    
    end = clock();
    time_spent = (double)(end - begin) / CLOCKS_PER_SEC;
    cout<<time_spent<<endl;

    imwrite(argv[3],dmin);

    namedWindow("WindowL", WINDOW_NORMAL ); // Create a window for display.
    namedWindow("WindowR", WINDOW_NORMAL);
    namedWindow("Depth",WINDOW_NORMAL);

    imshow("WindowL", imageL);  // Show our image inside it.
    imshow("WindowR", imageR);
    imshow("Depth",dmin);

    waitKey(0); // Wait for a keystroke in the window
    return 0;
}
