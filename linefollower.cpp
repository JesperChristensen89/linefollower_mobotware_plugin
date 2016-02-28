#include "linefollower.h"
#include "control.h"
//#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <iostream>
#include <stdio.h>

#define PI 3.14159265

using namespace cv;
using namespace std;

// for error detection
int k = 0;
int endOfLine = 0;
int imgNumber = 0; // #image has been processed during this run
vector<vector<Point> > contours;
vector<vector<Point> > contour_prev;
int imgError = 0;
bool err = false;

Control control;

void LineFollower::errorMatch(int im, int i)
{
   
    if (im!=0)
    {
        // match prev contour with current
        double match = matchShapes(contours[i],contour_prev[0],1,0.0);

        if (match > 10)
        {
            endOfLine = endOfLine + 1;
            
            err = true;

            printf("** Match error **");

            /*
            char errStr[50];
            sprintf(errStr,"img/imgERR%d.jpg",im);
            imwrite(errStr,img);
            */

            // if not coherent - reset
            if (im - imgError >= 4)
            {
                endOfLine = 0;
            }
            
            imgError = im;
        }
    }
    else
    {
        contour_prev.push_back(contours[i]);
    }
    
}

void LineFollower::errorContour(int im)
{
  
    if (k == 0)
        {
            endOfLine = endOfLine + 1;

            err = true;
            
            printf("** Contour error **");
            
            
             
            if (im - imgError >= 4)
            {
                endOfLine = 0;
            }
            imgError = im;
        }
        
}

void LineFollower::start(Mat imgCV, bool left, bool white, int errThresh)
{  
    
    // create roi
    
    Rect roi = Rect(10,imgCV.rows-60, imgCV.cols-20, imgCV.rows/12);
    Mat img_roi = imgCV(roi);
    
    // convert to gray
    Mat img_gray;
    //cvtColor(img_roi,img_gray,COLOR_RGB2GRAY);

    // blur
    Mat img_blur;
    GaussianBlur(img_roi,img_blur, Size(9,9),2);

    // threshold
    Mat img_thresh;
    
    int threshType = white ? CV_THRESH_BINARY | CV_THRESH_OTSU : CV_THRESH_BINARY_INV | CV_THRESH_OTSU;
    
    threshold(img_blur,img_thresh,0,255, threshType);
      
    // Opening
    Mat kernel(5,5, CV_8UC1, 1);
    Mat img_open;
    
    morphologyEx(img_thresh,img_open,2, kernel);
    
    // find contours
    vector<Vec4i> hierarchy;
     
    findContours(img_open,contours,hierarchy,CV_RETR_LIST,CV_CHAIN_APPROX_SIMPLE);
       
    vector<Rect> r;
    vector<Point> leftEdge;
    
    Point midRect;
    
    k = 0;
    
    for (uint i = 0; i < contours.size(); i++) // iterate through each contour
    {
      double a = contourArea(contours[i],false);
      
      
      // use only contours with area above 3500
      if (a > 3500)
      {
	// get bounding rec of contour
	r.push_back(boundingRect(contours[i]));
	r[k].x = r[k].x + 10;
	r[k].y = r[k].y + imgCV.rows-60;
	
	rectangle(imgCV, r[k], Scalar(0,255,0));
	
	// get mid of rect
	midRect = Point(r[k].x+r[k].width/2, r[k].y+r[k].height/2);
	
	circle(imgCV, midRect, 5, Scalar(0,0,255),-1,1,8);
	
	// get the left edge of rect
	// used as offset as raspicam is not
	// mounted on mid of regbot
	leftEdge.push_back( Point(r[k].x,r[k].y+r[k].height/2));

	circle(imgCV,leftEdge[k],5,Scalar(255,0,0),-1,1,8);
	
	
	k = k+1;
	
	//drawContours(imgCV,contours, i,Scalar(0,255,255),1,8,hierarchy,0,Point(10,imgCV.rows-60));
	
	// used for false-positive checking
	errorMatch(imgNumber, i);
	
      }
    }
    
    // if k = 0 no contour was found -> error
    errorContour(imgNumber);
    
    // error routine
    if (endOfLine == errThresh)
    {
	cout << "\n### REACHED END OF LINE ### ( or made an error :-) )" << endl;
	endOfLine = 0;
	
	//TODO stop run!!
    }
    
    
    // get the edge point
    int leftmostEdge = imgCV.cols;
    for (uint i = 0; i < leftEdge.size(); i++)
    {
      int edge = leftEdge[i].x;
      
      if (edge < leftmostEdge)
      {
	leftmostEdge = edge;
      }
    }
    
    int rightmostEdge = 0;
    for (uint i = 0; i < leftEdge.size(); i++)
    {
      int edge = leftEdge[i].x;
	  
      if (edge > rightmostEdge)
      {
	rightmostEdge = edge;
      }
    }
    
    // regulator //
    int angle = 0;
    if (left)
    {
      angle = control.regulator(leftmostEdge,imgCV.cols/2);
    }
    else
    {
      angle = control.regulator(rightmostEdge,imgCV.cols/2);
    }
    
    
    
    
    // send calculated angle to regbot
    UART uart;
    char angleStr[10];
    snprintf (angleStr,10,"%d\n",angle);
    uart.send(angleStr);
    
    imgNumber += 1;
}