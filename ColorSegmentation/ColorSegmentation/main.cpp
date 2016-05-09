#include "opencv2/highgui/highgui.hpp"
#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/video/tracking.hpp"
#include <opencv2/imgproc/imgproc.hpp>
#include "opencv2/videostab/videostab.hpp"

#include <vector>
#include <iostream>
#include <string.h>
#include <map>
#include <fstream>

using namespace cv;
using namespace std;

void readPoint (vector<Point2f>& points1, vector<Point2f>& point0, const char* filename, const int& number){
    char tempFilename[100];
    char *token;
    string line;
    strcpy(tempFilename,filename);
    sprintf(tempFilename, strcat(tempFilename, "%03d.txt"), number);
    ifstream textFile;
    textFile.open(tempFilename);
    if (textFile.is_open()){
        getline(textFile, line);
        while (getline(textFile, line)){
            Point2f tempPoint;
            int tempInt;
            char tokenArray[line.size()+1];//as 1 char space for null is also required
            strcpy(tokenArray, line.c_str());
            token = strtok(tokenArray, " ,");
            string strToken(token);
            tempPoint.x = stof(strToken);
            token = strtok(NULL, " ,");
            strToken = string(token);
            tempPoint.y = stof(strToken);
            token = strtok(NULL, " ,");
            strToken = string(token);
            tempInt = stoi(strToken);
            if (tempInt){
                points1.push_back(tempPoint);
            }
            else{
                point0.push_back(tempPoint);
            }
        }
    }
}

void readImage (Mat& image, const char* filename, const int& number){
    char tempFilename[100];
    strcpy(tempFilename,filename);
    sprintf(tempFilename, strcat(tempFilename, "%03d.jpg"), number);
    image = imread(tempFilename);
}

void getFrame(Mat& frame, int i){
    char frameImage[100];
    strcpy(frameImage, "/Users/student/Desktop/OpenCV/testForShoreline/frame");
    readImage(frame, frameImage, i);
}

void getTrackerImage (Mat& trackerImage, int i){
    char trackerImageFile[100];
    strcpy(trackerImageFile, "/Users/student/Desktop/OpenCV/testForShoreline/tracker");
    readImage(trackerImage, trackerImageFile, i);
}

void getPointValue (vector<Point2f>& pointWithValue1, vector<Point2f>& pointWithValue0, int i){
    char textCSVFile[100];
    strcpy(textCSVFile, "/Users/student/Desktop/OpenCV/testForShoreline/currentPointAndValues");
    readPoint(pointWithValue1, pointWithValue0, textCSVFile, i);
}

static void help(char** argv)
{
    cout << "\nDemonstrate mean-shift based color segmentation in spatial pyramid.\n"
    << "Call:\n   " << argv[0] << " image\n"
    << "This program allows you to set the spatial and color radius\n"
    << "of the mean shift window as well as the number of pyramid reduction levels explored\n"
    << endl;
}

//This colors the segmentations
static void floodFillPostprocess( Mat& img, vector<Point2f>& pointValue1, vector<Point2f>& pointValue0 , const Scalar& colorDiff=Scalar::all(1))
{
    CV_Assert( !img.empty() );
    RNG rng = theRNG();
    Mat mask( img.rows+2, img.cols+2, CV_8UC1, Scalar::all(0) );
//    Scalar ocean, land;
//    ocean = Scalar(0,0,255);
//    land = Scalar(0,255,0);
//    for( int y = 0; y < img.rows; y++ )
//    {
//        for( int x = 0; x < img.cols; x++ )
//        {
//            if( mask.at<uchar>(y+1, x+1) == 0 )
//            {
//                Scalar newVal( rng(256), rng(256), rng(256) );
//                floodFill( img, mask, Point(x,y), newVal, 0, colorDiff, colorDiff );
//            }
//        }
//    }
    Scalar ocean, land;
    ocean = Scalar(0,0,255);
    land = Scalar(0,255,0);
    for (int i = 0; i < pointValue0.size(); i++) {
        if (pointValue0[i].x > img.cols || pointValue0[i].y > img.rows){
            continue;
        }
        floodFill( img, mask, pointValue0[i], ocean, 0, colorDiff, colorDiff );
    }
    for (int i = 0; i < pointValue1.size(); i++) {
        if (pointValue1[i].x > img.cols || pointValue1[i].y > img.rows){
            continue;
        }
        floodFill( img, mask, pointValue1[i], land, 0, colorDiff, colorDiff );
    }

}



static void meanShiftSegmentation(Mat& img, Mat& res, int& spatialRad, int& colorRad, int&maxPyrLevel, string& winName, vector<Point2f>& pointValue1, vector<Point2f>& pointValue0)
{
    cout << "spatialRad=" << spatialRad << "; "
    << "colorRad=" << colorRad << "; "
    << "maxPyrLevel=" << maxPyrLevel << endl;
    pyrMeanShiftFiltering( img, res, spatialRad, colorRad, maxPyrLevel );
    
    imshow("res", res);
    floodFillPostprocess( res, pointValue1, pointValue0 , Scalar::all(2));
    imshow( winName, res );
}

int main(int argc, char** argv)
{
    string winName = "meanshift";
    int spatialRad, colorRad, maxPyrLevel;
    Mat img, res;
    int i = 0;
    vector<Point2f> pointWithValue1, pointWithValue0;
    getPointValue(pointWithValue1, pointWithValue0, i);
    
    
    getFrame(img, i);
    if( img.empty() )
        return -1;
    
    spatialRad = 5;
    colorRad = 20;
    maxPyrLevel = 2;
    
    namedWindow( winName, WINDOW_AUTOSIZE );
    namedWindow("res", WINDOW_AUTOSIZE);
    
//    createTrackbar( "spatialRad", winName, &spatialRad, 80, meanShiftSegmentation );
//    createTrackbar( "colorRad", winName, &colorRad, 60, meanShiftSegmentation );
//    createTrackbar( "maxPyrLevel", winName, &maxPyrLevel, 5, meanShiftSegmentation );
    
    meanShiftSegmentation(img,res,spatialRad,colorRad,maxPyrLevel,winName, pointWithValue1, pointWithValue0);
    waitKey();
    return 0;
}
