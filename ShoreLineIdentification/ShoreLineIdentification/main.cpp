//
//  main.cpp
//  ShoreLineIdentification
//
//  Created by Student on 4/8/16.
//  Copyright © 2016 Ting-Che Lin. All rights reserved.
//

#include <iostream>
#include <vector>
#include "opencv2/video/tracking.hpp"
#include <opencv2/imgproc/imgproc.hpp>
#include "opencv2/videostab/videostab.hpp"
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <string.h>
#include <map>
#include <fstream>

using namespace cv;
using namespace::std;

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

int main(int argc, const char * argv[]) {
    // insert code here...
    std::cout << "Hello, World!\n";
    vector<Point2f> a;
    vector<Point2f> b;

    int i = 2;
    getPointValue(a,b,i);
    Mat frame, trackers;
    getFrame(frame,i);
    namedWindow("Frame");
    imshow("Frame", frame);
    getTrackerImage(trackers,i);
    namedWindow("Trackers");
    imshow("Trackers", trackers);
    waitKey(0);
    return 0;
}
