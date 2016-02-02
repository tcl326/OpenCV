//
//  main.cpp
//  DissimilarityBetweenTrackers
//
//  Created by Student on 1/29/16.
//  Copyright © 2016 Student. All rights reserved.
//


#include "opencv2/video/tracking.hpp"
// Video write
#include <opencv2/core/core.hpp>
#include <ctype.h>
#include <cstdlib>
#include <list>
#include <iterator>
#include <math.h>
#include <typeinfo>


#include <string>
#include <iostream>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include "opencv2/core/core.hpp"
#include "opencv2/video/video.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/videostab/videostab.hpp"
#include <vector>
#include <stdarg.h>
#include "JenksNaturalBreak.hpp"

using namespace std;
using namespace cv;

double euclideanDistance (Point2f point1, Point2f point2){
    double distance;
    Point2f p;
    p = point1 - point2;
    distance = sqrt( p.x*p.x + p.y*p.y );
    return distance;
}

double euclideanMetric (std::vector<cv::Point2f> givenTracker, std::vector<cv::Point2f> neighbourTracker)
{
    size_t gT , nT, allSize;
    gT = givenTracker.size();
    nT = neighbourTracker.size();
    allSize = gT;
    double sumEuclideanDistance = 0;
    if (gT != nT)
    {
        if (gT > nT) {
            givenTracker.resize(nT);
            allSize = nT;
        }
        if (nT > gT) {
            neighbourTracker.resize(gT);
        }
    }
    for (int i = 0; i<allSize; i++){
        sumEuclideanDistance += euclideanDistance(givenTracker[i], neighbourTracker[i]);
    }
    
    return sumEuclideanDistance;
}

vector<int> findNeighbourIndexIterative (int givenTrackerIndex,std::vector< std::vector<cv::Point2f> > allTrackers){
    double radius;
    radius = 5.0;
    vector<int> neighbourIndex;
    size_t n = allTrackers.size();
    while (neighbourIndex.size() < 4) {
        for (int i = 0; i < n; i++){
            if (radius > euclideanDistance(allTrackers[givenTrackerIndex].back(), allTrackers[i].back())){
                neighbourIndex.push_back(i);
            }
        }
        radius++;
    }
    return neighbourIndex;
}

void updateNeighbour (vector< vector<int> >& neighbourIndexList, int removedPointIndex){
    bool remove = false;
    for (int i = 0; i < neighbourIndexList.size(); i++) {

        for (int j = 0; j < neighbourIndexList[i].size(); j++) {
            if (neighbourIndexList[i][j] > removedPointIndex) {
                neighbourIndexList[i][j]--;
            }
            if (neighbourIndexList[i][j] == removedPointIndex) {
                remove = true;
            }
        }
        if (remove) {
            neighbourIndexList[i].erase(std::remove(neighbourIndexList[i].begin(), neighbourIndexList[i].end(), 8), neighbourIndexList[i].end());
            
        }
    }
}

void dStarIterativeUpdate (int givenTrackerIndex, vector<int>& neighbourIndex, std::vector< std::vector<cv::Point2f> >& allTrackers, vector<double>& dStarList){
    double euclideanDis;
    size_t n = neighbourIndex.size();
    for (int i=0; i<n; i++){
        euclideanDis += euclideanDistance(allTrackers[givenTrackerIndex].back(), allTrackers[neighbourIndex[i]].back());
    }
    dStarList[givenTrackerIndex] += euclideanDis/n;
}


void mininimumDissimilarityIterative (int givenTrackerIndex , vector<int>& neighbourIndex, std::vector< std::vector<cv::Point2f> >& allTrackers, vector<double>& dStarList, vector<double>& minDissimilarity){
    double dissimilarity;
    size_t n = neighbourIndex.size();
    dStarIterativeUpdate(givenTrackerIndex, neighbourIndex, allTrackers, dStarList);
    minDissimilarity[givenTrackerIndex] = dStarList[neighbourIndex[0]];
    for (int i=1; i<n; i++){
        dissimilarity = dStarList[neighbourIndex[0]];
        if (minDissimilarity[givenTrackerIndex] > dissimilarity){
            minDissimilarity[givenTrackerIndex] = dissimilarity;
        }
    }
}

void updateDissimilarityIterative (std::vector< std::vector<cv::Point2f> >& allTrackers, vector<double>& dStarList, vector<double>& minDissimilarity, vector< vector<int> >& neighbourIndexList){
    for (int i = 0; i < allTrackers.size(); i++){
        mininimumDissimilarityIterative(i, neighbourIndexList[i], allTrackers, dStarList, minDissimilarity);
    }
}

vector<int> findNeighbourIndex (std::vector<cv::Point2f> givenTracker,std::vector< std::vector<cv::Point2f> > allTrackers){
    double radius;
    radius = 5.0;
    vector<int> neighbourIndex;
    size_t n = allTrackers.size();
    while (neighbourIndex.size() < 4) {
        for (int i = 0; i < n; i++){
            if (radius > euclideanDistance(givenTracker.back(), allTrackers[i].back())){
                neighbourIndex.push_back(i);
            }
        }
        radius++;
    }
    return neighbourIndex;
}




double dStar (std::vector<cv::Point2f> givenTracker, std::vector< std::vector<cv::Point2f> > allTrackers){
    double euclideanDistance;
    double dissimilarity;
    vector<int> neighbourIndex = findNeighbourIndex(givenTracker, allTrackers);
    size_t n = neighbourIndex.size();
    for (int i=0; i<n; i++){
        euclideanDistance += euclideanMetric(givenTracker, allTrackers[neighbourIndex[i]]);
    }
    dissimilarity = euclideanDistance/n;
    return dissimilarity;
}

double mininimumDissimilarity (std::vector<cv::Point2f> givenTracker,std::vector< std::vector<cv::Point2f> > allTrackers){
    vector<int> neighbourIndex = findNeighbourIndex(givenTracker, allTrackers);
    double minDissimilarity;
    double dissimilarity;
    size_t n = neighbourIndex.size();
    for (int i=0; i<n; i++){
        dissimilarity = dStar(allTrackers[neighbourIndex[i]], allTrackers);
        if (i == 0){
            minDissimilarity = dissimilarity;
        }
        if (minDissimilarity > dissimilarity){
            minDissimilarity = dissimilarity;
        }
    }
    
    return minDissimilarity;
}

int main(int argc, const char * argv[]) {
    
    
    char* movie;
    movie = "/Users/student/Desktop/OpenCV/RiverSegmentation/RiverSegmentation/MovieBoat.mp4";
    VideoCapture cap;
    TermCriteria termcrit(CV_TERMCRIT_ITER|CV_TERMCRIT_EPS, 20, 0.03);
    Size subPixWinSize(10,10), winSize(31,31);
    
    const int MAX_COUNT = 500;
    bool needToInit = true;
    bool addRemovePt = false;
    
    cap.open(movie);
    
    if( argc == 2 )
        cap.open(argv[1]);
    
    else
        cap.open(movie);
    
    if( !cap.isOpened() )
    {
        cout << "Could not initialize capturing...\n";
        return 0;
    }
    
    namedWindow( "Camera Invariat Motion Based River Segmentation", 1 );
    
    VideoWriter outputVideo;
    bool writeMovie = false; //write the output video to the place defined by filename
    if (writeMovie)
    {
        char* filename;
        filename = "/Users/student/Desktop/OpenCV/RiverSegmentation/RiverSegmentation/MovieBoatOutput.mp4";
        
        Size S = Size((int) cap.get(CV_CAP_PROP_FRAME_WIDTH),    // Acquire input size
                      (int) cap.get(CV_CAP_PROP_FRAME_HEIGHT));
        outputVideo.open(filename, CV_FOURCC('8','B','P','S'), 30, S);
        if (!outputVideo.isOpened())
        {
            cout  << "Could not open the output video for write: " << filename << endl;
            return -1;
        }
        
    }
    
    Mat gray, prevGray, image;
    vector<Point2f> points[2];
    vector<Point2f> motions;
    
    
    vector< vector<Point2f> > tracking(MAX_COUNT, vector<Point2f>());
    vector< vector<int> > neighbourIndexList(MAX_COUNT);
    vector<double> minDissimilarity (MAX_COUNT);
    vector<double> dissimilarity (MAX_COUNT);
    vector<double> naturalBreaks;
    /*
    vector<double> lengths(MAX_COUNT);
    vector<double> radius(MAX_COUNT);
    vector<Point2f> difference;
    vector<double> entropy(MAX_COUNT);
    vector<double> naturalBreaks;
     */
    
    float maxRadius = 0.0;
    int c = 0;
    
    for(;;)
    {
        Mat frame;
        cap >> frame;
        if( frame.empty() )
            break;
        
        frame.copyTo(image);
        cvtColor(image, gray, COLOR_BGR2GRAY);
        
        
        if( needToInit )
        {
            // Using goodFeaturesToTrack function to automatically find 500 features to track
            goodFeaturesToTrack(gray, points[1], MAX_COUNT, 0.01, 10, Mat(), 3, 0, 0.04);
            cornerSubPix(gray, points[1], subPixWinSize, Size(-1,-1), termcrit);
            addRemovePt = false;
        }
        
        else if( !points[0].empty() )
        {
            vector<uchar> status;
            vector<float> err;
            if(prevGray.empty())
                gray.copyTo(prevGray);
            
            calcOpticalFlowPyrLK(prevGray, gray, points[0], points[1], status, err, winSize,
                                 3, termcrit, 0, 0.001);
            size_t i, k;
            for( i = k = 0; i < points[1].size(); i++ )
            {
                
                if( !status[i] )
                    
                    continue;
                
                points[0][k] = points[0][i];
                points[1][k] = points[1][i];
                tracking[k] = tracking[i];
                tracking[k].push_back(points[0][i]);
                /*
                lengths[k] = lengths[i];
                
                 */
                /*
                 
                 if (c != 0)
                 {
                 lengths[k] += calcLength(tracking[k].rbegin()[0], tracking[k].rbegin()[1]);
                 entropy[k] = calcEntropyUsingMiniballRadius(lengths[k], tracking[k], maxRadius);
                 }
                 */
                k++;
            }
            //cout << k<< endl;
            //cout << k;
            
            points[0].resize(k);
            points[1].resize(k);
            tracking.resize(k);
            dissimilarity.resize(k);
            
            for ( int i = 0; i < k; i++ ){
                dissimilarity[i] = mininimumDissimilarity(tracking[i], tracking);
            }

            c += 1;
        }
        
        if (c > 3)
        {
            naturalBreaks = JenksNaturalBreak(dissimilarity,4);
            
            for (int p = 0; p<dissimilarity.size(); ++p)
            {
                if (dissimilarity[p] < naturalBreaks[0])
                {
                    // Draw Blue Circles
                    circle( image, tracking[p].rbegin()[0], 3, Scalar(255,0,0), -1, 8);
                }
                else if (dissimilarity[p] < naturalBreaks[1])
                {
                    // Draw Green Circles
                    circle( image, tracking[p].rbegin()[0], 3, Scalar(0,255,0), -1, 8);
                }
                else if (dissimilarity[p] < naturalBreaks[2])
                {
                    //Draw Red Circles
                    circle( image, tracking[p].rbegin()[0], 3, Scalar(0,0,255), -1, 8);
                }
                
                else if (dissimilarity[p] < naturalBreaks[3])
                {
                    //Draw Purple Circles
                    circle( image, tracking[p].rbegin()[0], 3, Scalar(255,0,255), -1, 8);
                }
                else
                {
                    // Draw Yellow Circles
                    circle( image, tracking[p].rbegin()[0], 3, Scalar(0,255,255), -1, 8);
                    
                }
                
            }
        }

        
        needToInit = false;
        imshow("Camera Invariant Motion Based River Segmentation", image);
        
        char d = (char)waitKey(30);
        if( d == 27 )
            break;
        
        std::swap(points[1], points[0]);
        cv::swap(prevGray, gray);
        
        if (writeMovie)
        {
            outputVideo.write(image);
        }
        
    }
    return 0;
/*
    vector<Point2f> ax, by,cz,cy;
    vector< vector<Point2f> > allTracks;
    ax.push_back(Point2f(2,3));
    ax.push_back(Point2f(2,3));
    ax.push_back(Point2f(2,4));
    by.push_back(Point2f(3,2));
    by.push_back(Point2f(3,3));
    by.push_back(Point2f(3,4));
    cy.push_back(Point2f(3,3));
    cy.push_back(Point2f(3,4));
    cy.push_back(Point2f(3,5));
    mininimumDissimilarity(ax, allTracks);
    cout << euclideanMetric(ax, by);
    return 0;
 */
}
