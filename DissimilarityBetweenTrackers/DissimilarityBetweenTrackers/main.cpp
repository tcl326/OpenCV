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

#include <cmath>
#include <cfloat>

using namespace std;
using namespace cv;

double euclideanDistance (Point2f point1, Point2f point2){
    double distance;
    Point2f p;
    p = point1 - point2;
    distance = sqrt( p.x*p.x + p.y*p.y );
    return distance;
}

Point2f angleDistance (Point2f point1, Point2f point2){
    double distance;
    double angle;
    Point2f angleDistance;
    Point2f p;
    p = point2 - point1;
    distance = sqrt( p.x*p.x + p.y*p.y );
    angle = atan2(p.y, p.x);
    angleDistance.x = angle;
    angleDistance.y = distance;
    return angleDistance;
}

vector<int> findNeighbourIndexIterative (int givenTrackerIndex, std::vector<cv::Point2f> points){
    double radius;
    radius = 10.0;
    vector<int> neighbourIndex;
    size_t n = points.size();
    while (neighbourIndex.size() < 4) {
        neighbourIndex = {};
        for (int i = 0; i < n; i++){
            if (i == givenTrackerIndex) {
                continue;
            }
            if (radius >= euclideanDistance(points[givenTrackerIndex], points[i])){
                neighbourIndex.push_back(i);
            }
        }
        radius++;
    }
    cout << radius << endl;
    return neighbourIndex;
}

void findNeighbourIndexList (vector< vector<int> >& neighbourIndexList, std::vector<cv::Point2f> points){
    for (int i = 0; i < points.size(); i++){
        neighbourIndexList[i] = findNeighbourIndexIterative(i, points);
    }
}

void updateNeighbour (vector< vector<int> >& neighbourIndexList, vector<int> removedIndexList){
    bool remove = false;
    for (int k = 0; k < removedIndexList.size(); k++){
        for (int i = 0; i < neighbourIndexList.size(); i++) {
            for (int j = 0; j < neighbourIndexList[i].size(); j++) {
                if (neighbourIndexList[i][j] > removedIndexList[k]) {
                    neighbourIndexList[i][j]--;
                }
                if (neighbourIndexList[i][j] == removedIndexList[k]) {
                    remove = true;
                }
            }
            if (remove) {
                neighbourIndexList[i].erase(std::remove(neighbourIndexList[i].begin(), neighbourIndexList[i].end(), removedIndexList[k]), neighbourIndexList[i].end());
                remove = false;
                
            }
        }
    }
}
/*
void dStarIterativeUpdate (int givenTrackerIndex, vector<int>& neighbourIndex, std::vector< std::vector<cv::Point2f> >& allTrackers, vector<double>& dStarList){
    double euclideanDis = 0.0;
    size_t n = neighbourIndex.size();
    for (int i=0; i<n; i++){
        euclideanDis += euclideanDistance(angleDistance( allTrackers[givenTrackerIndex].back(), allTrackers[givenTrackerIndex].end()[-2]), angleDistance (allTrackers[neighbourIndex[i]].back(), allTrackers[neighbourIndex[i]].end()[-2]));
    }
    dStarList[givenTrackerIndex] += (euclideanDis/n);
}
 */

void dStarListIterativeUpdate (vector< vector<int> >& neighbourIndexList, std::vector< std::vector<cv::Point2f> >& allTrackers, vector<double>& dStarList){
    
    //cout << "[ ";
    for (int j = 0; j < allTrackers.size(); j++){
        if (allTrackers[j].size() == 1){
            continue;
        }
        double euclideanDis = 0.0;
        size_t n = neighbourIndexList[j].size();
        for (int i=0; i<n; i++){
            Point2f angleDistance1;
            Point2f angleDistance2;
            double distance;
            angleDistance1 = angleDistance(allTrackers[j].back(), allTrackers[j].end()[-2]);
            angleDistance2 = angleDistance(allTrackers[neighbourIndexList[j][i]].back(), allTrackers[neighbourIndexList[j][i]].end()[-2]);
            //euclideanDis += euclideanDistance(allTrackers[j].back(), allTrackers[neighbourIndexList[j][i]].back());
            distance = euclideanDistance(angleDistance1, angleDistance2);
            euclideanDis += distance;
            //cout << euclideanDis << "; ";
            if (not isfinite(distance)){
                cout << euclideanDis << "; " << allTrackers[j].back() << "; " << allTrackers[j].end()[-2] << "; " << allTrackers[neighbourIndexList[j][i]].back() << "; " <<allTrackers[neighbourIndexList[j][i]].end()[-2] << "; " << angleDistance1 << "; " << angleDistance2 << endl;
                for (int k = 0; k < allTrackers[j].size(); k++) {
                    cout << allTrackers[j][k] << endl;
                }
            }
        }
        dStarList[j] += (euclideanDis/n);
        //if (not isfinite(dStarList[j])) {
        //    cout << euclideanDis << "; " << n << "; " << j << endl;
        //}
        //cout << dStarList[j] << ", ";
    }
    //cout << "]" << endl;
}


void mininimumDissimilarityIterative (int givenTrackerIndex , vector<int>& neighbourIndex, std::vector< std::vector<cv::Point2f> >& allTrackers, vector<double>& dStarList, vector<double>& minDissimilarity){
    double dissimilarity;
    size_t n = neighbourIndex.size();
    //dStarIterativeUpdate(givenTrackerIndex, neighbourIndex, allTrackers, dStarList);
    minDissimilarity[givenTrackerIndex] = dStarList[neighbourIndex[0]];
    for (int i=1; i<n; i++){
        dissimilarity = dStarList[neighbourIndex[i]];
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

void test(){
    vector<Point2f> points1;
    vector<Point2f> points2;
    vector<Point2f> points3;
    vector< vector<Point2f> > allPoints;
    vector< vector<Point2f> > tracking(9);
    vector< vector<int> > indexList(9);
    vector<double> minDissimilarity (9);
    vector<double> dissimilarity (9);
    vector<double> dStarList (9);
    vector<int> removedIndex;
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            Point2f p;
            p.x = i;
            p.y = j;
            points1.push_back(p);
        }
    }
    for (int i = 0; i < points1.size(); i++){
        Point2f p;
        p.x = points1[i].x;
        p.y = points1[i].y + 0.5;
        points2.push_back(p);
    }
    points2[0].x += 0.5;
    for (int i = 0; i < points2.size(); i++){
        if (i == 5) {
            continue;
        }
        Point2f p;
        p.x = points2[i].x;
        p.y = points2[i].y + 0.5;
        points3.push_back(p);
    }
    points3[0].x += 0.5;
    
    for (int i = 0 ; i < points1.size(); i++) {
        cout << points1[i];
    }
    for (int i = 0 ; i < points2.size(); i++) {
        cout << points2[i];
    }
    for (int i = 0 ; i < points3.size(); i++) {
        cout << points3[i];
    }
    allPoints.push_back(points1);
    allPoints.push_back(points2);
    allPoints.push_back(points3);
    
    for (int i = 0; i < allPoints.size(); i++) {
        if (i == 0){
            findNeighbourIndexList(indexList, points1);
        }
        if (i > 0){
            int j, k;
            for (j = k = 0; j < allPoints[i-1].size(); j++) {
                if (i == 2 && j == 4){
                    removedIndex.push_back(j);
                    continue;
                }
                tracking[j] = tracking[k];
                tracking[k].push_back(allPoints[i][j]);
                k++;
                cout << k << endl;
            }
            dStarList.resize(k);
            minDissimilarity.resize(k);
            indexList.resize(k);
            updateNeighbour(indexList, removedIndex);
            dStarListIterativeUpdate(indexList, tracking, dStarList);
            
            updateDissimilarityIterative(tracking, dStarList, minDissimilarity, indexList);
            for (int i = 0 ; i < minDissimilarity.size(); i++) {
                cout << minDissimilarity[i]<< ", ";
            }
        }
    }
    

    
}
/*
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
*/

int main(int argc, const char * argv[]) {
    //test();
    
    char* movie;
    movie = "/Users/student/Desktop/OpenCV/RiverSegmentation/RiverSegmentation/MovieBoat.mp4";
    //GP058145.m4v";
    //OpenCV/RiverSegmentation/RiverSegmentation/MovieBoat.mp4";
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
    vector<double> dStarList (MAX_COUNT);
    vector<double> radiuses (MAX_COUNT);
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
            findNeighbourIndexList(neighbourIndexList, points[1]);
            
        }
        
        else if( !points[0].empty() )
        {
            vector<uchar> status;
            vector<float> err;
            std::vector<int> removedIndex;
            if(prevGray.empty())
                gray.copyTo(prevGray);
            
            calcOpticalFlowPyrLK(prevGray, gray, points[0], points[1], status, err, winSize,
                                 3, termcrit, 0, 0.001);
            size_t i, k;
            for( i = k = 0; i < points[1].size(); i++ )
            {
                
                if( !status[i] ){
                    removedIndex.push_back(i);
                    continue;
                }
                
                points[0][k] = points[0][i];
                points[1][k] = points[1][i];
                tracking[k] = tracking[i];
                dStarList[k] = dStarList[i];
                minDissimilarity[k] = minDissimilarity[i];
                neighbourIndexList[k] = neighbourIndexList[i];
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
            dStarList.resize(k);
            minDissimilarity.resize(k);
            neighbourIndexList.resize(k);
            updateNeighbour(neighbourIndexList, removedIndex);
            dStarListIterativeUpdate(neighbourIndexList, tracking, dStarList);
            
            updateDissimilarityIterative(tracking, dStarList, minDissimilarity, neighbourIndexList);

            c += 1;
        }
        
        if (c > 3)
        {
            naturalBreaks = JenksNaturalBreak(dStarList,4);
            
            for (int p = 0; p<minDissimilarity.size(); ++p)
            {
                if (dStarList[p] < naturalBreaks[0])
                {
                    // Draw Blue Circles
                    circle( image, tracking[p].rbegin()[0], 3, Scalar(255,0,0), -1, 8);
                }
                else if (dStarList[p] < naturalBreaks[1])
                {
                    // Draw Green Circles
                    circle( image, tracking[p].rbegin()[0], 3, Scalar(0,255,0), -1, 8);
                }
                else if (dStarList[p] < naturalBreaks[2])
                {
                    //Draw Red Circles
                    circle( image, tracking[p].rbegin()[0], 3, Scalar(0,0,255), -1, 8);
                }
                
                else if (dStarList[p] < naturalBreaks[3])
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

}
