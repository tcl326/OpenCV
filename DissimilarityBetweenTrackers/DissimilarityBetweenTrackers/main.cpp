//
//  main.cpp
//  DissimilarityBetweenTrackers
//
//  Created by Student on 1/29/16.
//  Copyright © 2016 Student. All rights reserved.
//

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

using namespace std;
using namespace cv;

std::vector< std::vector<cv::Point2f> > findNeighbour (std::vector<cv::Point2f> givenTracker,std::vector< std::vector<cv::Point2f> > allTrackers){
    std::vector< std::vector<cv::Point2f> > neighbourTrackers;
    double radius;
    
    return neighbourTrackers;
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
        Point2f p;
        p = givenTracker[i] - neighbourTracker[i];
        sumEuclideanDistance += sqrt( p.x*p.x + p.y*p.y );
    }
    
    return sumEuclideanDistance;
}

double dStar (std::vector<cv::Point2f> givenTracker, std::vector< std::vector<cv::Point2f> > allTrackers){
    double euclideanDistance;
    double dissimilarity;
    size_t n = allTrackers.size();
    for (int i=0; i<n; i++){
        euclideanDistance += euclideanMetric(givenTracker, allTrackers[i]);
    }
    dissimilarity = euclideanDistance/n;
    return dissimilarity;
}

double mininmumDissimilarity (std::vector<cv::Point2f> givenTracker, std::vector< std::vector<cv::Point2f> > neighbourTrackers, std::vector< std::vector<cv::Point2f> > allTrackers){
    double minDissimilarity;
    double dissimilarity;
    size_t n = neighbourTrackers.size();
    for (int i=0; i<n; i++){
        dissimilarity = dStar(neighbourTrackers[i], allTrackers);
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
    vector<Point2f> ax, by,cz,cy;
    ax.push_back(Point2f(2,3));
    ax.push_back(Point2f(2,3));
    ax.push_back(Point2f(2,4));
    by.push_back(Point2f(3,2));
    by.push_back(Point2f(3,3));
    //by.push_back(Point2f(3,4));
    cy.push_back(Point2f(3,3));
    cy.push_back(Point2f(3,4));
    cy.push_back(Point2f(3,5));
    cout << euclideanMetric(ax, by);
    return 0;
}
