#include "opencv2/video/tracking.hpp"
// Video write
#include <opencv2/imgproc/imgproc.hpp>
#include "opencv2/videostab/videostab.hpp"
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <vector>
#include <iostream>
#include <ctype.h>
#include <cstdlib>
#include <list>
#include <iterator>
#include "Miniball.hpp"
#include <math.h>
#include <typeinfo>
#include "JenksNaturalBreak.hpp"
#include <fstream>
#include <ratio>

using namespace cv;
using namespace std;
using namespace cv::videostab;

void drawBasedOnBreaks(const vector<double>& naturalBreaks, const vector<double>& values, Mat& image, vector< vector<Point2f> >& tracking, vector<Point2f>& initialPoints){
    
    for (int p = 0; p<values.size(); ++p)
    {
        if (values[p] < naturalBreaks[0])
        {
            // Draw Blue Circles
            circle( image, tracking[p].rbegin()[0]+initialPoints[p], 3, Scalar(255,0,0), -1, 8);
        }
        else if (values[p] < naturalBreaks[1])
        {
            // Draw Green Circles
            circle( image, tracking[p].rbegin()[0]+initialPoints[p], 3, Scalar(0,255,0), -1, 8);
        }
        else if (values[p] < naturalBreaks[2])
        {
            //Draw Red Circles
            circle( image, tracking[p].rbegin()[0]+initialPoints[p], 3, Scalar(0,0,255), -1, 8);
        }
        
        
        else if (values[p] < naturalBreaks[3])
            
        {
            //Draw Purple Circles
            circle( image, tracking[p].rbegin()[0]+initialPoints[p], 3, Scalar(255,0,255), -1, 8);
        }
        else
        {
            // Draw Yellow Circles
            circle( image, tracking[p].rbegin()[0]+initialPoints[p], 3, Scalar(0,255,255), -1, 8);
            
        }
        
    }
    
}

void generateMask(cv::Mat& mask, cv::Mat& image, vector<Point2f>& points){
    double length = 20.0;
    int cols = image.cols;
    int rows = image.rows;
    mask = cv::Mat::ones(rows, cols, CV_8UC1)*255;
    for (int i = 0; i < points.size(); i++){
        float x = points[i].x-length/2;
        float y = points[i].y-length/2;
        if (x < 0 || y < 0 || x+length > cols || y+length > rows) {
            continue;
        }
        
        Rect ROI (x, y, length, length);
        mask(ROI).setTo(Scalar::all(0));
    }
}



void concatenateVectors (vector<Point2f>& a, vector<Point2f>& b){
    a.insert(std::end(a), std::begin(b), std::end(b));
}

void concatenateVectors (vector<uchar>& a, vector<uchar>& b){
    a.insert(std::end(a), std::begin(b), std::end(b));
}

void featuresInit (vector<Point2f>& pointsInit, const int& maxPoint, const int& numPoints, Mat& mask, const Mat& gray, const TermCriteria& termcrit, const Size& subPixWinSize){
    goodFeaturesToTrack(gray, pointsInit, maxPoint-numPoints, 0.01, 10, mask, 3, 0, 0.04);
    cornerSubPix(gray, pointsInit, subPixWinSize, Size(-1,-1), termcrit);
    
}

//------------------------- Entropy Calculation --------------------------

double euclideanDistance(const Point2f& p1,const Point2f& p2)
{
    Point2f difference;
    difference = p1-p2;
    return sqrt(pow(difference.x,2)+pow(difference.y,2));
}

// calculate the minimum radius that encloses all the points using the MiniBall Software(V3.0)
double calcMinRadius(const vector<Point2f>& points, vector<Point2f>& supportPoints, double& radius, Point2f& center)
{
    vector<int> supportPointIndices;
    int cases;
    std::vector<std::vector<double> > lp;
    if (!supportPoints.empty() && euclideanDistance(points.back(), center) <= radius) {
        return radius;
    }
    else if (!supportPoints.empty() && euclideanDistance(points.back(), center) > radius){
        supportPoints.push_back(points.back());
        lp.resize(supportPoints.size());
        for (int i = 0; i < supportPoints.size(); i++) {
            std::vector<double> p(2);
            p[0] = supportPoints[i].x;
            p[1] = supportPoints[i].y;
            lp[i] = p;
        }
        cases = 1;
    }
    else{
        // convert vector<Point2f> to vector<double>
        lp.resize(points.size());
        for (int i=0; i<points.size(); ++i) {
            std::vector<double> p(2);
            p[0] = points[i].x;
            p[1] = points[i].y;
            lp[i] = p;
        }
        cases = 0;
    }
    // define the types of iterators through the points and their coordinates
    typedef std::vector<std::vector<double> >::const_iterator PointIterator;
    typedef std::vector<double>::const_iterator CoordIterator;
    
    // create an instance of Miniball
    typedef Miniball::
    Miniball <Miniball::CoordAccessor<PointIterator, CoordIterator> >
    MB;
    MB mb (2, lp.begin(), lp.end());
    
    // return radius
    // support points on the boundary determine the smallest enclosing ball
    //    std::cout << "Number of support points:\n  ";
    //    std::cout << mb.nr_support_points() << std::endl;
    supportPointIndices.resize(mb.nr_support_points());
    
    //    std::cout << "Support point indices (numbers refer to the input order):\n  ";
    MB::SupportPointIterator it = mb.support_points_begin();
    PointIterator first = lp.begin();
    for (int i = 0; it != mb.support_points_end(); ++it, i++) {
        //std::cout << std::distance(first, *it) << " "; // 0 = first point
        supportPointIndices[i] = std::distance(first, *it);
    }
    //std::cout << std::endl;
    if (cases == 0) {
        supportPoints.resize(supportPointIndices.size());
        for (int i = 0; i < supportPoints.size(); i++) {
            supportPoints[i] = points[supportPointIndices[i]];
        }
    }
    
    if (cases == 1) {
        vector<Point2f> tmp(supportPointIndices.size());
        for (int i = 0; i<supportPointIndices.size(); i++) {
            tmp[i] = supportPoints[supportPointIndices[i]];
        }
        supportPoints = tmp;
    }
    
    //std::cout << "Center:\n  ";
    const double* center1 = mb.center();
    center.x = center1[0];
    center.y = center1[1];
    //    for(int i=0; i<2; ++i, ++center1)
    //        std::cout << *center1 << " ";
    //    std::cout << std::endl;
    radius = sqrt(mb.squared_radius());
    
    return sqrt(mb.squared_radius());
}


void calcEntropyUsingMiniballRadiusIterative (double& entropy, double& length, vector<Point2f>& points, float &maxRadius, vector<Point2f>& supportPoints, double& radius, Point2f& center)
{
    if (points.size() < 3){
        entropy = 0;
    }
    else {
        double radius;
        radius = calcMinRadius(points, supportPoints, radius, center);
        if (radius > maxRadius)
            maxRadius = radius;
        entropy = log(length/(2*radius))/(log(points.size()-1))*radius/maxRadius;
    }
}


void calcLengthIterative(double& length, Point2f& p1, Point2f& p2)
{
    Point2f difference;
    difference = p1-p2;
    length += sqrt(pow(difference.x,2)+pow(difference.y,2));
}

void entropyListUpdate (vector<double>& entropy, vector<double>& lengths, vector< vector<Point2f> >& allTrackers, float& maxRadius, vector< vector<Point2f> >& supportPoints,  vector<double>& radius, vector<Point2f>& center)
{
    size_t n = allTrackers.size();
    for (int i = 0 ; i < n; i++) {
        if (allTrackers[i].size() < 2){
            continue;
        }
        calcLengthIterative(lengths[i], allTrackers[i].rbegin()[0], allTrackers[i].rbegin()[1]);
        calcEntropyUsingMiniballRadiusIterative(entropy[i], lengths[i], allTrackers[i], maxRadius, supportPoints[i], radius[i], center[i]);
    }
}


//------------------------ Entropy Calculation End --------------------------

//------------------------ Dissimilarity Calculation ------------------------


Point2f angleDistance (const Point2f& point1, const Point2f& point2){
    double distance;
    double angle;
    Point2f angleDistance;
    Point2f p;
    p = point2 - point1;
    distance = sqrt( p.x*p.x + p.y*p.y );
    angle = atan(p.y/p.x);
    angleDistance.x = angle;
    angleDistance.y = distance;
    return angleDistance;
}

vector<int> findNeighbourIndexIterative (int& givenTrackerIndex, std::vector<cv::Point2f>& points){
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
    //cout << radius << endl;
    return neighbourIndex;
}

void findNeighbourIndexList (vector< vector<int> >& neighbourIndexList, std::vector<cv::Point2f>& points){
    for (int i = 0; i < points.size(); i++){
        neighbourIndexList.push_back(findNeighbourIndexIterative(i, points));
    }
}



void updateNeighbour (vector< vector<int> >& neighbourIndexList, vector<int>& removedIndexList){
    bool remove = false;
    for (int i = 0; i < neighbourIndexList.size(); i++) {
        for (int j = 0; j < neighbourIndexList[i].size(); j++) {
            int n = 0;
            for (int k = 0; k < removedIndexList.size(); k++) {
                if (neighbourIndexList[i][j] == removedIndexList[k]){
                    remove = true;
                }
                if (neighbourIndexList[i][j] > removedIndexList[k]) {
                    n++;
                }
            }
            if (remove){
                neighbourIndexList[i].erase(neighbourIndexList[i].begin()+j);
                remove = false;
            }
            else{
                neighbourIndexList[i][j] = neighbourIndexList[i][j] - n;
            }
        }
    }
}

void dStarListIterativeUpdate (vector< vector<int> >& neighbourIndexList, std::vector< std::vector<cv::Point2f> >& allTrackers, vector<double>& dStarList){
    for (int i = 0; i < allTrackers.size(); i++) {
        if (allTrackers[i].size() <= 1){
            continue;
        }
        double euclideanDis = 0.0;
        for (int j = 0; j < neighbourIndexList[i].size(); j++) {
            if (allTrackers[neighbourIndexList[i][j]].size() <= 1) {
                continue;
            }
            euclideanDis += euclideanDistance(allTrackers[i].back(), allTrackers[neighbourIndexList[i][j]].back());
        }
        dStarList[i] += (euclideanDis/neighbourIndexList[i].size());
    }
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


//------------------------ Dissimilarity Calculation End ------------------------

//------------------------ Fusing Dissimilarity and Entropy ---------------------

void fusionUpdate (vector<double>& entropyList, vector<double>& dStarList, vector<double>& fusionList, double& maxEntropy, double& maxDissimilarity){
    double entropyMax, dStarMax;
    entropyMax = *max_element(entropyList.begin(), entropyList.end());
    dStarMax = *max_element(dStarList.begin(), dStarList.end());
    if (entropyMax > maxEntropy) {
        maxEntropy = entropyMax;
    }
    if (dStarMax > maxDissimilarity) {
        maxDissimilarity = dStarMax;
    }
    for (int i = 0; i < entropyList.size(); i++){
        fusionList[i] = entropyList[i]/maxEntropy * dStarList[i]/maxDissimilarity;
        //cout << entropyList[i] << "; " << entropyMax << "; " << dStarList[i] << "; "<< dStarMax << endl;
        
    }
}

//----------------------- Fusiing Dissimilarity and Entropy End -----------------

vector<vector<Point2f>> initTracking (vector<Point2f>& initPoints){
    vector<vector<Point2f>> initTracking(initPoints.size());
    
    for (int i = 0; i < initPoints.size(); i++){
        initTracking[i].push_back (initPoints[i]);
    }
    return initTracking;
}

void updateNeighbourStatus (vector<int>& neighbourStatus, vector<vector<int>>& neighbourIndexList){
    for (int i = 0; i < neighbourIndexList.size(); i++) {
        if (neighbourIndexList[i].size() == 0) {
            neighbourStatus[i] = 0;
        }
        else{
            neighbourStatus[i] = 1;
        }
    }
}

int main(int argc, const char* argv[])
{
    int clicks = 0;
    bool takeSnap = false;
    char* movie;
    movie = "/Users/student/Desktop/OpenCV/RiverSegmentation/RiverSegmentation/TrimmedVideo.mp4";
    //OpenCV/RiverSegmentation/RiverSegmentation/TrimmedVideo.mp4";
    //OpenCV/RiverSegmentation/RiverSegmentation/2016-03-18-104718.webm";
    //GP058145.m4v";
    //OpenCV/RiverSegmentation/RiverSegmentation/MovieBoat.mp4";
    
    VideoCapture cap;
    TermCriteria termcrit(CV_TERMCRIT_ITER|CV_TERMCRIT_EPS, 20, 0.03);
    Size subPixWinSize(10,10), winSize(31,31);
    
    const int MAX_COUNT = 400;
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
    
    namedWindow("Entropy Based River Segmentation", 1 );
    namedWindow("Trackers",1);
    namedWindow("Mask",1);
    
    VideoWriter outputVideo;
    bool writeMovie = false; //write the output video to the place defined by filename
    if (writeMovie)
    {
        char* filename;
        filename = "/Users/student/Desktop/OpenCV/RiverSegmentation/RiverSegmentation/MovieBoatOutputDone.mp4";
        
        Size S = Size((int) cap.get(CV_CAP_PROP_FRAME_WIDTH),    // Acquire input size
                      (int) cap.get(CV_CAP_PROP_FRAME_HEIGHT));
        outputVideo.open(filename, CV_FOURCC('8','B','P','S'), 30, S);
        if (!outputVideo.isOpened())
        {
            cout  << "Could not open the output video for write: " << filename << endl;
            return -1;
        }
        
    }
    
    Mat gray, prevGray, image, mask, trackers;
    
    //---- Tracking Values ----
    
    vector< vector<Point2f> > points1;
    vector< vector<Point2f> > points0;
    vector< vector< vector<Point2f> > > tracking;
    vector< vector<Point2f> > initialPoints;
    
    //---- End Tracking Values
    
    
    //----- Entropy Values ------
    vector< vector<double> > lengths;//(1, vector<double> (MAX_COUNT));
    vector< vector<double> > entropy;//(1, vector<double> (MAX_COUNT));
    vector< vector< vector<Point2f> > > supportPoints;
    vector< vector<double> > radius;
    vector< vector<Point2f> > center;
    vector<float> maxRadius;
    
    //----- End Entropy Values ----
    
    
    //----- Dissimilarity Values ---
    
    vector< vector< vector<int> > > neighbourIndexList;//(1, vector< vector<int> >(MAX_COUNT));
    vector< vector<double> > minDissimilarity;// (1, vector<double> (MAX_COUNT));
    vector< vector<double> > dissimilarity;// (1, vector<double> (MAX_COUNT));
    vector< vector<double> > dStarList;// (1, vector<double> (MAX_COUNT));
    vector<vector<int>> neighbourStatus;
    
    //----- End Dissimilarity Values ---
    
    //----- Fusion Values-------
    
    vector< vector<double> > fusion;// (1, vector<double> (MAX_COUNT));
    vector<double> maxEntropy;
    vector<double> maxDissimilarity;
    
    //---- End Fusion Values
    
    vector<vector<double>> naturalBreaks;
    
    vector<int> count;
    int c = 0;
    int numPoints = 0;
    
    for(;;)
    {
        
        Mat frame;
        cap >> frame;
        if( frame.empty() )
            break;
        
        frame.copyTo(image);
        cvtColor(image, gray, COLOR_BGR2GRAY);
        
        int cols = image.cols;
        int rows = image.rows;
        trackers = cv::Mat::zeros(rows, cols, CV_8UC3);
        
        if( needToInit )
        {
            points1.push_back({});

            cout << "Generating NEW POINTS";

            // Using goodFeaturesToTrack function to automatically find 500 features to track
            goodFeaturesToTrack(gray, points1.back(), MAX_COUNT-numPoints, 0.00001, 10, mask, 3, 0, 0.04);
            cornerSubPix(gray, points1.back(), subPixWinSize, Size(-1,-1), termcrit);
            
            addRemovePt = false;
            
            numPoints = points1.back().size();
            
            initialPoints.push_back(points1.back());
            
            tracking.push_back(vector< vector<Point2f> > (points1.back().size(),{{0,0}}));//initTracking(points1.back()));
            
            //----- Initializing Entropy Values --------
            
            entropy.push_back(vector<double> (points1.back().size()));
            lengths.push_back(vector<double> (points1.back().size()));
            supportPoints.push_back(vector<vector<Point2f>> (points1.back().size()));
            center.push_back(vector<Point2f> (points1.back().size()));
            radius.push_back(vector<double> (points1.back().size()));
            maxRadius.push_back(0.0);
            
            //----- End Initializing Entropy Values -------
            
            //----- Initializing Dissimiilarity Values ----
            
            dStarList.push_back(vector<double> (points1.back().size()));
            minDissimilarity.push_back(vector<double> (points1.back().size()));
            neighbourStatus.push_back(vector<int> (points1.back().size(), 1));
            neighbourIndexList.push_back(vector< vector<int> > {});
            findNeighbourIndexList(neighbourIndexList.back(),points1.back());
            
            //---- End Initializing Dissimiilarity Values --
            
            //---- Start Initializing Fusion Values ----
            
            fusion.push_back(vector<double> (points1.back().size()));
            maxDissimilarity.push_back(0.0);
            maxEntropy.push_back(0.0);
            
            //---- End Initializing Fusion Values -----
            
            count.push_back(0);
            
            //cout << "initiating points" << endl;
            needToInit = false;
            points0.push_back({});
            naturalBreaks.push_back({});
        }
        
        for (int i = 0; i < tracking.size(); i++) {
            if( !points0[i].empty() )
            {
                vector<uchar> status;
                vector<float> err;
                vector<int> removedIndex;
                
                if(prevGray.empty()){
                    gray.copyTo(prevGray);
                }
                
                calcOpticalFlowPyrLK(prevGray, gray, points0[i], points1[i], status, err, winSize, 3, termcrit, 0, 0.0001);
                
                size_t l, k;
                for( l = k = 0; l < points1[i].size(); l++ )
                {
                    
                    if( !status[l] || !neighbourStatus[i][l] )
                    {
                        removedIndex.push_back(l);
                        continue;
                    }
                    points0[i][k] = points0[i][l];
                    points1[i][k] = points1[i][l];
                    initialPoints[i][k] = initialPoints[i][l];
                    tracking[i][k] = tracking[i][l];
                    tracking[i][k].push_back(points1[i][l]-initialPoints[i][l]);
                    
                    //---- Dissimilarity Values---
                    dStarList[i][k] = dStarList[i][l];
                    minDissimilarity[i][k] = minDissimilarity[i][l];
                    neighbourIndexList[i][k] = neighbourIndexList[i][l];
                    //---- End Dissimilarity Values---
                    
                    //---- Entropy Values ---
                    lengths[i][k] = lengths[i][l];
                    radius[i][k] = lengths[i][l];
                    center[i][k] = center[i][l];
                    supportPoints[i][k] = supportPoints[i][l];
                    
                    //---- End Entropy Values
                    
                    k++;
                }
                
                initialPoints[i].resize(k);
                points0[i].resize(k);
                points1[i].resize(k);
                tracking[i].resize(k);
                
                //---- Dissimilarity Values --
                dStarList[i].resize(k);
                minDissimilarity[i].resize(k);
                neighbourIndexList[i].resize(k);
                fusion[i].resize(k);
                neighbourStatus[i].resize(k);
                
                //---- End Dissimilarity Values --
                
                //---- Entropy Values ---
                
                entropy[i].resize(k);
                lengths[i].resize(k);
                center[i].resize(k);
                radius[i].resize(k);
                supportPoints[i].resize(k);
                
                //---- End Entropy Values
                

                updateNeighbour(neighbourIndexList[i], removedIndex);

                updateNeighbourStatus(neighbourStatus[i], neighbourIndexList[i]);

                dStarListIterativeUpdate(neighbourIndexList[i], tracking[i], dStarList[i]);
                
                entropyListUpdate(entropy[i], lengths[i], tracking[i], maxRadius[i], supportPoints[i],radius[i],center[i]);

                fusionUpdate(entropy[i],dStarList[i],fusion[i],maxEntropy[i],maxDissimilarity[i]);

                count[i]++;
            }
            
            
            if (count[i] > 3)
            {
                
                naturalBreaks[i] = JenksNaturalBreak(fusion[i], 4);
                
                drawBasedOnBreaks(naturalBreaks[i], fusion[i], trackers, tracking[i], initialPoints[i]);
                
                drawBasedOnBreaks(naturalBreaks[i], fusion[i], image, tracking[i], initialPoints[i]);
                
            }
            
            std::swap(points1[i], points0[i]);
        }
        numPoints = 0;
        vector<Point2f> allPoints;

        for (int i = 0; i < points0.size(); i++) {
            numPoints += points0[i].size();
            concatenateVectors(allPoints, points0[i]);
        }
        if (numPoints < MAX_COUNT*0.9) {
            needToInit = true;
            cout << "Init Points" << endl;
            generateMask(mask, image, allPoints);
        }
        
        c++;
        
        cv::swap(prevGray, gray);
        
        imshow("Entropy Based River Segmentation", image);

        
        char d = (char)waitKey(20);
        if( d == 27 )
            break;
        
        if (writeMovie)
        {
            outputVideo.write(image);
        }
        
    }
    
    
    return 0;
}