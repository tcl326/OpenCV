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
#include <iostream>
#include <list>
#include <iterator>
#include "Miniball.hpp"
#include <math.h>
#include <typeinfo>
#include "JenksNaturalBreak.hpp"

using namespace cv;
using namespace std;
using namespace cv::videostab;

vector<double> flatten (vector< vector<double> >& timedData){
    vector<double> flattenData;
    for (int i = 0; i < timedData.size(); i++) {
        for (int j = 0; j < timedData[i].size(); j++) {
            flattenData.push_back(timedData[i][j]);
        }
    }
    return flattenData;
}

vector< vector<Point2f> > flatten (vector< vector< vector<Point2f> >>& tracking){
    vector< vector<Point2f> > flattenData;
    for (int i = 0; i < tracking.size(); i++) {
        for (int j = 0; j < tracking[i].size(); j++) {
            flattenData.push_back(tracking[i][j]);
        }
    }
    return flattenData;

}

void timedDataValueResize(vector<vector<double>>& timedData, const vector<int>& numberPerTime){
    for (int i = 0; i < timedData.size(); i++) {
        timedData[i].resize(numberPerTime[i]);
    }
}

void timedDataValueResize(vector<vector<vector<Point2f>>>& timedData, const vector<int>& numberPerTime){
    for (int i = 0; i < timedData.size(); i++) {
        timedData[i].resize(numberPerTime[i]);
    }
}

void timedDataValueResize(vector<vector<vector<int>>>& timedData, const vector<int>& numberPerTime){
    for (int i = 0; i < timedData.size(); i++) {
        timedData[i].resize(numberPerTime[i]);
    }
}

void drawBasedOnBreaks(const vector<double>& naturalBreaks, const vector<double>& values, Mat& image, vector< vector<Point2f> > tracking){
    
    for (int p = 0; p<values.size(); ++p)
    {
        if (values[p] < naturalBreaks[0])
        {
            // Draw Blue Circles
            circle( image, tracking[p].rbegin()[0], 3, Scalar(255,0,0), -1, 8);
        }
        else if (values[p] < naturalBreaks[1])
        {
            // Draw Green Circles
            circle( image, tracking[p].rbegin()[0], 3, Scalar(0,255,0), -1, 8);
        }
        else if (values[p] < naturalBreaks[2])
        {
            //Draw Red Circles
            circle( image, tracking[p].rbegin()[0], 3, Scalar(0,0,255), -1, 8);
        }
        
        
        else if (values[p] < naturalBreaks[3])
            
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

void featuresInit (vector<Point2f>& pointsInit, const int& maxPoint, const int& numPoints, Mat& mask, const Mat& gray, const TermCriteria& termcrit, const Size& subPixWinSize){
    goodFeaturesToTrack(gray, pointsInit, maxPoint-numPoints, 0.01, 10, mask, 3, 0, 0.04);
    cornerSubPix(gray, pointsInit, subPixWinSize, Size(-1,-1), termcrit);
    
}

//------------------------- Entropy Calculation --------------------------

// calculate the minimum radius that encloses all the points using the MiniBall Software(V3.0)
double calcMinRadius(vector<Point2f>& points)
{
    // convert vector<Point2f> to vector<double>
    std::vector<std::vector<double> > lp;
    for (int i=0; i<points.size(); ++i) {
        std::vector<double> p(2);
        p[0] = points[i].x;
        p[1] = points[i].y;
        lp.push_back(p);
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
    return sqrt(mb.squared_radius());
}


void calcEntropyUsingMiniballRadiusIterative (double& entropy, double& length, vector<Point2f>& points, float &maxRadius)
{
    if (points.size() < 3){
        entropy = 0;
    }
    else {
        double radius;
        radius = calcMinRadius(points);
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

void entropyListUpdate (vector<double>& entropy, vector<double>& lengths, vector< vector<Point2f> >& allTrackers, float& maxRadius)
{
    size_t n = allTrackers.size();
    for (int i = 0 ; i < n; i++) {
        if (allTrackers[i].size() < 2){
            continue;
        }
        calcLengthIterative(lengths[i], allTrackers[i].rbegin()[0], allTrackers[i].rbegin()[1]);
        calcEntropyUsingMiniballRadiusIterative(entropy[i], lengths[i], allTrackers[i], maxRadius);
    }
}

void entropyTimedListUpdate (vector <vector<double> >& entropy, vector< vector<double> >& lengths, vector< vector< vector<Point2f> > >& allTrackers, vector<float>  & maxRadius)
{
    size_t n = allTrackers.size();
    for (int i = 0 ; i < n; i++) {
        entropyListUpdate(entropy[i], lengths[i], allTrackers[i], maxRadius[i]);
    }
}

//------------------------ Entropy Calculation End --------------------------

//------------------------ Dissimilarity Calculation ------------------------

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
    angle = atan(p.y/p.x);
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
    //cout << radius << endl;
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

void dStarListIterativeUpdate (vector< vector<int> >& neighbourIndexList, std::vector< std::vector<cv::Point2f> >& allTrackers, vector<double>& dStarList){
    
    //cout << "[ ";
    for (int j = 0; j < allTrackers.size(); j++){
        if (allTrackers[j].size() == 1){
            continue;
        }
        double euclideanDis = 0.0;
        size_t n = neighbourIndexList[j].size();
        for (int i=0; i<n; i++){
            if (allTrackers[j].size() <= 1 || allTrackers[neighbourIndexList[j][i]].size() <= 1)
                continue;
            Point2f angleDistance1;
            Point2f angleDistance2;
            double distance;
            angleDistance1 = angleDistance(allTrackers[j].back(), allTrackers[j].end()[-2]);
            angleDistance2 = angleDistance(allTrackers[neighbourIndexList[j][i]].back(), allTrackers[neighbourIndexList[j][i]].end()[-2]);
            //euclideanDis += euclideanDistance(allTrackers[j].back(), allTrackers[neighbourIndexList[j][i]].back());
            distance = euclideanDistance(angleDistance1, angleDistance2);
            
            //cout << euclideanDis << "; ";
            if (not isfinite(distance)){
                //cout << euclideanDis << "; " << allTrackers[j].back() << "; " << allTrackers[j].end()[-2] << "; " << allTrackers[neighbourIndexList[j][i]].back() << "; " <<allTrackers[neighbourIndexList[j][i]].end()[-2] << "; " << angleDistance1 << "; " << angleDistance2 << endl;
                //for (int k = 0; k < allTrackers[j].size(); k++) {
                //    cout << allTrackers[j][k] << endl;
                //}
                continue;
            }
            euclideanDis += distance;
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

//------------------------ Dissimilarity Calculation End ------------------------

//------------------------ Fusing Dissimilarity and Entropy ---------------------

void fusionUpdate (vector<double>& entropyList, vector<double>& dStarList, vector<double>& fusionList){
    double entropyMax, dStarMax;
    entropyMax = *max_element(entropyList.begin(), entropyList.end());
    dStarMax = *max_element(dStarList.begin(), dStarList.end());
    for (int i = 0; i < entropyList.size(); i++){
        fusionList[i] = entropyList[i]/entropyMax * dStarList[i]/dStarMax;
    }
}

//----------------------- Fusiing Dissimilarity and Entropy End -----------------

/*
void updateAllAndDraw(vector< vector<Point2f> >& allTrackers,const vector<uchar>& status, vector<Point2f>& oldPoints, vector<Point2f>& newPoints, vector<int>& removedIndex, vector<double>& lengths, Mat& image, int& numPoints, vector<double>& dStarList, vector<double>& minDissimilarity, vector< vector<int> >& neighbourIndexList){
    size_t i, k;
    for( i = k = 0; i < newPoints.size(); i++ )
    {
        
        if( !status[i] )
        {
            removedIndex.push_back(i);
            continue;
        }
        oldPoints[k] = oldPoints[i];
        newPoints[k] = newPoints[i];
        allTrackers[k] = allTrackers[i];
        lengths[k] = lengths[i];
        allTrackers[k].push_back(newPoints[i]);
        dStarList[k] = dStarList[i];
        minDissimilarity[k] = minDissimilarity[i];
        neighbourIndexList[k] = neighbourIndexList[i];
        
        
        k++;
        //circle( image, newPoints[i], 3, Scalar(0,255,0), -1, 8);
    }
    numPoints = k;
}
 */

void updateAllTimedAndDraw (vector< vector< vector<Point2f> > >& allTrackers,const vector<uchar>& status, vector<Point2f>& oldPoints, vector<Point2f>& newPoints, vector< vector<int> >& removedIndex, vector< vector<double> >& lengths, Mat& image, int& numPoints, vector<int>& numberPerTime, vector< vector<double> >& dStarList, vector< vector<double> >& minDissimilarity, vector< vector< vector<int> > >& neighbourIndexList){
    numberPerTime = {};
    int i = 0;
    int k = 0;
    for (int j = 0; j < allTrackers.size(); j++){
        int m = 0;
        for (int l = 0; l < allTrackers[j].size(); l++, i++) {
            if( !status[i] )
            {
                removedIndex[j].push_back(l);
                continue;
            }

            oldPoints[k] = oldPoints[i];
            newPoints[k] = newPoints[i];
            allTrackers[j][m] = allTrackers[j][l];
            lengths[j][m] = lengths[j][l];
            allTrackers[j][m].push_back(newPoints[i]);
            dStarList[j][m] = dStarList[j][l];
            minDissimilarity[j][m] = minDissimilarity[j][l];
            //neighbourIndexList[j][m] = neighbourIndexList[j][l];
            
            
            k++;
            m++;
        }
        numberPerTime.push_back(m);
    }

    numPoints = k;

}

vector<vector<Point2f>> initTracking (vector<Point2f>& initPoints){
    vector<vector<Point2f>> initTracking(initPoints.size());
    for (int i = 0; i < initPoints.size(); i++){
        initTracking[i].push_back (initPoints[i]);
    }
    return initTracking;
}

int main(int argc, const char* argv[])
{
    
    char* movie;
    movie = "/Users/student/Desktop/OpenCV/RiverSegmentation/RiverSegmentation/MovieBoat.mp4";
    
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
    
    namedWindow( "Entropy Based River Segmentation", 1 );
    namedWindow("Mask",1);
    
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
    
    Mat gray, prevGray, image, mask;
    vector<Point2f> points[2];
    vector<Point2f> pointsInit;
    vector< vector< vector<Point2f> > > tracking;// (1, vector< vector<Point2f> > (MAX_COUNT, vector<Point2f>()));
    
    vector< vector<double> > lengths;//(1, vector<double> (MAX_COUNT));
    vector< vector<double> > radius;//(1, vector<double> (MAX_COUNT));
    vector< vector<double> > entropy;//(1, vector<double> (MAX_COUNT));
    vector<double> naturalBreaks;
    
    
    //Variables for Dissimilarity
    
    vector< vector< vector<int> > > neighbourIndexList;//(1, vector< vector<int> >(MAX_COUNT));
    vector< vector<double> > minDissimilarity;// (1, vector<double> (MAX_COUNT));
    vector< vector<double> > dissimilarity;// (1, vector<double> (MAX_COUNT));
    vector< vector<double> > dStarList;// (1, vector<double> (MAX_COUNT));
    vector< vector<double> > radiuses;// (1, vector<double> (MAX_COUNT));
    vector< vector<double> > fusion;// (1, vector<double> (MAX_COUNT));
    
    vector<int> numberPerTime(1);
    vector< vector<int> >removedIndex;
    
    vector<float> maxRadius(1, 0.0);
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
        
        
        if( needToInit )
        {
            generateMask(mask, image, points[1]);
            // Using goodFeaturesToTrack function to automatically find 500 features to track
            goodFeaturesToTrack(gray, pointsInit, MAX_COUNT-numPoints, 0.01, 10, mask, 3, 0, 0.04);
            cornerSubPix(gray, pointsInit, subPixWinSize, Size(-1,-1), termcrit);
            addRemovePt = false;
            concatenateVectors(points[1], pointsInit);
            //neighbourIndexList.resize(points[1].size());
            
            cout << pointsInit.size() << endl;
            cout << numPoints << endl;
            
            numPoints = points[1].size();
            
            cout << numPoints << endl;
            
            tracking.push_back(initTracking(pointsInit));
            
            entropy.push_back(vector<double> (pointsInit.size()));
            
            lengths.push_back(vector<double> (pointsInit.size()));
            
            dStarList.push_back(vector<double> (pointsInit.size()));
            minDissimilarity.push_back(vector<double> (pointsInit.size()));
            timedDataValueResize(neighbourIndexList,numberPerTime);
            fusion.push_back(vector<double> (pointsInit.size()));
            
            removedIndex.push_back(vector<int> {});
            
            //updateNeighbour(neighbourIndexList, removedIndex);
            //dStarListIterativeUpdate(neighbourIndexList, tracking, dStarList);
            //updateDissimilarityIterative(tracking, dStarList, minDissimilarity, neighbourIndexList);
            entropyTimedListUpdate(entropy, lengths, tracking, maxRadius);
            //fusionUpdate(entropy,dStarList,fusion);
            
            
            //tracking.push_back(pointsInit);//(numPoints);
            
            //findNeighbourIndexList(neighbourIndexList, points[1]);
            
            cout << "initiating points" << endl;
            needToInit = false;
            std::swap(points[1], points[0]);
        }
        
        if( !points[0].empty() )
        {
            vector<uchar> status;
            vector<float> err;
            
            if(prevGray.empty())
                gray.copyTo(prevGray);
            
            calcOpticalFlowPyrLK(prevGray, gray, points[0], points[1], status, err, winSize,
                                 3, termcrit, 0, 0.001);
            
            updateAllTimedAndDraw(tracking, status, points[0], points[1], removedIndex, lengths, image,numPoints, numberPerTime, dStarList, minDissimilarity, neighbourIndexList);
            
            points[0].resize(numPoints);
            points[1].resize(numPoints);
            timedDataValueResize(tracking,numberPerTime);
            //tracking.resize(numPoints);
            
            timedDataValueResize(entropy, numberPerTime);
            timedDataValueResize(lengths, numberPerTime);
            //timedDataValueResize(minDissimilarity, numberPerTime);
            //timedDataValueResize(fusion, numberPerTime);
            //timedDataValueResize(dStarList, numberPerTime);
            
            //entropy.resize(numPoints);
            //lengths.resize(numPoints);
            
            //dStarList.resize(numPoints);
            //minDissimilarity.resize(numPoints);
            timedDataValueResize(neighbourIndexList,numberPerTime);
            //fusion.resize(numPoints);
            
            //updateNeighbour(neighbourIndexList, removedIndex);
            //dStarListIterativeUpdate(neighbourIndexList, tracking, dStarList);
            //updateDissimilarityIterative(tracking, dStarList, minDissimilarity, neighbourIndexList);
            entropyTimedListUpdate(entropy, lengths, tracking, maxRadius);
            //fusionUpdate(entropy,dStarList,fusion);
            
            c += 1;
            if (numPoints < MAX_COUNT*0.9) {
                needToInit = true;
                cout << "Init Points" << endl;
            }
        }
        
        
        if (c > 3)
        {
            naturalBreaks = JenksNaturalBreak(flatten(entropy),4);
            
            drawBasedOnBreaks(naturalBreaks, flatten(entropy), image, flatten(tracking));
            
        }
        
        imshow("Entropy Based River Segmentation", image);
        //imshow("Mask", mask);
        
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