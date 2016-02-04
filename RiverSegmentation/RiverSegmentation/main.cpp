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

Point2f point;

/*
// returns the coordinates of the tracked points if only camera motion is taken into account
// uses the points from t1 and t2 to estimate the perspective transformation matrix due to camera motion. Then use that matrix to calculate the supposed t2 only composed of camera motion.
void transformPerspective (std::vector<cv::Point2f> t1, std::vector<cv::Point2f> t2, std::vector<cv::Point2f>& dst, cv::Mat& m)
{
    //cout << t1 << ";" << t2;
    m = videostab::estimateGlobalMotionLeastSquares(t1,t2,3,0);
    Point2f temp;
    float x;
    float y;
    for (int i = 0; i<t1.size(); i++)
    {
        x = t1[i].x;
        y = t1[i].y;
        temp = Point2f((m.at<float>(0,0)*x+m.at<float>(0,1)*y+m.at<float>(0, 2))/(m.at<float>(2,0)*x+m.at<float>(2,1)*y+m.at<float>(2, 2)),(m.at<float>(1,0)*x+m.at<float>(1,1)*y+m.at<float>(1, 2))/(m.at<float>(2,0)*x+m.at<float>(2,1)*y+m.at<float>(2, 2)));
        dst.push_back(temp);
    }
}
 */

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

/* Entropy is calculated using the formula detailed in the paper "Water Detection with Segmentation Guided Dynamic Texture Recognition" by Pedro Santana, Ricardo Mendonca and Jose Barata
    The paper can be found here http://home.iscte-iul.pt/~pfsao/papers/robio_2012.pdf*/

// Calculate entropy using minimum radius found using default OpenCV function, inacurate as any radius
// smaller than 1 will return 1

/*
double calcEntropy(double length, vector<Point2f>& points, float &maxRadius)
{
    if (points.size() < 2){
        return 0;
    }
    float entropy, radius;
    Point2f center;
    cv::minEnclosingCircle(points, center, radius);
    if (radius > maxRadius)
        maxRadius = radius;
    entropy = log(length/(2*radius))/log(points.size()-1)*radius/maxRadius;
    return entropy;
}

// calculate entropy using minimum radius found using Miniball function, very accurate
double calcEntropyUsingMiniballRadius (double length, vector<Point2f>& points, float &maxRadius)
{
    if (points.size() < 2){
        return 0;
    }
    float entropy;
    double radius;
    radius = calcMinRadius(points);
    if (radius > maxRadius)
        maxRadius = radius;
    entropy = log(length/(2*radius))/(log(points.size()-1))*radius/maxRadius;
    return entropy;
}



// calculate the distance between two points
double calcLength(Point2f p1, Point2f p2)
{
    double length;
    Point2f difference;
    difference = p1-p2;
    length = sqrt(pow(difference.x,2)+pow(difference.y,2));
    return length;
}
*/

void calcEntropyUsingMiniballRadiusIterative (double& entropy, double& length, vector<Point2f>& points, float &maxRadius)
{
    if (points.size() < 2){
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
        calcLengthIterative(lengths[i], allTrackers[i].rbegin()[0], allTrackers[i].rbegin()[1]);
        calcEntropyUsingMiniballRadiusIterative(entropy[i], lengths[i], allTrackers[i], maxRadius);
    }
}

//Dissimilarity

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
        double euclideanDis = 0.0;
        size_t n = neighbourIndexList[j].size();
        for (int i=0; i<n; i++){
            euclideanDis += euclideanDistance(allTrackers[j].back(), allTrackers[neighbourIndexList[j][i]].back());
        }
        dStarList[j] += (euclideanDis/n);
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

void fusionUpdate (vector<double>& entropyList, vector<double>& dStarList, vector<double>& fusionList){
    for (int i = 0; i < entropyList.size(); i++){
        fusionList[i] = entropyList[i] * dStarList[i];
    }
}

int main(int argc, const char* argv[])
{

    char* movie;
    movie = "/Users/student/Desktop/GP058145.m4v";
    //GP058145.m4v"
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
    
    namedWindow( "Entropy Based River Segmentation", 1 );
    
    VideoWriter outputVideo;
    bool writeMovie = true; //write the output video to the place defined by filename
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
    vector<double> lengths(MAX_COUNT);
    vector<double> radius(MAX_COUNT);
    vector<Point2f> difference;
    vector<double> entropy(MAX_COUNT);
    vector<double> naturalBreaks;
    
    //Variables for Dissimilarity
    vector< vector<int> > neighbourIndexList(MAX_COUNT);
    vector<double> minDissimilarity (MAX_COUNT);
    vector<double> dissimilarity (MAX_COUNT);
    vector<double> dStarList (MAX_COUNT);
    vector<double> radiuses (MAX_COUNT);
    vector<double> fusion (MAX_COUNT);
    
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
                
                if( !status[i] )
                {
                    removedIndex.push_back(i);
                    continue;
                }
                points[0][k] = points[0][i];
                points[1][k] = points[1][i];
                tracking[k] = tracking[i];
                lengths[k] = lengths[i];
                tracking[k].push_back(points[1][i]);
                dStarList[k] = dStarList[i];
                minDissimilarity[k] = minDissimilarity[i];
                neighbourIndexList[k] = neighbourIndexList[i];
                
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

            /*
            vector< Point2f> dst;
            Mat m;
            
            transformPerspective(points[0], points[1], dst, m);
            
            size_t d;
            for (d = 0; d < dst.size(); d++)
            {
                lengths[d] += calcLength(dst[d], points[1][d]);
            }
            */
            
            points[0].resize(k);
            points[1].resize(k);
            tracking.resize(k);
            entropy.resize(k);
            lengths.resize(k);
            dStarList.resize(k);
            minDissimilarity.resize(k);
            neighbourIndexList.resize(k);
            fusion.resize(k);
            updateNeighbour(neighbourIndexList, removedIndex);
            dStarListIterativeUpdate(neighbourIndexList, tracking, dStarList);
            updateDissimilarityIterative(tracking, dStarList, minDissimilarity, neighbourIndexList);
            entropyListUpdate(entropy, lengths, tracking, maxRadius);
            fusionUpdate(entropy,dStarList,fusion);
            
            c += 1;
        }
        
            //cout << points[0] << ";" << points[1] << ";" << dst << endl;
            
            // Use Jenk's Natural Break algorithm (1D segmentation) to cluster features based on their
            // entropy value.
            // The code used here is a slight modified version of the code on http://svn.objectvision.nl/public/geodms/trunk/tic/tst/src/JenksTest.cpp
            
            if (c > 3)
            {
                naturalBreaks = JenksNaturalBreak(fusion,4);
                
                for (int p = 0; p<fusion.size(); ++p)
                {
                    if (fusion[p] < naturalBreaks[0])
                    {
                        // Draw Blue Circles
                        circle( image, tracking[p].rbegin()[0], 3, Scalar(255,0,0), -1, 8);
                    }
                    else if (fusion[p] < naturalBreaks[1])
                    {
                        // Draw Green Circles
                        circle( image, tracking[p].rbegin()[0], 3, Scalar(0,255,0), -1, 8);
                    }
                    else if (fusion[p] < naturalBreaks[2])
                    {
                        //Draw Red Circles
                        circle( image, tracking[p].rbegin()[0], 3, Scalar(0,0,255), -1, 8);
                    }

                    else if (fusion[p] < naturalBreaks[3])
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

        
            /* //Use Kmeans segmentation to cluster features based on their entropy value.
             if (c>3){
             Mat centers;
             Mat entropyMat(entropy.size(), 1, CV_32FC2);
             for (int row = 0; row<entropyMat.rows; ++row)
             {
             entropyMat.at<double>(row,0) = entropy[row];
             }
             //cout << entropyMat;
             cv::Mat labels;
             kmeans(entropyMat, 3, labels, TermCriteria( CV_TERMCRIT_EPS+CV_TERMCRIT_ITER, 10, 1.0),
             3, KMEANS_PP_CENTERS, centers);
             //cout << labels;
             //cout << centers;
             for(int p = 0; p<labels.rows; ++p)
             {
             if (labels.at<int>(p,0) == 0)
             {
             circle( image, tracking[p].rbegin()[0], 3, Scalar(0,255,255), -1, 8);
             }
             if (labels.at<int>(p,0) == 1)
             {
             circle( image, tracking[p].rbegin()[0], 3, Scalar(0,0,255), -1, 8);
             }
             if (labels.at<int>(p,0) == 2)
             {
             circle( image, tracking[p].rbegin()[0], 3, Scalar(255,0,255), -1, 8);
             }
             if (labels.at<int>(p,0) == 3)
             {
             circle( image, tracking[p].rbegin()[0], 3, Scalar(255,0,0), -1, 8);
             }
             if (labels.at<int>(p,0) == 4)
             {
             circle( image, tracking[p].rbegin()[0], 3, Scalar(255,255,255), -1, 8);
             }
             }
             }
            */


        
        
        needToInit = false;
        imshow("Entropy Based River Segmentation", image);
        
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