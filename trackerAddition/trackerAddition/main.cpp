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
//#include "Miniball.hpp"
#include <math.h>
#include <typeinfo>
//#include "JenksNaturalBreak.hpp"

using namespace cv;
using namespace std;
using namespace cv::videostab;

int main(int argc, const char* argv[])
{
    
    char* movie;
    movie = "/Users/student/Desktop/GP058145.m4v";
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
    
    namedWindow( "Entropy Based River Segmentation", 1 );
    
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
    vector<Point2f> pointsInit;
    vector<Point2f> motions;
    vector< vector<Point2f> > tracking(MAX_COUNT, vector<Point2f>());
    /*
    vector<double> lengths(MAX_COUNT);
    vector<double> radius(MAX_COUNT);
    vector<Point2f> difference;
    vector<double> entropy(MAX_COUNT);
    vector<double> naturalBreaks;
     */
    
    //Variables for Dissimilarity
    /*
    vector< vector<int> > neighbourIndexList(MAX_COUNT);
    vector<double> minDissimilarity (MAX_COUNT);
    vector<double> dissimilarity (MAX_COUNT);
    vector<double> dStarList (MAX_COUNT);
    vector<double> radiuses (MAX_COUNT);
    vector<double> fusion (MAX_COUNT);
     */
    
    float maxRadius = 0.0;
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
            // Using goodFeaturesToTrack function to automatically find 500 features to track
            goodFeaturesToTrack(gray, pointsInit, MAX_COUNT-numPoints, 0.01, 10, Mat(), 3, 0, 0.04);
            cornerSubPix(gray, pointsInit, subPixWinSize, Size(-1,-1), termcrit);
            addRemovePt = false;
            for (int i = 0; i < pointsInit.size(); i++) {
                points[1].push_back( pointsInit[i]);
            }
            cout << pointsInit.size() << endl;
            cout << numPoints << endl;
            numPoints = points[1].size();
            cout << numPoints << endl;
            tracking.resize(numPoints);
            //findNeighbourIndexList(neighbourIndexList, points[1]);
            cout << "initiating points" << endl;
            needToInit = false;
            std::swap(points[1], points[0]);
        }
        //if (points[0].empty()){
        //    cout << "empty";
       // }
        
        if( !points[0].empty() )
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
                //lengths[k] = lengths[i];
                tracking[k].push_back(points[1][i]);
                //dStarList[k] = dStarList[i];
                //minDissimilarity[k] = minDissimilarity[i];
                //neighbourIndexList[k] = neighbourIndexList[i];
                
                /*
                 if (c != 0)
                 {
                 lengths[k] += calcLength(tracking[k].rbegin()[0], tracking[k].rbegin()[1]);
                 entropy[k] = calcEntropyUsingMiniballRadius(lengths[k], tracking[k], maxRadius);
                 }
                 */
                
                k++;
                circle( image, points[1][i], 3, Scalar(0,255,0), -1, 8);
            }
            numPoints = k;
            
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
            
            points[0].resize(numPoints);
            points[1].resize(numPoints);
            tracking.resize(numPoints);
            /*
            entropy.resize(k);
            lengths.resize(k);
            dStarList.resize(k);
            minDissimilarity.resize(k);
            neighbourIndexList.resize(k);
            fusion.resize(k);
             */
            //updateNeighbour(neighbourIndexList, removedIndex);
            //dStarListIterativeUpdate(neighbourIndexList, tracking, dStarList);
            //updateDissimilarityIterative(tracking, dStarList, minDissimilarity, neighbourIndexList);
            //entropyListUpdate(entropy, lengths, tracking, maxRadius);
            //fusionUpdate(entropy,dStarList,fusion);
            
            c += 1;
            if (numPoints < 450) {
                needToInit = true;
                cout << "Init Points" << endl;
            }
        }
        
        
      
        
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