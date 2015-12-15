#include "opencv2/video/tracking.hpp"
// Video write
#include <opencv2/imgproc/imgproc.hpp>

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

Point2f point;

// calculate the minimum radius that encloses all the points using the MiniBall Software(V3.0)
double calcMinRadius(vector<Point2f> points)
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
double calcEntropy(double length, vector<Point2f> points, float &maxRadius)
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
double calcEntropyUsingMiniballRadius (double length, vector<Point2f> points, float &maxRadius)
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


int main(int argc, const char* argv[])
{

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
                
                points[1][k] = points[1][i];
                tracking[k] = tracking[i];
                lengths[k] = lengths[i];
                tracking[k].push_back(points[1][i]);
                
                if (c == 0)
                    continue;
                lengths[k] += calcLength(tracking[k].rbegin()[0], tracking[k].rbegin()[1]);
                entropy[k] = calcEntropyUsingMiniballRadius(lengths[k], tracking[k], maxRadius);

                k++;
            }
            
            // Use Jenk's Natural Break algorithm (1D segmentation) to cluster features based on their
            // entropy value.
            // The code used here is a slight modified version of the code on http://svn.objectvision.nl/public/geodms/trunk/tic/tst/src/JenksTest.cpp
            
            if (c > 3)
            {
                naturalBreaks = JenksNaturalBreak(entropy,3);
                cout << naturalBreaks[0] << ", " << naturalBreaks[1] << ", "<< naturalBreaks[2]<< "," << naturalBreaks[3] << endl;
                
                for (int p = 0; p<entropy.size(); ++p)
                {
                    if (entropy[p] < naturalBreaks[0])
                    {
                        // Draw Blue Circles
                        circle( image, tracking[p].rbegin()[0], 3, Scalar(255,0,0), -1, 8);
                    }
                    else if (entropy[p] < naturalBreaks[1])
                    {
                        // Draw Green Circles
                        circle( image, tracking[p].rbegin()[0], 3, Scalar(0,255,0), -1, 8);
                    }
                    else if (entropy[p] < naturalBreaks[2])
                    {
                        //Draw Red Circles
                        circle( image, tracking[p].rbegin()[0], 3, Scalar(0,0,255), -1, 8);
                    }

                    //else if (entropy[p] < naturalBreaks[3])
                    //{
                        //Draw Purple Circles
                    //    circle( image, tracking[p].rbegin()[0], 3, Scalar(255,0,255), -1, 8);
                    //}
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

            points[1].resize(k);
            tracking.resize(k);
            entropy.resize(k);
            lengths.resize(k);
            c += 1;
        }
        
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