#include "opencv2/video/tracking.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
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
bool addRemovePt = false;


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
    
    // squared radius
    //std::cout << "Squared radius:\n  ";
//    cout << points << endl;
//    cout << lp << endl;
    //std::cout << mb.squared_radius() <<  std::endl;
    return sqrt(mb.squared_radius());
}

double calcEntropy(double length, vector<Point2f> points, float &maxRadius)
{
    float entropy, radius;
    Point2f center;
    cv::minEnclosingCircle(points, center, radius);
    if (radius > maxRadius)
        maxRadius = radius;
    entropy = log(length/(2*radius))/log(points.size()-1)*radius/maxRadius;
    return entropy;
}

double calcEntropyUsingMiniballRadius (double length, vector<Point2f> points, float &maxRadius)
{
    float entropy;
    double radius;
    radius = calcMinRadius(points);
    if (radius > maxRadius)
        maxRadius = radius;
    entropy = log(length/(2*radius))/log(points.size()-1)*radius/maxRadius;
    return entropy;
}


double calcLength(Point2f p1, Point2f p2)
{
    double length;
    Point2f difference;
    difference = p1-p2;
    length = sqrt(pow(difference.x,2)+pow(difference.y,2));
    return length;
}

int main( int argc, char** argv )
{
    argc = 2;
    argv[1] = "/Users/student/Desktop/CV Canal/GP058145.M4v";
    VideoCapture cap;
    TermCriteria termcrit(CV_TERMCRIT_ITER|CV_TERMCRIT_EPS, 20, 0.03);
    Size subPixWinSize(10,10), winSize(31,31);
    
    const int MAX_COUNT = 500;
    bool needToInit = false;
    bool nightMode = false;
    

    cap.open(argv[1]);
    
    if( !cap.isOpened() )
    {
        cout << "Could not initialize capturing...\n";
        return 0;
    }
    
    namedWindow( "LK Demo", 1 );

    
    Mat gray, prevGray, image;
    vector<Point2f> points[2];
    vector<Point2f> motions;
    vector< vector<Point2f> > tracking(MAX_COUNT+1, vector<Point2f>());
    vector<double> lengths(MAX_COUNT+1);
    vector<double> radius(MAX_COUNT+1);
    vector<Point2f> difference;
    vector<Point2f> difference2;
    vector<double> entropy(MAX_COUNT+1);
    vector<double> naturalBreaks;
    //double distFromOrigin;
    //float maxRadius = 0.0,
    float maxRadius1 = 0.0;
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
            // automatic initialization
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
                if( addRemovePt )
                {
                    if( norm(point - points[1][i]) <= 5 )
                    {
                        addRemovePt = false;
                        continue;
                    }
                }
                
                if( !status[i] )
                    
                    continue;
                
                points[1][k++] = points[1][i];
                tracking[k] = tracking[i+1];
                tracking[k].push_back(points[1][i]);
                
                //cout << tracking[k].rbegin()[0]-tracking[k].rbegin()[1] << " " << tracking[k];
                
                //circle( image, points[1][i], 3, Scalar(0,255,0), -1, 8);
                if (c == 0)
                    continue;
                //difference = {tracking[k].rbegin()[0]-tracking[k].rbegin()[1]};
                /*difference2 = {tracking[k].rbegin()[0] - tracking[k][0]};
                distFromOrigin = sqrt(pow(difference2[0].x,2)+pow(difference2[0].y,2));
                if (distFromOrigin > radius[k])
                    radius[k] = distFromOrigin;
                if (distFromOrigin > maxRadius)
                    maxRadius = distFromOrigin;*/
                //Point2f center;
                //float radius, diameter;
                //minEnclosingCircle(tracking[k], center, radius);
                //diameter = 2*radius;
                //if (radius > maxRadius)
                //    maxRadius = radius;
                lengths[k] += calcLength(tracking[k].rbegin()[0], tracking[k].rbegin()[1]);//sqrt(pow(difference[0].x,2)+pow(difference[0].y,2));
                entropy[k] = calcEntropyUsingMiniballRadius(lengths[k], tracking[k], maxRadius1);
                //cout << entropy[k];
                /*if (entropy[k] <0)
                    continue;
                if (entropy[k] <= 0.025)
                    circle(image, tracking[k].rbegin()[0], 3, Scalar(255,0,0), -1, 8);*/
                //cout << sqrt(pow(difference[0].x,2)+pow(difference[0].y,2)) << ":" << calcLength(tracking[k].rbegin()[0], tracking[k].rbegin()[1]) << endl;
                //cout << "radius " << radius << endl;
                //cout << entropy[k] << "," << endl;
            }
            
            if (c > 3)
            {
                naturalBreaks = JenksNaturalBreak(entropy,10);
                cout << naturalBreaks[0] << ", " << naturalBreaks[1] << "," << naturalBreaks[2]<< endl;
            
            for (int p = 1; p<entropy.size(); ++p)
            {
                if (entropy[p] < 0)
                {
                    circle( image, tracking[p].rbegin()[0], 3, Scalar(0,255,255), -1, 8);
                }
                else if (entropy[p] < naturalBreaks[1])
                {
                    circle( image, tracking[p].rbegin()[0], 3, Scalar(0,0,255), -1, 8);
                }
                else if (entropy[p] < naturalBreaks[2])
                {
                    circle( image, tracking[p].rbegin()[0], 3, Scalar(255,0,255), -1, 8);
                }
                else if (entropy[p] < naturalBreaks[3])
                {
                    circle( image, tracking[p].rbegin()[0], 3, Scalar(255,0,0), -1, 8);
                }
                else if (entropy[p] < naturalBreaks[4])
                {
                    circle( image, tracking[p].rbegin()[0], 3, Scalar(255,255,255), -1, 8);
                }
                else if (entropy[p] <naturalBreaks[5])
                {
                    circle( image, tracking[p].rbegin()[0], 3, Scalar(255,255,0), -1, 8);
                }
                else
                {
                    circle( image, tracking[p].rbegin()[0], 3, Scalar(0,250,0), -1, 8);

                }
   
            }
            }
            /*
            if (c>3){
            Mat centers;
            Mat entropyMat(entropy.size(), 1, CV_32FC2);
            for (int row = 0; row<entropyMat.rows; ++row)
            {
                entropyMat.at<double>(row,0) = entropy[row];
            }
            //cout << entropyMat;
            cv::Mat labels;
            kmeans(entropyMat, 5, labels, TermCriteria( CV_TERMCRIT_EPS+CV_TERMCRIT_ITER, 10, 1.0),
                   3, KMEANS_PP_CENTERS, centers);
            //cout << labels;
            //cout << centers;
                for(int p = 1; p<labels.rows; ++p)
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
            //for (int i = 0; i<entropy.size(); ++i){
            //    cout << entropy[i];
            //}
            //cout << "end";
            //cout << c << ", " << tracking[2].size() << endl;
            points[1].resize(k);
            tracking.resize(k+1);
            entropy.resize(k+1);
            lengths.resize(k+1);
            c += 1;
            //cout << "lengths: " << lengths[1] << endl;
            //cout << "Radius: "<< radius[1]<< endl;
            //cout << "Entropy: " << entropy[1] << endl;
            //cout << tracking[1]<< points[1][0];
        }
        
        if( addRemovePt && points[1].size() < (size_t)MAX_COUNT )
        {
            vector<Point2f> tmp;
            tmp.push_back(point);
            cornerSubPix( gray, tmp, winSize, cvSize(-1,-1), termcrit);
            points[1].push_back(tmp[0]);
            addRemovePt = false;
        }
        
        needToInit = false;
        imshow("LK Demo", image);
        
        char c = (char)waitKey(10);
        if( c == 27 )
            break;
        switch( c )
        {
            case 'r':
                needToInit = true;
                break;
            case 'c':
                points[0].clear();
                points[1].clear();
                break;
            case 'n':
                nightMode = !nightMode;
                break;
        }

        
        std::swap(points[1], points[0]);
        cv::swap(prevGray, gray);
    }
    
    return 0;
}
