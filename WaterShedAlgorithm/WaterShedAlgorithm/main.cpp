#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"

#include <cstdio>
#include <iostream>

using namespace cv;
using namespace std;


#include <vector>
#include <string.h>
#include <map>
#include <fstream>

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

static void help()
{
    cout << "\nThis program demonstrates the famous watershed segmentation algorithm in OpenCV: watershed()\n"
    "Usage:\n"
    "./watershed [image_name -- default is fruits.jpg]\n" << endl;
    
    
    cout << "Hot keys: \n"
    "\tESC - quit the program\n"
    "\tr - restore the original image\n"
    "\tw or SPACE - run watershed segmentation algorithm\n"
    "\t\t(before running it, *roughly* mark the areas to segment on the image)\n"
    "\t  (before that, roughly outline several markers on the image)\n";
}
Mat markerMask, img;
Point prevPt(-1, -1);

static void onMouse( int event, int x, int y, int flags, void* )
{
    if( x < 0 || x >= img.cols || y < 0 || y >= img.rows )
        return;
    if( event == CV_EVENT_LBUTTONUP || !(flags & CV_EVENT_FLAG_LBUTTON) )
        prevPt = Point(-1,-1);
    else if( event == CV_EVENT_LBUTTONDOWN )
        prevPt = Point(x,y);
    else if( event == CV_EVENT_MOUSEMOVE && (flags & CV_EVENT_FLAG_LBUTTON) )
    {
        Point pt(x, y);
        if( prevPt.x < 0 )
            prevPt = pt;
        line( markerMask, prevPt, pt, Scalar::all(255), 5, 8, 0 );
        line( img, prevPt, pt, Scalar::all(255), 5, 8, 0 );
        prevPt = pt;
        imshow("image", img);
    }
}

void generateMask(vector<Point2f> seedPoint, int value){
    Point2f addition (0,0);
    if (value){
        addition = {0,-1};
    }
    else{
        addition = {0,1};
    }
    for (int i = 0; i < seedPoint.size(); i++) {
        line(markerMask, seedPoint[i], seedPoint[i]+addition, Scalar::all(255), 5, 8, 0 );
        line(img, seedPoint[i]+8*addition, seedPoint[i]+10*addition, Scalar::all(255), 5, 8, 0 );
        imshow("image", img);
    }
    
}

int main( int argc, char** argv )
{
//    char* filename ="/Users/student/Desktop/OpenCV/testForShoreline/frame000.jpg";
//    Mat img0 = imread(filename, 1), imgGray;
//    
//    if( img0.empty() )
//    {
//        cout << "Couldn'g open image " << filename << ". Usage: watershed <image_name>\n";
//        return 0;
//    }
//    help();
    
    Mat img0, imgGray;
    vector<Point2f> pointWithValue1, pointWithValue0;
    int i = 0;
    
    namedWindow( "image", 1 );
    
    getFrame(img0, i);
    getPointValue(pointWithValue1, pointWithValue0, i);
    
    img0.copyTo(img);

    cvtColor(img, markerMask, COLOR_BGR2GRAY);
    cvtColor(markerMask, imgGray, COLOR_GRAY2BGR);
    markerMask = Scalar::all(0);
    imshow( "image", img );
    setMouseCallback( "image", onMouse, 0 );
    
    pointWithValue1.insert(pointWithValue1.end(), pointWithValue0.begin(),pointWithValue0.end ());
    
    generateMask(pointWithValue1, 1);
    
    for(;;)
    {
        int c = waitKey(0);
        
        if( (char)c == 27 )
            break;
        
//        if( (char)c == 'r' )
//        {
//            markerMask = Scalar::all(0);
//            img0.copyTo(img);
//            imshow( "image", img );
//        }
        
        if( (char)c == 'w' || (char)c == ' ' )
        {
            int i, j, compCount = 0;
            vector<vector<Point> > contours;
            vector<Vec4i> hierarchy;
            
            findContours(markerMask, contours, hierarchy, CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE);
            
            if( contours.empty() )
                continue;
            Mat markers(markerMask.size(), CV_32S);
            markers = Scalar::all(0);
            int idx = 0;
            for( ; idx >= 0; idx = hierarchy[idx][0], compCount++ )
                drawContours(markers, contours, idx, Scalar::all(compCount+1), -1, 8, hierarchy, INT_MAX);
            
            if( compCount == 0 )
                continue;
            
            vector<Vec3b> colorTab;
            for( i = 0; i < compCount; i++ )
            {
                int b = theRNG().uniform(0, 255);
                int g = theRNG().uniform(0, 255);
                int r = theRNG().uniform(0, 255);
                
                colorTab.push_back(Vec3b((uchar)b, (uchar)g, (uchar)r));
            }
            
            double t = (double)getTickCount();
            watershed( img0, markers );
            t = (double)getTickCount() - t;
            printf( "execution time = %gms\n", t*1000./getTickFrequency() );
            
            Mat wshed(markers.size(), CV_8UC3);
            
            // paint the watershed image
            for( i = 0; i < markers.rows; i++ )
                for( j = 0; j < markers.cols; j++ )
                {
                    int index = markers.at<int>(i,j);
                    if( index == -1 )
                        wshed.at<Vec3b>(i,j) = Vec3b(255,255,255);
                    else if( index <= 0 || index > compCount )
                        wshed.at<Vec3b>(i,j) = Vec3b(0,0,0);
                    else
                        wshed.at<Vec3b>(i,j) = colorTab[index - 1];
                }
            
            wshed = wshed*0.5 + imgGray*0.5;
            imshow( "watershed transform", wshed );
        }
    }
    
    return 0;
}
