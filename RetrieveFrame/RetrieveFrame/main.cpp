#include <iostream>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include "opencv2/imgproc/imgproc.hpp"

using namespace std;
using namespace cv;

int main(int argc, const char* argv[]) {
    
    VideoCapture cap("/Users/student/Desktop/CV Canal/GP058145.MP4");
    if(!cap.isOpened())
    {
        cout<<"can't open video file"<<endl;
        return -1;
    }
    namedWindow("Myvideo",CV_WINDOW_AUTOSIZE);
    
    Mat frame;
    cap.read(frame);
    cap.read(frame);
    Mat src = frame;
    Mat dst, tmp;
    dst = src;
    pyrDown( src, src, Size( tmp.cols/2, tmp.rows/2 ) );
    pyrDown( src, src, Size( tmp.cols/2, tmp.rows/2 ) );
    //pyrDown( src, src, Size( tmp.cols/2, tmp.rows/2 ) );
    //pyrUp( src, src, Size( tmp.cols*2, tmp.rows*2 ) );
    imwrite( "/Users/student/Desktop/CV Canal/frame2.jpg", src );
    imshow("Myvideo",src);
    waitKey(0);
    return 0;
}