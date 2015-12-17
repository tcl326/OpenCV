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

using namespace std;
using namespace cv;
using namespace cv::videostab;

void transformPerspective (std::vector<cv::Point2f> original, std::vector<cv::Point2f>& dst, cv::Mat m)
{
    Point2f temp;
    float x;
    float y;
    for (int i = 0; i<original.size(); i++)
    {
        x = original[i].x;
        y = original[i].y;
        cout << x << "," << y << endl;
        cout << m.at<float>(0,0);
        temp = Point2f((m.at<float>(0,0)*x+m.at<float>(0,1)*y+m.at<float>(0, 2))/(m.at<float>(2,0)*x+m.at<float>(2,1)*y+m.at<float>(2, 2)),(m.at<float>(1,0)*x+m.at<float>(1,1)*y+m.at<float>(1, 2))/(m.at<float>(2,0)*x+m.at<float>(2,1)*y+m.at<float>(2, 2)));
        cout << temp;
        dst.push_back(temp);
    }
}

int addition (int a, int b)
{
    int r;
    r=a+b;
    return r;
}


int main(){
    vector<Point2f> ax, by,cz;
    ax.push_back(Point2f(2,2));
    ax.push_back(Point2f(2,3));
    ax.push_back(Point2f(2,4));
    by.push_back(Point2f(3,2));
    by.push_back(Point2f(3,3));
    by.push_back(Point2f(3,4));
    Mat t = videostab::estimateGlobalMotionLeastSquares(ax,by,3,0);
    cout << t;
    Mat c;
    Mat cvt(by);
    cout << cvt<<endl;
    Mat cvt1;
    Mat t1;
    //cvt.convertTo(cvt1, CV_32FC2);
    //t.convertTo(t1, CV_32FC2);
    //cout << t.size() << t1.size() << endl;
    //cout << cvt.reshape(1,0).size() << cvt1.t().size() << endl;
    //c = t*cvt.reshape(1,0);
    //Mat dst;
    //warpPerspective(cvt, dst, t, dst.size());
    int z;
    z = addition (5,3);
    cout << z;

    transformPerspective(ax, cz, t);
    cout << cz;
    //cout << dst;
    //cout << c;
    return 0;
}