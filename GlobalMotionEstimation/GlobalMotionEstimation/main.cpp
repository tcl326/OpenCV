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

void transformPerspective (std::vector<cv::Point2f> original, std::vector<cv::Point2f> transformed, std::vector<cv::Point2f>& dst, cv::Mat& m)
{
    m = videostab::estimateGlobalMotionLeastSquares(original,transformed,3,0);
    Point2f temp;
    float x;
    float y;
    for (int i = 0; i<original.size(); i++)
    {
        x = original[i].x;
        y = original[i].y;
        temp = Point2f((m.at<float>(0,0)*x+m.at<float>(0,1)*y+m.at<float>(0, 2))/(m.at<float>(2,0)*x+m.at<float>(2,1)*y+m.at<float>(2, 2)),(m.at<float>(1,0)*x+m.at<float>(1,1)*y+m.at<float>(1, 2))/(m.at<float>(2,0)*x+m.at<float>(2,1)*y+m.at<float>(2, 2)));
        dst.push_back(temp);
    }
}



int main(){
    vector<Point2f> ax, by,cz;
    ax.push_back(Point2f(2,2));
    ax.push_back(Point2f(2,3));
    ax.push_back(Point2f(2,4));
    by.push_back(Point2f(3,2));
    by.push_back(Point2f(3,3));
    by.push_back(Point2f(3,4));
    Mat t;
    transformPerspective(ax,by, cz, t);
    cout << cz;

    return 0;
}