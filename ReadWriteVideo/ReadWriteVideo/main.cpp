#include <iostream>	// for standard I/O
#include <string>   // for strings
#include <fstream>
#include "opencv2/video/tracking.hpp"
#include <opencv2/core/core.hpp>        // Basic OpenCV structures (cv::Mat)
#include <opencv2/highgui/highgui.hpp>  // Video write
#include <opencv2/imgproc/imgproc.hpp>
using namespace std;
using namespace cv;

static void help()
{
    cout
    << "------------------------------------------------------------------------------" << endl
    << "This program shows how to write video files."                                   << endl
    << "You can extract the R or G or B color channel of the input video."              << endl
    << "Usage:"                                                                         << endl
    << "./video-write inputvideoName [ R | G | B] [Y | N]"                              << endl
    << "------------------------------------------------------------------------------" << endl
    << endl;
}

inline bool isFlowCorrect(Point2f u)
{
    return !cvIsNaN(u.x) && !cvIsNaN(u.y) && fabs(u.x) < 1e9 && fabs(u.y) < 1e9;
}

static Vec3b computeColor(float fx, float fy)
{
    static bool first = true;
    
    // relative lengths of color transitions:
    // these are chosen based on perceptual similarity
    // (e.g. one can distinguish more shades between red and yellow
    //  than between yellow and green)
    const int RY = 15;
    const int YG = 6;
    const int GC = 4;
    const int CB = 11;
    const int BM = 13;
    const int MR = 6;
    const int NCOLS = RY + YG + GC + CB + BM + MR;
    static Vec3i colorWheel[NCOLS];
    
    if (first)
    {
        int k = 0;
        
        for (int i = 0; i < RY; ++i, ++k)
            colorWheel[k] = Vec3i(255, 255 * i / RY, 0);
        
        for (int i = 0; i < YG; ++i, ++k)
            colorWheel[k] = Vec3i(255 - 255 * i / YG, 255, 0);
        
        for (int i = 0; i < GC; ++i, ++k)
            colorWheel[k] = Vec3i(0, 255, 255 * i / GC);
        
        for (int i = 0; i < CB; ++i, ++k)
            colorWheel[k] = Vec3i(0, 255 - 255 * i / CB, 255);
        
        for (int i = 0; i < BM; ++i, ++k)
            colorWheel[k] = Vec3i(255 * i / BM, 0, 255);
        
        for (int i = 0; i < MR; ++i, ++k)
            colorWheel[k] = Vec3i(255, 0, 255 - 255 * i / MR);
        
        first = false;
    }
    
    const float rad = sqrt(fx * fx + fy * fy);
    const float a = atan2(-fy, -fx) / (float)CV_PI;
    
    const float fk = (a + 1.0f) / 2.0f * (NCOLS - 1);
    const int k0 = static_cast<int>(fk);
    const int k1 = (k0 + 1) % NCOLS;
    const float f = fk - k0;
    
    Vec3b pix;
    
    for (int b = 0; b < 3; b++)
    {
        const float col0 = colorWheel[k0][b] / 255.f;
        const float col1 = colorWheel[k1][b] / 255.f;
        
        float col = (1 - f) * col0 + f * col1;
        
        if (rad <= 1)
            col = 1 - rad * (1 - col); // increase saturation with radius
        else
            col *= .75; // out of range
        
        pix[2 - b] = static_cast<uchar>(255.f * col);
    }
    
    return pix;
}

static void drawOpticalFlow(const Mat_<Point2f>& flow, Mat& dst, float maxmotion = -1)
{
    dst.create(flow.size(), CV_8UC3);
    dst.setTo(Scalar::all(0));
    
    // determine motion range:
    float maxrad = maxmotion;
    
    if (maxmotion <= 0)
    {
        maxrad = 1;
        for (int y = 0; y < flow.rows; ++y)
        {
            for (int x = 0; x < flow.cols; ++x)
            {
                Point2f u = flow(y, x);
                
                if (!isFlowCorrect(u))
                    continue;
                
                maxrad = max(maxrad, sqrt(u.x * u.x + u.y * u.y));
            }
        }
    }
    
    for (int y = 0; y < flow.rows; ++y)
    {
        for (int x = 0; x < flow.cols; ++x)
        {
            Point2f u = flow(y, x);
            
            if (isFlowCorrect(u))
                dst.at<Vec3b>(y, x) = computeColor(u.x / maxrad, u.y / maxrad);
        }
    }
}

// binary file format for flow data specified here:
// http://vision.middlebury.edu/flow/data/
static void writeOpticalFlowToFile(const Mat_<Point2f>& flow, const string& fileName)
{
    static const char FLO_TAG_STRING[] = "PIEH";
    
    ofstream file(fileName.c_str(), ios_base::binary);
    
    file << FLO_TAG_STRING;
    
    file.write((const char*) &flow.cols, sizeof(int));
    file.write((const char*) &flow.rows, sizeof(int));
    
    for (int i = 0; i < flow.rows; ++i)
    {
        for (int j = 0; j < flow.cols; ++j)
        {
            const Point2f u = flow(i, j);
            
            file.write((const char*) &u.x, sizeof(float));
            file.write((const char*) &u.y, sizeof(float));
        }
    }
}

int main(int argc, char *argv[])
{
    help();
    argc = 4;
    argv[1] = "/Users/student/Desktop/CV Canal/GP058145.m4v";
    argv[3][0] = 'Y';
    argv[2][0] = 'R';
    
    if (argc != 4)
    {
        cout << "Not enough parameters" << endl;
        return -1;
    }
    
    const string source      = argv[1];           // the source file name
    const bool askOutputType = argv[3][0] =='Y';  // If false it will use the inputs codec type
    
    VideoCapture inputVideo(source);              // Open input
    if (!inputVideo.isOpened())
    {
        cout  << "Could not open the input video: " << source << endl;
        return -1;
    }
    
    string::size_type pAt = source.find_last_of('.');                  // Find extension point
    const string NAME ="/Users/student/Desktop/Output.MP4/";   // Form the new name with container
    int ex = static_cast<int>(inputVideo.get(CV_CAP_PROP_FOURCC));     // Get Codec Type- Int form
    
    // Transform from int to char via Bitwise operators
    char EXT[] = {(char)(ex & 0XFF) , (char)((ex & 0XFF00) >> 8),(char)((ex & 0XFF0000) >> 16),(char)((ex & 0XFF000000) >> 24), 0};
    
    Size S = Size((int) inputVideo.get(CV_CAP_PROP_FRAME_WIDTH),    // Acquire input size
                  (int) inputVideo.get(CV_CAP_PROP_FRAME_HEIGHT));
    
    VideoWriter outputVideo;                                        // Open the output
    if (askOutputType)
        outputVideo.open(NAME, CV_FOURCC('8','B','P','S'), 30, S);
    else
        outputVideo.open(NAME, CV_FOURCC('8','B','P','S'), 30, S);
    
    if (!outputVideo.isOpened())
    {
        cout  << "Could not open the output video for write: " << source << endl;
        return -1;
    }
    
    cout << "Input frame resolution: Width=" << S.width << "  Height=" << S.height
    << " of nr#: " << inputVideo.get(CV_CAP_PROP_FRAME_COUNT) << endl;
    cout << "Input codec type: " << EXT << endl;
    
    int channel = 2; // Select the channel to save
    switch(argv[2][0])
    {
        case 'R' : channel = 2; break;
        case 'G' : channel = 1; break;
        case 'B' : channel = 0; break;
    }
    Mat src, res, src1, greySrc, greySrc1;
    vector<Mat> spl;
    int c = 0;
    for(;;) //Show the image captured in the window and repeat
    {
        inputVideo >> src1;
        inputVideo >> src;              // read
        cout << inputVideo.get(CV_CAP_PROP_POS_MSEC);
        cout << "/n";   
        if (inputVideo.get(CV_CAP_PROP_POS_MSEC) > 10000) break;         // check if at end
        /*
        split(src, spl);                // process - extract only the correct channel
        for (int i =0; i < 3; ++i)
            if (i != channel)
                spl[i] = Mat::zeros(S, spl[0].type());
        merge(spl, res);*/
        
        //outputVideo.write(res); //save or
        cvtColor(src, greySrc, CV_BGR2GRAY);
        cvtColor(src1, greySrc1, CV_BGR2GRAY);
       
        Mat_<Point2f> flow;
        Ptr<DenseOpticalFlow> tvl1 = createOptFlow_DualTVL1();
        
        const double start = (double)getTickCount();
        tvl1->calc(greySrc1, greySrc, flow);
        const double timeSec = (getTickCount() - start) / getTickFrequency();
        cout << "calcOpticalFlowDual_TVL1 : " << timeSec << " sec" << endl;
        
        Mat out;
        drawOpticalFlow(flow, out);
        imshow("Grey1",greySrc);
        imshow("Grey2",greySrc1);
        imshow("Flow", out);
        outputVideo << out;
    }
    
    cout << "Finished writing" << endl;
    return 0;
}
