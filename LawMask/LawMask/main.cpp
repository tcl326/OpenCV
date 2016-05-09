#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <stdlib.h>
#include <stdio.h>
#include <iostream>
#include <fstream>
#include <math.h>

using namespace cv;
using namespace std;

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
    image = imread(tempFilename, CV_LOAD_IMAGE_COLOR);
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

static void floodFillPostprocess( Mat& img, vector<Point2f>& pointValue1, vector<Point2f>& pointValue0 , const Scalar& colorDiff=Scalar::all(2))
{
    CV_Assert( !img.empty() );
    RNG rng = theRNG();
    Mat mask( img.rows+2, img.cols+2, CV_8UC1, Scalar::all(0) );

    Scalar ocean, land;
    ocean = Scalar(0,0,255);
    land = Scalar(0,255,0);
    for (int i = 0; i < pointValue0.size(); i++) {
        if (pointValue0[i].x > img.cols || pointValue0[i].y > img.rows){
            continue;
        }
        floodFill( img, mask, pointValue0[i], ocean, 0, colorDiff, colorDiff );
    }
    for (int i = 0; i < pointValue1.size(); i++) {
        if (pointValue1[i].x > img.cols || pointValue1[i].y > img.rows){
            continue;
        }
        floodFill( img, mask, pointValue1[i], land, 0, colorDiff, colorDiff );
    }
    
}

void show_result(const cv::Mat& labels, const cv::Mat& centers, int height, int width, const vector<Vec3b>& colorTable, vector<Point2f>& point1, vector<Point2f>& point0)
{
    std::cout << "===\n";
    std::cout << "labels: " << labels.rows << " " << labels.cols << std::endl;
    std::cout << "centers: " << centers.rows << " " << centers.cols << std::endl;
    assert(labels.type() == CV_32SC1);
    assert(centers.type() == CV_32FC1);
    
    cv::Mat rgb_image(height, width, CV_8UC3);
    cv::MatIterator_<cv::Vec3b> rgb_first = rgb_image.begin<cv::Vec3b>();
    cv::MatIterator_<cv::Vec3b> rgb_last = rgb_image.end<cv::Vec3b>();
    cv::MatConstIterator_<int> label_first = labels.begin<int>();
    
//    cv::Mat centers_u8;
//    centers.convertTo(centers_u8, CV_8UC1, 255.0);
//    cv::Mat centers_u8c3 = centers_u8.reshape(3);
    
    while ( rgb_first != rgb_last ) {
        const cv::Vec3b& rgb = (colorTable[*label_first]);
        *rgb_first = rgb;
        ++rgb_first;
        ++label_first;
    }
    Mat final;
    
    //pyrUp(rgb_image, final, Size( rgb_image.cols*2, rgb_image.rows*2 ));
    
    //floodFillPostprocess(rgb_image, point1, point0);
    
    cv::imshow("tmp", rgb_image);
    cv::waitKey();
}

static Vec3b randomColor( RNG& rng )
{
    int icolor = (unsigned) rng;
    return Vec3b( icolor&255, (icolor>>8)&255, (icolor>>16)&255 );
}

void generateColorTable (vector<Vec3b>& colorTable, const int& numberOfColors){
    RNG rng(0xFFFFFFFF);
    for (int i = 0; i < numberOfColors; i++) {
        colorTable.push_back(randomColor(rng));
    }
}

// Generate the 9 Law's Masks and store them in a vector, more about Law Masks be read from here https://courses.cs.washington.edu/courses/cse455/09wi/Lects/lect12.pdf

void generateLawMasks (vector<Mat>& LawMasks){
    Mat L5, E5, S5, R5, W5;
    
    L5 = (Mat_<float>(1,5) << 1,4,6,4,1);
    E5 = (Mat_<float>(1,5) << -1,-2,0,2,1);
    S5 = (Mat_<float>(1,5) << -1,0,2,0,-1);
    R5 = (Mat_<float>(1,5) << 1,-4,6,-4,1);
    W5 = (Mat_<float>(1,5) << -1,2,0,-2,1);
    
    
    Mat L5E5, L5S5, L5R5, E5E5, E5L5, E5S5, E5R5, S5S5, S5L5, S5R5, S5E5, R5R5, R5L5, R5E5, R5S5, M1, M2, M3, M4, M5 ,M6;
    
    L5E5 = L5.t()*E5;
    E5L5 = L5E5.t();
    L5S5 = L5.t()*S5;
    S5L5 = L5S5.t();
    L5R5 = L5.t()*R5;
    R5L5 = L5R5.t();
    E5E5 = E5.t()*E5;
    E5S5 = E5.t()*S5;
    S5E5 = E5S5.t();
    E5R5 = E5.t()*R5;
    R5E5 = E5R5.t();
    S5S5 = S5.t()*S5;
    S5R5 = S5.t()*R5;
    R5S5 = S5R5.t();
    R5R5 = R5.t()*R5;
    
    addWeighted(L5E5, 0.5, E5L5, 0.5, 0, M1);
    LawMasks.push_back(M1);
    addWeighted(L5S5, 0.5, S5L5, 0.5, 0, M2);
    LawMasks.push_back(M2);
    addWeighted(L5R5, 0.5, R5L5, 0.5, 0, M3);
    LawMasks.push_back(M3);
    addWeighted(E5S5, 0.5, S5E5, 0.5, 0, M4);
    LawMasks.push_back(M4);
    addWeighted(E5R5, 0.5, R5E5, 0.5, 0, M5);
    LawMasks.push_back(M5);
    addWeighted(S5R5, 0.5, R5R5, 0.5, 0, M6);
    LawMasks.push_back(M6);
//    LawMasks.push_back(L5E5);
//    LawMasks.push_back(E5L5);
//    LawMasks.push_back(L5S5);
//    LawMasks.push_back(S5L5);
//    LawMasks.push_back(L5R5);
//    LawMasks.push_back(R5L5);
//    LawMasks.push_back(E5S5);
//    LawMasks.push_back(S5E5);
//    LawMasks.push_back(E5R5);
//    LawMasks.push_back(R5E5);
//    LawMasks.push_back(S5R5);
//    LawMasks.push_back(R5R5);
    
    
    LawMasks.push_back(S5S5);
    LawMasks.push_back(E5E5);
    LawMasks.push_back(R5R5);
}
// Following the algorithm outlined in https://courses.cs.washington.edu/courses/cse455/09wi/Lects/lect12.pdf, the Law Mask texture Energy of the input greyImage is computed and stored in a vector.
void generateLawMaskTextureEnergy (const vector<Mat>&  lawMasks, const Mat& greyImage, vector<Mat>& textureEnergy){
    
    Mat averageIntensity, intensityRemoved;
    
    double delta;
    int ddepth;
    int kernel_size;
    Point anchor;
    
    anchor = Point( -1, -1 );
    delta = 0;
    ddepth = -1;
    
    //Compute the average intensity of the image in a 15 by 15 neighbour and remove it from the image
    
    boxFilter(greyImage, averageIntensity, CV_8UC1, Size (15,15), anchor);
    
    intensityRemoved = greyImage - averageIntensity;
    
    int ind = 0;
    while( ind < lawMasks.size() )
    {
        Mat result, sumResult, lawMask;
        
        lawMask = lawMasks[ind];
        
        //Convolute the individual Law Mask with the image with intensity removed
        filter2D(intensityRemoved, result, CV_32F , lawMask, anchor, delta, BORDER_DEFAULT );
        
        //Sum the convolute value in a 15 by 15 neighbour
        boxFilter(result, sumResult, CV_32F, Size (15,15), anchor, false, BORDER_DEFAULT);

        textureEnergy.push_back(sumResult);
        ind++;
    }
}

// create a mat vector that contains the c1, c2, and c3 color space of the BGR image
// c1c2c3 is an illumination (shadow) invariant colorspace based in this paper http://digital.csic.es/bitstream/10261/30345/1/Robust%20color%20contour.pdf
void convertBGRtoC1C2C3 (const Mat& bgrColorImage, vector<Mat>& c1c2c3){
    vector<Mat> bgrChannels, maxGBRBRG;
    
    split(bgrColorImage, bgrChannels);
    
    //Create maxGB, maxRB, maxRG lookup matrix;
    maxGBRBRG.push_back(max(bgrChannels[1], bgrChannels[0]));//, maxGB);
    maxGBRBRG.push_back(max(bgrChannels[2], bgrChannels[0]));//, maxRB);
    maxGBRBRG.push_back(max(bgrChannels[2], bgrChannels[1]));//, maxRG);
    
    MatIterator_<float> it, end;
    MatIterator_<uchar> maxIt, maxEnd, frameIt, frameEnd;
    
    for (int i = 0; i < bgrChannels.size(); i++) {
        Mat max = maxGBRBRG[i];
        Mat c(max.rows,max.cols,CV_32FC1);
        it = c.begin<float>();
        maxIt = max.begin<uchar>();
        frameIt = bgrChannels[i].begin<uchar>();
        
        for (; it != c.end<float>(); ) {
            
            *it = atan2(float(*frameIt),float( *maxIt));
            it++;
            maxIt++;
            frameIt++;
            
        }
        c1c2c3.push_back(c);
    }
}

int main ()
{
    /// Declare variables
    vector<Mat> dsts;
    Mat dst, integral, colorSrc, frame, smallFrame;
    int frameNumber = 0;
    vector<Mat> kernels;
    
    //get a single frame
    getFrame(frame, frameNumber);
    
    //create Law Masks
    generateLawMasks(kernels);
    
    cv::Mat greyMat, colorMat;
    cv::cvtColor(frame, greyMat, CV_BGR2GRAY);
    
    if( !frame.data )
    { return -1; }
    
    //calculate the Texture Energy using the grey image
    generateLawMaskTextureEnergy(kernels,greyMat,dsts);
    
    //convert the image from BGR colorspace to c1c2c3 colorspace
    convertBGRtoC1C2C3(frame, dsts);
    
    // create a matrix that contains only the height (row) info of the matrix.
    Mat height(greyMat.rows,greyMat.cols,CV_32FC1);
    MatIterator_<float> heightIt;
    
    heightIt = height.begin<float>();
    
    for (int c = 0; heightIt != height.end<float>(); heightIt++, c++) {
        *heightIt = int(c)/height.cols;
        //cout << *heightIt << endl;
    }
    dst.push_back(height);
    
    //reshape the matrix to be used by the openCV kmeans function
    vector<Mat> points;
    
    for (int i = 0; i < dsts.size(); i++) {
        Mat reshaped;
        reshaped = dsts[i].reshape(1,dsts[i].rows*dsts[i].cols);
        points.push_back(reshaped);
    }
    
    Mat labels, centers, sample;
    
    hconcat(points, sample);
    
    //kmeans segmentation
    cv::TermCriteria criteria {cv::TermCriteria::COUNT, 100, 1};
    
    kmeans(sample, 4, labels, criteria, 1, KMEANS_RANDOM_CENTERS, centers);
    
    
    //demonstrate the result
    vector<Vec3b> colorTable;
    generateColorTable(colorTable, 32);
    
    vector<Point2f> points1, points0;
    
    getPointValue(points1, points0, frameNumber);
    
    show_result(labels, centers, greyMat.rows, greyMat.cols, colorTable, points1, points0);
    
    
    return 0;
}