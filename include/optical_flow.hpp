#include "ORBextractor.h"
#include <iostream>
#include <sstream>
#include <fstream>
#include <opencv2/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/video.hpp>
#include <string>
#include <chrono>

using namespace ORB_SLAM2;
using namespace cv;
using namespace std;

// Golbal var
// ORBextractor
ORBextractor *mpORBextractor;
// Tracking points
vector<Point2f> p0, p1;
// Load images
vector<string> vstrImageFilenames;
vector<double> vTimestamps;
// ORBextractor Parameters
const float scaleFactor = 1.2;
const int nfeatures = 2000, nlevels = 8, iniThFAST = 20, minThFAST = 7;
// Total Num of Images, num of keypoints
int nImages;
// mean optical flow of images and threshold N
float moptf, N;
// Total Running Time, timeframe
double total_time = 0, tframe = 0, ltframe = 0;
// Input Output Path
string inpath, outpath;
// CMD Argument
bool dense, save, show;
// Image
Mat frame1, imprvs;
Mat frame2, imnext;
Mat keypointstemp;
// Keypoints and Descriptors
std::vector<cv::KeyPoint> mvKeys;
cv::Mat mDescriptors;


//cmd parser
CommandLineParser parser();

// for cmd parser
const char* params
    = "{ h help     |   | print usage }"
      "{ p path     |   | path to dataset }"
      "{ d dense    |   | dense or sparse}"
      "{ N  n       |   | threshold for optical flow}"
      "{ o output   |   | Save DIR}"
      "{ s show     |   | if Show image}"
      ;

// function definition
void LoadImages(const string &strFile, vector<string> &vstrImageFilenames, vector<double> &vTimestamps);
void SaveImages(const string &strFile, const string &filename, const Mat &im);
vector<string> split (string s, string delimiter);
void InitORBextractor(const int nfeatures, const float scaleFactor, const int nlevels, const int iniThFAST, const int minThFAST);
void Calmoptflmag(vector<Point2f> prvs, vector<Point2f> next, const int nkpt);
void cmdpars(CommandLineParser parser);
void ReadImages(const string Image, Mat &OutputArray);
void ReadImages(const string Image, Mat &OutputArray, Mat &OutputArray2);
void SelectGoodPts(vector<Point2f> &good_old, vector<Point2f> &good_new, const vector<uchar> status);













// drawKeypoints(prvs,mvKeys,keypointstemp,Scalar::all(-1),DrawMatchesFlags::DEFAULT);
        // imshow("ORB Keypoints",keypointstemp);
        // waitKey(0);
    // goodFeaturesToTrack(prvs, p0, 100, 0.3, 7, Mat(), 7, false, 0.04);
    
    // cout << "p0 size: " << p0.size() << endl;



    // float sum=0;
            // for (uint i=0; i<nkpt; i++){
            //     sum += sqrt((good_new[i].x -good_old[i].x)*(good_new[i].x -good_old[i].x)+(good_new[i].y -good_old[i].y)*(good_new[i].y -good_old[i].y));
            // }
            // float moptf = sum/nkpt;