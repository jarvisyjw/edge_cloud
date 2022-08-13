#include "ORBextractor.h"
#include "pd.hpp"
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
// #include <string>
#include <iostream>

using namespace cv;
using namespace std;

class KFDSample
{
private:
    // Tracking points
    vector<Point2f> old, next;
    // ORBextractor
    ORB_SLAM2::ORBextractor *mpORBextractor;
    // ORBextractor Parameters
    float scaleFactor = 1.2;
    int nfeatures = 2000, nlevels = 8, iniThFAST = 20, minThFAST = 7;
    // Tracking Keypoints and Descriptors
    vector<KeyPoint> mvKeys;
    Mat mDescriptors;
    // Image
    Mat frame1, imprvs;
    Mat frame2, imnext;
    // PD controller
    PD *mpPDcontroller;
    
    

public:
    // Constructor Initialization
    KFDSample( int nfeatures, int nlevels, int iniThFAST, int minThFAST, Mat &Frame, float TimeStamp);
    // Destructor
    ~KFDSample();
};

KFDSample::KFDSample(int nfeatures, int nlevels, int iniThFAST, int minThFAST, Mat &Frame, float TimeStamp)
{
    this->nfeatures = nfeatures;
    this->minThFAST = minThFAST;
    this->nlevels = nlevels;
    this->iniThFAST = iniThFAST;
    this->scaleFactor = scaleFactor;
    this->mpORBextractor = new ORB_SLAM2::ORBextractor(nfeatures, 
    scaleFactor, nlevels, iniThFAST, minThFAST);
    cout << endl  << "ORB Extractor Parameters: " << endl;
    cout << "- Number of Features: " << nfeatures << endl;
    cout << "- Scale Levels: " << nlevels << endl;
    cout << "- Scale Factor: " << scaleFactor << endl;
    cout << "- Initial Fast Threshold: " << iniThFAST << endl;
    cout << "- Minimum Fast Threshold: " << minThFAST << endl;
}

void InitORBextractor(const int nfeatures, const float scaleFactor, const int nlevels, const int iniThFAST, const int minThFAST);
