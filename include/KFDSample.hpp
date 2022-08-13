#include "ORBextractor.h"
#include "pd.hpp"
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

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
    KFDSample(/* args */);
    ~KFDSample();
};

KFDSample::KFDSample(/* args */)
{
}

// KFDSample::~KFDSample()
// {
// }
