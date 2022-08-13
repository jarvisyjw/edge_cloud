#include "ORBextractor.h"
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

using namespace cv;
using namespace std;

class KFDSample
{
private:
    // ORBextractor
    ORB_SLAM2::ORBextractor *mpORBextractor;
    // ORBextractor Parameters
    float scaleFactor = 1.2;
    int nfeatures = 2000, nlevels = 8, iniThFAST = 20, minThFAST = 7;
    // Tracking points
    vector<Point2f> old, next;
    // Last Frame
    Mat mLastFrame, mLastFrameGray;
    // Last Timestamp
    float mLastT;

public:
    KFDSample( int nfeatures, int nlevels, int iniThFAST, int minThFAST, Mat &Frame, float TimeStamp);

    ~KFDSample();
};

KFDSample::KFDSample(/* args */)
{
}

// KFDSample::~KFDSample()
// {
// }
