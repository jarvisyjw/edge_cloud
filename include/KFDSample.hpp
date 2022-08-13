#include "ORBextractor.h"
#include "pd.hpp"
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
// #include <opencv2/features2d/features2d.hpp>
#include <opencv2/video.hpp>
// #include <string>
#include <iostream>

using namespace cv;
using namespace std;

class KFDSample
{
private:
    // Tracking points
    vector<Point2f> old, next, good_old, good_next;
    // ORBextractor
    ORB_SLAM2::ORBextractor *mpORBextractor;
    // ORBextractor Parameters Default for TUM1 Dataset
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
    // Timestamp
    float ltframe = 0;
    // Mean Optical Flow
    float moptf = 0;
    // status
    vector<uchar> status;
    vector<float> err;
    TermCriteria criteria = TermCriteria((TermCriteria::COUNT) + (TermCriteria::EPS), 20, 0.03);
    
    

public:
    // Constructor Initialization
    KFDSample( int nfeatures, int nlevels, int iniThFAST, int minThFAST, float scaleFactor, Mat &Frame, float TimeStamp);
    KFDSample();
    // Destructor
    ~KFDSample();
    // ORBextractor Init
    void InitORBextractor(const int nfeatures, const float scaleFactor, const int nlevels, const int iniThFAST, const int minThFAST){
        this->mpORBextractor = new ORB_SLAM2::ORBextractor(nfeatures, scaleFactor, nlevels, iniThFAST, minThFAST);
        Printinfo (nfeatures, nlevels, iniThFAST, minThFAST, scaleFactor);
    }
    // Init PD keyframe select controller
    void InitPDKFselector(float Kp, float Kd){
        
    }
    // Set Initial Frame
    void SetInitialFrame(const Mat &InputArray){
        this->frame1 = InputArray;
        cvtColor(this->frame1, this->imprvs, CV_BGR2GRAY);
        (*mpORBextractor)(this->imprvs,cv::Mat(),this->mvKeys,this->mDescriptors);
        KeyPoint::convert(mvKeys,this->old);
    }
    // Input Frame
    void SetNextFrame(const Mat &InputArray){
        this->frame2 = InputArray;
        cvtColor(this->frame1, this->imnext, CV_BGR2GRAY);
    }

    // Select Good points After tracking
    void SelectGoodPts(){
    for(uint i = 0; i < this->old.size(); i++)
                {
                    if(this->status[i] == 1) {
                        this->good_next.push_back(this->next[i]);                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                            
                        this->good_old.push_back(this->old[i]);
                    }
                }
    }

    // Calculate Mean Optical FLOW
    void CalcMOptFLOW(const Mat &inputIm, float timeStamp){
        this->SetNextFrame(inputIm);
        calcOpticalFlowPyrLK(this->imprvs, this->imnext, this->old, 
                            this->next, this->status, this->err, Size(31,31), 2, this->criteria);
        this->SelectGoodPts();
        float nkpt = good_next.size();
        cout.precision(16);
        cout << "At Timestamp: " << timeStamp << endl;
        cout.precision(5);
        cout << "Num of Good Points:" << nkpt << endl;
        Calmoptflmag(this->good_old, this->good_next, nkpt);
        float tempN = moptf;
        float pd = mpPDcontroller.update(tempN, tframe-ltframe);
        float TH = tempN + pd;
        cout << "PD Output: " << pd << endl;
        cout << "Select Threshold: " << TH << endl;
        ltframe = tframe;


        
    }   
    
};

KFDSample::KFDSample(int nfeatures, int nlevels, int iniThFAST, int minThFAST, float scaleFactor, Mat &Frame, float TimeStamp)
{
    // Initialize
    this->nfeatures = nfeatures;
    this->minThFAST = minThFAST;
    this->nlevels = nlevels;
    this->iniThFAST = iniThFAST;
    this->scaleFactor = scaleFactor;
    this->InitORBextractor(this->nfeatures,this->scaleFactor,this->nlevels,this->iniThFAST,this->minThFAST);
    this->frame1 = Frame;
    this->ltframe = TimeStamp;

}

KFDSample::KFDSample(){
    InitORBextractor(this->nfeatures,this->scaleFactor,this->nlevels,this->iniThFAST,this->minThFAST);
}

void Printinfo (int nfeatures, int nlevels, int iniThFAST, int minThFAST, float scaleFactor){
        cout << endl  << "ORB Extractor Parameters: " << endl;
        cout << "- Number of Features: " << nfeatures << endl;
        cout << "- Scale Levels: " << nlevels << endl;
        cout << "- Scale Factor: " << scaleFactor << endl;
        cout << "- Initial Fast Threshold: " << iniThFAST << endl;
        cout << "- Minimum Fast Threshold: " << minThFAST << endl;
}

float Calmoptflmag(vector<Point2f> prvs, vector<Point2f> next, const int nkpt){
        float sum = 0;
        float moptf = 0;
        for (uint i=0; i<nkpt; i++){
            float a = sqrt((next[i].x -prvs[i].x)*(next[i].x -prvs[i].x)+(next[i].y -prvs[i].y)*(next[i].y -prvs[i].y));
            sum += a;
        }
        moptf = sum/nkpt;
    // cout.precision(4);
    // cout << "Magnitude of Optical flow:" << moptf << endl;
    return moptf;
}




