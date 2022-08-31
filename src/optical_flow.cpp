#include "ORBextractor.h"
#include "optical_flow.hpp"
#include "pd.hpp"
//run time clock
using std::chrono::high_resolution_clock;
using std::chrono::duration_cast;
using std::chrono::duration;
using std::chrono::milliseconds;

int main( int argc, char* argv[])
{
    
    //cmd parser
    CommandLineParser parser(argc, argv, params);
    // parse the cmd input
    cmdpars(parser);
    // load image from dataset
    LoadImages(inpath+"/rgb.txt", vstrImageFilenames, vTimestamps);
    // create ORBextractor
    InitORBextractor(nfeatures,scaleFactor,nlevels,iniThFAST,minThFAST);
    // create PD controller
    PD keyselect = PD(0.8,0.08);
    keyselect.setSetpoint(N);
    int nImg = 0;
    // Start the Loop, in LoadImages, the first image is retrived and saved (Optional)
    // If Sparse, extract keypoints and descriptor from the first image
    if (!dense){
        // Extract keypoints and descriptor
        (*mpORBextractor)(imprvs,cv::Mat(),mvKeys,mDescriptors);
        KeyPoint::convert(mvKeys,p0);
        if (show){
                // Mat nkeypointstemp;
                drawKeypoints(imprvs,mvKeys,keypointstemp,Scalar::all(-1),DrawMatchesFlags::DEFAULT);
                imshow("ORB Keypoints",keypointstemp);
                waitKey(20);
        }
    }

    // Start the Loop
    for (int ni = 1; ni < nImages; ni++){
        //Read Next image
        ReadImages(vstrImageFilenames[ni],frame2, imnext);
        if (frame2.empty())
            break;
        // insert time beacon
        auto t1 = high_resolution_clock::now();
        if(dense){
            Mat flow(imprvs.size(), CV_32FC2);
            // Dense Optical flow
            calcOpticalFlowFarneback(imprvs, imnext, flow, 0.5, 3, 15, 3, 5, 1.2, 0);
            // Calculate magnitude
            Mat flow_parts[2];
            split(flow, flow_parts);
            Mat magnitude, angle, magn_norm;
            cartToPolar(flow_parts[0], flow_parts[1], magnitude, angle, true);
            cv::Scalar tempVal = cv::mean( magnitude );
            moptf = tempVal.val[0];
            cout << "Magnitude of Mean Optical flow:" << moptf << endl;
            // Visualization
            if (show){
                Mat nkeypointstemp;
                drawKeypoints(imprvs,mvKeys,nkeypointstemp,Scalar::all(-1),DrawMatchesFlags::DEFAULT);
                imshow("ORB Feature", nkeypointstemp);
                // imshow("Prvs",imprvs);
                // imshow("Next", imnext);
                waitKey(20);
            }
            // Save Image
            if ((moptf > N) && save){
                SaveImages(outpath, vstrImageFilenames[ni], frame2);
            }
        }
        else{
                // Declare Var
                int nkpt = 0;
                vector<uchar> status;
                vector<float> err;
                TermCriteria criteria = TermCriteria((TermCriteria::COUNT) + (TermCriteria::EPS), 20, 0.03);

                // Track and Calculate the Optical Flow
                calcOpticalFlowPyrLK(imprvs, imnext, p0, p1, status, err, Size(31,31), 2, criteria);    
                vector<Point2f> good_new;
                vector<Point2f> good_old;

                // Select good points
                SelectGoodPts(good_old,good_new,status);
                nkpt = good_new.size();
                tframe = vTimestamps[ni];
                cout.precision(16);
                cout << "At Timestamp: " << tframe << endl;
                cout.precision(5);
                cout << "Num of Good Points:" << nkpt << endl;

                // Calculate Mean Optical Flow
                Calmoptflmag(good_old, good_new, nkpt);

                // Display
                if (show){
                vector<KeyPoint> keys;
                KeyPoint::convert(good_new, keys);
                drawKeypoints(imnext,keys,keypointstemp,Scalar::all(-1),DrawMatchesFlags::DEFAULT);
                imshow("ORB Keypoints",keypointstemp);
                waitKey(20);
                }

                // Thresholding
                // float tempN = moptf;
                // float pd = keyselect.update(tempN, tframe-ltframe);
                // float TH = tempN + pd;
                // cout << "PD Output: " << pd << endl;
                // cout << "Select Threshold: " << TH << endl;
                // ltframe = tframe;

                if (moptf > N){
                    //extract new features
                    nImg++;
                    cout << "New Frame Extracted" << endl;
                    (*mpORBextractor)(imnext,cv::Mat(),mvKeys,mDescriptors);
                    KeyPoint::convert(mvKeys,p0);
                    if(save){
                        SaveImages(outpath, vstrImageFilenames[ni], frame2);
                    }}       
                else{
                    p0 = good_new;
                    }
            }
        // Time Beacon
        auto t2 = high_resolution_clock::now();
        imprvs = imnext.clone(); //remember to clone!!!!
        auto ms_int = duration_cast<milliseconds>(t2 - t1);
        duration<double, std::milli> ms_double = t2 - t1;
        total_time += ms_double.count();
    }
    double average_time = total_time / nImages ;
    cout << "Average excution time of algorithm is " << average_time << "ms." << endl;
    cout << "Total Selected Keyframes is " << nImg << endl;
}

// Adopted from ORB-SLAM2
void LoadImages(const string &strFile, vector<string> &vstrImageFilenames, vector<double> &vTimestamps)
{
    ifstream f;
    f.open(strFile.c_str());

    // skip first three lines
    string s0;
    getline(f,s0);
    getline(f,s0);
    getline(f,s0);

    while(!f.eof())
    {
        string s;
        getline(f,s);
        if(!s.empty())
        {
            stringstream ss;
            ss << s;
            double t;
            string sRGB;
            ss >> t;
            vTimestamps.push_back(t);
            ss >> sRGB;
            vstrImageFilenames.push_back(sRGB);
        }
    }
    nImages = vstrImageFilenames.size();
    cout << "Total number of Image is " << nImages << endl;
    ReadImages(vstrImageFilenames[0], frame1, imprvs);
    cout << "Image size: " << frame1.cols << "X" << frame1.rows << endl;
    ltframe = vTimestamps[0];
    cout << "Start TimeStamps: " << ltframe << endl;
    if (save){
        SaveImages(outpath, vstrImageFilenames[0], frame1);
    }
    // Original Code using cv::glob()
    // vector<String> filename;
    // glob(path, filename, false);
    // double count = filename.size();
}

// Read Images
void ReadImages(const string Image, Mat &OutputArray){
   OutputArray = imread(inpath+"/"+Image,CV_LOAD_IMAGE_UNCHANGED);
}
void ReadImages(const string Image, Mat &OutputArray, Mat &OutputArray2){
    OutputArray = imread(inpath+"/"+Image,CV_LOAD_IMAGE_UNCHANGED);
    cvtColor(OutputArray, OutputArray2, CV_BGR2GRAY);
}


// Save Image using filename under the strfile dir
void SaveImages(const string &strFile, const string &filename, const Mat &im)
{     
    string delimiter = "/";
    vector<string> v = split (filename, delimiter);
    string sp = strFile + "/" + v[1];
    cout << "Image save on " << sp << endl;
    imwrite(sp,im);
}
// function for split "rgb" in tum filename "rgb/xxxxxx.png"
vector<string> split (string s, string delimiter) {
    size_t pos_start = 0, pos_end, delim_len = delimiter.length();
    string token;
    vector<string> res;

    while ((pos_end = s.find (delimiter, pos_start)) != string::npos) {
        token = s.substr (pos_start, pos_end - pos_start);
        pos_start = pos_end + delim_len;
        res.push_back (token);
    }

    res.push_back (s.substr (pos_start));
    return res;
}

// Initiate the ORBextractor which is adopted from the ORBSLAM2 ORBextractor.cc
void InitORBextractor(const int nfeatures, const float scaleFactor, const int nlevels, const int iniThFAST, const int minThFAST){
        mpORBextractor = new ORBextractor(nfeatures, scaleFactor, nlevels, iniThFAST, minThFAST);
        cout << endl  << "ORB Extractor Parameters: " << endl;
        cout << "- Number of Features: " << nfeatures << endl;
        cout << "- Scale Levels: " << nlevels << endl;
        cout << "- Scale Factor: " << scaleFactor << endl;
        cout << "- Initial Fast Threshold: " << iniThFAST << endl;
        cout << "- Minimum Fast Threshold: " << minThFAST << endl;
}

// Calculate and print the mean magnitude of the optical flow with the matching pairs
void Calmoptflmag(vector<Point2f> prvs, vector<Point2f> next, const int nkpt){
    float sum=0;
    for (uint i=0; i<nkpt; i++){
        
        float a = sqrt((next[i].x -prvs[i].x)*(next[i].x -prvs[i].x)+(next[i].y -prvs[i].y)*(next[i].y -prvs[i].y));
        sum += a;
        // cout << a << endl;
    }
    moptf = sum/nkpt;
    cout.precision(4);
    cout << "Magnitude of Optical flow:" << moptf << endl;
}

// parser
void cmdpars(CommandLineParser parser){
    if ( parser.has("help") )
    {
        parser.about( "Code for Keyframe Selection of Cloud Base SLAM.\n"
                      "Include dataset preprocessing and runtime evaluation. \n");
        parser.printMessage();
    }
    inpath = parser.get<string>("path");
    outpath = parser.get<string>("output");
    dense = parser.has("dense");
    save = parser.has("output");
    show = parser.has("show");
    N = parser.get<float>("N"); // threshold for optical flow
    cout << "Experiments Parameters: " << endl;
    cout << "Evaluate on " << inpath << endl;
    cout << "Save on " << outpath <<endl;
    cout << "Sparse Optical Flow " << !dense << endl;
    cout << "Show Image" << show << endl;
    cout << "Mean Optical Flow Threshold " << N << endl;
}

void SelectGoodPts(vector<Point2f> &good_old, vector<Point2f> &good_new, const vector<uchar> status){
    for(uint i = 0; i < p0.size(); i++)
                {
                    if(status[i] == 1) {
                        good_new.push_back(p1[i]);                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                            
                        good_old.push_back(p0[i]);
                    }
                }
}