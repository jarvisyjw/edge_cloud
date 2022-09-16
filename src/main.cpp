#include "KFDSample.hpp"
#include <iostream>
#include <sstream>
#include <fstream>


const char* params
    = "{ h help     |   | print usage }"
      "{ p path     |   | path to dataset }"
      "{ d dense    |   | dense or sparse}"
      "{ N  n       |   | threshold for optical flow}"
      "{ o output   |   | Save DIR}"
      "{ s show     |   | if Show image}"
      ;


void cmdpars(CommandLineParser parser);
void LoadImages(const string &strFile, vector<string> &vstrImageFilenames, vector<double> &vTimestamps);
void ReadImages(const string Image, Mat &OutputArray);
void ReadImages(const string Image, Mat &OutputArray, Mat &OutputArray2);\
void SaveImages(const string &strFile, const string &filename, const Mat &im);
vector<string> split (string s, string delimiter);

// Load images
vector<string> vstrImageFilenames;
vector<double> vTimestamps;
// Input Output Path
string inpath, outpath;
// CMD Argument
bool dense, save, show;
bool kf=false;
// Total Num of Images, num of keypoints
int nImages;
// mean optical flow of images and threshold N
float moptf, N;
// Image
Mat frame1, imprvs;
Mat frame2, imnext;
// Total Running Time, timeframe
double total_time = 0, tframe = 0, ltframe = 0;
// ORBextractor Parameters
const float scaleFactor = 1.2;
const int nfeatures = 2000, nlevels = 8, iniThFAST = 20, minThFAST = 7;


int main( int argc, char* argv[]){

    //cmd parser
    CommandLineParser parser(argc, argv, params);
    // parse the cmd input
    cmdpars(parser);
    // load image from dataset
    LoadImages(inpath+"/rgb.txt", vstrImageFilenames, vTimestamps);

    // init selector
    KFDSample kfd = KFDSample(nfeatures, nlevels, iniThFAST, minThFAST, scaleFactor, frame1, vTimestamps[0]);
    // init PD controller
    kfd.InitPDKFselector(0.8, 0.01, N);
    kfd.SetThreshold(N);
    // start the loop
    if (show){
    kfd.SetDisplay();
    }

    for (uint i = 1; i < nImages; i++){
        Mat nextframe;
        double t = vTimestamps[i];
        ReadImages(vstrImageFilenames[i],nextframe);
        if (nextframe.empty()){
            break;
        }
        if (kfd.CalcMOptFLOW(nextframe, t)){
            if(save){
                SaveImages(outpath, vstrImageFilenames[i],nextframe);
            }
        }
        vector<Mat> allkf = kfd.GetAllKF();
        cout << "number of keyframes"<<allkf.size() << endl;
    }
    // return 0;
    // delete []kfd;
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