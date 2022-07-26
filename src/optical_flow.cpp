#include <iostream>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/video.hpp>
#include <string>
#include <chrono>
// #include <iomanip>
// #include <sstream>

using namespace cv;
using namespace std;

const char* params
    = "{ help h         |       | print usage }"
      "{ @path p         |  /home/jarvis/jw_ws/edge_cloud/optical_flow/dataset/rgbd_dataset_freiburg1_teddy/rgb | path to dataset }";

int main( int argc, char* argv[])
{
    // VideoCapture capture(samples::findFile("vtest.avi"));
    // if (!capture.isOpened()){
    //     //error in opening the video input
    //     cerr << "Unable to open file!" << endl;
    //     return 0;
    // }

    // VideoCapture capture("rgbd_dataset_freiburg1_teddy/rgb/*.png");

    // if ( (argc <= 1) || (argv[argc-1] == NULL) || (argv[argc-1][0] == '-') ) {  // there is NO input...
    //     cerr << "No argument provided!" << endl;
    //     //return 1;
    // }
    CommandLineParser parser(argc, argv, params);
    if ( parser.has("help") )
    {
        parser.about( "Code for test running time of dense optical flow algorithm (Farneback).\n"
                      "Running time include perform optical flow estimation algorithm and mean optical flow calucation. \n");
        parser.printMessage();
        return 0;
    }

    using std::chrono::high_resolution_clock;
    using std::chrono::duration_cast;
    using std::chrono::duration;
    using std::chrono::milliseconds;
    double total_time=0;

    string path = parser.get<string>("@path");
    // "/data/SLAMdatasets/tum/rgbd_dataset_freiburg1_room";
    cout << "Evaluate on " << path << endl;
    vector<String> filename;
    glob(path, filename, false);
    double count = filename.size();
    cout << "Total number of Image is " << count << endl;

    Mat frame1, prvs;
    frame1 = imread(filename[0]);
    cout << "Image size: " << frame1.rows << "X" << frame1.cols << endl;
    cvtColor(frame1, prvs, COLOR_BGR2GRAY);

    for (int i = 1; i < count; i++){
        Mat frame2, next;
        frame2 = imread(filename[i]);
        if (frame2.empty())
            break;
        cvtColor(frame2, next, COLOR_BGR2GRAY);

        Mat flow(prvs.size(), CV_32FC2);
        auto t1 = high_resolution_clock::now();
        calcOpticalFlowFarneback(prvs, next, flow, 0.5, 3, 15, 3, 5, 1.2, 0);
        // visualization
        Mat flow_parts[2];
        split(flow, flow_parts);
        Mat magnitude, angle, magn_norm;
        cartToPolar(flow_parts[0], flow_parts[1], magnitude, angle, true);
        cv::Scalar tempVal = cv::mean( magnitude );
        float mean = tempVal.val[0];
        auto t2 = high_resolution_clock::now();
        // normalize(magnitude, magn_norm, 0.0f, 1.0f, NORM_MINMAX);
        // angle *= ((1.f / 360.f) * (180.f / 255.f));
        // std::cout << "Mean optical flow is" << mean << endl;
        //build hsv image
        // Mat _hsv[3], hsv, hsv8, bgr;
        // _hsv[0] = angle;
        // _hsv[1] = Mat::ones(angle.size(), CV_32F);
        // _hsv[2] = magn_norm;
        // merge(_hsv, 3, hsv);
        // hsv.convertTo(hsv8, CV_8U, 255.0);
        // cvtColor(hsv8, bgr, COLOR_HSV2BGR);

        // imshow("frame2", bgr);

        // int keyboard = waitKey(30);
        // if (keyboard == 'q' || keyboard == 27)
        //     break;
        prvs = next;
        auto ms_int = duration_cast<milliseconds>(t2 - t1);
        duration<double, std::milli> ms_double = t2 - t1;
        total_time += ms_double.count();
    }
    double average_time = total_time / count ;
    cout << "Average excution time of algorithm is " << average_time << "ms." << endl;
}
