#include <iostream>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/video.hpp>
#include <string>
// #include <iomanip>
// #include <sstream>

using namespace cv;
using namespace std;

int main()
{
    // VideoCapture capture(samples::findFile("vtest.avi"));
    // if (!capture.isOpened()){
    //     //error in opening the video input
    //     cerr << "Unable to open file!" << endl;
    //     return 0;
    // }

    // VideoCapture capture("rgbd_dataset_freiburg1_teddy/rgb/*.png");

    string path = "/data/SLAMdatasets/tum/rgbd_dataset_freiburg1_room";
    cout << "Evaluate on " << path << endl;
    vector<String> filename;
    glob(path, filename, false);
    size_t count = filename.size();
    cout << "Total name of Image is" << count << endl;

    Mat frame1, prvs;
    frame1 = imread(filename[0]);
    cvtColor(frame1, prvs, COLOR_BGR2GRAY);

    for (int i = 1; i < count; i++){
        Mat frame2, next;
        frame2 = imread(filename[i]);
        if (frame2.empty())
            break;
        cvtColor(frame2, next, COLOR_BGR2GRAY);

        Mat flow(prvs.size(), CV_32FC2);
        calcOpticalFlowFarneback(prvs, next, flow, 0.5, 3, 15, 3, 5, 1.2, 0);
         
        // visualization
        Mat flow_parts[2];
        split(flow, flow_parts);
        Mat magnitude, angle, magn_norm;
        cartToPolar(flow_parts[0], flow_parts[1], magnitude, angle, true);
        cv::Scalar tempVal = cv::mean( magnitude );
        float mean = tempVal.val[0];
        // normalize(magnitude, magn_norm, 0.0f, 1.0f, NORM_MINMAX);
        // angle *= ((1.f / 360.f) * (180.f / 255.f));
        std::cout << "Mean optical flow is" << mean << endl;
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
    }
}
