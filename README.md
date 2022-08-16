# edge_cloud

## Usage
- Header File: `KFDSample.hpp` and `pd.hpp`



### reference usage:
As shown in `main.cpp`
    
```c

// construct a object
KFDSample kfd = KFDSample(nfeatures, nlevels, iniThFAST, minThFAST, scaleFactor, frame1, vTimestamps[0]);
// init PD controller
float N = "$threshold";
kfd.InitPDKFselector(0.8, 0.01, N);
// set threshold
kfd.SetThreshold(N);
// wether to show the result, default false
kfd.SetDisplay();

/*            initialization finished       */
/*            New Frame comes              */

cv::Mat nextframe;
double t = nexttimestamp;
kfd.CalcMOptFLOW(nextframe, t);


/*            Get the Latest Selected Keyframe */

cv::Mat lastkf = kfd.GetKF();
std::vector<cv::Mat> allkf = kfd.GetAllKF();

```



