#ifndef DEPTHMAP_H
#define DEPTHMAP_H
#include <opencv2/ximgproc/disparity_filter.hpp>
#include <opencv2/ximgproc.hpp>
#include <opencv2/core/ocl.hpp>
#include <opencv2/core/base.hpp>
#include <opencv2/core/cvdef.h>
#include <opencv2/core/core_c.h>
#include <opencv2/ccalib.hpp>
#include <opencv2/core/utility.hpp>
#include "opencv2/core/core.hpp"
#include "opencv2/calib3d.hpp"
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <limits.h>
#include "opencv2/hal/intrin.hpp"
#include <opencv2/stereo.hpp>
#include "opencv2/hal/intrin.hpp"
#include <opencv2/stereo.hpp>
#include "utils.h"

//using namespace cv;
//using namespace std;
//using namespace cv::ximgproc;
struct DepthMapSettings
{
    int dispNum;
    int prefilter;
    int speckleSize;
    int speckleRange;
    int minDisp;
    int dif12;
    int unique;
    int blockSize;
    int P1;
    int P2;

    DepthMapSettings()
    {
        dispNum = 160;
        prefilter = 0;
        speckleSize = 200;
        speckleRange = 2;
        minDisp = -32;
        dif12 = -1;
        unique = 0;
        blockSize = 1;
        P1 = 8;
        P2 = 32;
    }
};

class DepthMap
{
public:
    cv::Mat left,right,g1,g2,disp,disp8,disp12,dispf,dispf8;
    cv::Size imSize;
    cv::String L,R,D;
    DepthMap();
    void run();
    void clean();
    void run(matPair p,DepthMapSettings dispSet);
    //void SGBMdisparityCalc(cv::Mat l, cv::Mat r);
    void SGBMdisparityCalc(cv::Mat l, cv::Mat r);
    void BMdisparityCalc(cv::Mat l, cv::Mat r);
    void DisparityFilter(cv::Mat l, cv::Mat r);
    cv::String GetDisp();
    cv::String GetRGB();
    DepthMapSettings dispSet;

};
#endif // DEPTHMAP_H
