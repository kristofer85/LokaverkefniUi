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


//a struct with the purpose of relaying disparity settings from the ui to the depthmap class
struct DepthMapSettings
{
    //Maximum disparity minus minimum disparity. The value is always greater than zero. In the current implementation,
    //this parameter must be divisible by 16.
    int dispNum;

    //Truncation value for the prefiltered image pixels.
    //The algorithm first computes x-derivative at each pixel and clips its value
    //by [-preFilterCap, preFilterCap] interval. The result values are passed to
    //the Birchfield-Tomasi pixel cost function.
    int prefilter;

    //Maximum size of smooth disparity regions to consider their noise speckles and invalidate.
    //Set it to 0 to disable speckle filtering.
    int speckleSize;

    //Maximum disparity variation within each connected component. If you do speckle filtering,
    //set the parameter to a positive value, it will be implicitly multiplied by 16.
    //Normally, 1 or 2 is good enough.
    int speckleRange;

    //Minimum possible disparity value. Normally, it is zero but sometimes rectification algorithms can shift images,
    //so this parameter needs to be adjusted accordingly.
    int minDisp;

    //Maximum allowed difference (in integer pixel units) in the left-right disparity check.
    //Set it to a non-positive value to disable the check.
    int dif12;

    //Margin in percentage by which the best (minimum) computed cost function value should "win"
    //the second best value to consider the found match correct.
    int unique;

    //Matched block size. It must be an odd number >=1 . Normally, it should be somewhere in the 3..11 range./
    int blockSize;

    //The first parameter controlling the disparity smoothness.
    //This parameter is used for the case of slanted surfaces (not fronto parallel).
    int P1;

    //The second parameter controlling the disparity smoothness.
    //This parameter is used for "solving" the depth discontinuities problem.
    //The larger the values are, the smoother the disparity is.
    //P1 is the penalty on the disparity change by plus or minus 1 between neighbor pixels.
    //P2 is the penalty on the disparity change by more than 1 between neighbor pixels.
    //The algorithm requires P2 > P1
    int P2;

    //DepthMapSettings initialized
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
//the class that createss depthmaps
class DepthMap
{
public:
    //opencv mats for right and left image both color and grayscale as well as numorus disparity maps
    cv::Mat left,right,g1,g2,disp,disp8,disp12,dispf,dispf8;

    //variable that contains the size of the left image
    cv::Size imSize;

    //strings containing the default paths to load and save images and depthmaps
    cv::String L,R,D;

    //class constructor
    DepthMap();

    //run and create disparity map with with default values
    void run();

    //clean up and dealocate the memory used by the mats
    void clean();

    //run and create disparity map using the input image pair
    void run(matPair p,DepthMapSettings dispSet);

    //void SGBMdisparityCalc(cv::Mat l, cv::Mat r);

    //use the semi global block matching algorithm to create depth map
    void SGBMdisparityCalc(cv::Mat l, cv::Mat r);

    //use the block matching algorithm to create depth map
    void BMdisparityCalc(cv::Mat l, cv::Mat r);

    //use the semi global block matching algorithm to create depth map and then try to refine it using the a disparity filter
    void DisparityFilter(cv::Mat l, cv::Mat r);

    //returns the path to where the disparity map was saved
    cv::String GetDisp();

    //returns the path where the left image was saved
    cv::String GetRGB();

    //struct with disparity map settings used by the class
    DepthMapSettings dispSet;

};
#endif // DEPTHMAP_H
