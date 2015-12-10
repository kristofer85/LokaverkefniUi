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

    //testing disparity refinement
    const float EVAL_BAD_THRESH = 1.f;
    const int EVAL_TEXTURELESS_WIDTH = 3;
    const float EVAL_TEXTURELESS_THRESH = 4.f;
    const float EVAL_DISP_THRESH = 1.f;
    const float EVAL_DISP_GAP = 2.f;
    const int EVAL_DISCONT_WIDTH = 9;
    const int EVAL_IGNORE_BORDER = 10;

    void computeTextureBasedMasks( const cv::Mat& _img, cv::Mat* texturelessMask, cv::Mat* texturedMask,
                 int texturelessWidth, float texturelessThresh);
    void checkTypeAndSizeOfDisp( const cv::Mat& dispMap, const cv::Size* sz );
    void checkTypeAndSizeOfMask( const cv::Mat& mask, cv::Size sz );
    void checkDispMapsAndUnknDispMasks( const cv::Mat& leftDispMap, const cv::Mat& rightDispMap,
                                        const cv::Mat& leftUnknDispMask, const cv::Mat& rightUnknDispMask );
    void computeOcclusionBasedMasks(const cv::Mat& leftDisp,const  cv::Mat& _rightDisp,cv::Mat* occludedMask,cv::Mat* nonOccludedMask,const cv::Mat& leftUnknDispMask,const cv::Mat& rightUnknDispMask,float dispThresh);
    void computeDepthDiscontMask( const cv::Mat& disp, cv::Mat& depthDiscontMask, const cv::Mat& unknDispMask,
                                     float dispGap, int discontWidth);
    cv::Mat getBorderedMask( cv::Size maskSize, int border);
    float dispRMS( const cv::Mat& computedDisp, const cv::Mat& groundTruthDisp, const cv::Mat& mask );
    float badMatchPxlsFraction( const cv::Mat& computedDisp, const cv::Mat& groundTruthDisp, const cv::Mat& mask,
                                float _badThresh);

    void fillOcclusion(cv::Mat& src, int invalidvalue);// for disparity map
    void fillOcclusionDepth(cv::Mat& src, int invalidvalue);//for depth map
    void fillOcclusionDepth(cv::Mat& depth, cv::Mat& image, int invalidvalue, int threshold);
    void jointColorDepthFillOcclusion(const cv::Mat& src, const cv::Mat& guide, cv::Mat& dest, const cv::Size ksize, double threshold);
    //remove Streaking Noise in stereo DP matching and hole filling function
    void removeStreakingNoise(cv::Mat& src, cv::Mat& dest, int th);

};
#endif // DEPTHMAP_H
