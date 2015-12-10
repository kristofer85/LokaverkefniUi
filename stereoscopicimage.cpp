#include "stereoscopicimage.h"
#include "stereocalibrate.h"
#include <opencv2/core/utility.hpp>
#include <opencv2/ximgproc/disparity_filter.hpp>
#include <opencv2/ximgproc.hpp>
#include <opencv2/core/ocl.hpp>
#include <opencv2/core/base.hpp>
#include <opencv2/core/cvdef.h>
#include <opencv2/core/core_c.h>
#include <opencv2/ccalib.hpp>
#include <opencv/cv.h>
#include <opencv2/core.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
using namespace cv;
using namespace std;
StereoScopicImage::StereoScopicImage()
{

}


/*******************************************
 * The functions in this section use a     *
 * so-called pinhole camera model. In this *
 * model, a scene view is formed by        *
 * projecting 3D points into the image     *
 * plane using a perspective               *
 * transformation.                         *
 ******************************************/
void StereoScopicImage::rectifyCamera()
{
    Mat fullImg = imread("myndir/calib3_fixed.JPG",IMREAD_COLOR);
    Size imSize = fullImg.size();
    Mat img1 = fullImg(Range(0, imSize.height),Range(0, imSize.width/2)).clone();
    imwrite("leftS.png",img1);
    Mat img2 = fullImg(Range(0, imSize.height),Range(imSize.width/2, imSize.width)).clone();
    imwrite("rightS.png",img2);
    Mat CM1, D1, CM2, D2,R, T, R1, P1, R2, P2;
    Rect roi1, roi2;
    Mat Q;
    Q = Mat(4,4, CV_64F);
    setIdentity(Q);
    cout << "Q"<< Q << endl;
    FileStorage fs1 = FileStorage("stereoCalibration.yml", FileStorage::READ);
    fs1["CM1"] >> CM1;
    fs1["D1"] >> D1;
    fs1["CM2"] >> CM2;
    fs1["D2"] >> D2;
    fs1["R"] >> R;
    fs1["T"] >> T;
    stereoRectify( CM1, D1, CM2, D2, img1.size(), R, T, R1, R2, P1, P2, Q, CALIB_ZERO_DISPARITY );
    fs1.open("stereoCalibration.yml", FileStorage::APPEND);
    fs1 << "R1" << R1;
    fs1 << "R2" << R2;
    fs1 << "P1" << P1;
    fs1 << "P2" << P2;
    fs1 << "Q" << Q;

}

void StereoScopicImage::disparityMap()
{
    Mat g1,g2;
    Mat fullImg = imread("InputPicts/My with a_mighty_chessbord.JPG",IMREAD_COLOR);
    Size imSize = fullImg.size();
    Mat img1 = fullImg(Range(0, imSize.height-1),Range(0, imSize.width/2)).clone();
    Mat img2 = fullImg(Range(0, imSize.height-1),Range(imSize.width/2, imSize.width-1)).clone();
    resize(img1, img1, Size(), 0.25, 0.25);
    resize(img2, img2, Size(), 0.25, 0.25);
    imwrite("leftColor.png",img1);
    imwrite("rightColor.png",img2);
    g1 = imread("leftColor.png",IMREAD_GRAYSCALE);
    g2 = imread("rightColor.png",IMREAD_GRAYSCALE);

    Mat disp,disp8;
    Ptr<StereoSGBM> sgbm = StereoSGBM::create(0,16,3);
    int sgbmWinSize = 3;
    sgbm->setBlockSize(sgbmWinSize);
    int cn = g1.channels();
    cout << "channels "<< cn << endl;
    sgbm->setDisp12MaxDiff(12);
    sgbm->setUniquenessRatio(5);
    sgbm->setMode(StereoSGBM::MODE_SGBM);
    sgbm->setMinDisparity(0);
    sgbm->setNumDisparities(384);
    sgbm->setP1(8*cn*sgbmWinSize*sgbmWinSize);
    sgbm->setP2(32*cn*sgbmWinSize*sgbmWinSize);
    sgbm->setPreFilterCap(32);
    sgbm->setSpeckleRange(5);
    sgbm->setSpeckleWindowSize(100);
    sgbm->compute(g1, g2, disp);
    normalize(disp, disp8, 0, 255, CV_MINMAX, CV_8U);
    imwrite("dispF.png",disp);
    imwrite("disp8F.png",disp8);
    //OpenCvUtilities openCvUtilities;
    //openCvUtilities.depthMapFilter(disp,disp8,g1,g2,sgbm);
    Mat raw_disp_vis2;
    ximgproc::getDisparityVis(disp,raw_disp_vis2,1.0);
    Ptr<ximgproc::DisparityWLSFilter> wls_filter;
    Mat left_for_matcher, right_for_matcher;
    Mat left_disp,right_disp;
    Mat filtered_disp;
    Mat conf_map = Mat(g1.rows,g1.cols,CV_8U);
    conf_map = Scalar(255);
    Rect ROI;
    double matching_time, filtering_time;
    left_for_matcher  = g1.clone();
    right_for_matcher = g2.clone();
    String filter = "wls_conf";

    int max_disp = 160;
    double lambda = 8000.0;
    double sigma  = 1.5;
    double vis_mult = 1.0;
    int wsize = 3;

    Ptr<StereoSGBM> left_matcher  = StereoSGBM::create(0,max_disp,wsize);
    left_matcher->setP1(8*wsize*wsize);
    left_matcher->setP2(32*wsize*wsize);
    left_matcher->setPreFilterCap(63);
    left_matcher->setSpeckleRange(5);
    left_matcher->setSpeckleWindowSize(10);
    left_matcher->setMinDisparity(0);
    left_matcher->setMode(StereoSGBM::MODE_SGBM);
    wls_filter = cv::ximgproc::createDisparityWLSFilter(left_matcher);

    Ptr<StereoMatcher> right_matcher = ximgproc::createRightMatcher(left_matcher);
    matching_time = (double)getTickCount();
    left_matcher->compute(left_for_matcher, right_for_matcher,left_disp);
    right_matcher->compute(right_for_matcher, left_for_matcher, right_disp);
    matching_time = ((double)getTickCount() - matching_time)/getTickFrequency();

    wls_filter->setLambda(lambda);
    wls_filter->setSigmaColor(sigma);
    filtering_time = (double)getTickCount();
    wls_filter->filter(left_disp,g1,filtered_disp,right_disp);
    filtering_time = ((double)getTickCount() - filtering_time)/getTickFrequency();
    conf_map = wls_filter->getConfidenceMap();

    ROI = wls_filter->getROI();
    normalize(left_disp, disp8, 0, 255, CV_MINMAX, CV_8U);
    Mat raw_disp_vis;
    ximgproc::getDisparityVis(left_disp,raw_disp_vis,vis_mult);
    Mat filtered_disp_vis;
    ximgproc::getDisparityVis(filtered_disp,filtered_disp_vis,vis_mult);
    imwrite("img1Path.png",filtered_disp_vis);
    //destroyAllWindows();

    Mat leftColorClone = imread("leftColor.png",IMREAD_GRAYSCALE);
    Mat rightColorClone = imread("rightColor.png",IMREAD_GRAYSCALE);
}

void StereoScopicImage::disparityMap(string filename)
{
    //int SADWindowSize = 9, numberOfDisparities = 0;
    //FileStorage fs;
    //fs.open(filename, FileStorage::READ);
    //
    //if (!fs.isOpened())
    //{
    //    cerr << "Failed to open " << filename << endl;
    //    //quit(0);
    //}
    //
    //FileNode n = fs["images"];                         // Read string sequence - Get node
    //if (n.type() != FileNode::SEQ)
    //{
    //    cerr << "strings is not a sequence! FAIL" << endl;
    //    //quit(0);
    //}
    //string bleh;
    //Mat fullImg;
    //Mat img1, img2;
    //Mat g1, g2, disp, disp8;
    //namedWindow("disp",WINDOW_NORMAL| WINDOW_KEEPRATIO);
    //Size imgSize;
    //FileNodeIterator it = n.begin(), it_end = n.end(); // Go through the node
    //for (; it != it_end; ++it)
    //{
    //    bleh.append((string)*it);
    //    fullImg = imread(bleh,IMREAD_COLOR);
    //    Size imSize = fullImg.size();
    //    Mat img1 = fullImg(Range(300, imSize.height-800),Range(650, imSize.width/2)).clone();
    //    Mat img2 = fullImg(Range(300, imSize.height-800),Range(imSize.width/2, imSize.width-650)).clone();
    //    imgSize = img1.size();
    //    cvtColor(img1, g1, CV_BGR2GRAY);
    //    cvtColor(img2, g2, CV_BGR2GRAY);
    //    int sgbmWinSize = 6;
    //    sgbm->setBlockSize(sgbmWinSize);
    //    int cn = img1.channels();
    //    sgbm->setDisp12MaxDiff(1);
    //    sgbm->setUniquenessRatio(1);
    //    sgbm->setMode(StereoSGBM::MODE_SGBM);
    //    sgbm->setMinDisparity(-64);
    //    sgbm->setNumDisparities(192);
    //    sgbm->setP1(8*cn*sgbmWinSize*sgbmWinSize);
    //    sgbm->setP2(32*cn*sgbmWinSize*sgbmWinSize);
    //    sgbm->setPreFilterCap(4);
    //    sgbm->setSpeckleRange(32);
    //    sgbm->setSpeckleWindowSize(200);
    //    sgbm->compute(g1, g2, disp);
    //    normalize(disp, disp8, 0, 255, CV_MINMAX, CV_8U);
    //    imshow("disp", disp8);
    //    namedWindow("g1",WINDOW_NORMAL| WINDOW_KEEPRATIO);
    //    namedWindow("g2",WINDOW_NORMAL| WINDOW_KEEPRATIO);
    //    imshow("g1", g1);
    //    imshow("g2", g2);
    //    waitKey(0);
    //}
}
