#include "depthmap.h"
#include "defines.h"
using namespace cv;
using namespace std;
using namespace cv::ximgproc;

DepthMap::DepthMap()
{

    L = "im0.png";
    R = "im1.png";
    //L = "im0.jpg";
    //R = "im1.jpg";
    D = "disp8SGBM.png";
}

void DepthMap::run()
{

    //loads the image im0.png into the mat left
    left = imread(L,IMREAD_COLOR);
    //loads the image im1.png into the mat right
    right = imread(R,IMREAD_COLOR);
    //resize(left ,left ,Size(),0.5,0.5);
    //resize(right,right,Size(),0.5,0.5);
    imwrite(L,left);
    imwrite(R,right);
    cvtColor(left, g1, CV_BGR2GRAY);
    cvtColor(right, g2, CV_BGR2GRAY);
    imwrite("leftGray.png", g1);
    imwrite("rightGray.png", g2);



    SGBMdisparityCalc(g1,g2);
    //SGBMdisparityCalc(g1,g2);
    //BMdisparityCalc(g1,g2);

    //DisparityFilter(left,right);
}
//this function dealocates and returns the memory the varius mats
//in this class are usin and clears their contents
void DepthMap::clean()
{
    left.release();
    right.release();
    g1.release();
    g2.release();
    disp.release();
    disp8.release();
    disp12.release();
    dispf.release();
    dispf8.release();
}

void DepthMap::run(matPair p,DepthMapSettings DMS)
{
    //DepthMapSettings containing values from the UI
    dispSet = DMS;
    left = p.left;
    right = p.right;
    imwrite(L,left);
    imwrite(R,right);
    //turns color images into grayscale images
    cvtColor(left, g1, CV_BGR2GRAY);
    cvtColor(right, g2, CV_BGR2GRAY);
    imwrite("leftGray.png", g1);
    imwrite("rightGray.png", g2);
    p.release();
    //releases the memory used by the mats left and right
    left.release();
    right.release();

    //calls a function that uses the semi global block matching
    //algorithm using the grayscale versions of the images
    SGBMdisparityCalc(g1,g2);
    //SGBMdisparityCalc(g1,g2);
    //BMdisparityCalc(g1,g2);

    //DisparityFilter(left,right);
    //DisparityFilter(left,right);
}

void DepthMap::DisparityFilter(Mat l,Mat r)
{
    Ptr<DisparityWLSFilter> wls_filter;

    Mat left_for_matcher, right_for_matcher;
    Mat left_disp,right_disp;
    Mat filtered_disp;
    //Get the confidence map that was used in the last filter call.
    //It is a CV_32F one-channel image with values ranging from 0.0
    //(totally untrusted regions of the raw disparity map) to 255.0
    //(regions containing correct disparity values with a high degree of confidence).
    Mat conf_map = Mat(g1.rows,g1.cols,CV_8U);
    conf_map = Scalar(255);
    Rect ROI;
    double matching_time, filtering_time;
    left_for_matcher  = l.clone();
    right_for_matcher = r.clone();
    //chose waht kind of filter you want to use
    String filter = "wls_conf";

    int max_disp = 64;

    //Lambda is a parameter defining the amount of regularization during filtering.
    //Larger values force filtered disparity map edges to adhere more to
    //source image edges.
    double lambda = 18000.0;

    //SigmaColor is a parameter defining how sensitive the filtering process
    //is to source image edges. Large values can lead to disparity leakage
    //through low-contrast edges. Small values can make the filter too sensitive
    //to noise and textures in the source image.
    double sigma  = 1.5;

    //disparity map will be multiplied by this value for visualization
    double vis_mult = 1.0;
    int wsize = 3;
    int wsize2 = 3;

    Ptr<StereoSGBM> left_matcher  = StereoSGBM::create(0,max_disp,wsize);
    left_matcher->setUniquenessRatio(0);
    left_matcher->setDisp12MaxDiff(1000000);
    left_matcher->setP1(24*wsize2*wsize2);
    left_matcher->setP2(96*wsize2*wsize2);
    left_matcher->setPreFilterCap(0);
    left_matcher->setSpeckleRange(-1);
    left_matcher->setSpeckleWindowSize(-1);;
    left_matcher->setMinDisparity(0);
    left_matcher->setBlockSize(wsize);
    left_matcher->setMode(StereoSGBM::MODE_SGBM_3WAY);

    wls_filter = createDisparityWLSFilterGeneric(true);

    //DepthDiscontinuityRadius is a parameter used in confidence computation.
    //It defines the size of low-confidence regions around depth discontinuities.
    wls_filter->setDepthDiscontinuityRadius((int)ceil(0.5*wsize));

    wls_filter = cv::ximgproc::createDisparityWLSFilter(left_matcher);

    //convert color images to grayscale images
    cvtColor(left_for_matcher,  left_for_matcher,  COLOR_BGR2GRAY);
    cvtColor(right_for_matcher, right_for_matcher, COLOR_BGR2GRAY);

    Ptr<StereoMatcher> right_matcher = createRightMatcher(left_matcher);
    matching_time = (double)getTickCount();
    left_matcher-> compute(left_for_matcher, right_for_matcher,left_disp);
    right_matcher->compute(right_for_matcher,left_for_matcher, right_disp);
    matching_time = ((double)getTickCount() - matching_time)/getTickFrequency();

                //! [filtering]
    wls_filter->setLambda(lambda);
    wls_filter->setSigmaColor(sigma);
    filtering_time = (double)getTickCount();
    //apply the filter on the left and right disparity images
    wls_filter->filter(left_disp,g1,filtered_disp,right_disp);
    Mat right_disp_norm,left_disp_norm;
    normalize(right_disp, right_disp_norm, 0, 255, CV_MINMAX, CV_8U);
    normalize(left_disp, left_disp_norm, 0, 255, CV_MINMAX, CV_8U);
    //wls_filter->filter(right_disp2,g2,filtered_disp,left_disp,ROI,g1);
    //wls_filter->filter(right_disp_norm,g2,filtered_disp,left_disp_norm,ROI,g1);
    filtering_time = ((double)getTickCount() - filtering_time)/getTickFrequency();
                        //! [filtering]
    conf_map = wls_filter->getConfidenceMap();
    //ROI = wls_filter->getROI();
    //normalize(left_disp, disp8, 0, 255, CV_MINMAX, CV_8U);

    //! [visualization]
    Mat raw_disp_vis;
    //Function for creating a disparity map visualization (clamped CV_8U image)
    getDisparityVis(left_disp,raw_disp_vis,vis_mult);
    //getDisparityVis(right_disp2,raw_disp_vis,vis_mult);

    Mat filtered_disp_vis;
    //Function for creating a disparity map visualization (clamped CV_8U image)
    getDisparityVis(filtered_disp,filtered_disp_vis,vis_mult);

    //raw disp
    namedWindow("left",WINDOW_NORMAL| CV_WINDOW_KEEPRATIO);
    namedWindow("right",WINDOW_NORMAL| CV_WINDOW_KEEPRATIO);
    namedWindow("right2",WINDOW_NORMAL| CV_WINDOW_KEEPRATIO);
    //norm
    namedWindow("norm_left",WINDOW_NORMAL| CV_WINDOW_KEEPRATIO);
    namedWindow("norm_right",WINDOW_NORMAL| CV_WINDOW_KEEPRATIO);
    //vis
    namedWindow("rawVis",WINDOW_NORMAL| CV_WINDOW_KEEPRATIO);
    namedWindow("filteredVis",WINDOW_NORMAL| CV_WINDOW_KEEPRATIO);
    namedWindow("conf",WINDOW_NORMAL| CV_WINDOW_KEEPRATIO);
    imshow("left",left_disp);
    imshow("right",right_disp);
    //imshow("right2",right_disp2);
    imshow("norm_left",left_disp_norm);
    imshow("norm_right",right_disp_norm);
    imshow("rawVis",raw_disp_vis);
    imshow("filteredVis",filtered_disp_vis);
    imshow("conf",conf_map);
    waitKey(0);

    imwrite("rawDisp.png",filtered_disp_vis);
    imwrite("filteredDisp.png",filtered_disp_vis);
    destroyAllWindows();





}

void DepthMap::BMdisparityCalc(Mat g1,Mat g2)
{
    cv::Ptr<cv::StereoBM> bm = cv::StereoBM::create(16,9);

    int SADWindowSize = 5;
    bm->setBlockSize(SADWindowSize > 0 ? SADWindowSize : 9);
    bm->setDisp12MaxDiff(32);
    bm->setUniquenessRatio(10);
    bm->setMinDisparity(0);
    bm->setNumDisparities(1600);
    bm->setPreFilterCap(63);
    bm->setSpeckleRange(5);
    bm->setSpeckleWindowSize(10);
    bm->compute(g1, g2, disp);
    normalize(disp, disp8, 0, 255, CV_MINMAX, CV_16S);
    imwrite("dispBM.png", disp);
    imwrite("disp8BM.png", disp8);
}

void DepthMap::SGBMdisparityCalc(Mat g1,Mat g2)
{
    //sets im
    cv::Ptr<cv::StereoSGBM> sgbm = cv::StereoSGBM::create(0,16,3);
    int sgbmWinSize = 1;
    sgbm->setBlockSize(dispSet.blockSize);
    sgbm->setDisp12MaxDiff(dispSet.dif12);
    sgbm->setUniquenessRatio(dispSet.unique);
    sgbm->setMode(StereoSGBM::MODE_SGBM);
    sgbm->setMinDisparity(dispSet.minDisp);
    sgbm->setNumDisparities(dispSet.dispNum);
    //sgbm->setP1(16*g1.channels()*sgbmWinSize*sgbmWinSize);
    //sgbm->setP2(64*g2.channels()*sgbmWinSize*sgbmWinSize);
    sgbm->setP1(dispSet.P1);
    sgbm->setP2(dispSet.P2);
    cout << "p1 = " << sgbm->getP1() << " p2 = " << sgbm->getP2() << endl;
    sgbm->setPreFilterCap(dispSet.prefilter);
    sgbm->setSpeckleRange(dispSet.speckleRange);
    sgbm->setSpeckleWindowSize(dispSet.speckleSize);
    sgbm->compute(g1, g2, disp);
    Mat raw_disp_vis,dispR;
    //evens out the image the image is easyer on the eyes
    normalize(disp, disp8, 0, 255, CV_MINMAX, CV_8U);
    imwrite("dispSGBM.png", disp);
    imwrite(D, disp8);
    medianBlur(disp,dispf,5);
    medianBlur(disp8,dispf8,9);
    Mat dispf82;
    medianBlur(dispf8,dispf82,9);
    imwrite("dispSGBMMedian.png", dispf);
    imwrite("disp8SGBMMedian.png", dispf8);
    imwrite("disp8fSGBMMedian.png", dispf82);

    cout << "done making depthMap" << endl;

}

cv::String DepthMap::GetDisp()
{
    return D;
}

cv::String DepthMap::GetRGB()
{
    return L;
}


