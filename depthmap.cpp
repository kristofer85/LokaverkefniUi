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

    left = imread(L,IMREAD_COLOR);
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

    dispSet = DMS;
    left = p.left;
    right = p.right;
    imwrite(L,left);
    imwrite(R,right);
    cvtColor(left, g1, CV_BGR2GRAY);
    cvtColor(right, g2, CV_BGR2GRAY);
    imwrite("leftGray.png", g1);
    imwrite("rightGray.png", g2);
    p.release();
    left.release();
    right.release();


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
    Mat conf_map = Mat(g1.rows,g1.cols,CV_8U);
    conf_map = Scalar(255);
    Rect ROI;
    double matching_time, filtering_time;
    left_for_matcher  = l.clone();
    right_for_matcher = r.clone();
    String filter = "wls_conf";

    int max_disp = 64;
    double lambda = 18000.0;
    double sigma  = 1.5;
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
    wls_filter->setDepthDiscontinuityRadius((int)ceil(0.5*wsize));

    wls_filter = cv::ximgproc::createDisparityWLSFilter(left_matcher);

    cvtColor(left_for_matcher,  left_for_matcher,  COLOR_BGR2GRAY);
    cvtColor(right_for_matcher, right_for_matcher, COLOR_BGR2GRAY);

    Ptr<StereoMatcher> right_matcher = createRightMatcher(left_matcher);
    matching_time = (double)getTickCount();
    left_matcher-> compute(left_for_matcher, right_for_matcher,left_disp);
    right_matcher->compute(right_for_matcher,left_for_matcher, right_disp);
    matching_time = ((double)getTickCount() - matching_time)/getTickFrequency();

                //! [filtering]
    //wls_filter = createDisparityWLSFilterGeneric(true);
    //wls_filter->setDepthDiscontinuityRadius((int)ceil(0.5*wsize));


    //wls_filter = cv::ximgproc::createDisparityWLSFilter(right_matcher2);
    wls_filter->setLambda(lambda);
    wls_filter->setSigmaColor(sigma);
    filtering_time = (double)getTickCount();
    wls_filter->filter(left_disp,g1,filtered_disp,right_disp);
  //  wls_filter->filter(left_disp,g1,filtered_disp,right_disp2,ROI,g2);
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
    getDisparityVis(left_disp,raw_disp_vis,vis_mult);
    //getDisparityVis(right_disp2,raw_disp_vis,vis_mult);

    Mat filtered_disp_vis;
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

    cv::Ptr<cv::StereoSGBM> sgbm = cv::StereoSGBM::create(0,16,3);
    int sgbmWinSize = 1;
    sgbm->setBlockSize(dispSet.blockSize);
    sgbm->setDisp12MaxDiff(dispSet.dif12);
    sgbm->setUniquenessRatio(dispSet.unique);
    sgbm->setMode(StereoSGBM::MODE_SGBM);
    sgbm->setMinDisparity(dispSet.minDisp);
    sgbm->setNumDisparities(dispSet.dispNum);
    //sgbm->setP1(8*g1.channels()*sgbmWinSize*sgbmWinSize);
    //sgbm->setP2(32*g2.channels()*sgbmWinSize*sgbmWinSize);
    sgbm->setP1(16*g1.channels()*sgbmWinSize*sgbmWinSize);
    sgbm->setP2(64*g2.channels()*sgbmWinSize*sgbmWinSize);
    cout << "p1 = " << sgbm->getP1() << " p2 = " << sgbm->getP2() << endl;
    sgbm->setPreFilterCap(dispSet.prefilter);
    sgbm->setSpeckleRange(dispSet.speckleRange);
    sgbm->setSpeckleWindowSize(dispSet.speckleSize);
    sgbm->compute(g1, g2, disp);
    Mat raw_disp_vis,dispR;

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


