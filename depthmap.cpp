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
    //Mat stereo = imread("calib5_fixed.JPG",IMREAD_COLOR);
    /*
    Mat stereo = imread("../Lokaverkefni2/chessboardImages/DSC_0071_sbs.jpg",IMREAD_COLOR);
    imSize = stereo.size();
    left = stereo(Range(0, imSize.height),Range(0, imSize.width/2)).clone();
    right = stereo(Range(0, imSize.height),Range(imSize.width/2, imSize.width)).clone();

    //left = stereo(Range(0, imSize.height),Range(0, imSize.width/2)).clone();
    //right = stereo(Range(0, imSize.height),Range(imSize.width/2, imSize.width)).clone();
    matPair mats;
    mats = splitImage(stereo);
    left = mats.left;
    right = mats.right;
    */
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
    //Mat stereo = imread("calib5_fixed.JPG",IMREAD_COLOR);
    /*
    Mat stereo = imread("../Lokaverkefni2/chessboardImages/DSC_0071_sbs.jpg",IMREAD_COLOR);
    imSize = stereo.size();
    left = stereo(Range(0, imSize.height),Range(0, imSize.width/2)).clone();
    right = stereo(Range(0, imSize.height),Range(imSize.width/2, imSize.width)).clone();

    //left = stereo(Range(0, imSize.height),Range(0, imSize.width/2)).clone();
    //right = stereo(Range(0, imSize.height),Range(imSize.width/2, imSize.width)).clone();
    matPair mats;
    mats = splitImage(stereo);
    left = mats.left;
    right = mats.right;
    */
    dispSet = DMS;
    left = p.left;
    right = p.right;
    //normalize(left,left,0,255, CV_MINMAX, CV_8U);
    //normalize(right,right,0,255, CV_MINMAX, CV_8U);
    //resize(left ,left ,Size(),0.5,0.5);
    //resize(right,right,Size(),0.5,0.5);
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
/*
    //test custom createRightMatcher
    Ptr<StereoMatcher> right_matcher2 = createRightMatcher2(left_matcher);
    Mat right_disp2;
    right_matcher2->compute(right_for_matcher,left_for_matcher, right_disp2);
*/
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
/*
void DepthMap::SGBMdisparityCalc(Mat g1,Mat g2)
{
    cv::Ptr<cv::StereoSGBM> sgbm = cv::StereoSGBM::create(0,16,3);
    int sgbmWinSize = 1;
    sgbm->setBlockSize(3);
    sgbm->setDisp12MaxDiff(50);
    sgbm->setUniquenessRatio(2);
    sgbm->setMode(StereoSGBM::MODE_HH);
    sgbm->setMinDisparity(-48);
    sgbm->setNumDisparities(128);
    //sgbm->setP1(8*g1.channels()*sgbmWinSize*sgbmWinSize);
    //sgbm->setP2(32*g2.channels()*sgbmWinSize*sgbmWinSize);
    sgbm->setP1(16*g1.channels()*sgbmWinSize*sgbmWinSize);
    sgbm->setP2(64*g2.channels()*sgbmWinSize*sgbmWinSize);
    cout << "p1 = " << sgbm->getP1() << " p2 = " << sgbm->getP2() << endl;
    sgbm->setPreFilterCap(63);
    sgbm->setSpeckleRange(2);
    sgbm->setSpeckleWindowSize(200);
    sgbm->compute(g1, g2, disp);
    Mat raw_disp_vis,dispR;

    Ptr<StereoMatcher> sgbm2 = createRightMatcher(sgbm);

    cout << "compute right disp" << endl;
    sgbm2->compute(g2, g1, dispR);
    imwrite("dispSGBMR.png", dispR);
    Mat occ,nonocc;
    cout << "compute occ and nonocc" << endl;
    Mat L,R;
    disp.convertTo( L, CV_32FC1 );
    dispR.convertTo( R, CV_32FC1 );
    imwrite("L.png",L);
    imwrite("R.png",R);

    computeOcclusionBasedMasks(L,R,&occ,&nonocc,Mat(),Mat(),1.f);
    imwrite("dispOcc.png", occ);
    imwrite("dispnonOcc.png", nonocc);


    getDisparityVis(disp,raw_disp_vis,1);
    imwrite("dispVis.png",raw_disp_vis);
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
    Mat dispFill,dispFillNorm,dispNoNoise;
    dispFill = disp;
    fillOcclusion(dispFill, 0);
    removeStreakingNoise(dispFill,dispFill,9);
    int sigS = 70;
        int sigC = 100;
        int sigWS = 100;
        int sigWC = 100;
    Mat dispbil,jointBil,jointBilNorm;
    Mat weight = Mat::ones(disp.size(),CV_32F);
            int r = 33;
            int d = 2*r+1;
            double swc=sigWC/10.0;
            double sws=sigWS/10.0;
            double sc=sigC/10.0;
            double ss=sigS/10.0;

    bilateralFilter(disp8,dispbil,d,sc*3,ss*3,0);
    jointBilateralFilter(g1,disp8,jointBil,d,sc,ss);
    imshow("joint",jointBil);
    imwrite("dispbil.png",dispbil);
    //normalize(jointBil, jointBilNorm, 0, 255, CV_MINMAX, CV_8U);
    //imwrite("jointBil",jointBilNorm);
    normalize(dispFill, dispFillNorm, 0, 255, CV_MINMAX, CV_8U);
    imwrite("dispFill.png",dispFill);
    imwrite("dispFillNorm.png",dispFillNorm);

    cout << "done making depthMap" << endl;

}
*/

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

// testing disparity refinement

void DepthMap::computeTextureBasedMasks( const Mat& _img, Mat* texturelessMask, Mat* texturedMask,
             int texturelessWidth = 3, float texturelessThresh = 4.f )
{
    if( !texturelessMask && !texturedMask )
        return;
    if( _img.empty() )
        CV_Error( Error::StsBadArg, "img is empty" );

    Mat img = _img;
    if( _img.channels() > 1)
    {
        Mat tmp; cvtColor( _img, tmp, COLOR_BGR2GRAY ); img = tmp;
    }
    Mat dxI; Sobel( img, dxI, CV_32FC1, 1, 0, 3 );
    Mat dxI2; pow( dxI / 8.f/*normalize*/, 2, dxI2 );
    Mat avgDxI2; boxFilter( dxI2, avgDxI2, CV_32FC1, Size(texturelessWidth,texturelessWidth) );

    if( texturelessMask )
        *texturelessMask = avgDxI2 < texturelessThresh;
    if( texturedMask )
        *texturedMask = avgDxI2 >= texturelessThresh;
}

void DepthMap::checkTypeAndSizeOfDisp( const Mat& dispMap, const Size* sz )
{
    if( dispMap.empty() )
        CV_Error( Error::StsBadArg, "dispMap is empty" );
    if( dispMap.type() != CV_32FC1 )
        CV_Error( Error::StsBadArg, "dispMap must have CV_32FC1 type" );
    if( sz && (dispMap.rows != sz->height || dispMap.cols != sz->width) )
        CV_Error( Error::StsBadArg, "dispMap has incorrect size" );
}

void DepthMap::checkTypeAndSizeOfMask( const Mat& mask, Size sz )
{
    if( mask.empty() )
        CV_Error( Error::StsBadArg, "mask is empty" );
    if( mask.type() != CV_8UC1 )
        CV_Error( Error::StsBadArg, "mask must have CV_8UC1 type" );
    if( mask.rows != sz.height || mask.cols != sz.width )
        CV_Error( Error::StsBadArg, "mask has incorrect size" );
}

void DepthMap::checkDispMapsAndUnknDispMasks( const Mat& leftDispMap, const Mat& rightDispMap,
                                    const Mat& leftUnknDispMask, const Mat& rightUnknDispMask )
{
    // check type and size of disparity maps
    checkTypeAndSizeOfDisp( leftDispMap, 0 );
    if( !rightDispMap.empty() )
    {
        Size sz = leftDispMap.size();
        checkTypeAndSizeOfDisp( rightDispMap, &sz );
    }

    // check size and type of unknown disparity maps
    if( !leftUnknDispMask.empty() )
        checkTypeAndSizeOfMask( leftUnknDispMask, leftDispMap.size() );
    if( !rightUnknDispMask.empty() )
        checkTypeAndSizeOfMask( rightUnknDispMask, rightDispMap.size() );

    // check values of disparity maps (known disparity values musy be positive)
    double leftMinVal = 0, rightMinVal = 0;
    if( leftUnknDispMask.empty() )
        minMaxLoc( leftDispMap, &leftMinVal );
    else
        minMaxLoc( leftDispMap, &leftMinVal, 0, 0, 0, ~leftUnknDispMask );
    if( !rightDispMap.empty() )
    {
        if( rightUnknDispMask.empty() )
            minMaxLoc( rightDispMap, &rightMinVal );
        else
            minMaxLoc( rightDispMap, &rightMinVal, 0, 0, 0, ~rightUnknDispMask );
    }
    if( leftMinVal < 0 || rightMinVal < 0)
        CV_Error( Error::StsBadArg, "known disparity values must be positive" );
}

/*
  Calculate occluded regions of reference image (left image) (regions that are occluded in the matching image (right image),
  i.e., where the forward-mapped disparity lands at a location with a larger (nearer) disparity) and non occluded regions.
*/
void DepthMap::computeOcclusionBasedMasks(const Mat& leftDisp,const Mat& _rightDisp,Mat* occludedMask,Mat* nonOccludedMask,
                             const Mat& leftUnknDispMask = Mat(),const Mat& rightUnknDispMask = Mat(),float dispThresh = 1.f )
{
    if( !occludedMask && !nonOccludedMask )
        return;
    checkDispMapsAndUnknDispMasks( leftDisp, _rightDisp, leftUnknDispMask, rightUnknDispMask );

    Mat rightDisp;
    if( _rightDisp.empty() )
    {
        if( !rightUnknDispMask.empty() )
           CV_Error( Error::StsBadArg, "rightUnknDispMask must be empty if _rightDisp is empty" );
        rightDisp.create(leftDisp.size(), CV_32FC1);
        rightDisp.setTo(Scalar::all(0) );
        for( int leftY = 0; leftY < leftDisp.rows; leftY++ )
        {
            for( int leftX = 0; leftX < leftDisp.cols; leftX++ )
            {
                if( !leftUnknDispMask.empty() && leftUnknDispMask.at<uchar>(leftY,leftX) )
                    continue;
                float leftDispVal = leftDisp.at<float>(leftY, leftX);
                int rightX = leftX - cvRound(leftDispVal), rightY = leftY;
                if( rightX >= 0)
                    rightDisp.at<float>(rightY,rightX) = max(rightDisp.at<float>(rightY,rightX), leftDispVal);
            }
        }
    }
    else
        _rightDisp.copyTo(rightDisp);

    if( occludedMask )
    {
        occludedMask->create(leftDisp.size(), CV_8UC1);
        occludedMask->setTo(Scalar::all(0) );
    }
    if( nonOccludedMask )
    {
        nonOccludedMask->create(leftDisp.size(), CV_8UC1);
        nonOccludedMask->setTo(Scalar::all(0) );
    }
    for( int leftY = 0; leftY < leftDisp.rows; leftY++ )
    {
        for( int leftX = 0; leftX < leftDisp.cols; leftX++ )
        {
            if( !leftUnknDispMask.empty() && leftUnknDispMask.at<uchar>(leftY,leftX) )
                continue;
            float leftDispVal = leftDisp.at<float>(leftY, leftX);
            int rightX = leftX - cvRound(leftDispVal), rightY = leftY;
            if( rightX < 0 && occludedMask )
                occludedMask->at<uchar>(leftY, leftX) = 255;
            else
            {
                if( !rightUnknDispMask.empty() && rightUnknDispMask.at<uchar>(rightY,rightX) )
                    continue;
                float rightDispVal = rightDisp.at<float>(rightY, rightX);
                if( rightDispVal > leftDispVal + dispThresh )
                {
                    if( occludedMask )
                        occludedMask->at<uchar>(leftY, leftX) = 255;
                }
                else
                {
                    if( nonOccludedMask )
                        nonOccludedMask->at<uchar>(leftY, leftX) = 255;
                }
            }
        }
    }
}

/*
  Calculate depth discontinuty regions: pixels whose neiboring disparities differ by more than
  dispGap, dilated by window of width discontWidth.
*/
void DepthMap::computeDepthDiscontMask( const Mat& disp, Mat& depthDiscontMask, const Mat& unknDispMask = Mat(),
                                 float dispGap = 2.f, int discontWidth = 9 )
{
    if( disp.empty() )
        CV_Error( Error::StsBadArg, "disp is empty" );
    if( disp.type() != CV_32FC1 )
        CV_Error( Error::StsBadArg, "disp must have CV_32FC1 type" );
    if( !unknDispMask.empty() )
        checkTypeAndSizeOfMask( unknDispMask, disp.size() );

    Mat curDisp; disp.copyTo( curDisp );
    if( !unknDispMask.empty() )
        curDisp.setTo( Scalar(numeric_limits<float>::min()), unknDispMask );
    Mat maxNeighbDisp; dilate( curDisp, maxNeighbDisp, Mat(3, 3, CV_8UC1, Scalar(1)) );
    if( !unknDispMask.empty() )
        curDisp.setTo( Scalar(numeric_limits<float>::max()), unknDispMask );
    Mat minNeighbDisp; erode( curDisp, minNeighbDisp, Mat(3, 3, CV_8UC1, Scalar(1)) );
    depthDiscontMask = max( (Mat)(maxNeighbDisp-disp), (Mat)(disp-minNeighbDisp) ) > dispGap;
    if( !unknDispMask.empty() )
        depthDiscontMask &= ~unknDispMask;
    dilate( depthDiscontMask, depthDiscontMask, Mat(discontWidth, discontWidth, CV_8UC1, Scalar(1)) );
}

/*
   Get evaluation masks excluding a border.
*/
Mat DepthMap::getBorderedMask( Size maskSize, int border = 10 )
{
    CV_Assert( border >= 0 );
    Mat mask(maskSize, CV_8UC1, Scalar(0));
    int w = maskSize.width - 2*border, h = maskSize.height - 2*border;
    if( w < 0 ||  h < 0 )
        mask.setTo(Scalar(0));
    else
        mask( Rect(Point(border,border),Size(w,h)) ).setTo(Scalar(255));
    return mask;
}

/*
  Calculate root-mean-squared error between the computed disparity map (computedDisp) and ground truth map (groundTruthDisp).
*/
float DepthMap::dispRMS( const Mat& computedDisp, const Mat& groundTruthDisp, const Mat& mask )
{
    checkTypeAndSizeOfDisp( groundTruthDisp, 0 );
    Size sz = groundTruthDisp.size();
    checkTypeAndSizeOfDisp( computedDisp, &sz );

    int pointsCount = sz.height*sz.width;
    if( !mask.empty() )
    {
        checkTypeAndSizeOfMask( mask, sz );
        pointsCount = countNonZero(mask);
    }
    //dont have ground trouth so i will just return 1.f
    return 1.f;
    //return 1.f/sqrt((float)pointsCount) * (float)cvtest::norm(computedDisp, groundTruthDisp, NORM_L2, mask);
}

/*
  Calculate fraction of bad matching pixels.
*/
float DepthMap::badMatchPxlsFraction( const Mat& computedDisp, const Mat& groundTruthDisp, const Mat& mask,float _badThresh = 1.f)
{
    int badThresh = cvRound(_badThresh);
    checkTypeAndSizeOfDisp( groundTruthDisp, 0 );
    Size sz = groundTruthDisp.size();
    checkTypeAndSizeOfDisp( computedDisp, &sz );

    Mat badPxlsMap;
    absdiff( computedDisp, groundTruthDisp, badPxlsMap );
    badPxlsMap = badPxlsMap > badThresh;
    int pointsCount = sz.height*sz.width;
    if( !mask.empty() )
    {
        checkTypeAndSizeOfMask( mask, sz );
        badPxlsMap = badPxlsMap & mask;
        pointsCount = countNonZero(mask);
    }
    return 1.f/pointsCount * countNonZero(badPxlsMap);
}

template <class T>
static void fillOcclusionInv_(Mat& src, Mat& image, const T invalidvalue, const T minval, int threshold)
{
    int bb=1;
    const int MAX_LENGTH=(int)(src.cols*0.9);
    //#pragma omp parallel for
    for(int j=bb;j<src.rows-bb;j++)
    {
        T* s = src.ptr<T>(j);
        uchar* imap = image.ptr<uchar>(j);

        s[0]=minval;
        s[src.cols-1]=minval;
        for(int i=1;i<src.cols-1;i++)
        {
            if(s[i]<=invalidvalue)
            {
                int t=i;
                int emax=0;
                int argt;
                do
                {
                    const int kstep = 2;
                    int E =
                        max(max(
                        abs(imap[3*(t-kstep)+0]-imap[3*(t+kstep)+0]),
                        abs(imap[3*(t-kstep)+1]-imap[3*(t+kstep)+1])),
                        abs(imap[3*(t-kstep)+2]-imap[3*(t+kstep)+2])
                        );
                    if(E>emax)
                    {
                        emax = E;
                        argt = t;
                    }
                    t++;
                    if(t>src.cols-2)break;
                }while(s[t]<=invalidvalue);

                if(emax>threshold && t-argt>2 && argt-i>2)
                {
                    const T dl = s[i-1];
                    const T	dr = s[t];
                    if(t-i>MAX_LENGTH)
                    {
                        for(;i<t;i++)
                        {
                            s[i]=invalidvalue;
                        }
                    }
                    else
                    {
                        //for(;i<t;i++)
                        //{
                        //	s[i]=invalidvalue;
                        //}
                        for(;i<argt;i++)
                        {
                            s[i]=dl;
                        }
                        for(;i<t;i++)
                        {
                            s[i]=dr;
                        }
                    }
                }
                else
                {
                    const T dd = max(s[i-1],s[t]);
                    if(t-i>MAX_LENGTH)
                    {
                        for(;i<t;i++)
                        {
                            s[i]=invalidvalue;
                        }
                    }
                    else
                    {
                        for(;i<t;i++)
                        {
                            s[i]=dd;
                        }
                    }
                }
            }
        }
        s[0]=s[1];
        s[src.cols-1]=s[src.cols-2];
    }

    T* s1 = src.ptr<T>(0);
    T* s2 = src.ptr<T>(1);
    T* s3 = src.ptr<T>(src.rows-2);
    T* s4 = src.ptr<T>(src.rows-1);
    for(int i=0;i<src.cols;i++)
    {
        s1[i]=s2[i];
        s4[i]=s3[i];
    }
}
template <class T>
static void fillOcclusionInv_(Mat& src, const T invalidvalue, const T minval)
{
    int bb=1;
    const int MAX_LENGTH=(int)(src.cols*0.3);
    //#pragma omp parallel for
    for(int j=bb;j<src.rows-bb;j++)
    {
        T* s = src.ptr<T>(j);

        s[0]=minval;
        s[src.cols-1]=minval;
        for(int i=1;i<src.cols-1;i++)
        {
            if(s[i]<=invalidvalue)
            {
                int t=i;
                do
                {
                    t++;
                    if(t>src.cols-2)break;
                }while(s[t]<=invalidvalue);

                const T dd = max(s[i-1],s[t]);
                if(t-i>MAX_LENGTH)
                {
                    for(;i<t;i++)
                    {
                        s[i]=invalidvalue;
                    }
                }
                else
                {
                    for(;i<t;i++)
                    {
                        s[i]=dd;
                    }
                }
            }
        }
        s[0]=s[1];
        s[src.cols-1]=s[src.cols-2];
    }

    T* s1 = src.ptr<T>(0);
    T* s2 = src.ptr<T>(1);
    T* s3 = src.ptr<T>(src.rows-2);
    T* s4 = src.ptr<T>(src.rows-1);
    for(int i=0;i<src.cols;i++)
    {
        s1[i]=s2[i];
        s4[i]=s3[i];
    }
}
void DepthMap::fillOcclusionDepth(Mat& src, int invalidvalue)
{
    {
        if(src.type()==CV_8U)
        {
            fillOcclusionInv_<uchar>(src, (uchar)invalidvalue, 0);
        }
        else if(src.type()==CV_16S)
        {
            fillOcclusionInv_<short>(src, (short)invalidvalue, 0);
        }
        else if(src.type()==CV_16U)
        {
            fillOcclusionInv_<unsigned short>(src, (unsigned short)invalidvalue, 0);
        }
        else if(src.type()==CV_32F)
        {
            fillOcclusionInv_<float>(src, (float)invalidvalue,0);
        }
    }
}

void DepthMap::fillOcclusionDepth(Mat& depth, Mat& image, int invalidvalue, int threshold)
{
    if(depth.type()==CV_8U)
    {
        fillOcclusionInv_<uchar>(depth, image, (uchar)invalidvalue, 0, threshold);
    }
    else if(depth.type()==CV_16S)
    {
        fillOcclusionInv_<short>(depth, image, (short)invalidvalue, 0, threshold);
    }
    else if(depth.type()==CV_16U)
    {
        fillOcclusionInv_<unsigned short>(depth, image, (ushort)invalidvalue, 0, threshold);
    }
    else if(depth.type()==CV_32F)
    {
        fillOcclusionInv_<float>(depth, image, (float)invalidvalue, 0, threshold);
    }
}

template <class T>
static void fillOcclusion_(Mat& src, const T invalidvalue, const T maxval)
{
    int bb=0;
    const int MAX_LENGTH=(int)(src.cols*1.0-5);

    T* s = src.ptr<T>(0);
    const int step = src.cols;
    //Mat testim = Mat::zeros(src.size(),CV_8UC3);const int lineth = 30;
    //check right
    {
        T* s = src.ptr<T>(0);
        s+=(src.cols-1);
        const int step = src.cols;
        for(int j=0;j<src.rows;j++)
        {
            if(s[0]==invalidvalue)
                s[0] = s[-step];
            s+=step;
        }
    }

    //int table[500];
    const int leftmax=64;
    for(int j=0;j<src.rows;j++)
    {
        s[0]=maxval;//可能性のある最大値を入力
        s[src.cols-1]=maxval;//可能性のある最大値を入力
        //もし視差が0だったら値の入っている近傍のピクセル（エピポーラ線上）の最小値で埋める
        int i=1;
        // check left
        //if(j!=0)
        {
            for(;i<src.cols-1;i++)
            {
                if(s[i]<=invalidvalue)
                {
                    int t=i;
                    do
                    {
                        t++;
                        if(t>leftmax)break;
                    }while(s[t]<=invalidvalue);
                    T dd = s[t];
                    //table[j]=t;

                    if(t>leftmax)
                    {
                        //for(int n=0;n<src.cols;n++)s[n]=invalidvalue;
                        //memcpy(s,s-step,step*sizeof(T));
                        i=1;break;
                    }
                    else
                    {
                        double dsub=0.0;
                        int n=1;
                        for(;n<128;n++)
                        {
                            if( abs(s[t+n] - dd)> 1)break;
                        }
                        const int n1=n;
                        T d2 = s[t+n];
                        n++;
                        for(;n<128;n++)
                        {
                            if( abs(s[t+n] - d2)> 0)break;
                        }
                        dsub = 2.0/(double)(n-n1);

                        //dsub = 1.0/(double)(n1);


                        //if(s[t+n]-s[t+n1]>0)dsub*=-1;

                        //cout<<j<<": "<<dsub<<endl;
                        for(;i<t+1;i++)
                        {
                            //s[i]=(0.33333333*s[i-step] + 0.66666667*(dd +dsub*(t-i))) ;
                            s[i]=(T)(dd +dsub*(t-i)+0.5);
                        }
                    }
                    break;
                }
            }
            //main
            for(;i<src.cols-1;i++)
            {
                if(s[i]<=invalidvalue)
                {
                    if(s[i+1]>invalidvalue)
                    {
                        s[i]=min(s[i+1],s[i-1]);
                        i++;continue;
                    }

                    int t=i;
                    do
                    {
                        t++;
                        if(t>src.cols-2)break;
                    }while(s[t]<=invalidvalue);

                    //if(t-i>lineth)line(testim,Point(i,j),Point(t,j),CV_RGB(255,0,0));

                    T dd;
                    //if(s[i-1]<=invalidvalue)dd=s[t];
                    //else if(s[t]<=invalidvalue)dd=s[i-1];
                    //else dd = min(s[i-1],s[t]);
                    dd = min(s[i-1],s[t]);
                    if(t-i>MAX_LENGTH)
                    {
                        //for(int n=0;n<src.cols;n++)s[n]=invalidvalue;
                        memcpy(s,s-step,step*sizeof(T));
                    }
                    else
                    {
                        /*const int n=i-1;
                        double dsub=0.0;
                        if(abs(s[i-1]-s[t])<21)
                        dsub=abs(s[n]-s[t])/(double)(t-n+1);
                        if(s[i-1]==dd)
                        {
                        for(;i<t;i++)
                        {
                        s[i]=(dd + dsub*(i-n));
                        }
                        }
                        else
                        {
                        for(;i<t;i++)
                        {
                        s[i]=(dd -dsub*(i-n));
                        }
                        }*/

                        for(;i<t;i++)s[i]=dd;
                    }
                }
            }
            s[0]=s[1];
            s[src.cols-1]=s[src.cols-2];
            s+=step;
        }
        //imshow("test",testim);
    }
}
void DepthMap::fillOcclusion(Mat& src, int invalidvalue)
{
    {
        if(src.type()==CV_8U)
        {
            fillOcclusion_<uchar>(src, (uchar)invalidvalue, UCHAR_MAX);
        }
        else if(src.type()==CV_16S)
        {
            fillOcclusion_<short>(src, (short)invalidvalue, SHRT_MAX);
        }
        else if(src.type()==CV_16U)
        {
            fillOcclusion_<unsigned short>(src, (unsigned short)invalidvalue, USHRT_MAX);
        }
        else if(src.type()==CV_32F)
        {
            fillOcclusion_<float>(src, (float)invalidvalue,FLT_MAX);
        }
    }
}

void DepthMap::removeStreakingNoise(Mat& src, Mat& dest, int th)
{
    Mat dst;
    src.copyTo(dst);
    int bb=1;
    int j;

    int t2 = th;
    const int step=2*src.cols;
    const int step2=3*src.cols;
    //#pragma omp parallel for
    if(src.type()==CV_8U)
    {
        for(j=bb;j<src.rows-bb;j++)
        {
            int i;
            uchar* d = dst.ptr(j);
            uchar* s = src.ptr(j-1);
            for(i=0;i<src.cols;i++)
            {
                if(abs(s[i]-s[step+i])<t2)
                    d[i]=(s[i]+s[step+i])>>1;
            }
        }
    }
    if(src.type()==CV_16S)
    {
        const int istep = src.cols;
        short* d = dst.ptr<short>(bb);
        short* s = src.ptr<short>(bb-1);

        for(j=bb;j<src.rows-bb-1;j++)
        {
            int i;
            for(i=0;i<src.cols;i++)
            {
                if(abs(s[i]-s[step2+i])<t2)
                {
                    short v=(s[i]+s[step2+i])>>1;
                    if(abs(v-d[i])>t2)
                    {
                        d[i]=v;
                        d[i+istep]=v;
                    }
                }
                if(abs(s[i]-s[step+i])<t2)
                {
                    short v=(s[i]+s[step+i])>>1;
                    if(abs(v-d[i])>t2)
                        d[i]=v;
                }
            }
            s+=istep;
            d+=istep;
        }
    }
        if(src.type()==CV_16U)
    {
        const int istep = src.cols;
        ushort* d = dst.ptr<ushort>(bb);
        ushort* s = src.ptr<ushort>(bb-1);

        for(j=bb;j<src.rows-bb-1;j++)
        {
            int i;
            for(i=0;i<src.cols;i++)
            {
                if(abs(s[i]-s[step2+i])<t2)
                {
                    ushort v=(s[i]+s[step2+i])>>1;
                    if(abs(v-d[i])>t2)
                    {
                        d[i]=v;
                        d[i+istep]=v;
                    }
                }
                if(abs(s[i]-s[step+i])<t2)
                {
                    ushort v=(s[i]+s[step+i])>>1;
                    if(abs(v-d[i])>t2)
                        d[i]=v;
                }
            }
            s+=istep;
            d+=istep;
        }
    }

    dst.copyTo(dest);
}
void removeStreakingNoiseV(Mat& src, Mat& dest, int th)
{
    Mat dst;
    src.copyTo(dst);

    int bb=0;
    int j;

    int t2 = th;
    const int step=2*src.cols;
    const int step2=3*src.cols;
    //#pragma omp parallel for
    if(src.type()==CV_8U)
    {
        for(j=bb;j<src.rows-bb;j++)
        {
            int i;
            uchar* d = dst.ptr(j);
            uchar* s = src.ptr(j);
            for(i=1;i<src.cols-1;i++)
            {
                if(abs(s[i-1]-s[i+1])<t2)
                    d[i]=(s[i-1]+s[i+1])>>1;
            }
        }
    }
    if(src.type()==CV_16S)
    {
        const int istep = src.cols;
        short* d = dst.ptr<short>(0);d++;
        short* s = src.ptr<short>(0);

        for(j=0;j<src.rows;j++)
        {
            int i;
            for(i=2;i<src.cols-2;i++)
            {
                if(abs(s[i]-s[3+i])<t2)
                {
                    short v=(s[i]+s[3+i])>>1;
                    if(abs(v-d[i])>t2)
                    {
                        d[i]=v;
                        d[i+1]=v;
                    }
                }
                if(abs(s[i]-s[2+i])<t2)
                {
                    short v=(s[i]+s[2+i])>>1;
                    if(abs(v-d[i])>t2)
                        d[i]=v;
                }
            }
            s+=istep;
            d+=istep;
        }
    }
    if(src.type()==CV_16U)
    {
        const int istep = src.cols;
        ushort* d = dst.ptr<ushort>(0);d++;
        ushort* s = src.ptr<ushort>(0);

        for(j=0;j<src.rows;j++)
        {
            int i;
            for(i=2;i<src.cols-2;i++)
            {
                if(abs(s[i]-s[3+i])<t2)
                {
                    ushort v=(s[i]+s[3+i])>>1;
                    if(abs(v-d[i])>t2)
                    {
                        d[i]=v;
                        d[i+1]=v;
                    }
                }
                if(abs(s[i]-s[2+i])<t2)
                {
                    ushort v=(s[i]+s[2+i])>>1;
                    if(abs(v-d[i])>t2)
                        d[i]=v;
                }
            }
            s+=istep;
            d+=istep;
        }
    }
    dst.copyTo(dest);
}
