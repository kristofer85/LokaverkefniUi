#include "stereocalibrate.h"
#include <opencv2/core/utility.hpp>
#include <opencv2/ximgproc/disparity_filter.hpp>
#include <opencv2/ximgproc.hpp>
#include <opencv2/core/ocl.hpp>
#include <opencv2/core/base.hpp>
#include <opencv2/core/cvdef.h>
#include <opencv2/core/core_c.h>
#include <opencv2/ccalib.hpp>
#include <opencv2/stereo/stereo.hpp>
#include "utils.h"
#include "string"
using namespace cv;
using namespace std;
using namespace cv::ximgproc;


StereoCalibrate::StereoCalibrate()//
{

    patternSize = 24.0;
    //patternSize = 5;

}

void StereoCalibrate::clean()
{
    ChessHd.release();
    img1.release();
    img2.release();
    gray1.release();
    gray2.release();
    foundImages.release();

    R1.release();
    R2.release();
    P1.release();
    P2.release();
    Q.release();
    map1x.release();
    map1y.release();
    map2x.release();
    map2y.release();
    imgU1.release();
    imgU2.release();

    CM1.release();// = Mat(3, 3, CV_32FC1);
    CM2.release();// = Mat(3, 3, CV_32FC1);
    D1.release();
    D2.release();
    R.release();
    T.release();
    E.release();
    F.release();

    objectpoints.clear();
    imagePoints1.clear();
    imagePoints2.clear();
    corners1.clear();
    corners2.clear();
    obj.clear();

}




void StereoCalibrate::findAndDrawChessBoardCorners(string filename)
{
    String l = "left_color_536x712.png";
    String r = "right_color_536x712.png";
    String l_gGray = "left_color_536x712.png";
    String r_gGray = "right_color_536x712.png";
    numBoards = 24;
    board_w = 10;
    board_h = 7;
    board_sz = Size(board_w, board_h);
    board_n = board_w*board_h;

    FileStorage fs;
    fs.open(filename, FileStorage::READ);
    if (!fs.isOpened()){cerr << "Failed to open " << filename << endl;}
    FileNode n = fs["images"];                         // Read string sequence - Get node
    if (n.type() != FileNode::SEQ){cerr << "strings is not a sequence! FAIL" << endl;}
    string ChessboardImageList,left, right;
    FileNodeIterator it = n.begin(), it_end = n.end(); // Go through the node
    int success = 0;
    bool first = true;
    for (int i=0; i<board_h; i++)
        for (int j=0; j<board_w; j++)
            obj.push_back(Point3f(i *patternSize, j *patternSize, 0.0f));
    for (; it != it_end; ++it)
    {
        //ChessboardImageList = CHESSBOARDIMAGES;
        ChessboardImageList = CHESSBOARDKULAIMAGESOLD;
        ChessboardImageList.append((string)*it);
        ChessHd = imread(ChessboardImageList,IMREAD_COLOR);
        imSize = ChessHd.size();
        img1 = ChessHd(Range(0, imSize.height),Range(0, imSize.width/2)).clone();
        img2 = ChessHd(Range(0, imSize.height),Range(imSize.width/2, imSize.width)).clone();
        //resize(img1,img1,Size(),0.25,0.25);
        //resize(img2,img2,Size(),0.25,0.25);
        resize(img1,img1,Size(),0.50,0.50);
        resize(img2,img2,Size(),0.50,0.50);
        //cout << "chessboard img size" << img1.cols <<" " << img1.rows << endl;

        cutSize = img1.size();
        cvtColor(img1,gray1,CV_RGB2GRAY);
        cvtColor(img2,gray2,CV_RGB2GRAY);
/*
        left = LEFTRIGHTSCALED;
        left.append(l);
        imwrite(left,img1);         // Left Color image used for coloring the polyMesh
        right = LEFTRIGHTSCALED;
        right.append(r);
        imwrite(right,img2);        // Right Color not as inportant.
        left = LEFTRIGHTSCALED;
        left.append(l_gGray);
        imwrite(l_gGray,gray1);            // Left B&W Image used for comarison in depth mapping
        right = LEFTRIGHTSCALED;
        right.append(r_gGray);
        imwrite(r_gGray,gray2);           // Left B&W Image, the image left compares itself with.
*/
        bool found1 = false;
        bool found2 = false;

        found1 = findChessboardCorners(img1, board_sz, corners1, CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FILTER_QUADS);
        found2 = findChessboardCorners(img2, board_sz, corners2, CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FILTER_QUADS);
        //found1 = findChessboardCorners(img1, board_sz, corners1, CALIB_CB_ADAPTIVE_THRESH | CALIB_CB_NORMALIZE_IMAGE);
        //found2 = findChessboardCorners(img2, board_sz, corners2, CALIB_CB_ADAPTIVE_THRESH | CALIB_CB_NORMALIZE_IMAGE);

        if (found1)
        {
            cornerSubPix(gray1, corners1, Size(11, 11), Size(-1, -1), TermCriteria(CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 30, 0.1));
            drawChessboardCorners(img1, board_sz, corners1, found1);
        }

        if (found2)
        {
            cornerSubPix(gray2, corners2, Size(11, 11), Size(-1, -1), TermCriteria(CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 30, 0.1));
            drawChessboardCorners(img2, board_sz, corners2, found2);
        }

        if (found1 !=0 && found2 != 0)
        {
            if(first == true)
            {
                imwrite("chessboardDrawL.png",img1);
                imwrite("chessboardDrawR.png",img2);
                first = false;
            }
            success++;
            //find translation between corners in
            cout << " image: " << ChessboardImageList << " "<< success << endl;
            int cornerNr = (int)corners1.size();
            /*
            for(int i = 0; i < cornerNr; i++)
            {
                Point2d left = corners1[i];
                Point2d right = corners2[i];
                cout << "Y left = " <<left.y << " Y right = " << right.y << " difference =" << left.y-right.y << endl;
                cout << "X left = " <<left.x << " X right = " << right.x << " difference =" << left.x-right.x << endl;
            }
            */

            imagePoints1.push_back(corners1);
            imagePoints2.push_back(corners2);
            objectpoints.push_back(obj);

            /*
            ChessboardImageList = CHESSBOARDIMAGES;
            ChessboardImageList.append("leftFound"+(string)*it);
            imwrite(ChessboardImageList,gray1);
            ChessboardImageList = CHESSBOARDIMAGES;
            ChessboardImageList.append("rightFound"+(string)*it);
            imwrite(ChessboardImageList+(string)*it,gray2);
            */

        }

    }
    cout << "found " << success << " of " << numBoards << " pairs" << endl;
    fs.release();
    ChessHd.release();
}

void StereoCalibrate::CalibrateStereoCamera()
{

    //CM1 = getCameraMatrix(zoom_valueC*focalResC, imSize.width, imSize.height);
    //CM2 = getCameraMatrix(zoom_valueC*focalResC, imSize.width, imSize.height);

    cout << "cutSize is "<< cutSize << endl;

    string file_name = "../Lokaverkefni2/chessboardKulaImages/DSC_0209_sbs.jpg";
    //string file_name = "../Lokaverkefni2/chessboardImages/calib1_fixed.jpg";


    Mat Image = imread(file_name,IMREAD_COLOR);
    Size imSize = Image.size();
    Mat img1 = Image(Range(0, imSize.height),Range(0, imSize.width/2)).clone();
    Mat img2 = Image(Range(0, imSize.height),Range(imSize.width/2, imSize.width)).clone();
    //resize(img1,img1,Size(),0.50,0.50);
    //resize(img2,img2,Size(),0.50,0.50);
    resize(img1,img1,Size(),0.25,0.25);
    resize(img2,img2,Size(),0.25,0.25);
    cout << "calib size" << img1.cols <<" " << img1.rows << endl;

    /*
    string imagePath = "C:/calibmyndir/DSC_0071.JPG";
    Mat distCoeffs;

    double zoom_value = getZoomValue(imagePath);

    if(!getDistCoeffs(distCoeffs, zoom_value, file_name))
    {
            qDebug()<<"Failed to get distortion coeffs";
    }
    double focalRes = getFocalResolution(imagePath);
    cout << "focal = " << focalRes << endl;
    cout << "Zoom = " << zoom_value << endl;
    if (focalRes*zoom_value < 1)
    {
            qDebug()<<"No focal resolution found";
    }
    */
    //CM1 = getCameraMatrix(1, img1.cols, img1.rows);
    //CM2 = getCameraMatrix(1, img2.cols+100, img2.rows+50);
    CM1 = initCameraMatrix2D(objectpoints,imagePoints1,cutSize,0);
    CM2 = initCameraMatrix2D(objectpoints,imagePoints2,cutSize,0);


    cout << "distCoeff " << D1 << endl;
    cout << "camera1 = " << CM1 << endl;
    cout << "camera2 = " << CM2 << endl;

    FileStorage stereoCalibrationStored = FileStorage("stereoCalibration.yml", FileStorage::WRITE);
    double rms = stereoCalibrate(objectpoints,
                                 imagePoints1,
                                 imagePoints2,
                                 CM1, D1, CM2, D2,
                                 cutSize,
                                 R, T, E, F,
                                 CV_CALIB_FIX_ASPECT_RATIO +
                                 CV_CALIB_ZERO_TANGENT_DIST +
                                 //CV_CALIB_FIX_INTRINSIC +
                                 CV_CALIB_RATIONAL_MODEL +
                                 //CV_CALIB_FIX_PRINCIPAL_POINT +
                                 //CV_CALIB_FIX_K1+CV_CALIB_FIX_K2+CV_CALIB_FIX_K3+
                                 CV_CALIB_USE_INTRINSIC_GUESS+
                                 //CALIB_FIX_K3 + CALIB_FIX_K4 + CALIB_FIX_K5 +
                                 CV_CALIB_SAME_FOCAL_LENGTH
                                 ,
                                 cvTermCriteria(CV_TERMCRIT_ITER+CV_TERMCRIT_EPS, 100, 1e-5));
    cout << "done with RMS error=" << rms << endl;




/*
    CM1 = limit_precision_mat(CM1,4);
    CM2 = limit_precision_mat(CM2,4);
    R = limit_precision_matF(R,4);
    T = limit_precision_mat(T,4);
    F = limit_precision_mat(F,4);
    E = limit_precision_mat(E,4);

    cout << "distCoeff " << D1 << endl;
    cout << "camera1 = " << CM1 << endl;
    cout << "camera2 = " << CM2 << endl;
    cout << "Rotation = " << R << endl;
    cout << "Translation = " << T << endl;
*/

    double err = 0;
    int npoints = 0;
    vector<Vec3f> lines[2];
    for(int i = 0; i < 9; i++ )
    {
      //  cout << "I = " << i << endl;
        int npt = (int)imagePoints1[i].size();
        Mat imgpt[2];

            imgpt[0] = Mat(imagePoints1[i]);
            undistortPoints(imgpt[0], imgpt[0], CM1, D1, Mat(), CM1);
            computeCorrespondEpilines(imgpt[0], 1, F, lines[0]);

            imgpt[1] = Mat(imagePoints2[i]);
            undistortPoints(imgpt[1], imgpt[1], CM2, D2, Mat(), CM2);
            computeCorrespondEpilines(imgpt[1], 1+1, F, lines[1]);

        for(int j = 0; j < npt; j++ )
        {
            double errij = fabs(imagePoints1[i][j].x*lines[1][j][0] +
                                imagePoints1[i][j].y*lines[1][j][1] + lines[1][j][2]) +
                           fabs(imagePoints2[i][j].x*lines[0][j][0] +
                                imagePoints2[i][j].y*lines[0][j][1] + lines[0][j][2]);
            err += errij;
        }
        npoints += npt;
    }
    cout << "average reprojection err = " <<  err/npoints << endl;



    stereoCalibrationStored << "CM1" << CM1;
    stereoCalibrationStored << "CM2" << CM2;
    stereoCalibrationStored << "D1" << D1;
    stereoCalibrationStored << "D2" << D2;
    stereoCalibrationStored << "R" << R;
    stereoCalibrationStored << "T" << T;
    stereoCalibrationStored << "E" << E;
    stereoCalibrationStored << "F" << F;
    stereoCalibrationStored.release();


}

void StereoCalibrate::rectifyCamera()
{

    FileStorage stereoCalibrationStored = FileStorage("stereoCalibration.yml", FileStorage::READ);
    stereoCalibrationStored["CM1"] >> CM1;
    stereoCalibrationStored["D1"] >> D1;
    stereoCalibrationStored["CM2"] >> CM2;
    stereoCalibrationStored["D2"] >> D2;
    stereoCalibrationStored["R"] >> R;
    stereoCalibrationStored["T"] >> T;
    cout << "rectify img1 size" << img1.cols <<" " << img1.rows << endl;
    cout << "rectify cutsize" << cutSize.width <<" " << cutSize.height << endl;
    //stereoRectify( CM1, D1, CM2, D2, img1.size(), R, T, R1, R2, P1, P2, Q, CALIB_ZERO_DISPARITY );
    stereoRectify( CM1, D1, CM2, D2, img1.size(), R, T, R1, R2, P1, P2, Q,0,0,cutSize);
    //stereoRectify( CM1, D1, CM2, D2, img1.size(), R, T, R1, R2, P1, P2, Q,0,0);
//*******************************
// ToDo Write Data to DATA_HOLDER
//*******************************

    stereoCalibrationStored = FileStorage("stereoCalibration.yml", FileStorage::APPEND);
    stereoCalibrationStored << "R1" << R1;
    stereoCalibrationStored << "R2" << R2;
    stereoCalibrationStored << "P1" << P1;
    stereoCalibrationStored << "P2" << P2;
    //stereoCalibrationStored << "Q" << Q;
    stereoCalibrationStored.release();
}

Rect computeROI(Size2i src_sz, Ptr<StereoMatcher> matcher_instance)
{
    int min_disparity = matcher_instance->getMinDisparity();
    int num_disparities = matcher_instance->getNumDisparities();
    int block_size = matcher_instance->getBlockSize();

    int bs2 = block_size/2;
    int minD = min_disparity, maxD = min_disparity + num_disparities - 1;

    int xmin = maxD + bs2;
    int xmax = src_sz.width + minD - bs2;
    int ymin = bs2;
    int ymax = src_sz.height - bs2;

    Rect r(xmin, ymin, xmax - xmin, ymax - ymin);
    return r;
}

matPair StereoCalibrate::initUndistort(matPair Pair)
{
    /*
    //string bleh = CALIBFOLDER;
    string bleh = CHESSBOARDKULAIMAGESOLD;
    //string bleh = CHESSBOARDIMAGES;

    bleh.append("DSC_0209_sbs.jpg");
    //bleh.append("calib1_fixed.jpg");
    //bleh.append("DSC_0025.JPG");
    Mat fullImg = imread(bleh,IMREAD_COLOR);

    //pyrDown(fullImg,fullImg,Size(fullImg.cols/2,fullImg.rows/2));
    //split sterio pic into two images
    //Size imSize = fullImg.size();

    Mat img1 = fullImg(Range(0, imSize.height),Range(0, imSize.width/2)).clone();
    Mat img2 = fullImg(Range(0, imSize.height),Range(imSize.width/2, imSize.width)).clone();
    */
    img1 = Pair.left;
    img2 = Pair.right;
    Mat i1;
    Mat i2;
    //resize(img1, img1, Size(), 0.50, 0.50);
    //resize(img2, img2, Size(), 0.50, 0.50);

    cout << "remap input img1 size" << img1.cols <<" " << img1.rows << endl;
    cout << "remap input img2 size" << img2.cols <<" " << img2.rows << endl;

    FileStorage stereoCalibrationStored = FileStorage("stereoCalibration.yml", FileStorage::READ);
    stereoCalibrationStored["CM1"] >> CM1;
    stereoCalibrationStored["CM2"] >> CM2;
    stereoCalibrationStored["R1"] >> R1;
    stereoCalibrationStored["R2"] >> R2;
    stereoCalibrationStored["P1"] >> P1;
    stereoCalibrationStored["P2"] >> P2;
    stereoCalibrationStored.release();

    //Mat img1 = fullImg(Range(0, imSize.height),Range(0, imSize.width/2)).clone();
    //imwrite("leftColor.png",img1);
    //Mat img2 = fullImg(Range(0, imSize.height),Range(imSize.width/2, imSize.width)).clone();
    //imwrite("rightColor.png",img2);
    //Mat newImg= Mat::zeros(img1.size().height, img1.size().width, img1.type());
    //shift_image(img1,&newImg,100,10);
       //CM1 = getCameraMatrix(zoom_value*focalRes, img1.cols, img1.rows);
       //CM2 = getCameraMatrix(zoom_value*focalRes, img2.cols, img2.rows);
    cout << " R matrix" << endl;
    cout << R << endl;
    //T.at<double>(0) += 50;
    //stereoRectify( CM1, D1, CM2, D2, img1.size(), R, T, R1, R2, P1, P2, Q ,0,0);
    //initUndistortRectifyMap(CM1, D1, Mat(), P1, img1.size(), CV_16SC2, map1x, map1y);
    initUndistortRectifyMap(CM1, D1, R1, P1, img1.size(), CV_16SC2, map1x, map1y);
    //initUndistortRectifyMap(CM2, D2, Mat(), P2, img2.size(), CV_16SC2, map2x, map2y);
    initUndistortRectifyMap(CM2, D2, R2, P2, img2.size(), CV_16SC2, map2x, map2y);

    remap(img1, i1, map1x, map1y, INTER_LINEAR);
    remap(img2, i2, map2x, map2y, INTER_LINEAR);
/*
    namedWindow("imageOrg",CV_WINDOW_KEEPRATIO);
    namedWindow("image1",CV_WINDOW_KEEPRATIO);
    namedWindow("imageOrg2",CV_WINDOW_KEEPRATIO);
    namedWindow("image2",CV_WINDOW_KEEPRATIO);
    imshow("imageOrg",img1);
    imshow("image1",i1);
    imshow("imageOrg2",img2);
    imshow("image2",i2);
    cout << "remap i1 size" << i1.cols <<" " << i1.rows << endl;
    cout << "remap i2 size" << i2.cols <<" " << i2.rows << endl;
    waitKey();
    destroyAllWindows();
*/

/*
    string img1Path = CHESSBOARDKULAIMAGES;
    img1Path.append("leftUndistorted.png");
    imwrite(img1Path,i1);
    img1Path = CHESSBOARDKULAIMAGES;
    img1Path.append("rightUndistorted.png");
    imwrite(img1Path,i2);


    int rows = i1.rows;
    int cols = i2.cols;
    Mat dst = cv::Mat(rows, 2 * cols, i1.type());
    cv::Mat tmp = dst(cv::Rect(0, 0, cols, rows));
    i1.copyTo(tmp);
    tmp = dst(cv::Rect(cols, 0, cols, rows));
    i2.copyTo(tmp);
    imwrite("rectifiedNoShift.png",dst);
*/

    cout << "DONE " << endl;
    matPair returnPair;
    //int shift = 0.2*i1.cols;
    int shift = 0;
    cout << "shift = " << shift << endl;
    returnPair.left = i1(Range(0, i1.rows),Range(0+shift, i1.cols)).clone();
     //Mat img2 = fullImg(Range(0, imSize.height),Range(imSize.width/2, imSize.width)).clone();
    returnPair.right = i2(Range(0, i2.rows),Range(0, i2.cols-shift)).clone();
    cout << "returnPair.left size" << returnPair.left.cols <<" " << returnPair.left.rows << endl;
    cout << "returnPair.right size" << returnPair.right.cols <<" " << returnPair.right.rows << endl;
    return returnPair;
}




