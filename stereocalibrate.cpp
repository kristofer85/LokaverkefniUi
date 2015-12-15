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


//initializes sterio calibrate information patern size as 24 since the the size
//of the squares the patternwe used is 24 mm wide
StereoCalibrate::StereoCalibrate()//
{
    patternSize = 24.0;
    //patternSize = 5;
}

<<<<<<< HEAD
//releases the memory used by the variables in this class and sets them to null
=======
//  deallocates allocated memory for the following Mats
// clears vectors containing corner positions for CameraCalibration
>>>>>>> origin/master
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
//reads in an xml file containing a list of calibration images it then loads each
//image in that list and checks if it can find a patern 10 squeres wide and 7 squere high
//if it can find the pattern in both left and right side of the image it then inserts
//the x and y values of the corners of every square into vectors for left and righ side
//along with points 3d point for each corner in the pattern
void StereoCalibrate::findAndDrawChessBoardCorners(string filename)
{
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

            imagePoints1.push_back(corners1);
            imagePoints2.push_back(corners2);
            objectpoints.push_back(obj);
        }
    }
    cout << "found " << success << " of " << numBoards << " pairs" << endl;
    fs.release();
    ChessHd.release();
}

//uses the points gotten by findAndDrawChessBoardCorners to
//it starts by loading in on image that has had lens distortion fixed for sample
//it then  initializes two camera matrixes using the object points and image points
//it then uses those camera matrixes along with the object and image points to calculate
//how much points are shifted between left and right image
//finaly it saves the information about the shifting to a file in to form of several
//matrixes that will later be used by the function rectifyCamera
void StereoCalibrate::CalibrateStereoCamera()
{
    cout << "cutSize is "<< cutSize << endl;
    string file_name = "../LokaverkefniUi/chessboardKulaImages/DSC_0209_sbs.jpg";

    Mat Image = imread(file_name,IMREAD_COLOR);
    Size imSize = Image.size();
    Mat img1 = Image(Range(0, imSize.height),Range(0, imSize.width/2)).clone();
    Mat img2 = Image(Range(0, imSize.height),Range(imSize.width/2, imSize.width)).clone();
    resize(img1,img1,Size(),0.50,0.50);
    resize(img2,img2,Size(),0.50,0.50);
    //resize(img1,img1,Size(),0.25,0.25);
    //resize(img2,img2,Size(),0.25,0.25);
    cout << "calib size" << img1.cols <<" " << img1.rows << endl;

    // provided initial cameramatrix for the CV_CALIB_USE_INTRINSIC_GUESS flag in stereoCalibrate
    CM1 = initCameraMatrix2D(objectpoints,imagePoints1,cutSize,0);
    CM2 = initCameraMatrix2D(objectpoints,imagePoints2,cutSize,0);
    cout << "distCoeff " << D1 << endl;
    cout << "camera1 = " << CM1 << endl;
    cout << "camera2 = " << CM2 << endl;
    // File storage for Stereo calibrate matrixes
    FileStorage stereoCalibrationStored = FileStorage("stereoCalibration.yml", FileStorage::WRITE);
    // This function is where the stereo calibration is done, It takes objectpoints that contain vector of obj that contain world
    //   coord for every corner found starting at X = 0, Y = 0 increasing one patternSize for every corner The Z position is always 0.
    // imagePoints are the found corners and are mapped to he coord in the obj vectors in objectpoints vector giving us scale.
    // Cm1 & Cm2 are 3x3 matrix containing  focal length and image center as variables fx,fy,cx and cy.
    // D1 & D2 are 5x1 matrix containing the distortion matrix of the image.
    // R = rotation matrix that holds the information of how to rotate the camera to have the same angle as the first one.
    // T = translate matrix that holds the information of how to translate the camera to have the same position as the first one
    // The matrix E contains information about the translation and rotation that relate the two cameras in physical space
    // The matrix F F contains the same information as E in addition to information about the intrinsics of both cameras.
    // CV_CALIB_FIX_ASPECT_RATIO will kepp the ratio of fx and fy to whatever value is set in the intrinsic_matrix
    // fl ag turns off fi tting the tangential distortion parameters p1 and
    // CV_CALIB_ZERO_TANGENT_DIST if this flag is set it turns off fitting the tangential distortion parameters p1 and p2, which are thereby both set to 0.
    // CV_CALIB_USE_INTRINSIC_GUESS if this flag is set then intrinsic_matrix is assumed to contain valid values that will be used as an initial guess to be further optimized by cvCalibrateCamera2().
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

    // prints out RMS error
    cout << "done with RMS error=" << rms << endl;

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

//this function uses the matrixes that CalibrateStereoCamera created to
//create seperate rotation and projection matrixes for left and right side
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

    // Return parameters are Rl and Rr, the 3-by-3 row-aligned rectifi cation rotations for the
    // left and right image planes as derived in the preceding equations. Similarly, we get back
    // the 3-by-4 left and right projection equations Pl and Pr. An optional return parameter is
    // Q, the 4-by-4 reprojection matrix
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
<<<<<<< HEAD
//this function takes an image pair and then using matrixes from rectifyCamera
//along with the camera matrixes takes takes each pixel in the left and right
//image and remaps them based on the matrixes it then returns an image pair that
//has been remaped so that each pixel on the right image is in same height as
//the pixel on the left image that it corresponds to
=======

// Gets parameters from filestorage to call initUndistortRectifyMap
// return a pair of new images
>>>>>>> origin/master
matPair StereoCalibrate::initUndistort(matPair Pair)
{
    img1 = Pair.left;
    img2 = Pair.right;
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
    cout << " R matrix" << endl;
    cout << R << endl;
    //T.at<double>(0) += 50;
    //stereoRectify( CM1, D1, CM2, D2, img1.size(), R, T, R1, R2, P1, P2, Q ,0,0);
    //initUndistortRectifyMap(CM1, D1, Mat(), P1, img1.size(), CV_16SC2, map1x, map1y);

    // returns lookup maps mapx and mapy as output.These maps indicate from where we
    // should interpolate source pixels for each pixel of the destination image
    initUndistortRectifyMap(CM1, D1, R1, P1, img1.size(), CV_16SC2, map1x, map1y);
    //initUndistortRectifyMap(CM2, D2, Mat(), P2, img2.size(), CV_16SC2, map2x, map2y);
    initUndistortRectifyMap(CM2, D2, R2, P2, img2.size(), CV_16SC2, map2x, map2y);
    remap(img1, imgU1, map1x, map1y, INTER_LINEAR);
    remap(img2, imgU2, map2x, map2y, INTER_LINEAR);

    cout << "DONE " << endl;
    matPair returnPair;
    //int shift = 0.2*i1.cols;
    int shift = 0;
    cout << "shift = " << shift << endl;
    returnPair.left = imgU1(Range(0, imgU1.rows),Range(0+shift, imgU1.cols)).clone();
     //Mat img2 = fullImg(Range(0, imSize.height),Range(imSize.width/2, imSize.width)).clone();
    returnPair.right = imgU2(Range(0, imgU2.rows),Range(0, imgU2.cols-shift)).clone();
    cout << "returnPair.left size" << returnPair.left.cols <<" " << returnPair.left.rows << endl;
    cout << "returnPair.right size" << returnPair.right.cols <<" " << returnPair.right.rows << endl;
    return returnPair;
}
