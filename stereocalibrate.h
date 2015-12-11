#ifndef STEREOCALIBRATE_H
#define STEREOCALIBRATE_H
#include "opencv2/core/core.hpp"
#include "opencv2/calib3d.hpp"
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <stdio.h>
#include <iostream>
#include <string>
#include <limits.h>
#include "opencv2/hal/intrin.hpp"
#include <opencv2/stereo.hpp>
#include "utils.h"
//using namespace cv;
//using namespace std;
class StereoCalibrate
{
public:
    int numBoards; // Number of left & right pairs
    int board_w; // Number of corners in chessbord width
    int board_h; // Number of corners in chessbord height
    cv::Size board_sz; //  size of chessbord(board_w,board_h)
    int board_n; // board_w*board_h
    double patternSize; // size of pattern
    size_t fullResImage;
    size_t leftAndRightImages;
    std::vector<std::vector<cv::Point3f> > objectpoints; // 3d points where patternsize is used for x and y position z = 0
    std::vector<std::vector<cv::Point2f> > imagePoints1, imagePoints2;  // vector of corners
    std::vector<cv::Point2f> corners1, corners2;  // position of corners
    std::vector<cv::Point3f> obj; // vector of object points
    cv::Mat ChessHd,img1, img2, gray1, gray2, foundImages; // image from kula depper, left, right, leftgray,rightgray, foundImages
    cv::Mat R1, R2, P1, P2, Q; // output from rectify
    cv::Mat map1x, map1y, map2x, map2y; // parameters for remap
    cv::Mat imgU1, imgU2; // new remaped images
    cv::Mat CM1;// camera matrix1
    cv::Mat CM2;// camera matrix2
    cv::Mat D1, D2; // distortion matrix 1 & 2
    cv::Mat R, T, E, F; // rotation, translation essential and fundamental matrices.
    cv::Size imSize;
    cv::Size cutSize;
    float zoom_valueC,focalResC;
    StereoCalibrate();

    // deallocates the allocated  memory
    void clean();

    // Finds the corners of left & right chessbord images
    void findAndDrawChessBoardCorners(std::string filename);

    // get distortion matrices and position of the cameras from each others
    void CalibrateStereoCamera();

    // outputs matrices used in rectifying the images
    void rectifyCamera();

    // maps the rectification of  images
    // creates new pair of rectified images
    matPair initUndistort(matPair);
};
#endif // STEREOCALIBRATE_H
