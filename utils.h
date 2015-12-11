#ifndef UTILS_H
#define UTILS_H
#include "defines.h"
#include <string>
#include <iostream>
#include <opencv/cv.hpp>
#include "opencv2/core/core.hpp"
#include "opencv2/calib3d.hpp"
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include "opencv2/objdetect.hpp"
#include "json/json.h"
#include "json/value.h"
#include <QDebug>
#include <vector>
#include <cmath>

#include "exiv2/exiv2.hpp"
#include <vector>
//#include "lensfun.h"
#include "lensfun/lensfun.h"
#include <locale.h>
//#include <glib.h>
#include <getopt.h>
#define DISTORTION 0.0694

//struct containing left and right part of an image
struct matPair
{
   cv::Mat left;
   cv::Mat right;
   matPair()
   {

   }
   //resize both images
   void resize(double scale)
   {
       cv::resize(left,left,cv::Size(),scale,scale);
       cv::resize(right,right,cv::Size(),scale,scale);
       ;
   }
   //set both left and right mats to NULL and relese the memory used by them
   void release()
   {
       left.release();
       right.release();
   }
   //check if both left and right image are the same size
   bool SameSize()
   {
       if(left.cols == right.cols && left.rows == right.rows)
       {
           return true;
       }
       else
       {
           return false;
       }
   }
};

//struct containing information of how much pixels are shifted between left and right image
struct shiftInfo {
        int dx;
        int dy;
        double distortion;
};

struct areaByHeightAndWidthParams{
        std::vector<cv::Point> contour;
        cv::Mat innerPoints;
        int width;
        int height;
};
    //takes one stereo scopic image and splits it into left and right part
    matPair splitImage(cv::Mat fullImage);

    //this only works for nikon D90 to do implement lensfun
    //to do implement lensfun and exiv2

    //functions to get distortion coefficients from
    bool getDistCoeffs(cv::Mat &distCoeffs, float zoom, std::string filename);

    //function to get
    bool getDistortionParameters(std::string lens, double focal, double &k1, double &k2, double &k3);
    //function that reads the meta information of the image to get the FocalPlaneXResolution value
    //or the size of the pixel on the camera sensor
    double getFocalResolution(std::string imagePath);

    //function that fixes tangens distortion in images
    void tangent_distortion_correction(cv::Mat src_mat, cv::Mat * dst_mat, float left, float right);


    //This function tries to find a gray (everything in grayscale depending on
    //grayScaleSize, white and black count as gray here if 1 <= grayScaleSize,
    //the scale is from black to (length between white and black)*grayScaleSize) box in one
    //of the sides of the image image specified by the selection parameter.
    //The values for selection are:
    //  0   |  top
    //  1   |  right
    //  2   |  bottom
    //  3   |  right
    //Anything else will most likely resault in an error from opencv but it might also just write some random stuff
    //at random places in the memory so just don't.
    //
    //This function defines a gray box as a box where all the color values are very simular. So a low stDev and
    //the mean values of the color intension are simular. This is controled with the maxStdev and maxColorDiff
    //parameters.
    //The way it then works is that numberOfLines+1 lines are selected at the edge to use as a base mark and
    //then we take the mean and stDev of those lines and one more line. Thats the line we move further and further
    //away from the edge and if those lines combined are not gray we stop and the last line was the last line
    //of the graybox.
    int findSideBox(const cv::Mat &image, double maxStdev,int numberOfLines, float maxSizeRatio, int maxColorDiff, double grayScaleSize, char
     selection);

    //This process takes in 2 images and tries to find the gray borders and remove them.
    //Note both images will be croped in the same way so that their size is still the same.
    //All the work (currently but ofc a new function might be added later) is done in the
    // findSideBox function. Further comments on functuality are in the findSideBox function.
    matPair BorderRemoveal(matPair pair);

    //returns a image where top pixels have been cut from the top etc....
    cv::Mat cropImageBorders(cv::Mat image, int top, int right, int bottom, int left);

    //initializes a camera matrix based on size of image and zoom value
    cv::Mat getCameraMatrix(float zoom, int width, int height);

    //opens up a reader to read meta information from an image
    Exiv2::Image::AutoPtr unicodeExiv2Open(QString srcPath, QString *linkPath);

    //reads meta information of an image to find its zoom value
    float getZoomValue(std::string imagePath);

    //fixes tangens destortion using a value previusly discovered by the kula team
    matPair undestort(matPair mats);

    //fixes distortion in an image caused by zooming based on meta information in the image
    cv::Mat undestortZoom(std::string file);

    //a function that takes an unprocessed image and proccesses it using the following steps
    //1 undestortZoom
    //2 splitImage
    //3 undestort
    //and then returns an image pair where lens distortion has been fixed
    matPair Proccess_image(std::string impath);

    //This function attempts to find a suitable search region of an image
    //This is done via facedetection and later via other methods aswell.
    //The function returns a vector a where
    //a[0] = x0
    //a[1] = y0
    //a[2] = width
    //a[3] = height
    //where (x0,y0) is the upper left corner with the width and height
    cv::Vec4i findROI(cv::Mat image);

    //scales the image based on the value of scale
    cv::Mat ScaleImage(cv::Mat image,int scale,bool scaleX);

    //this function crops from the top, bottom,left or right sides of the image based on the values of
    //finalDx and finalDy
    cv::Mat cropImage(cv::Mat img,int finalDx,int finalDy);

    //a function to help find localMaxima of two images
    std::vector<cv::Vec2i> findLocalMaxima(const cv::Mat &image1, const cv::Mat &image2, int dx1, int dx2, int dy1, int dy2, float threshold,float maximaSize,
                                  int top1, int top2, int right1, int right2, int bottom1, int bottom2, int left1, int left2,int eps);

    //compares the localMaxima of x and y axis
    bool localMaximaCompare(cv::Vec3i x,cv::Vec3i y);

    //finds the ratio of pixels that match between two images
    float getCorrectPixelsRatioROI(const cv::Mat &image1, const cv::Mat &image2, int dx, int dy, int top1, int top2, int right1, int right2,
                      int bottom1, int bottom2, int left1, int left2, int eps = 5, float maxDistRatio = 0.7);

    int deltaCostROI(const cv::Mat &image1, const cv::Mat &image2, int dx, int dy, int top1, int top2, int right1, int right2,
                          int bottom1, int bottom2, int left1, int left2,int eps = 5);

    int deltaCostColorROI(const cv::Mat &image1, const cv::Mat &image2, int dx, int dy, int top1, int top2, int right1, int right2,
                          int bottom1, int bottom2, int left1, int left2,int eps = 5);

    shiftInfo gradiantShiftAndDistortion(matPair undistorted_images);

    cv::Mat getThresholdedDiff(matPair images, int threshold = 35);

    cv::Mat thresholdGrayToWhite(cv::Mat image, int grayThreshold = 12, int darkness = 150);

    bool isVerticalRectangle(std::vector<cv::Point> contour, cv::Mat innerPoints, int * left, int * right);

    //function that implements a binary search
    int binarySearch(int lowerBound, int upperBound, double x , void * params,double(*f)(int, void *));


    double areaByHeightAndWidth(int height,void * params);



#endif // UTILS_H
