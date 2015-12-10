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
struct matPair
{
   cv::Mat left;
   cv::Mat right;
   matPair()
   {

   }
   void resize(double scale)
   {
       cv::resize(left,left,cv::Size(),scale,scale);
       cv::resize(right,right,cv::Size(),scale,scale);
       ;
   }
   void release()
   {
       left.release();
       right.release();
   }
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

double limit_precision(double val, int precision);
float limit_precision2(float val, int precision);

cv::Mat limit_precision_mat(cv::Mat M, int precision);
cv::Mat limit_precision_matF(cv::Mat M, int precision);

    //matPair splitImage(cv::Mat fullImage);
    matPair splitImage(cv::Mat fullImage);
    //this only works for nikon D90 to do implement lensfun
    //to do implement lensfun and exiv2
    bool getDistCoeffs(cv::Mat &distCoeffs, float zoom, std::string filename);
    bool getDistortionParameters(std::string lens, double focal, double &k1, double &k2, double &k3);
    double getFocalResolution(std::string imagePath);
    void tangent_distortion_correction(cv::Mat src_mat, cv::Mat * dst_mat, float left, float right);
    int findSideBox(const cv::Mat &image, double maxStdev,int numberOfLines, float maxSizeRatio, int maxColorDiff, double grayScaleSize, char
     selection);
    matPair BorderRemoveal(matPair pair);
    cv::Mat cropImageBorders(cv::Mat image, int top, int right, int bottom, int left);
    cv::Mat getCameraMatrix(float zoom, int width, int height);
    Exiv2::Image::AutoPtr unicodeExiv2Open(QString srcPath, QString *linkPath);
    float getZoomValue(std::string imagePath);
    void keystone(cv::Mat src, cv::Mat dst);
    cv::Ptr<cv::StereoMatcher> createRightMatcher2(cv::Ptr<cv::StereoMatcher> matcher_left);
    void shift_image(cv::Mat src_mat, cv::Mat * dst_mat, float left, float right);
    void proccess(std::string imagepath);
    matPair undestort(matPair mats);
    cv::Mat undestortZoom(std::string file);
    matPair Proccess_image(std::string impath);
    cv::Vec4i findROI(cv::Mat image);
    cv::Mat ScaleImage(cv::Mat image,int scale,bool scaleX);
    cv::Mat cropImage(cv::Mat img,int finalDx,int finalDy);
    std::vector<cv::Vec2i> findLocalMaxima(const cv::Mat &image1, const cv::Mat &image2, int dx1, int dx2, int dy1, int dy2, float threshold,float maximaSize,
                                  int top1, int top2, int right1, int right2, int bottom1, int bottom2, int left1, int left2,int eps);
    bool localMaximaCompare(cv::Vec3i x,cv::Vec3i y);
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
    int binarySearch(int lowerBound, int upperBound, double x , void * params,double(*f)(int, void *));
    double areaByHeightAndWidth(int height,void * params);
    //std::string cameraName;
    //std::string lens;



#endif // UTILS_H
