#ifndef STEREOSCOPICIMAGE_H
#define STEREOSCOPICIMAGE_H
#include "opencv2/core/core.hpp"
#include "opencv2/calib3d.hpp"
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <stdio.h>
#include <iostream>
#include <string>

#include <limits.h>
#include "opencv2/hal/intrin.hpp"


class StereoScopicImage
{
public:
    //cv::Ptr<cv::StereoSGBM> sgbm = cv::StereoSGBM::create(0,16,3);
    StereoScopicImage();
    void rectifyCamera();
    void disparityMap();
    void disparityMap(std::string images);
};

#endif // STEREOSCOPICIMAGE_H
