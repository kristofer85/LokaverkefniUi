#ifndef REPROJECTIMAGETO3D_H
#define REPROJECTIMAGETO3D_H
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/core/core.hpp>
#include <iostream>
#include <fstream>
#include <string>
#include "opencv2/highgui/highgui.hpp"
#include "reprojectimageto3d.h"
//using namespace std;
//using namespace cv;
class ReprojectImageTo3d
{
public:
    ReprojectImageTo3d();
    void reproject(const cv::Mat& disparity, const cv::Mat& Q, cv::Mat& out3D);
    void save(const cv::Mat& image3D, const std::string& fileName);
};

#endif // REPROJECTIMAGETO3D_H
