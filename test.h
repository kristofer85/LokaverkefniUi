#ifndef TEST_H
#define TEST_H
#ifndef Q_MOC_RUN
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/features/normal_3d.h>
#include <boost/thread/thread.hpp>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/surface/gp3.h>
#include <pcl/io/vtk_io.h>
#include <boost/thread/thread.hpp>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/point_types.h>
#include <iostream>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/surface/mls.h>
#include <pcl/outofcore/visualization/camera.h>
#endif
#include "opencv2/core.hpp"
#include "opencv2/calib3d.hpp"
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

class Test
{
public:
    Test();
    void runTriangulate();

};

#endif // TEST_H
