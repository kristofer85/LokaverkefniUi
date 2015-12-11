#ifndef CONVERT_H
#define CONVERT_H
#include <opencv/cv.h>
#include <opencv2/core.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <string>
#include <iostream>
#ifndef Q_MOC_RUN
#include <pcl/surface/poisson.h>
#include <pcl/surface/boost.h>
#include <pcl/surface/texture_mapping.h>
#include <pcl/surface/gp3.h>
#include <pcl/surface/mls.h>
#include <pcl/surface/organized_fast_mesh.h>
#include <pcl/features/integral_image_normal.h>
#include <pcl/features/normal_3d.h>
#include <pcl/TextureMesh.h>
#include <pcl/common/common_headers.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/vtk_io.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <boost/thread/thread.hpp>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/surface/gp3.h>
#include <pcl/point_cloud.h>
#include <pcl/io/vtk_io.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/stereo/disparity_map_converter.h>
#include "visualizer.h"
#endif
#include "stereocalibrate.h"
//#include <QApplication>
#include "convert.h"
class Convert
{
    public:
        Convert();
        //pcl::PolygonMesh possitionMesh(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud);
        pcl::PointCloud<pcl::PointNormal> disparityToPointCloud(cv::Mat color, cv::Mat depth);
        pcl::PointCloud<pcl::PointNormal> smoothNormals(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud);

        // converts depth map and color map for the right image to point cloud
        // creates one point for each pixel with color info and position in 3d space
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr matToCloud(cv::Mat rgb,cv::Mat disp,cv::Mat Q,pcl::PointCloud<pcl::PointXYZRGB>::Ptr Cloud);

         // filter out points that are out of certain range from other points
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr SOR_filter(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud);

        // draws polygons between points in the point cloud
        pcl::PolygonMesh triangulate(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud);

        // smooths out point clouds normal based on average depth change
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr curveNormals(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud);

        cv::Mat Q;
        cv::FileStorage fs;
        std::string cameraCal;
};
#endif // CONVERT_H
