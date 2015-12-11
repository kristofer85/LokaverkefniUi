#include "visualizer.h"
#include "convert.h"
using namespace cv;
using namespace std;
using namespace pcl;
using namespace pcl::visualization;
Visualizer::Visualizer()
{
    displayPoly = false;
    displayPoints = true;
    renderFrameWidth = 1024;
    renderFrameHeight = 1024;
}
boost::shared_ptr<PCLVisualizer> Visualizer::displayPointCloudColor (PointCloud<PointXYZRGB>::ConstPtr cloud,Mat color)
{
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    viewer->setBackgroundColor (0, 0, 0);
    viewer->getRenderWindow();
    viewer->setSize(color.size().width,color.size().height);
    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
    viewer->addPointCloud<pcl::PointXYZRGB> (cloud, rgb, "3D Viewer");
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "3D Viewer");
    viewer->setCameraPosition(0.0, 0.0, -60.0, 0.0, 0.0, 0.0, 0.0,-1.0, 1,0 );
    return (viewer);
}
boost::shared_ptr<pcl::visualization::PCLVisualizer> Visualizer::displayPolyMesh (pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud,pcl::PolygonMesh triangles,Mat  color)
{
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    viewer->setBackgroundColor (0, 0, 0);
    viewer->getRenderWindow();
    viewer->setSize(color.size().width,color.size().height);
    viewer->setCameraPosition(0.0, 0.0, -60.0, 0.0, 0.0, 0.0, 0.0,-1.0, 1,0 );
    return (viewer);
}
