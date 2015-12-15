#include "visualizer.h"
#include "convert.h"
using namespace cv;
using namespace std;
using namespace pcl;
using namespace pcl::visualization;
Visualizer::Visualizer()
{
    displayPoly = false;        // Render polygonMesh

    // default PCL window size
    renderFrameWidth = 1024;
    renderFrameHeight = 1024;
}
// This function displays point cloud with color information
// The parameter Mat color is passed with this function only to be able to resize the render window to be
// the same as the image viwed, the reason for this was to save screenshots from the window as quick and durty
// rendering method.
boost::shared_ptr<PCLVisualizer> Visualizer::displayPointCloudColor (PointCloud<PointXYZRGB>::ConstPtr cloud,Mat color)
{
    // makes a instance of PCLVisualizer
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    viewer->setBackgroundColor (0, 0, 0);
    // get access to the render window, necessary for changing it´s size
    viewer->getRenderWindow();
    // set the size of the render window
    viewer->setSize(color.size().width,color.size().height);
    // specifies what kind of points to render
    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
    // adds a point cloud to the viewer
    viewer->addPointCloud<pcl::PointXYZRGB> (cloud, rgb, "3D Viewer");
    // Set to render points and sets the size for the points to 3
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "3D Viewer");
    // sets the camera back 60 units to prevent the camera to be inside the object it is suposed to view
    // also we where getting all object up side down so we changed the camera up axis to be -1.0
    viewer->setCameraPosition(0.0, 0.0, -60.0, 0.0, 0.0, 0.0, 0.0,-1.0, 1,0 );
    return (viewer);
}

// This function displays polygon mesh with color information.
// Set the render window size equal to the size of the color image
boost::shared_ptr<pcl::visualization::PCLVisualizer> Visualizer::displayPolyMesh (pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud,pcl::PolygonMesh triangles,Mat  color)
{
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    viewer->setBackgroundColor (0, 0, 0);
    viewer->getRenderWindow();
    viewer->setSize(color.size().width,color.size().height);
    // adds a point polyMesh to the viewer
    viewer->addPolygonMesh(triangles);
    viewer->setCameraPosition(0.0, 0.0, -60.0, 0.0, 0.0, 0.0, 0.0,-1.0, 1,0 );
    return (viewer);
}
