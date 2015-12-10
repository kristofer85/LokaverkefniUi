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

void Visualizer::setRenderWindowWidth(cv::Mat image)
{
    Size s = image.size();
    renderFrameWidth = s.width;
}

void Visualizer::setRenderWindowHeight(cv::Mat image)
{
    Size s = image.size();
    renderFrameWidth = s.height;
}

int Visualizer::getRenderWindowWidth()
{
    return renderFrameHeight;
}

int Visualizer::getRenderWindowHeight()
{
    return renderFrameWidth;
}

PointCloud<PointNormal> Visualizer::loadToCload(Mat color, Mat depth)
{
    if(color.size().height != depth.size().height && color.size().width != depth.size().width)
    {
        cout << "color image and depth image have to be the same size" << endl;
        exit(0);
    }
    if (color.empty() || depth.empty())
    {
        cout << "You need two pictures to convert images to 3d Point cloud" << endl;
        exit(0);
    }
    setRenderWindowWidth(color);
    setRenderWindowHeight(color);

    Convert convert;
    cameraCal = "stereoCalibration.yml";
    fs = FileStorage(cameraCal, FileStorage::READ);
    fs["Q"] >> Q;
    PointCloud<PointXYZRGB>::Ptr cloud (new PointCloud<PointXYZRGB>);
    PointCloud<PointNormal>::Ptr normals (new PointCloud<PointNormal> ());
    search::KdTree<PointXYZRGB>::Ptr tree (new search::KdTree<PointXYZRGB>);

    convert.matToCloud(color,depth,Q,cloud);
    MovingLeastSquares<PointXYZRGB, PointNormal> mls;
    mls.setComputeNormals (true);

    // Set parameters
    mls.setInputCloud (cloud);
    mls.setPolynomialFit (true);
    mls.setSearchMethod (tree);
    mls.setSearchRadius (0.03);
    // Get the normals caculated nice
    // before converting to polygon mesh.
    mls.process(*normals);
    //displayPointCloudColor(cloud);

    return *normals;
}

//pcl::PCLPointCloud2 cloud_blob;
//  loadPCDFile (argv[1], cloud_blob);
//  fromPCLPointCloud2 (cloud_blob, *cloud);



/***********Display point cloud***********
 *  Loop untils pcl viewer is turned off *
******************************************/
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
    viewer->getRenderWindow();
    //viewer->setSize(getRenderWindowWidth,getRenderWindowHeight);
    //viewer->initCameraParameters ();

    return (viewer);
}

/***********Display polygon mesh**********
 *  Loop untils pcl viewer is turned off *
******************************************/

boost::shared_ptr<pcl::visualization::PCLVisualizer> Visualizer::displayPolyMesh (pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud,pcl::PolygonMesh triangles,Mat  color)
{
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    viewer->setBackgroundColor (0, 0, 0);
    viewer->getRenderWindow();
    viewer->setSize(color.size().width,color.size().height);
    pcl::PointXYZ center (0, 0, 0);
    //viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1, 0,0,"3D Viewer");
    viewer->setCameraPosition(0.0, 0.0, 60.0, 0.0, 0.0, 0.0, 0.0,-1.0, 1,0 );
    viewer->setPointCloudSelected(true,"cloud");


    return (viewer);
}
//viewer->saveCameraParameters("frameCamera.cam");
//viewer->saveScreenshot("beginingPose.png");
//viewer->spinOnce(100);
//viewer->saveScreenshot("switchCamposition.png");
//boost::this_thread::sleep (boost::posix_time::microseconds (100000));
//viewer->setCameraPosition(0.0, -0.0, -50.0, 0.0, 0.0, 0.0, 0.0,-1.0, 1,0 );
//viewer->saveCameraParameters("frameCamera2.cam");
//viewer->saveScreenshot("beginingPose2.png");
//viewer->spinOnce(100);
//viewer->saveScreenshot("switchCamposition2.png");
//boost::this_thread::sleep (boost::posix_time::microseconds (100000));

/******Compare locations of camera*********
 *  Test visualizer: adds small spheres   *
 *  with specific transform and rotate.   *
 *  Compare positions from the camera     *
 *  matrix from stereo calibration with   *
 *  transformed cubes. This will hopfully *
 *  tell something about the quality of   *
 *  the stereo calibration.               *
*******************************************/
