#include "convert.h"
using namespace cv;
using namespace std;
using namespace boost;
using namespace pcl;
using namespace pcl::visualization;


// converts q which is a ray that intersects the image plane at (x,y,f) for all x and y
// of the image into (X,Y,Z) corrdinates.
Convert::Convert(){}
PointCloud<PointXYZRGB>::Ptr Convert::matToCloud(Mat rgb,Mat disp,Mat Q,PointCloud<PointXYZRGB>::Ptr Cloud)
{
    double px, py, pz;                                         // x,y,z position for each pixel
    uchar pr, pg, pb;                                           // represents rgb color information at each pixel
    double Q03, Q13, Q23, Q32, Q33;
    Q03 = Q.at<double>(0,3);                     //Fx
    Q13 = Q.at<double>(1,3);                       //Fy
    Q23 = Q.at<double>(2,3);                      //Cx
    Q32 = Q.at<double>(3,2);                      //Cy
    Q33 = Q.at<double>(3,3);                     // (Cx - C´x)/Tx
    for (int i = 0; i < rgb.rows; i++)
    {
        uchar* rgb_ptr = rgb.ptr<uchar>(i);
        uchar* disp_ptr = disp.ptr<uchar>(i);
        for (int j = 0; j < rgb.cols; j++)
        {
            uchar d = disp_ptr[j];
            if ( d == 0 ) continue;                            // If the depth value is 0 dDiscard the pixel
            double pw = 1.0 * static_cast<double>(d) * Q32 + Q33;
            px = static_cast<double>(j) + Q03 ;
            py = static_cast<double>(i) + Q13 ;
            pz = Q23 ;
            px = px/pw;
            py = py/pw;
            pz = pz/pw;
            //Get RGB info
            pb = rgb_ptr[3*j];
            pg = rgb_ptr[3*j+1];
            pr = rgb_ptr[3*j+2];
            //Insert info into point cloud structure
            PointXYZRGB point;
            point.x = px;
            point.y = py;
            point.z = pz;
            uint32_t rgb = (static_cast<uint32_t>(pr) << 16 | static_cast<uint32_t>(pg) << 8 | static_cast<uint32_t>(pb));
            point.rgb = *reinterpret_cast<float*>(&rgb);
            Cloud->points.push_back (point);
        }
    }
    // prints out the values from the Q matrix
    for (int y = 0; y < Q.rows; y++)
    {
      const double* Qy = Q.ptr<double>(y);
      for (int x = 0; x < Q.cols; x++)
      {
        std::cout << "Q(" << x << "," << y << ") = " << Qy[x] << std::endl;
      }
    }

    Cloud->width = (int) Cloud->points.size();          // Sets point cloud width
    Cloud->height = 1;
    return Cloud;
}

// This function tries to recreate missing parts of the surface by higher order
// polynomial interpolations between the surrounding data points. By performing resampling,
// these small errors can be corrected and the “double walls” artifacts resulted from
// registering multiple scans together can be smoothed.
pcl::PointCloud<pcl::PointNormal> Convert::smoothNormals(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud)
{
    // Create a KD-Tree
    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB>);
    // Output has the PointNormal type in order to store the normals calculated by MLS
    pcl::PointCloud<pcl::PointNormal> mls_points;
    // Init object (second point type is for the normals, even if unused)
    pcl::MovingLeastSquares<pcl::PointXYZRGB, pcl::PointNormal> mls;
    mls.setComputeNormals (true);
    mls.setInputCloud(cloud);
    mls.setPolynomialFit (true);
    mls.setSearchMethod (tree);
    mls.setSearchRadius (0.09);
    // Reconstruct
    mls.process (mls_points);
   // pcl::io::savePCDFile ("SmoothNormals.pcd", mls_points);
    return (mls_points);
}

// Our sparse outlier removal is based on the computation of the distribution of point to neighbors
// distances in the input dataset. For each point, we compute the mean distance from it to all its neighbors.
// By assuming that the resulted distribution is Gaussian with a mean and a standard deviation, all points
// whose mean distances are outside an interval defined by the global distances mean and standard
// deviation can be considered as outliers and trimmed from the dataset.
pcl::PointCloud<pcl::PointXYZRGB>::Ptr Convert::SOR_filter(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud)
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr filter (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor;
    sor.setInputCloud (cloud);
    sor.setMeanK (50);
    sor.setStddevMulThresh (1.0);
    sor.filter (*filter);
    cloud->clear();
    return filter;
}

// This function takes a list of points an draws polygons between them In the begining
//  one point is selected and looks for two points witch have the shortest distance from the
// selected point. The algorithm keeps running until there are no more valid  points.
pcl::PolygonMesh Convert::triangulate(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud)
{
    // polygon normal is a vector that is perpendicular to a single polygon.
    pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> n;
    pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
    // KdTree is a search method that searches nearest point for x,y and z dimensions
    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB>);
    tree->setInputCloud (cloud);
    n.setInputCloud (cloud);
    n.setSearchMethod (tree);
    // set maximum distance to next normal
    n.setKSearch (200);
    n.compute (*normals);
    // Concatenate the XYZRGB and normal fields
    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_with_normals (new pcl::PointCloud<pcl::PointXYZRGBNormal>);
    pcl::concatenateFields (*cloud, *normals, *cloud_with_normals);
    pcl::PolygonMesh polygon;
    pcl::search::KdTree<pcl::PointXYZRGBNormal>::Ptr tree2 (new pcl::search::KdTree<pcl::PointXYZRGBNormal>);
    tree2->setInputCloud (cloud_with_normals);
    // Triangulation algorithm
    pcl::GreedyProjectionTriangulation<pcl::PointXYZRGBNormal> triangulation;
     //Set the maximum distance between connected points (maximum edge length)
    triangulation.setSearchRadius (2.0);
    // Set typical values for the parameters
    triangulation.setMu (2.5);
    triangulation.setMaximumNearestNeighbors (200);
    triangulation.setMaximumSurfaceAngle(M_PI/4); // 45 degrees
    triangulation.setMinimumAngle(M_PI/18); // 10 degrees
    triangulation.setMaximumAngle(2*M_PI/6); // 120 degrees
    triangulation.setNormalConsistency(false);
    // Get result
    triangulation.setInputCloud (cloud_with_normals);
    triangulation.setSearchMethod (tree2);
    triangulation.reconstruct (polygon);
   // pcl::io::saveVTKFile("triangulate.vtk", polygon);
    return polygon;
}
pcl::PointCloud<pcl::PointXYZRGB>::Ptr Convert::curveNormals(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud)
{
    pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
    pcl::IntegralImageNormalEstimation<pcl::PointXYZRGB, pcl::Normal> ne;
    ne.setNormalEstimationMethod (ne.AVERAGE_DEPTH_CHANGE);
    ne.setMaxDepthChangeFactor(20.5f);
    ne.setNormalSmoothingSize(10.0f);
    ne.setInputCloud(cloud);
    ne.compute(*normals);
    return cloud;
}
PointCloud<PointNormal> Convert::disparityToPointCloud(Mat color, Mat depth)
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
    mls.setInputCloud (cloud);
    mls.setPolynomialFit (true);
    mls.setSearchMethod (tree);
    mls.setSearchRadius (0.03);
    mls.process(*normals);
    return *normals;
}
