#include "convert.h"
using namespace cv;
using namespace std;
using namespace boost;
using namespace pcl;
using namespace pcl::visualization;

Convert::Convert(){}

PointCloud<PointXYZRGB>::Ptr Convert::matToCloud(Mat rgb,Mat disp,Mat Q,PointCloud<PointXYZRGB>::Ptr Cloud)
{
    double px, py, pz;
    uchar pr, pg, pb;
    double Q03, Q13, Q23, Q32, Q33;
    Q03 = Q.at<double>(0,3);
    Q13 = Q.at<double>(1,3);
    Q23 = Q.at<double>(2,3);
    Q32 = Q.at<double>(3,2);
    Q33 = Q.at<double>(3,3);

    for (int i = 0; i < rgb.rows; i++)
    {
        uchar* rgb_ptr = rgb.ptr<uchar>(i);
        uchar* disp_ptr = disp.ptr<uchar>(i);
        for (int j = 0; j < rgb.cols; j++)
        {
            uchar d = disp_ptr[j];
            if ( d == 0 ) continue; //Discard bad pixels
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
            uint32_t rgb = (static_cast<uint32_t>(pr) << 16 |
                    static_cast<uint32_t>(pg) << 8 | static_cast<uint32_t>(pb));
            point.rgb = *reinterpret_cast<float*>(&rgb);
            Cloud->points.push_back (point);
        }
    }
    for (int y = 0; y < Q.rows; y++)
    {
      const double* Qy = Q.ptr<double>(y);
      for (int x = 0; x < Q.cols; x++)
      {
        std::cout << "Q(" << x << "," << y << ") = " << Qy[x] << std::endl;
      }
    }
    Cloud->width = (int) Cloud->points.size();
    Cloud->height = 1;
    //Cloud->resize(Cloud->width*Cloud->height);
    //Cloud->width = disp.rows;
    //Cloud->height = disp.cols;
    return Cloud;
}

pcl::PointCloud<pcl::PointNormal> Convert::smoothNormals(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud)
{
    // Create a KD-Tree
    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB>);

    // Output has the PointNormal type in order to store the normals calculated by MLS
    pcl::PointCloud<pcl::PointNormal> mls_points;

    // Init object (second point type is for the normals, even if unused)
    pcl::MovingLeastSquares<pcl::PointXYZRGB, pcl::PointNormal> mls;

    mls.setComputeNormals (true);

    // Set parameters
    mls.setInputCloud(cloud);
    mls.setPolynomialFit (true);
    mls.setSearchMethod (tree);
    mls.setSearchRadius (0.09);

    // Reconstruct
    mls.process (mls_points);
    pcl::io::savePCDFile ("SmoothNormals.pcd", mls_points);

    return (mls_points);
}



/*******************************************
 *  Our sparse outlier removal is based on *
 *  the computation of the distribution of *
 *  point to neighbors distances in the    *
 *  input dataset                          *
 ******************************************/
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
/*******************************************
 *  The method works by maintaining a list *
 *  of points from which the mesh can be   *
 *  grown and extending it until all       *
 *  possible points are  connected.        *
 *******************************************/

pcl::PolygonMesh Convert::triangulate(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud)
{

    // Normal estimation

    pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> n;
    pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB>);
    tree->setInputCloud (cloud);
    n.setInputCloud (cloud);
    n.setSearchMethod (tree);
    n.setKSearch (200);
    n.compute (*normals);

    // Concatenate the XYZRGB and normal fields*
    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_with_normals (new pcl::PointCloud<pcl::PointXYZRGBNormal>);
    pcl::concatenateFields (*cloud, *normals, *cloud_with_normals);
    //* cloud_with_normals = cloud + normals
    pcl::PolygonMesh polygon;
    // Create search tree*
    //std::vector<std::string> tex_files;
    //tex_files.push_back("c.jpg");
    //
    // initialize texture mesh
    //TextureMesh tex_mesh;
    //
    //tex_mesh.cloud = polygon.cloud;
    //
    // add the 1st mesh
    //tex_mesh.tex_polygons.push_back(polygon.polygons);
    pcl::search::KdTree<pcl::PointXYZRGBNormal>::Ptr tree2 (new pcl::search::KdTree<pcl::PointXYZRGBNormal>);
    tree2->setInputCloud (cloud_with_normals);
    //pcl::Poisson<PointXYZRGBNormal> possition;
    //possition.setInputCloud(cloud_with_normals);
    //possition.setConfidence(true);
    //possition.setSearchMethod(tree2);
    //possition.setDepth(8);
    //possition.setOutputPolygons(true);
    //std::cout << possition.getDepth();
    //possition.setSolverDivide(7);
    //possition.reconstruct(polygon);
    //
    //tex_mesh.cloud = polygon.cloud;

    // add the 1st mesh
    //tex_mesh.tex_polygons.push_back(polygon.polygons);

    // Initialize objects
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

     //Additional vertex information
    //std::vector<int> parts = triangulation.getPartIDs();
    //std::vector<int> states = triangulation.getPointStates();
    pcl::io::saveVTKFile("triangulate.vtk", polygon);



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

pcl::PointCloud<pcl::PointXYZI>::Ptr Convert::disparityToPointCloud(string disparity)
{
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::RGB>::Ptr left_image (new pcl::PointCloud<pcl::RGB>);
    pcl::DisparityMapConverter<pcl::PointXYZI> disparityMapConverter;
    disparityMapConverter.setBaseline(0.78f);
    disparityMapConverter.setFocalLength(368.5f);
    disparityMapConverter.setImageCenterX (536.0f);
    disparityMapConverter.setImageCenterY (712.0f);
    disparityMapConverter.setDisparityThresholdMin(15.0f);
    disparityMapConverter.setImage (left_image);
    // Disparity map of the scene.
    disparityMapConverter.loadDisparityMap (disparity, 1072, 1424);
    disparityMapConverter.compute(*cloud);
    return cloud;
}

