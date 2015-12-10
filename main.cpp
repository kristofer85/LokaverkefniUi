//#include <pcl/pcl_tests.h>
#include "mainwindow.h"
#include "rectify.h"
#include <QApplication>

using namespace cv;
using namespace std;
void process_batch(string filename)
{
    StereoCalibrate SC;
    FileStorage fs;
    //path - .jpg = path = path.substring(0, path.length() - 4);
    fs.open(filename, FileStorage::READ);
    if (!fs.isOpened()){cerr << "Failed to open " << filename << endl;}
    FileNode n = fs["images"];                         // Read string sequence - Get node
    if (n.type() != FileNode::SEQ){cerr << "strings is not a sequence! FAIL" << endl;}
    string ChessboardImageList,left, right;
    FileNodeIterator it = n.begin(), it_end = n.end(); // Go through the node
    for (; it != it_end; ++it)
    {
        ChessboardImageList = RAWIMG;
        ChessboardImageList.append((string)*it);
        String file = (string)*it;
        cout << "file is " << file << endl;
        matPair lensUndestortedPair = Proccess_image(ChessboardImageList);
        matPair UnDestort = SC.initUndistort(lensUndestortedPair);
        Rectify croped(UnDestort);
        UnDestort = croped.getCroppedPair();
        int rows = UnDestort.left.rows;
        int cols = UnDestort.right.cols;
        Mat dst = cv::Mat(rows, 2 * cols, UnDestort.left.type());
        cv::Mat tmp = dst(cv::Rect(0, 0, cols, rows));
        UnDestort.left.copyTo(tmp);
        tmp = dst(cv::Rect(cols, 0, cols, rows));
        UnDestort.right.copyTo(tmp);
        ChessboardImageList = ChessboardImageList.substr(0, ChessboardImageList.length() - 4);
        ChessboardImageList.append("_rect.jpg");
        imwrite(ChessboardImageList,dst);
    }
}

void depthmap_batch(string filename)
{
    FileStorage fs;
    matPair depthPair;
    //path - .jpg = path = path.substring(0, path.length() - 4);
    fs.open(filename, FileStorage::READ);
    if (!fs.isOpened()){cerr << "Failed to open " << filename << endl;}
    FileNode n = fs["images"];                         // Read string sequence - Get node
    if (n.type() != FileNode::SEQ){cerr << "strings is not a sequence! FAIL" << endl;}
    string ChessboardImageList,left, right;
    FileNodeIterator it = n.begin(), it_end = n.end(); // Go through the node
    for (; it != it_end; ++it)
    {
        ChessboardImageList = "../Lokaverkefni2/kulaCropped/";
        string fileD = "../Lokaverkefni2/kulaCropped/";
        fileD.append((string)*it);
        string fileL = "../Lokaverkefni2/kulaCropped/";
        fileL.append((string)*it);
        string fileR = "../Lokaverkefni2/kulaCropped/";
        fileR.append((string)*it);
        fileL = fileL.substr(0, fileL.length() - 4);
        fileL.append("_rect_L.jpg");
        fileR = fileR.substr(0, fileR.length() - 4);
        fileR.append("_rect_R.jpg");
        cout << "paths are L = "<< fileL << " and R = " << fileR << endl;
        depthPair.left = imread(fileL);
        depthPair.right = imread(fileR);
        depthPair.resize(0.50);
        DepthMap depthMap;
        DepthMapSettings dispSet;
        depthMap.run(depthPair,dispSet);
        Mat disp = imread("disp8SGBM.png");
        Mat dispVis = imread("dispVis.png");
        fileD = fileD.substr(0, fileD.length() - 4);
        string fileD2 = fileD;
        fileD2.append("_rect_Disp_vis.jpg");
        fileD.append("_rect_Disp.jpg");
        imwrite(fileD,disp);
        imwrite(fileD2,dispVis);
        cout << "file is " << fileL << endl;
    }
}
void sharpen(const cv::Mat &image, cv::Mat &result)
{
    cv::Mat kernel(3, 3, CV_32F, cv::Scalar(0));

    kernel.at<float>(1, 1) = 5.0;
    kernel.at<float>(0, 1) = -1.0;
    kernel.at<float>(2, 1) = -1.0;
    kernel.at<float>(1, 0) = -1.0;
    kernel.at<float>(1, 2) = -1.0;

    cv::filter2D(image, result, image.depth(), kernel);
}

int main(int argc, char *argv[])
{
    bool debug = true;
    cvUseOptimized(true);
    //cvUseOptimized(false);
    cout << "optimized" << cv::useOptimized() << endl;
/*
    matPair bleh = Proccess_image("../Lokaverkefni2/orginal4.jpg");
    StereoCalibrate stereoCalibrate;
    matPair correctedPair = stereoCalibrate.initUndistort(bleh);
    Rectify croped(correctedPair);
    matPair proccessedPair = croped.getCroppedPair();
    imwrite("../Lokaverkefni2/croped4L.jpg",proccessedPair.left);
    imwrite("../Lokaverkefni2/croped4R.jpg",proccessedPair.right);
    cout << "testing sharpen croped img" << endl;
    Mat sharp,sharpNorm,cropedNorm;
    sharpen(proccessedPair.left,sharp);
    imwrite("../Lokaverkefni2/sharp4L.jpg",sharp);
    cout << "testing normalize croped img" << endl;
    normalize(proccessedPair.left,cropedNorm,0,255, CV_MINMAX, CV_16S);
    normalize(sharp,sharpNorm,0,255, CV_MINMAX, CV_16S);
    imwrite("../Lokaverkefni2/sharp4NormL.jpg",sharpNorm);
    imwrite("../Lokaverkefni2/crop4NormL.jpg",cropedNorm);
 */
    //process_batch("../Lokaverkefni2/Y2.xml");
   //depthmap_batch("../Lokaverkefni2/Y2.xml");
    Mat temp = imread("disp8SGBM.png");
    Mat temp2;
    medianBlur(temp,temp2,3);
    imwrite("disp8SGBM.png",temp2);
    QApplication a(argc, argv);
    MainWindow w;
    w.show();

    return a.exec();
    /*
    //StereoCalibrate stereoCalibrate;

    stereoCalibrate.findAndDrawChessBoardCorners("../lokaverkefni2/Y.xml");
    stereoCalibrate.CalibrateStereoCamera();
    stereoCalibrate.rectifyCamera();



    string impath = "../lokaverkefni2/orginal5.jpg";
    matPair lensUndestortedPair;
    if(debug)
    {
    namedWindow("left",CV_WINDOW_KEEPRATIO);
    namedWindow("right",CV_WINDOW_KEEPRATIO);
    }
    lensUndestortedPair = Proccess_image(impath);
    if(debug)
    {
    imshow("left",lensUndestortedPair.left);
    imshow("right",lensUndestortedPair.right);
    imwrite("leftLensCor.png", lensUndestortedPair.left);
    imwrite("rightLensCor.png", lensUndestortedPair.right);
    cout <<"len corrected left "<< lensUndestortedPair.left.cols << " "<< lensUndestortedPair.left.rows << endl;
    cout <<"len corrected right "<< lensUndestortedPair.right.cols << " " << lensUndestortedPair.right.rows << endl;
    waitKey();
    }
    matPair correctedPair;
    correctedPair = stereoCalibrate.initUndistort(lensUndestortedPair);
    lensUndestortedPair.left.release();
    lensUndestortedPair.right.release();

    if(debug)
    {
    imshow("left",correctedPair.left);
    imshow("right",correctedPair.right);
    waitKey();
    cout << correctedPair.left.cols << " "<< correctedPair.left.rows << endl;
    cout << correctedPair.right.cols << " " << correctedPair.right.rows << endl;
    }
    //a special mat pair where left is the left image and right is the disparity image

    DepthMap depthMap;
    //depthMap.run(correctedPair);

    depthMap.run();
    String imgC = depthMap.GetRGB();
    String imgD = depthMap.GetDisp();

    Convert utilities;
    Visualizer visualizer;

       // PCL variables & other temp location*****
       pcl::PointCloud<pcl::PointXYZRGB>::Ptr mainCloud (new pcl::PointCloud<pcl::PointXYZRGB>);
       pcl::PointCloud<pcl::PointNormal>::Ptr pointNormal (new pcl::PointCloud<pcl::PointNormal>);
       pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZRGBNormal>);
       //pcl::PointCloud<pcl::PointXYZRGB>::Ptr normal (new pcl::PointCloud<pcl::PointXYZRGB>);
       pcl::PolygonMesh triangles;
       Mat Q;                                     //Load Matrix Q
       //string cameraCal = "stereoCalibration.yml";
       string cameraCal = "../lokaverkefni2/Q.xml";
       FileStorage fs = FileStorage(cameraCal, FileStorage::READ);
       fs["Q"] >> Q;
       //Q.at<double>(3,3)=40.0;

       //Mat color = imread(imgC, CV_LOAD_IMAGE_COLOR);
       //Mat disparity = imread(imgD, CV_LOAD_IMAGE_GRAYSCALE);

       //Mat temp = imread("im0.png", CV_LOAD_IMAGE_COLOR);
       //Mat color = temp(Range(0, temp.rows),Range(0, temp.cols-35)).clone();
       //resize(color,color,Size(),0.50 ,0.50);
       //Mat disparity = imread("testdisp8SGBM.png", CV_LOAD_IMAGE_GRAYSCALE);

       Mat color = imread("im0.png", CV_LOAD_IMAGE_COLOR);
       //resize(color,color,Size(),0.50 ,0.50);

       Mat disparity = imread("disp8fSGBMMedian.png", CV_LOAD_IMAGE_GRAYSCALE);
       //Mat disparity = imread("disp8SGBM.png", CV_LOAD_IMAGE_GRAYSCALE);
       Mat temp = imread("test1.png",CV_LOAD_IMAGE_COLOR);
       Mat temp2;
       resize(temp,temp,Size(),0.5 ,0.5);
       imwrite("test1.png",temp);

       //resize(color,color,Size(),2.00 ,2.00);
       //resize(disparity,disparity,Size(),2.00 ,2.00);
       //imshow("col",color);
       //imshow("disp",disparity);
       //waitKey();
       //destroyAllWindows();
       //resize(color,color,Size(),0.50 ,0.50);
       //point cloud 2 xyzrgb, filers,triangulate*
       // *  point cloud from rgb image, depth     *
       // *  image & an empty point cloud of       *
       // *  pointXYZRGB.                          *
       // *  Point cloud filters applied           *
       // *****************************************
       utilities.matToCloud(color,disparity,Q,mainCloud);

       mainCloud = utilities.SOR_filter(mainCloud);

       utilities.smoothNormals(mainCloud);
       //mainCloud.reset();
       //pcl::PCLPointCloud2 cloud_blob;
       //pcl::io::loadPCDFile ("SmoothNormals.pcd", cloud_blob);
       //fromPCLPointCloud2 (cloud_blob, *mainCloud);
       //triangles = utilities.triangulate(mainCloud);
       //normal = utilities.curveNormals(mainCloud);


       if(visualizer.displayPoly == false && visualizer.displayPoints == true)
           visualizer.viewer = visualizer.displayPointCloudColor(mainCloud,color);      // view point cloud
       else if(visualizer.displayPoly == true && visualizer.displayPoints == false)
           visualizer.viewer = visualizer.displayPolyMesh(mainCloud,triangles,color); // view polyMesh

       //------------Main render loop------------
       //   Loop untils pcl viewer is turned off *
       //----------------------------------------
       while ( !visualizer.viewer->wasStopped())
       {
         visualizer.viewer->spinOnce(100);
         //viewer->saveScreenshot("screenshot.png");
         boost::this_thread::sleep (boost::posix_time::microseconds (100000));
       }


       return 0;
       */
}
