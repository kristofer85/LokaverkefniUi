#ifndef MAINWINDOW_H
#define MAINWINDOW_H
#include <QMainWindow>
#include<QFileDialog>
#include<QtCore>

#include <iostream>
#include <string>
#include <iostream>
#include<opencv/cv.h>
#include<opencv2/core/core.hpp>
#include<opencv2/highgui/highgui.hpp>
#include<opencv2/imgproc/imgproc.hpp>
#include <math.h>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "opencv2/imgcodecs.hpp"
#include "opencv2/core/utility.hpp"
#include "opencv2/features2d/features2d.hpp"

#ifndef Q_MOC_RUN
#include <QVTKApplication.h>
#include <pcl/common/common_headers.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/surface/gp3.h>
#include <pcl/io/vtk_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <boost/thread/thread.hpp>
#endif

#include "stereocalibrate.h"
#include "depthmap.h"
#include "convert.h"
#include "visualizer.h"
#include "rectify.h"


//using namespace cv;
namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();

    //QStrings used to contain keep track of the filepaths to the images loaded with the open buttons
    QString strFileName;
    QString strLeftImgName;
    QString strRightImgName;
    QString strDispImgName;

    //a Q matrix for rendering a 3D model
    cv::Mat Q;
    //cv::Mat matL;
    //cv::Mat matR;
    StereoCalibrate stereoCalibrate;
    matPair lensUndestortedPair;
    matPair correctedPair;
    matPair depthPair;
    DepthMap depthMap;
    Convert utilities;
    Visualizer visualizer;
    //pcl::PointCloud<pcl::PointXYZRGB>::Ptr mainCloud;
    //pcl::PointCloud<pcl::PointNormal>::Ptr pointNormal;
    //pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_filtered;
    //pcl::PolygonMesh triangles;

    //bool variables to keep track of what images are chosen
    bool opened = false;
    bool processed = false;
    bool viewer = false;
    bool img1Chosen = false;
    bool img2Chosen = false;
    bool dispChosen = false;
    DepthMapSettings dispSet;

private slots:

    //converts a opencv Mat variable into a Qt QImage variable
    QImage matToQImage(cv::Mat mat);

    //the functions here beneth all implement some kind of input from the ui such as button presses or
    //textbox input

    //if you click the open button
    void on_btnLeft_clicked();

    //after you click the process button it the program first checks if you have opened a unprocessed image
    //if you have not it reminds you to open an unproccessed image
    //if you have it takes that image and runs it through
    //1 Proccess_image
    //2 stereoCalibrate.initUndistort
    //3 Rectify;
    //and returns a fully processed image
    void on_btnProcess_clicked();

    //using the calibration images in the XML file y.xml
    //it resets the calibration information or if this is the first time you run the program it
    //initializes them
    void on_pushButton_clicked();

    //clicking the button depthmap will check if you have left and right images loaded if you have
    //it will then use that image pair to create a depthmap using the disparity settings you have inputed
    //if you do not it will create a depth map using the default images im0.png and im1.png
    void on_btnDepthmap_clicked();

    //if you have loaded a left color image and a depth map it will then use those images to
    //create and show a point cloud based on the images
    void on_btnPCL_clicked();

    //clicking the close pcl button will terminate a pcl viewer window if one is open
    void on_btnClose_clicked();

    //clicking the close button will terminate the program
    void on_pushButton_2_clicked();


    //if you change the text in the dispNr textbox it will set that value as the value for
    //dispSet.dispNum
    void on_dispNrText_textChanged(const QString &arg1);

    //if you change the text in the minDisparity textbox it will set that value as the value for
    //dispSet.minDisp
    void on_minDispText_textChanged(const QString &arg1);

    //if you change the text in the speckleSize textbox it will set that value as the value for
    //dispSet.speckleSize
    void on_speckleSizeText_textChanged(const QString &arg1);

    //if you change the text in the uniqueness textbox it will set that value as the value for
    //dispSet.unique
    void on_uniqueText_textChanged(const QString &arg1);

    //if you change the text in the p1 textbox it will set that value as the value for
    //dispSet.p1
    void on_P1Text_textChanged(const QString &arg1);

    //if you change the text in the p2 textbox it will set that value as the value for
    //dispSet.p2
    void on_P2Text_textChanged(const QString &arg1);

    //if you change the text in the speckleRange textbox it will set that value as the value for
    //dispSet.speckleRange
    void on_speckleRangeText_textChanged(const QString &arg1);

    //if you change the text in the disp12MaxDiff textbox it will set that value as the value for
    //dispSet.dif12
    void on_disp12Text_textChanged(const QString &arg1);

    //if you click open left you can load a proccessed left part of an image into the left image frame
    void on_btnOpenLeft_clicked();

    //if you click open right you can load a proccessed right part of an image into the right image frame
    void on_btnOpenRight_clicked();

    //if you click open disp you can load an existing disparity map into the right image frame
    void on_btnOpenDisp_clicked();

private:
    Ui::MainWindow *ui;
};

#endif // MAINWINDOW_H
