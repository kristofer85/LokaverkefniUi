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

#include "reprojectimageto3d.h"
#include "stereocalibrate.h"
#include "test.h"
#include "depthmap.h"
#include "stereoscopicimage.h"
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
    QString strFileName;
    QString strLeftImgName;
    QString strRightImgName;
    QString strDispImgName;
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
    bool opened = false;
    bool processed = false;
    bool viewer = false;
    bool img1Chosen = false;
    bool img2Chosen = false;
    bool dispChosen = false;
    DepthMapSettings dispSet;

private slots:
    QImage matToQImage(cv::Mat mat);
    void on_btnLeft_clicked();

    void on_btnProcess_clicked();

    void on_pushButton_clicked();

    void on_btnDepthmap_clicked();

    void on_btnPCL_clicked();

    void on_btnClose_clicked();

    void on_pushButton_2_clicked();

    void on_dispNrText_textChanged(const QString &arg1);

    void on_minDispText_textChanged(const QString &arg1);

    void on_speckleSizeText_textEdited(const QString &arg1);

    void on_speckleSizeText_textChanged(const QString &arg1);

    void on_uniqueText_textChanged(const QString &arg1);

    void on_P1Text_textChanged(const QString &arg1);

    void on_P2Text_textChanged(const QString &arg1);

    void on_speckleRangeText_textChanged(const QString &arg1);

    void on_disp12Text_textChanged(const QString &arg1);

    void on_btnOpenLeft_clicked();

    void on_btnOpenRight_clicked();

    void on_btnOpenDisp_clicked();

private:
    Ui::MainWindow *ui;
};

#endif // MAINWINDOW_H
