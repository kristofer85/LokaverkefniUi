#include "mainwindow.h"
#include "ui_mainwindow.h"
using namespace cv;
using namespace std;

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
    {
    dispSet.blockSize = 1;
    dispSet.dif12 = 0;
    dispSet.dispNum = 160;
    dispSet.minDisp = 0;
    dispSet.prefilter = 33;
    dispSet.speckleRange = 2;
    dispSet.speckleSize = 200;
    dispSet.unique = 0;
    string cameraCal = "../lokaverkefniUi/Q.xml";
    FileStorage fs = FileStorage(cameraCal, FileStorage::READ);  
    fs["Q"] >> Q;
    fs.release();
    //pcl::PointCloud<pcl::PointXYZRGB>::Ptr normal (new pcl::PointCloud<pcl::PointXYZRGB>);

        ui->setupUi(this);
    }

MainWindow::~MainWindow()
{
    delete ui;
}
QImage MainWindow::matToQImage(cv::Mat mat)
{
    if(mat.channels() == 1) {                   // if grayscale image
        return QImage((uchar*)mat.data, mat.cols, mat.rows, mat.step, QImage::Format_Indexed8);     // declare and return a QImage
    } else if(mat.channels() == 3) {            // if 3 channel color image
        cv::cvtColor(mat, mat, CV_BGR2RGB);     // invert BGR to RGB
        return QImage((uchar*)mat.data, mat.cols, mat.rows, mat.step, QImage::Format_RGB888);       // declare and return a QImage
    } else {
        qDebug() << "in openCVMatToQImage, image was not 1 channel or 3 channel, should never get here";
    }
    return QImage();        // return a blank QImage if the above did not work
}

void MainWindow::on_btnLeft_clicked()
{

    strFileName = QFileDialog::getOpenFileName();       // bring up open file dialog



    Mat matL = cv::imread(strFileName.toStdString());        // open image
    ui->Impath->setText(strFileName);
    Mat small_matL;
    cv::resize(matL,matL,Size(),0.25,0.25);
     QImage QorginalL = matToQImage(matL);         // convert original and Canny images to QImage
     ui->picL->setPixmap(QPixmap::fromImage(QorginalL));   // show original and Canny images on labels
     ui->statusL->setText("unProcessed image");
     opened = true;
     processed = false;
     matL.release();
     //small_matL.release();
     QorginalL = QImage();
}

void MainWindow::on_btnProcess_clicked()
{
    if(processed == true)
    {
        ui->output->setText("this image has already been proccessed");
    }
    else if(opened == true)
    {
        string impath = strFileName.toStdString();
        lensUndestortedPair = Proccess_image(impath);
        cout << "first stage proccessing complete" << endl;
        imwrite("leftLensCor.png", lensUndestortedPair.left);
        imwrite("rightLensCor.png", lensUndestortedPair.right);
        correctedPair = stereoCalibrate.initUndistort(lensUndestortedPair);
        lensUndestortedPair.left.release();
        lensUndestortedPair.right.release();
        Rectify croped(correctedPair);
        //matPair proccessedPair
        correctedPair= croped.getCroppedPair();
        Mat matL,matR;
        cv::resize(correctedPair.left,matL,Size(),0.25,0.25);
        cv::resize(correctedPair.right,matR,Size(),0.25,0.25);
        QImage QmatL = matToQImage(matL);
        QImage QmatR = matToQImage(matR);
        qDebug()<<"done making Qmats";
        ui->picL->setPixmap(QPixmap::fromImage(QmatL));
        ui->picR->setPixmap(QPixmap::fromImage(QmatR));
        ui->statusL->setText("corrected left");
        ui->statusR->setText("corrected right");
        ui->output->setText("image undestort complete");
        processed = true;
        matL.release();
        matR.release();
        QmatR = QImage();
        QmatL = QImage();

    }
    else
    {
        ui->output->setText("Image has not been selected please select an image to proccess");
    }
}

void MainWindow::on_pushButton_clicked()
{
    stereoCalibrate.findAndDrawChessBoardCorners("../lokaverkefniUi/Y.xml");
    stereoCalibrate.CalibrateStereoCamera();
    stereoCalibrate.rectifyCamera();
    stereoCalibrate.clean();
    ui->output->setText("stereo recalibration complete");
}

void MainWindow::on_btnDepthmap_clicked()
{
    if(img1Chosen == true && img2Chosen == true)
    {
        matPair tempPair;
        tempPair.left = imread(strLeftImgName.toStdString());
        tempPair.right = imread(strRightImgName.toStdString());

        if(tempPair.left.cols != tempPair.right.cols || tempPair.left.rows != tempPair.right.rows)
        {
            ui->output->setText("The two images are of different size please make sure the both images are the same size.");
        }
        else
        {
            if(tempPair.left.cols > 1000 || tempPair.left.rows > 1000)
            {
                tempPair.resize(0.50);
            }

            cout << tempPair.left.cols << " "<< tempPair.left.rows << endl;
            cout << tempPair.right.cols << " " << tempPair.right.rows << endl;
            depthMap.run(tempPair,dispSet);
            tempPair.release();
            Mat disparity = imread("disp8SGBM.png", CV_LOAD_IMAGE_GRAYSCALE);
            QImage QmatR = matToQImage(disparity);
            ui->picR->setPixmap(QPixmap::fromImage(QmatR));
            if(img2Chosen == true)
            {
                img2Chosen = false;
                ui->ImpathR->setText("");
            }
            ui->statusR->setText("depthmap");
            ui->output->setText("done creating depthmap from selected image pair");
        }
    }
    else if(processed == true)
    {
        correctedPair.resize(0.50);
        cout << correctedPair.left.cols << " "<< correctedPair.left.rows << endl;
        cout << correctedPair.right.cols << " " << correctedPair.right.rows << endl;
        depthMap.run(correctedPair,dispSet);
        correctedPair.release();
        Mat disparity = imread("disp8SGBM.png", CV_LOAD_IMAGE_GRAYSCALE);
        QImage QmatR = matToQImage(disparity);
        ui->picR->setPixmap(QPixmap::fromImage(QmatR));
        ui->statusR->setText("depthmap");
        ui->output->setText("done creating depthmap from proccessed picture");


    }
    else
    {
        depthMap.run();
        Mat disparity = imread("disp8SGBM.png", CV_LOAD_IMAGE_GRAYSCALE);
        QImage QmatR = matToQImage(disparity);
        ui->picR->setPixmap(QPixmap::fromImage(QmatR));
        ui->statusR->setText("depthmap");
        ui->output->setText("image not processed so used default value with default settings");
        QmatR = QImage();
    }
    depthMap.clean();
}

void MainWindow::on_btnPCL_clicked()
{

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr mainCloud (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointNormal>::Ptr pointNormal (new pcl::PointCloud<pcl::PointNormal>);
    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZRGBNormal>);
    //pcl::PointCloud<pcl::PointXYZRGB>::Ptr normal (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PolygonMesh triangles;
    Mat disparity;
    if(dispChosen == true)
    {
        disparity = imread(strDispImgName.toStdString(), CV_LOAD_IMAGE_GRAYSCALE);
        Mat tempDisp;
        if(disparity.rows < 300 || disparity.cols < 400)
        {
            cv::resize(disparity,tempDisp,Size(),2.0,2.0);
            disparity = tempDisp;
        }
        else if(disparity.rows > 900 || disparity.cols > 900)
        {
            cv::resize(disparity,tempDisp,Size(),0.5,0.5);
            disparity = tempDisp;
        }
    }
    else
    {
        disparity = imread("disp8SGBM.png", CV_LOAD_IMAGE_GRAYSCALE);
    }

    Mat color,temp;
    if(img1Chosen == true)
    {
        color = imread(strLeftImgName.toStdString());
        if( color.rows < 300 ||  color.cols < 400)
        {
            cv::resize(color,temp,Size(),2.0,2.0);
            color = temp;
        }
        else if( color.rows > 900 ||  color.cols > 900)
        {
            cv::resize(color,temp,Size(),0.5,0.5);
            color = temp;
        }
    }
    else
    {
        color = imread("im0.png");
    }

    if(color.rows == disparity.rows && color.cols == disparity.cols)
    {
        cout << "size is " << color.rows << " X " << color.cols << endl;
        utilities.matToCloud(color,disparity,Q,mainCloud);

        //mainCloud = utilities.SOR_filter(mainCloud);
        utilities.smoothNormals(mainCloud);
        if(visualizer.displayPoly == false)
            visualizer.viewer = visualizer.displayPointCloudColor(mainCloud,color);      // view point cloud
        else
            visualizer.viewer = visualizer.displayPolyMesh(mainCloud,triangles,color); // view polyMesh
        mainCloud->clear();
        QImage QmatR = matToQImage(disparity);
        QImage QmatL = matToQImage(color);
        ui->picL->setPixmap(QPixmap::fromImage(QmatL));
        ui->picR->setPixmap(QPixmap::fromImage(QmatR));
        ui->statusR->setText("depthmap");
        ui->statusL->setText("image");
        ui->output->setText("done creating 3D model");
        disparity.release();
        color.release();
        QmatR = QImage();
        QmatL = QImage();

        //------------Main render loop------------
        //   Loop untils pcl viewer is turned off *
        //----------------------------------------
        viewer = true;

        while ( !visualizer.viewer->wasStopped())
        {
          visualizer.viewer->spinOnce(100);
          //viewer->saveScreenshot("screenshot.png");
          boost::this_thread::sleep (boost::posix_time::microseconds (100000));

        }
    }
    else
    {
        ui->output->setText("the image and the depthmap are not the same size");
    }

}

void MainWindow::on_btnClose_clicked()
{
    if(viewer == true)
    {
        visualizer.viewer->close();
        visualizer.viewer->removeAllPointClouds();
        visualizer.viewer->removePolygonMesh();
        visualizer.viewer->removeAllCoordinateSystems();
    }
    else
    {
        ui->output->setText("viewer is not active");
    }
}

void MainWindow::on_pushButton_2_clicked()
{
    this->close();
}

void MainWindow::on_dispNrText_textChanged(const QString &arg1)
{
    dispSet.dispNum = ui->dispNrText->text().toInt();
}

void MainWindow::on_minDispText_textChanged(const QString &arg1)
{
    dispSet.minDisp = ui->minDispText->text().toInt();
}

void MainWindow::on_speckleSizeText_textChanged(const QString &arg1)
{
    dispSet.speckleSize = ui->speckleSizeText->text().toInt();
}


void MainWindow::on_uniqueText_textChanged(const QString &arg1)
{
    dispSet.unique = ui->uniqueText->text().toInt();
}

void MainWindow::on_P1Text_textChanged(const QString &arg1)
{
    dispSet.P1 = ui->P1Text->text().toInt();
}

void MainWindow::on_P2Text_textChanged(const QString &arg1)
{
    dispSet.P2 = ui->P2Text->text().toInt();
}

void MainWindow::on_speckleRangeText_textChanged(const QString &arg1)
{
    dispSet.speckleRange = ui->speckleRangeText->text().toInt();
}

void MainWindow::on_disp12Text_textChanged(const QString &arg1)
{
    dispSet.dif12 = ui->disp12Text->text().toInt();
}

void MainWindow::on_btnOpenLeft_clicked()
{
    strLeftImgName = QFileDialog::getOpenFileName();       // bring up open file dialog

    Mat matL = cv::imread(strLeftImgName.toStdString());        // open image
    ui->ImpathL->setText(strLeftImgName);
    Mat small_matL;
    cv::resize(matL,matL,Size(),0.25,0.25);
     QImage QorginalL = matToQImage(matL);         // convert original and Canny images to QImage
     ui->picL->setPixmap(QPixmap::fromImage(QorginalL));   // show original and Canny images on labels
     ui->statusL->setText("proccessed left image");
     img1Chosen = true;
     opened = false;
     processed = false;
     //matL.release();
     //small_matL.release();
     QorginalL = QImage();
}

void MainWindow::on_btnOpenRight_clicked()
{
    strRightImgName = QFileDialog::getOpenFileName();       // bring up open file dialog

    Mat matR = cv::imread(strRightImgName.toStdString());        // open image
    ui->ImpathR->setText(strRightImgName);
    Mat small_matR;
    cv::resize(matR,matR,Size(),0.25,0.25);
     QImage QorginalR = matToQImage(matR);         // convert original and Canny images to QImage
     ui->picR->setPixmap(QPixmap::fromImage(QorginalR));   // show original and Canny images on labels
     ui->statusR->setText("proccessed right image");
     opened = false;
     processed = false;
     img2Chosen = true;
     if(dispChosen == true)
     {
         dispChosen = false;
         ui->ImpathDisp->setText("");
     }
     matR.release();
     //small_matL.release();
     QorginalR = QImage();
}

void MainWindow::on_btnOpenDisp_clicked()
{
    strDispImgName = QFileDialog::getOpenFileName();       // bring up open file dialog

    Mat matR = cv::imread(strDispImgName.toStdString());        // open image
    ui->ImpathDisp->setText(strDispImgName);
    Mat small_matR;
    cv::resize(matR,matR,Size(),0.25,0.25);
     QImage QorginalR = matToQImage(matR);         // convert original and Canny images to QImage
     ui->picR->setPixmap(QPixmap::fromImage(QorginalR));   // show original and Canny images on labels
     ui->statusR->setText("pre existing depth map");
     opened = false;
     processed = false;
     if(img2Chosen == true)
     {
         img2Chosen = false;
         ui->ImpathR->setText("");
     }
     dispChosen = true;
     matR.release();
     //small_matL.release();
     QorginalR = QImage();
}
