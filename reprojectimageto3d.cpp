#include "reprojectimageto3d.h"
using namespace cv;
using namespace std;
ReprojectImageTo3d::ReprojectImageTo3d()
{

}


void ReprojectImageTo3d::reproject(const cv::Mat& disparity, const cv::Mat& Q, cv::Mat& out3D)
{


    out3D = cv::Mat::zeros(disparity.size(), CV_32FC3);

    double Q03 = Q.at<double>(0, 3);
    double Q13 = Q.at<double>(1, 3);
    double Q23 = Q.at<double>(2, 3);
    double Q32 = Q.at<double>(3, 2);
    double Q33 = Q.at<double>(3, 3);

    for (int i = 0; i < disparity.rows; i++)
    {
        const double* disp_ptr = disparity.ptr<double>(i);
        cv::Vec3f* out3D_ptr = out3D.ptr<cv::Vec3f>(i);

        for (int j = 0; j < disparity.cols; j++)
        {
            const double pw = 1.0f / (disp_ptr[j] * Q32 + Q33);

            cv::Vec3f& point = out3D_ptr[j];
            point[0] = (static_cast<double>(j)+Q03) * pw;
            point[1] = (static_cast<double>(i)+Q13) * pw;
            point[2] = Q23 * pw;
        }
    }
}

void ReprojectImageTo3d::save(const Mat& image3D, const string& fileName)
{


    ofstream outFile("geggjad.txt");

    if (!outFile.is_open())
    {
        std::cerr << "ERROR: Could not open " << fileName << std::endl;
        return;
    }

    for (int i = 0; i < image3D.rows; i++)
    {
        const cv::Vec3f* image3D_ptr = image3D.ptr<cv::Vec3f>(i);

        for (int j = 0; j < image3D.cols; j++)
        {
            outFile << image3D_ptr[j][0] << " " << image3D_ptr[j][1] << " " << image3D_ptr[j][2] << std::endl;
        }
    }

    outFile.close();
}
