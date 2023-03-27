#include "loadCalib.h"
#include "projectPoints.h"
#include "opencv2/opencv.hpp"
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>

using namespace std;
using namespace cv;
using namespace pcl;

int main() {
    // Load calibration data
    Mat camMat, distCoeffs, trans, rot;
    loadCalib(camMat, distCoeffs, trans, rot);

    // Load point cloud from PCD file using PCL library
    PointCloud<PointXYZI>::Ptr cloud(new PointCloud<PointXYZI>());
    if (pcl::io::loadPCDFile<pcl::PointXYZI>(MAIN_DIR"/calib/lidar_calib.pcd", *cloud) == -1) //* load the file
    {
        PCL_ERROR ("Couldn't read file test_pcd.pcd \n");
        return -1;
    }
    std::cout << "Loaded " << cloud->width * cloud->height << " data points" << std::endl;

    // Load the calib image using opencv
    auto filename = MAIN_DIR"/calib/basler_calib.png";
    Mat image = imread(filename, cv::IMREAD_COLOR);
    if (image.empty()) {
        std::cout << "Cannot read image: " << filename << std::endl;
        return -1;
    }
    std::cout << "Loaded image" << std::endl;

    // Project points and get output image
    Mat init_img = Projector::getProjectionImg(image, trans, rot, camMat, distCoeffs, cloud);
    auto winname = "Projected Points";
    namedWindow(winname, WINDOW_NORMAL);
    resizeWindow(winname, 2000, 2000);
    imshow(winname, init_img);

    [[maybe_unused]] char c = (char) waitKey(0);
    return 0;
}