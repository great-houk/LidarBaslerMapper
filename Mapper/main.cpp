#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/calib3d.hpp"
#include "yaml-cpp/yaml.h"
#include <iostream>

using namespace std;
using namespace cv;

const float MIN_INTENSITY = 10.0;

void project_points(const pcl::PointCloud<pcl::PointXYZI>::Ptr &lidar_cloud, Mat &tvec, Mat &rvec, Mat &camera_mat, Mat &distCoeffs, Mat &output) {
    // Get max depth to render, make intensity and depth lists,
    // and transform the point cloud into a format opencv can handle.
    vector<Point3f> pts_3d;
    vector<float> depths;
    int width = output.cols;
    int height = output.rows;
    for (size_t i = 0; i < lidar_cloud->size(); i++) {
        pcl::PointXYZI p = lidar_cloud->points[i];
        if (p.intensity > MIN_INTENSITY) {
            pts_3d.emplace_back(p.x, p.y, p.z);
            float depth = sqrtf(powf(p.x, 2) + powf(p.y, 2) + powf(p.z, 2));
            depths.emplace_back(depth);
        }
    }
    // project 3d-points into image view
    vector<Point2f> pts_2d;
    projectPoints(pts_3d, rvec, tvec, camera_mat, distCoeffs, pts_2d);
    // Write depths to matrix
    Mat projected_depths = Mat::zeros(height, width, CV_32FC1);
    for (size_t i = 0; i < pts_2d.size(); i++) {
        Point2f point_2d = pts_2d[i];
        float depth = depths[i];
        int col = (int) round(point_2d.x);
        int row = (int) round(point_2d.y);

        if (col > 0 && col < width && row > 0 && row < height) {
            if (projected_depths.at<float>(row, col) == 0) {
                *projected_depths.ptr<float>(row, col) = depth;
            } else {
                *projected_depths.ptr<float>(row, col) += depth;
                *projected_depths.ptr<float>(row, col) /= 2.0;
            }
        }
    }
    // Copy depth map over to return value
    for (int j = 0; j < height; j++) {
        for (int i = 0; i < width; i++) {
            if (projected_depths.at<float>(j, i) != 0) {
                *output.ptr<float>(j, i) = projected_depths.at<float>(j, i);
            }
        }
    }
}

void mapTurbo(double v, double vmin, double vmax, uint8_t &r, uint8_t &g, uint8_t &b) {
#include "turbo_colormap.c"

    int intensity = (int) round((v - vmin) / (vmax - vmin) * 255.0);
    if (intensity < 0) {
        intensity = 0;
    } else if (intensity > 255) {
        intensity = 255;
    }
    auto color = turbo_srgb_bytes[intensity];
    r = color[0];
    g = color[1];
    b = color[2];
}

Mat getProjectionImg(
        const Mat &image,
        Mat &tvec,
        Mat &rvec,
        Mat &camera_mat,
        Mat &distCoeffs,
        const pcl::PointCloud<pcl::PointXYZI>::Ptr &lidar_cloud) {

    int width = image.cols, height = image.rows;
    Mat projected_depths = Mat::zeros(height, width, CV_32FC1);
    project_points(lidar_cloud, tvec, rvec, camera_mat, distCoeffs, projected_depths);

    float avg_depth = 0.0;
    int count = 0;
    for (int j = 0; j < height; j++) {
        for (int i = 0; i < width; i++) {
            auto depth = projected_depths.at<float>(j, i);
            if (depth != 0.0) {
                avg_depth += depth;
                count++;
            }
        }
    }
    avg_depth /= (float) count;
    float max_depth = avg_depth * 2.0f;

    Mat map_img = Mat::zeros(height, width, CV_8UC3);
    for (int x = 0; x < width; x++) {
        for (int y = 0; y < height; y++) {
            uint8_t r, g, b;
            float norm = (float) projected_depths.at<float>(y, x) / max_depth;
            mapTurbo(norm, 0, 1, r, g, b);
            map_img.at<Vec3b>(y, x)[0] = b;
            map_img.at<Vec3b>(y, x)[1] = g;
            map_img.at<Vec3b>(y, x)[2] = r;
        }
    }
    Mat merge_img;
    merge_img = 0.5 * map_img + 0.5 * image;
    return merge_img;
}

int main() {
    // Load point cloud from PCD file using PCL library
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
    if (pcl::io::loadPCDFile<pcl::PointXYZI>(MAIN_DIR"/calib/lidar_calib.pcd", *cloud) == -1) //* load the file
    {
        PCL_ERROR ("Couldn't read file test_pcd.pcd \n");
        return (-1);
    }
    cout << "Loaded " << cloud->width * cloud->height << " data points" << endl;
    // Load the calib image using opencv
    auto filename = MAIN_DIR"/calib/basler_calib.png";
    Mat image = imread(filename, IMREAD_COLOR);
    if (image.empty()) {
        cout << "Cannot read image: " << filename << endl;
        return -1;
    }
    cout << "Loaded image" << endl;
    // Load the camera calib yaml file using yaml-cpp
    YAML::Node node = YAML::LoadFile(MAIN_DIR"/calib/basler_calib.yaml");
    // Grab the dist coeffs
    auto distCoeffsYaml = node["camera"]["dist_coeffs"];
    Mat distCoeffs = Mat_<float>(1, 5);
    for (int i = 0; i < 5; i++) {
        distCoeffs.at<float>(0, i) = distCoeffsYaml[i].as<float>();
    }
    // Grab the camera matrix
    auto camMatYaml = node["camera"]["camera_matrix"];
    Mat camMat = Mat_<float>(3, 3);
    for (int j = 0; j < 3; j++) {
        for (int i = 0; i < 3; i++) {
            camMat.at<float>(j, i) = camMatYaml[j * 3 + i].as<float>();
        }
    }
    cout << "Loaded Camera Calib Params" << endl;
    // Load the extrinsic matrix from the txt
    ifstream extrinsic_file(MAIN_DIR"/calib/extrinsic.txt");
    if (!extrinsic_file.is_open()) {
        return -1;
    }
    Mat init_rot = Mat_<float>(3, 3), rot = Mat_<float>(3, 1), trans = Mat_<float>(3, 1);
    for (int i = 0; i < 3; i++) {
        float r0, r1, r2, t;
        char temp;
        extrinsic_file >> r0 >> temp >> r1 >> temp >> r2 >> temp >> t;

        init_rot.at<float>(i, 0) = r0;
        init_rot.at<float>(i, 1) = r1;
        init_rot.at<float>(i, 2) = r2;
        trans.at<float>(i) = t;
    }
    Rodrigues(init_rot, rot);
    cout << "Loaded Extrinsic Calib" << endl;
    // Project points and get output image
    Mat init_img = getProjectionImg(image, trans, rot, camMat, distCoeffs, cloud);
    auto winname = "Projected Points";
    namedWindow(winname, WINDOW_NORMAL);
    resizeWindow(winname, 2000, 2000);
    imshow(winname, init_img);

    char c = (char) waitKey(0);
    return 0;
}