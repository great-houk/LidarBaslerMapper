#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include "opencv2/calib3d.hpp"
#include "livox_sdk.h"

const double MIN_INTENSITY = 5.0;
const float DIV = 150.0;

void project_points_pcl(const pcl::PointCloud<pcl::PointXYZI>::Ptr &lidar_cloud, cv::Mat &tvec, cv::Mat &rvec, cv::Mat &camera_mat, cv::Mat &distCoeffs, cv::Mat &output) {
    // Transform the point cloud into a format opencv can handle.
    std::vector<cv::Point3d> pts_3d;
    int width = output.cols;
    int height = output.rows;
    for (size_t i = 0; i < lidar_cloud->size(); i++) {
        pcl::PointXYZI p = lidar_cloud->points[i];
        if (p.intensity > MIN_INTENSITY) {
            pts_3d.emplace_back(p.x, p.y, p.z);
        }
    }
    // project 3d-points into image view
    std::vector<cv::Point2d> pts_2d;
    projectPoints(pts_3d, rvec, tvec, camera_mat, distCoeffs, pts_2d);
    // Write depths to matrix
    cv::Mat projected_depths = cv::Mat::zeros(height, width, CV_64FC3);
    for (size_t i = 0; i < pts_2d.size(); i++) {
        cv::Point2d p2d = pts_2d[i];
        cv::Point3d p3d = pts_3d[i];
        int col = (int) round(p2d.x);
        int row = (int) round(p2d.y);

        if (col > 0 && col < width && row > 0 && row < height) {
            auto p = projected_depths.at<cv::Vec3d>(row, col);
            if (norm(p3d) > norm(p)) {
                *projected_depths.ptr<cv::Vec3d>(row, col) = p3d;
            }
        }
    }
    // Copy depth map over to return value
    for (int j = 0; j < height; j++) {
        for (int i = 0; i < width; i++) {
            if (projected_depths.at<cv::Vec3d>(j, i) != cv::Vec3d::zeros()) {
                *output.ptr<cv::Vec3d>(j, i) = projected_depths.at<cv::Vec3d>(j, i);
            }
        }
    }
}

void project_points_livox(const LivoxRawPoint *points, const int size, cv::Mat &tvec, cv::Mat &rvec, cv::Mat &camera_mat, cv::Mat &distCoeffs, cv::Mat &output) {
    // Transform the point cloud into a format opencv can handle.
    std::vector<cv::Point3d> pts_3d;
    int width = output.cols;
    int height = output.rows;
    for (size_t i = 0; i < size; i++) {
        auto p = points[i];
        if (p.reflectivity > MIN_INTENSITY) {
            pts_3d.emplace_back(double(p.x) / DIV, double(p.y) / DIV, double(p.z) / DIV);
        }
    }
    // project 3d-points into image view
    std::vector<cv::Point2d> pts_2d;
    projectPoints(pts_3d, rvec, tvec, camera_mat, distCoeffs, pts_2d);
    // Write depths to matrix
    cv::Mat projected_depths = cv::Mat::zeros(height, width, CV_64FC3);
    for (size_t i = 0; i < pts_2d.size(); i++) {
        cv::Point2d p2d = pts_2d[i];
        cv::Point3d p3d = pts_3d[i];
        int col = (int) round(p2d.x);
        int row = (int) round(p2d.y);

        if (col > 0 && col < width && row > 0 && row < height) {
            auto p = projected_depths.at<cv::Vec3d>(row, col);
            if (norm(p3d) > norm(p)) {
                *projected_depths.ptr<cv::Vec3d>(row, col) = p3d;
            }
        }
    }
    // Copy depth map over to return value
    for (int j = 0; j < height; j++) {
        for (int i = 0; i < width; i++) {
            if (projected_depths.at<cv::Vec3d>(j, i) != cv::Vec3d::zeros()) {
                *output.ptr<cv::Vec3d>(j, i) = projected_depths.at<cv::Vec3d>(j, i);
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

cv::Mat getProjectionImg(const cv::Mat &image, cv::Mat &tvec, cv::Mat &rvec, cv::Mat &camera_mat, cv::Mat &distCoeffs, const pcl::PointCloud<pcl::PointXYZI>::Ptr &lidar_cloud) {
    int width = image.cols, height = image.rows;
    cv::Mat projected_depths = cv::Mat::zeros(height, width, CV_64FC3);
    project_points_pcl(lidar_cloud, tvec, rvec, camera_mat, distCoeffs, projected_depths);

    double avg_depth = 0.0;
    double min_depth = -1.0;
    int count = 0;
    for (int j = 0; j < height; j++) {
        for (int i = 0; i < width; i++) {
            auto depth = norm(projected_depths.at<cv::Vec3d>(j, i));
            if (depth != 0.0) {
                avg_depth += depth;
                count++;
            }
            if(min_depth < 0.0 || depth < min_depth) {
                min_depth = depth;
            }
        }
    }
    avg_depth /= (double) count;
    double max_depth = avg_depth * 2.0;

    cv::Mat map_img = cv::Mat::zeros(height, width, CV_8UC3);
    for (int x = 0; x < width; x++) {
        for (int y = 0; y < height; y++) {
            uint8_t r, g, b;
            double dist = norm(projected_depths.at<cv::Vec3d>(y, x));
            mapTurbo(dist, min_depth, max_depth, r, g, b);
            map_img.at<cv::Vec3b>(y, x)[0] = b;
            map_img.at<cv::Vec3b>(y, x)[1] = g;
            map_img.at<cv::Vec3b>(y, x)[2] = r;
        }
    }
    cv::Mat merge_img;
    merge_img = 0.8 * map_img + 0.5 * image;
    return merge_img;
}
