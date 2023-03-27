#ifndef PROJECT_POINTS_LIDAR_MAPPER
#define PROJECT_POINTS_LIDAR_MAPPER

#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include "opencv2/opencv.hpp"
#include "livox_sdk.h"
#include "../CaptureLidarBuffered/LidarCapture.h"

const double MIN_INTENSITY = 0.0;
const float DIV = 150.0;

class Projector {
public:
    static void project_points_pcl(const pcl::PointCloud<pcl::PointXYZI>::Ptr &lidar_cloud, cv::Mat &tvec, cv::Mat &rvec, cv::Mat &camera_mat, cv::Mat &distCoeffs, cv::Mat &output);

    template<typename T>
    static void
    project_points_livox(const T *points, const size_t size, cv::Mat &tvec, cv::Mat &rvec, cv::Mat &camera_mat, cv::Mat &distCoeffs, lidarPoint *buffer, size_t buffer_size, char &bufferInd,
                         int &position) {
        // Transform the point cloud into a format opencv can handle.
        std::vector<cv::Point3d> pts_3d;
        pts_3d.reserve(size);
        for (size_t i = 0; i < size; i++) {
            auto p = points[i];
            if (p.reflectivity > MIN_INTENSITY) {
                pts_3d.emplace_back(p.x / DIV, p.y / DIV, p.z / DIV);
            }
        }
        if (pts_3d.empty()) {
            return;
        }
        // project 3d-points into image view
        std::vector<cv::Point2d> pts_2d;
        projectPoints(pts_3d, rvec, tvec, camera_mat, distCoeffs, pts_2d);
        // Write _lidarDepths to matrix
        for (int i = 0; i < pts_2d.size(); i++) {
            cv::Point2d p2d = pts_2d[i];
            auto x = round(p2d.x);
            auto y = round(p2d.y);

            if (position + i >= buffer_size) {
                bufferInd += 1;
                bufferInd %= 2;
                position = -i;
            }

            buffer[bufferInd * buffer_size + position + i] = {
                    (float) x, (float) y, (float) pts_3d[i].x, (float) pts_3d[i].y, (float) pts_3d[i].z
            };
        }
        position += (int) pts_2d.size();
    }

    static void mapTurbo(double v, double vmin, double vmax, uint8_t &r, uint8_t &g, uint8_t &b);

    static cv::Mat getProjectionImg(const cv::Mat &image, cv::Mat &tvec, cv::Mat &rvec, cv::Mat &camera_mat, cv::Mat &distCoeffs, const pcl::PointCloud<pcl::PointXYZI>::Ptr &lidar_cloud);

};

#endif