#ifndef LOAD_CALIB_LIDAR_MAPPER
#define LOAD_CALIB_LIDAR_MAPPER

#include "opencv2/imgcodecs.hpp"
#include "opencv2/calib3d.hpp"
#include "yaml-cpp/yaml.h"
#include <iostream>
#include <fstream>

void loadCalib(cv::Mat &camMat, cv::Mat &distCoeffs, cv::Mat &trans, cv::Mat &rot) {
    // Load the camera calib yaml file using yaml-cpp
    YAML::Node node = YAML::LoadFile(MAIN_DIR"/calib/basler_calib.yaml");
    // Grab the dist coeffs
    auto distCoeffsYaml = node["camera"]["dist_coeffs"];
    distCoeffs = cv::Mat_<double>(1, 5);
    for (int i = 0; i < 5; i++) {
        distCoeffs.at<double>(0, i) = distCoeffsYaml[i].as<double>();
    }
    // Grab the camera matrix
    auto camMatYaml = node["camera"]["camera_matrix"];
    camMat = cv::Mat_<double>(3, 3);
    for (int j = 0; j < 3; j++) {
        for (int i = 0; i < 3; i++) {
            camMat.at<double>(j, i) = camMatYaml[j * 3 + i].as<double>();
        }
    }
    std::cout << "Loaded Camera Calib Params" << std::endl;
    // Load the extrinsic matrix from the txt
    std::ifstream extrinsic_file(MAIN_DIR"/calib/extrinsic.txt");
    if (!extrinsic_file.is_open()) {
        std::cout << "Cannot read extrinsic: " << MAIN_DIR"/calib/extrinsic.txt" << std::endl;
        return;
    }
    cv::Mat init_rot = cv::Mat_<double>(3, 3);
    rot = cv::Mat_<double>(3, 1);
    trans = cv::Mat_<double>(3, 1);
    for (int i = 0; i < 3; i++) {
        double r0, r1, r2, t;
        char temp;
        extrinsic_file >> r0 >> temp >> r1 >> temp >> r2 >> temp >> t;

        init_rot.at<double>(i, 0) = r0;
        init_rot.at<double>(i, 1) = r1;
        init_rot.at<double>(i, 2) = r2;
        trans.at<double>(i) = t;
    }
    Rodrigues(init_rot, rot);
    std::cout << "Loaded Extrinsic Calib" << std::endl;
}

#endif