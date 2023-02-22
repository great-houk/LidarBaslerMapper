#include "../Mapper/loadCalib.cpp"
#include "../Mapper/projectPoints.cpp"
#include "captureBasler.cpp"
#include "captureLidar.cpp"
#include "opencv2/opencv.hpp"
#include <iostream>

using namespace std;
using namespace cv;
using namespace pcl;

int depth = 0;
int range = 10;

const char *WINNAME = "Projected Points";

void onDepthChange(int pos, void*) {
    depth = pos;
}

void onRangeChange(int pos, void*) {
    range = pos;
}

int main() {
    cout << "Loading..." << endl;

    // Load calibration data
    Mat camMat, distCoeffs, trans, rot;
    loadCalib(camMat, distCoeffs, trans, rot);
    CalibData cdata {
        camMat, distCoeffs, trans, rot
    };

    // Start grabbing
    BaslerCapturer grabber;
    grabber.startGrabbing();

    // Start capturing lidar
    start_lidar_capture(&cdata);

    // Project points and get output image
    namedWindow(WINNAME, WINDOW_NORMAL);
    resizeWindow(WINNAME, 2000, 2000);
    createTrackbar("Depth Slider", WINNAME, nullptr, 300, onDepthChange);
    createTrackbar("Range +/-", WINNAME, nullptr, 60, onRangeChange);

    while (true) {
        auto mat = BaslerCapturer::getMat();
        if (mat.size[0] == 0) { continue; }

        if ((char) cv::pollKey() == 'q') {
            break;
        }

        // Draw image
        Mat depths = getDepths().clone();
        Mat clipped = Mat::zeros(AOI_HEIGHT, AOI_WIDTH, CV_8UC3);
        for(int j = 0; j < AOI_HEIGHT; j++) {
            for(int i = 0; i < AOI_WIDTH; i++) {
                // Find first non-zero pos
                auto pos = depths.at<Vec<double, 3>>(j, i);
                if(pos == Vec<double, 3>(0.0, 0.0, 0.0)) {
                    for(int d = 1; d < 5; d++) {
                        for(int x = -d; x <= d; x++) {
                            // Check top
                            int i2 = i + x, j2 = j + d;
                            if(i2 < AOI_WIDTH && i2 > 0 && j2 < AOI_HEIGHT && j2 > 0) {
                                auto p = depths.at<Vec<double, 3>>(j2, i2);
                                if(p != Vec<double, 3>(0.0, 0.0, 0.0)) {
                                    pos = p;
                                    d = 100;
                                    break;
                                }
                            }
                            // Check right
                            i2 = i + d, j2 = j + x;
                            if(i2 < AOI_WIDTH && i2 > 0 && j2 < AOI_HEIGHT && j2 > 0) {
                                auto p = depths.at<Vec<double, 3>>(j2, i2);
                                if(p != Vec<double, 3>(0.0, 0.0, 0.0)) {
                                    pos = p;
                                    d = 100;
                                    break;
                                }
                            }
                            // Check bottom
                            i2 = i + x, j2 = j - d;
                            if(i2 < AOI_WIDTH && i2 > 0 && j2 < AOI_HEIGHT && j2 > 0) {
                                auto p = depths.at<Vec<double, 3>>(j2, i2);
                                if(p != Vec<double, 3>(0.0, 0.0, 0.0)) {
                                    pos = p;
                                    d = 100;
                                    break;
                                }
                            }
                            // Check left
                            i2 = i - d, j2 = j + x;
                            if(i2 < AOI_WIDTH && i2 > 0 && j2 < AOI_HEIGHT && j2 > 0) {
                                auto p = depths.at<Vec<double, 3>>(j2, i2);
                                if(p != Vec<double, 3>(0.0, 0.0, 0.0)) {
                                    pos = p;
                                    d = 100;
                                    break;
                                }
                            }
                        }
                    }
                }

                auto depth2 = pos[0] * pos[0] + pos[1] * pos[1] + pos[2] * pos[2];
                if(depth2 > depth * depth - range * range && depth2 < depth * depth + range * range) {
                    *clipped.ptr<Vec<char, 3>>(j, i) = mat.at<Vec<char, 3>>(j, i);
                }
            }
        }
        imshow(WINNAME, clipped);
    }

    stop_lidar_capture();

    return 0;
}