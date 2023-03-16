#include <iostream>
#include <pcl/io/pcd_io.h>
#include "opencv2/opencv.hpp"
#include "captureBasler.cpp"
#include "../Mapper/loadCalib.cpp"
#include "../CaptureLidarBuffered/LidarCapture.h"

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
    LidarCapture::start(&cdata);

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
        auto depths = LidarCapture::get_raw_data();
        for(int i = 0; i < BUFFER_SIZE; i++) {
            // Find first non-zero pos
            auto &pos = depths[i];

            if(pos.px > 0 && pos.px < AOI_WIDTH && pos.py > 0 && pos.py < AOI_HEIGHT) {
                int x = (int)pos.px;
                int y = (int)pos.py;

                mat.at<Vec3b>(x, y) = Vec3b(255, 0, 0);
            }
        }

        imshow(WINNAME, mat);
    }

    return LidarCapture::stop();
}