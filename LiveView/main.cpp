#include <iostream>
#include "opencv2/opencv.hpp"
#include "captureBasler.cpp"
#include "../CaptureLidarBuffered/LidarCapture.h"
#include "../Mapper/loadCalib.h"
#include "../Mapper/projectPoints.h"

using namespace std;
using namespace cv;
using namespace pcl;

int depth = 0;
float opacity = 1.0;

const char *WINNAME = "Projected Points";

void onDepthChange(int pos, void*) {
    depth = pos;
}

void onOpacityChange(int pos, void*) {
    opacity = (float)pos / 100;
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
    createTrackbar("Opacity", WINNAME, nullptr, 100, onOpacityChange);

    while (true) {
        auto mat = BaslerCapturer::getMat();
        Mat gray(mat);
        if (mat.size[0] == 0) { continue; }

        cvtColor(gray, gray, COLOR_BGR2GRAY);
        vector<Vec4f> output;
        HoughCircles(gray, output, HOUGH_GRADIENT, 1, gray.rows / 3.0, 80);
        Vec4f best(0, 0, 0, 0);
        for(auto &v : output) {
            if(v[3] > best[3]) {
                best = v;
            }
        }
        circle(mat, Point(best[0], best[1]), best[2], Scalar(0, 0, 255), 5);

        if ((char) cv::pollKey() == 'q') {
            break;
        }

        // Draw image
        auto depths = LidarCapture::get_raw_data();
        if(best[0] != 0 && best[1] != 0) {
            sphereCenter s = LidarCapture::findSphere(best[0], best[1], best[2]);
            cout << s.x << ", " << s.y << ", " << s.z << ", " << s.r << endl;
        }

        for(int i = 0; i < BUFFER_SIZE; i++) {
            // Find first non-zero pos
            auto &pos = depths[i];

            if(pos.px > 0 && pos.px < AOI_WIDTH && pos.py > 0 && pos.py < AOI_HEIGHT) {
                int x = (int)pos.px;
                int y = (int)pos.py;
                float v = pos.x * pos.x + pos.y * pos.y + pos.z * pos.z;
                uint8_t r, g, b;
                Projector::mapTurbo(v, 0, depth * depth, r, g, b);

                mat.at<Vec3b>(y, x) *= 1.0 - opacity;
                mat.at<Vec3b>(y, x) += Vec3b(b, g, r) * opacity;
            }
        }

        imshow(WINNAME, mat);
    }

    return LidarCapture::stop();
}