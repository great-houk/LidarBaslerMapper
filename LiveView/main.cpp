#include "../Mapper/loadCalib.cpp"
#include "../Mapper/projectPoints.cpp"
#include "captureBasler.cpp"
#include "opencv2/opencv.hpp"
#include <iostream>

using namespace std;
using namespace cv;
using namespace pcl;

Mat depths, camMat, distCoeffs, trans, rot;
const char *WINNAME = "Projected Points";

int main() {
    cout << "Loading..." << endl;

    // Load calibration data
    loadCalib(camMat, distCoeffs, trans, rot);

    // Start grabbing
    BaslerCapturer grabber;
    grabber.startGrabbing();

    // Project points and get output image
    namedWindow(WINNAME, WINDOW_NORMAL);
    resizeWindow(WINNAME, 2000, 2000);

    while (true) {
        auto mat = BaslerCapturer::getMat();
        if (mat.size[0] == 0) { continue; }
        imshow(WINNAME, mat);

        if ((char) cv::pollKey() == 'q') {
            break;
        }
    }

    return 0;
}