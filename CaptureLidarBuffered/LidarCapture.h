//
// Created by tyler on 3/10/23.
//

#ifndef LIDARBASLERMAPPER_LIDARCAPTURE_H
#define LIDARBASLERMAPPER_LIDARCAPTURE_H

#include "opencv2/opencv.hpp"

const size_t BUFFER_SIZE = 100 * /* Milliseconds -> */ 10;

typedef struct {
    cv::Mat camMat;
    cv::Mat distCoeffs;
    cv::Mat trans;
    cv::Mat rot;
} CalibData;

typedef struct {
    float px;
    float py;
    float x;
    float y;
    float z;
} lidarPoint;

typedef struct {
    float x;
    float y;
    float z;
    float r;
} sphereCenter;

class LidarCapture {
public:
    static int start(CalibData *cdata);
    static int init();
    static int stop();
    static sphereCenter findSphere(float px, float py, float r);
    static lidarPoint *get_raw_data();
};


#endif //LIDARBASLERMAPPER_LIDARCAPTURE_H
