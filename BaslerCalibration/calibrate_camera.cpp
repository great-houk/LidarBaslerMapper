#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>
#include <filesystem>
#include <fstream>

using namespace cv;

int calibrate_camera() {
    Size boardSize(6, 9);
    vector<vector<Point3f>> boardCorners(1);
    for (int i = 0; i < boardSize.height; i++)
        for (int j = 0; j < boardSize.width; j++)
            boardCorners[0].emplace_back(float(j), float(i), 0);
    vector<vector<Point2f>> imgCorners;
    int width, height;

    cout << "Reading images" << endl;

    for (auto &filename: filesystem::directory_iterator(BASLER_DIR"/images/")) {
        Mat img = imread(filename.path(), IMREAD_GRAYSCALE);
        width = img.cols;
        height = img.rows;
        vector<Point2f> corners;
        bool found = findChessboardCorners(img, boardSize, corners,
                                           CALIB_CB_ADAPTIVE_THRESH + CALIB_CB_FAST_CHECK + CALIB_CB_NORMALIZE_IMAGE);
        if (found) {
            cornerSubPix(img, corners, Size(11, 11), Size(-1, -1),
                         TermCriteria(TermCriteria::EPS + TermCriteria::COUNT, 30, 0.0001));
            imgCorners.push_back(corners);
        }
    }

    cout << "Doing Math" << endl;

    Mat cameraMatrix, distCoeffs;
    vector<Mat> rvecs;
    vector<Mat> tvecs;
    boardCorners.resize(imgCorners.size(), boardCorners[0]);
    calibrateCamera(boardCorners, imgCorners, Size(width, height), cameraMatrix, distCoeffs, rvecs, tvecs);

    cout << "Outputting" << endl;

    Mat inputImg = imread(filesystem::directory_iterator(BASLER_DIR"/images/")->path());
    Mat outputImg;
    undistort(inputImg, outputImg, cameraMatrix, distCoeffs);
    imwrite(BASLER_DIR"/undistorted.png", outputImg);

    auto mat = [&](int x, int y) -> double {return cameraMatrix.at<double>(x, y);};
    auto dist = [&](int i) -> double {return distCoeffs.at<double>(i);};
    string yaml = format(
            ""
            "# Data path. adjust them!\n"
            "common:\n"
            "    image_file: \"" MAIN_DIR"/calib/basler_calib.png\"\n"
            "    pcd_file: \"" MAIN_DIR"/calib/lidar_calib.pcd\"\n"
            "    result_file: \"" MAIN_DIR"/calib/extrinsic.txt\"\n"
            ""
            ""
            "# Camera Parameters\n"
            "camera:\n"
            "    camera_matrix: [%f, %f, %f,\n"
            "                    %f, %f, %f,\n"
            "                    %f, %f, %f]\n"
            "    dist_coeffs: [%f, %f, %f, %f, %f]\n"
            ""
            ""
            "# Calibration Parameters.\n"
            "calib:\n"
            "    calib_config_file: \"" MAIN_DIR"/calib/config.yaml\"\n"
            "    use_rough_calib: true # set true if your initial_extrinsic is bad\n",
            mat(0, 0), mat(0, 1), mat(0, 2),
            mat(1, 0), mat(1, 1), mat(1, 2),
            mat(2, 0), mat(2, 1), mat(2, 2),
            dist(0), dist(1), dist(2), dist(3), dist(4)
    );
    ofstream basler_calib;
    basler_calib.open(MAIN_DIR"/calib/basler_calib.yaml");
    basler_calib << yaml;
    basler_calib.close();

    return 0;
}