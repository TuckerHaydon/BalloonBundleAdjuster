#pragma once

#include <cv.h>
#include <Eigen/Core>
#include <vector>

enum Color {red=0, blue=1};

typedef struct {
    /* X, Y, Z */
    Eigen::Vector3d balloonLocation;

    double balloonRadius;

    /* Red=0, Blue=1 */
    Color color;
} BalloonInfo;

const std::vector<BalloonInfo> processImage(const cv::Mat& img);
