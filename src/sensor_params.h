#pragma once
#include <Eigen/Core>
#include <vector>

typedef struct {
    /* Position of the camera with respect to body origin in body frame */
    Eigen::Vector3d rcB;

    /* Position of inertial origin with respect to ECEF origin in ECEF frame */
    Eigen::Vector3d riG;

    /* Focal length of camera in pixels */
    double fx;
    double fy;

    /* Size of a pixel in meters */
    double pixel_size;

    /* Camera intrinsics */
    // c_x c_y k_1 k_2 p_1 p_2 k_3
    // https://docs.opencv.org/3.4.1/da/d54/group__imgproc__transform.html#ga69f2545a8b62a6b0fc2ee060dc30559d
    std::vector<double> intrinsics;

    /* Image size */
    int image_width;
    int image_height;

    /* Feature uncertainty in pixels */
    double stdev_feature;

    /* Rotation of camera away from body */
    Eigen::Matrix3d RCB;

    /* Uncertainty in roll */
    double sig_roll;

} SensorParams;

extern SensorParams sensor_params;
