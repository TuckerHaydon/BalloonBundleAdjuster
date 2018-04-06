#pragma once
#include <Eigen/Core>

typedef struct {
    /* Position of the camera with respect to body origin in body frame */
    Eigen::Vector3d rcB;

    /* Position of primary with respect to body origin in body frame */
    Eigen::Vector3d rpB;

    /* Position of reference antenna with respect to ECEF origin in ECEF frame */
    Eigen::Vector3d rrG;

    /* Position of inertial origin with respect to ECEF origin in ECEF frame */
    Eigen::Vector3d riG;

    /* Focal length of camera in pixels */
    double f;

    /* Size of a pixel in meters */
    double pixel_size;

    /* Camera intrinsics */
    double k1;

    /* Image size */
    uint16_t image_width;
    uint16_t image_height;

    /* Feature uncertainty in pixels */
    double stdev_feature;


} SensorParams;

extern SensorParams sensor_params;
