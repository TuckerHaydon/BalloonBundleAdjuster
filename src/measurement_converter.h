/* Author: Tucker Haydon */

#pragma once

#include <Eigen/Core>

#include "sensor_params.h"

typedef struct {
    double az;              // Azimuth
    double el;              // Elevation
    double roll;            // Roll
    Eigen::Vector3d rG;     // Primary antenna in ECEF
} RawMeasurement;

typedef struct {
    Eigen::Vector3d rciC;   // Vector from camera origin to ENU origin in camera frame
    Eigen::Matrix3d RIC;    // Rotation of inertial frame away from camera frame
} ProcessedMeasurement;

ProcessedMeasurement TransformRawMeasurement(
    const RawMeasurement& raw_measurement
    ) {

}
