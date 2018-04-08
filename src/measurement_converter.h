/* Author: Tucker Haydon */

#pragma once

#include <Eigen/Core>

#include "sensor_params.h"
#include "transform.h"

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

inline ProcessedMeasurement TransformRawMeasurement(
    const RawMeasurement& raw_measurement
    ) {
        
        // Unpack input
        const double az_          = raw_measurement.az;
        const double el_          = raw_measurement.el;
        const double roll_        = raw_measurement.roll;
        const Eigen::Vector3d rG_ = raw_measurement.rG;

        const Eigen::Vector3d roll_pitch_yaw = AzElRoll2RollPitchYaw(az_, el_, roll_);

        // Transforms between body and ENU
        const Eigen::Matrix3d RBI = Eul2Rot321(
            roll_pitch_yaw(0),
            roll_pitch_yaw(1),
            roll_pitch_yaw(2));
        const Eigen::Matrix3d RIB = RBI.transpose();

        // Transforms between body and camera
        const Eigen::Matrix3d RCB = sensor_params.RCB;
        const Eigen::Matrix3d RBC = RCB.transpose();

        // Transforms between ENU and ECEF
        const Eigen::Matrix3d DG2I = DECEF2ENU(sensor_params.riG);
        const Eigen::Matrix3d RGI = DG2I; 
        const Eigen::Matrix3d RIG = RGI.transpose();

        // Transforms between inertial and camera
        const Eigen::Matrix3d RCI = RBI * RCB;
        const Eigen::Matrix3d RIC = RCI.transpose();
        const Eigen::Matrix3d DI2C = RIC;

        // Determine vector from camera origin to ENU origin in camera frame
        const Eigen::Vector3d rigI = DG2I * -1*sensor_params.riG;
        const Eigen::Vector3d ribI = ECEF2ENU(rG_, RGI, rigI);
        const Eigen::Vector3d ricI = Body2ENU(sensor_params.rcB, RBI, ribI);
        const Eigen::Vector3d rciC = DI2C * -1*ricI;

        // Output
        return ProcessedMeasurement{rciC, RIC}; 
}
