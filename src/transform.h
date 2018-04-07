/* Author: Tucker Haydon */

#pragma once

#include <Eigen/Core>

// Converts azimuth, elevation, and roll to 321 roll, pitch, yaw euler angles
// Azimuth - Rotation away from true north with positive towards east
// Elevation - Rotation away from ground with positive towards zenith
inline Eigen::Vector3d AzElRoll2RollPitchYaw(
    const double& az,
    const double& el, 
    const double& roll) {
        return Eigen::Vector3d(roll, -1*el, M_PI/2 - az);
}

inline Eigen::Vector3d Body2ENU(
    const Eigen::Vector3d& vB,      // Vector in body frame
    const Eigen::Matrix3d& RBI,     // Rotation of body away from ENU
    const Eigen::Vector3d& vbI      // Vector from ENU origin to body origin in ENU frame
    ) {
        return vbI + RBI * vB;
}

inline Eigen::Vector3d ENU2Body(
    const Eigen::Vector3d& vI,      // Vector in ENU frame
    const Eigen::Matrix3d& RIB,     // Rotation of ENU away from body
    const Eigen::Vector3d& riB      // Vector from body origin to ENU origin in body frame
    ) {
        retunr riB + RIB * vI;
}

inline Eigen::Vector3d ENU2ECEF(
    const Eigen::Vector3d& vI,      // Vector in ENU frame
    const Eigen::Matrix3d& RIG,     // Rotation of ENU away from ECEF
    const Eigen::Vector3d& viG      // Vector from ECEF origin to ENU origin in ECEF frame
    ) {
        return viG + RIG * vI;
}


