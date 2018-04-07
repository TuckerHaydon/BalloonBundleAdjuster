/* Author: Tucker Haydon */

#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>


inline Eigen::Vector4d Rot2Quat(const Eigen::Matrix3d& R) {
    const Eigen::Quaterniond q(R);
    return Eigen::Vector4d(q.w(), q.x(), q.y(), q.z());
}

inline Eigen::Matrix3d Quat2Rot(const Eigen::Vector4d& q) {
    const Eigen::Quaterniond q_(q(0), q(1), q(2), q(3));
    return Eigen::Matrix3d(q_);
}

inline Eigen::Vector4d Eul2Quat321(const double& roll,
                                   const double& pitch,
                                   const double& yaw) {
    Eigen::Quaterniond q(
        Eigen::AngleAxisd(yaw,   Eigen::Vector3d::UnitZ()) *
        Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
        Eigen::AngleAxisd(roll,  Eigen::Vector3d::UnitX()));

    return Eigen::Vector4d(q.w(), q.x(), q.y(), q.z());
}

inline Eigen::Matrix3d Eul2Rot321(const double& roll,
                                  const double& pitch,
                                  const double& yaw) {
    Eigen::Quaterniond q(
        Eigen::AngleAxisd(yaw,   Eigen::Vector3d::UnitZ()) *
        Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
        Eigen::AngleAxisd(roll,  Eigen::Vector3d::UnitX()));

    return Eigen::Matrix3d(q);
}

inline Eigen::Vector3d QuatRotatePoint(const Eigen::Vector4d& q,
                                       const Eigen::Vector3d& p) {
    return Quat2Rot(q) * p;
}

// Requires q to be unit vector
inline Eigen::Vector4d InvertQuat(const Eigen::Vector4d& q) {
    return Eigen::Vector4d(q.w(), -q.x(), -q.y(), -q.z());
}
