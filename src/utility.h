/* Author: Tucker Haydon */

#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>


inline Eigen::Vector4d Rot2Quat(
    const Eigen::Matrix3d& R
    ) {
        const Eigen::Quaterniond q(R);
        return Eigen::Vector4d(q.w(), q.x(), q.y(), q.z());
}

inline Eigen::Matrix3d Quat2Rot(
    const Eigen::Vector4d& q
    ) {
        const Eigen::Quaterniond q_(q(0), q(1), q(2), q(3));
        return Eigen::Matrix3d(q_);
}

inline Eigen::Vector4d Eul2Quat321(
    const double& roll,
    const double& pitch,
    const double& yaw
    ) {
        Eigen::Quaterniond q(
            Eigen::AngleAxisd(yaw,   Eigen::Vector3d::UnitZ()) *
            Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
            Eigen::AngleAxisd(roll,  Eigen::Vector3d::UnitX()));

        return Eigen::Vector4d(q.w(), q.x(), q.y(), q.z());
}

inline Eigen::Matrix3d Eul2Rot321(
    const double& roll,
    const double& pitch,
    const double& yaw
    ) {
        Eigen::Quaterniond q(
            Eigen::AngleAxisd(yaw,   Eigen::Vector3d::UnitZ()) *
            Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
            Eigen::AngleAxisd(roll,  Eigen::Vector3d::UnitX()));

        return Eigen::Matrix3d(q);
}

inline Eigen::Vector3d QuatRotatePoint(
    const Eigen::Vector4d& q,
    const Eigen::Vector3d& p
    ) {
        return Quat2Rot(q) * p;
}

// Requires q to be unit vector
inline Eigen::Vector4d InvertQuat(
    const Eigen::Vector4d& q
    ) {
        return Eigen::Vector4d(q.w(), -q.x(), -q.y(), -q.z());
}

inline Eigen::Vector3d Quat2Eul321(
    const Eigen::Vector4d q
    ) {
        const double q0 = q(0);
        const double q1 = q(1);
        const double q2 = q(2);
        const double q3 = q(3);

        const double roll  = std::atan2((2*(q0*q1 + q2*q3)) , (1-  2*(q1*q1 + q2*q2)));
        const double pitch = std::asin(2*q0*q2 - q3*q1);
        const double yaw   = std::atan2((2*(q0*q3 + q1*q2)), (1 - 2*(q2*q2 + q3*q3)));

        return Eigen::Vector3d(roll, pitch, yaw);
}

inline Eigen::Vector3d Rot2Eul321(
    const Eigen::Matrix3d R
    ) {
        return Quat2Eul321(Rot2Quat(R));
}

