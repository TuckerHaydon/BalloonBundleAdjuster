/* Author: Tucker Haydon */

#pragma once

#include <Eigen/Core>


Eigen::Vector4d Rot2Quat(const Eigen::Matrix3d& R) {
    const Eigen::Quaterniond q(R);
    return Eigen::Vector4d(q.w(), q.x(), q.y(), q.z());
}

Eigen::Matrix3d Quat2Rot(const Eigen::Vector4d& q) {
    const Eigen::Quaterniond q_(q(0), q(1), q(2), q(3));
    return Eigen::Matrix3d(q_);
}
