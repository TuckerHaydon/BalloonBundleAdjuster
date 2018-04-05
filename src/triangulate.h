
#pragma once

#include <Eigen/Core>
#include <vector>
#include <utility>

#include "utility.h"

Eigen::Vector3d triangulate(const std::vector< std::tuple<Eigen::Vector4d, Eigen::Vector3d, Eigen::Vector2d> > camInfoVec) {
    const double f = 1660; // pixels
    const double pixelSize = 1.12e-6; // meters/pixel
    Eigen::Matrix<double, 3, 4> k = Eigen::Matrix<double, 3, 4>().setZero(); 
    k.topLeftCorner<3,3>().diagonal() = Eigen::Vector3d(f, f, 1);


    for(const auto& [qIC, tciC, f] : camInfoVec) {
        Eigen::Matrix4d P_ = Eigen::Matrix4d().setZero();
        P_.topLeftCorner<3,3>() = Quat2Rot(qIC);
        P_.topRightCorner<1,3>() = tciC;
        P_(3,3) = 1;

        std::cout << P_ << std::endl;
    }

    return Eigen::Vector3d(0,0,0);
}
