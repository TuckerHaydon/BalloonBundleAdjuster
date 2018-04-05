
#pragma once

#include <Eigen/Core>
#include <Eigen/SVD>

#include <vector>
#include <utility>


#include "utility.h"

Eigen::Vector3d triangulate(const std::vector< std::tuple<Eigen::Vector4d, Eigen::Vector3d, Eigen::Vector2d> > camInfoVec) {
    const double pixelSize = 1.12e-6; // meters/pixel
    const double f = 1660 * pixelSize; // meters
    Eigen::Matrix<double, 3, 4> k = Eigen::Matrix<double, 3, 4>().setZero(); 
    k.topLeftCorner<3,3>().diagonal() = Eigen::Vector3d(f, f, 1);

    const size_t nz = camInfoVec.size();
    Eigen::MatrixXd H(2*nz, 4);


    for(const auto& [qIC, tciC, feat] : camInfoVec) {
        static int i = 0;
        Eigen::Matrix4d P_ = Eigen::Matrix4d().setZero();
        P_.topLeftCorner<3,3>() = Quat2Rot(qIC);
        P_.topRightCorner<3,1>() = tciC;
        P_(3,3) = 1;

        const Eigen::Matrix<double, 3, 4> P = k * P_;
        const Eigen::Vector4d P1 = P.row(0);
        const Eigen::Vector4d P2 = P.row(1);
        const Eigen::Vector4d P3 = P.row(2);

        const double x_tilde = feat(0) * pixelSize;
        const double y_tilde = feat(1) * pixelSize;

        H.row(2*i)   = x_tilde * P3 - P1;
        H.row(2*i+1) = y_tilde * P3 - P2;

        i++;
    }

    // const Eigen::JacobiSVD<Eigen::MatrixXd> svd(H, Eigen::ComputeThinU | Eigen::ComputeThinV);
    // const Eigen::MatrixXd V = svd.matrixV();
    // std::cout << V.rows() << " :: " << V.cols() << std::endl;
    // const Eigen::Matrix4d X = V.rightCols(1);
    // std::cout << Eigen::Vector3d(X(0)/X(3), X(1)/X(3), X(2)/X(3)) << std::endl;

    const Eigen::MatrixXd H_r = H.leftCols(3);
    const Eigen::MatrixXd z = -1 * H.rightCols(1);
    return (H_r.transpose() * H_r).inverse() * H_r.transpose() * z;
}
