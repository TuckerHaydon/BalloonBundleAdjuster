/* Author: Tucker Haydon */

#pragma once

#include <memory>
#include <utility>
#include <Eigen/Core>

#include "observation.h"

class Camera {

public:
    Camera(const Eigen::Vector3d t_vec_prior,
           const Eigen::Vector4d q_vec_prior,
           const Eigen::Matrix<double, 6, 6> cov);

    Eigen::Vector3d& TVec();
    Eigen::Vector3d& TVecPrior();

    Eigen::Vector4d& QVec();
    Eigen::Vector4d& QVecPrior();

    Eigen::Matrix<double, 6, 6>& CovPrior();

    std::vector<Observation> Observations();
    void AddObservation(const Observation observation);

private:
    // Observations
    std::vector<Observation> observations_;

    // Position from camera origin to inertial origin in camera frame
    Eigen::Vector3d t_vec_;

    // Rotation of inertial frame away from camera frame
    Eigen::Vector4d q_vec_;

    Eigen::Vector3d t_vec_prior_;
    Eigen::Vector4d q_vec_prior_;
    Eigen::Matrix<double, 6, 6> cov_prior_;

};

inline Camera::Camera(const Eigen::Vector3d t_vec_prior,
               const Eigen::Vector4d q_vec_prior,
               const Eigen::Matrix<double, 6, 6> cov)
    : t_vec_prior_(t_vec_prior),
      q_vec_prior_(q_vec_prior),
      cov_prior_(cov),
      t_vec_(t_vec_prior),
      q_vec_(q_vec_prior)
{}

inline Eigen::Vector3d& Camera::TVec() {
    return t_vec_;
}

inline Eigen::Vector3d& Camera::TVecPrior() {
    return t_vec_prior_;
}

inline Eigen::Vector4d& Camera::QVec() {
    return q_vec_;
}

inline Eigen::Vector4d& Camera::QVecPrior() {
    return q_vec_prior_;
}

inline Eigen::Matrix<double, 6, 6>& Camera::CovPrior() {
    return cov_prior_;
}

inline std::vector<Observation> Camera::Observations() {
    return observations_;
}

inline void Camera::AddObservation(const Observation observation) {
    observations_.push_back(observation);
}
