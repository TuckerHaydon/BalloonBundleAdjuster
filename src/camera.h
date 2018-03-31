/* Author: Tucker Haydon */

#pragma once

#include <memory>
#include <utility>
#include <Eigen/Core>

class Camera {

public:
    Camera(const std::pair<Eigen::Vector2d, std::shared_ptr<FeaturePoint> > fp,
           const Eigen::Vector3d tVecPrior,
           const Eigen::Vector4d qVecPrior,
           const Eigen::Matrix<double, 6, 6> cov);

    Eigen::Vector3d& tVec();
    Eigen::Vector3d& tVecPrior();

    Eigen::Vector4d& qVec();
    Eigen::Vector4d& qVecPrior();

    Eigen::Matrix<double, 6, 6>& covPrior();

    std::pair<Eigen::Vector2d, std::shared_ptr<FeaturePoint> > fp();


private:
    // Observation and feature point id
    std::pair<Eigen::Vector2d, std::shared_ptr<FeaturePoint> > fp_;

    // Position from camera origin to inertial origin in camera frame
    Eigen::Vector3d tVec_;

    // Rotation of inertial frame away from camera frame
    Eigen::Vector4d qVec_;

    Eigen::Vector3d tVecPrior_;
    Eigen::Vector4d qVecPrior_;
    Eigen::Matrix<double, 6, 6> covPrior_;

};

Camera::Camera(const std::pair<Eigen::Vector2d, std::shared_ptr<FeaturePoint> > fp,
               const Eigen::Vector3d tVecPrior,
               const Eigen::Vector4d qVecPrior,
               const Eigen::Matrix<double, 6, 6> cov)
    : fp_(fp),
      tVecPrior_(tVecPrior),
      qVecPrior_(qVecPrior),
      covPrior_(cov),
      tVec_(tVecPrior),
      qVec_(qVecPrior)
{}

inline Eigen::Vector3d& Camera::tVec() {
    return this->tVec_;
}

inline Eigen::Vector3d& Camera::tVecPrior() {
    return this->tVecPrior_;
}

inline Eigen::Vector4d& Camera::qVec() {
    return this->qVec_;
}

inline Eigen::Vector4d& Camera::qVecPrior() {
    return this->qVecPrior_;
}

inline Eigen::Matrix<double, 6, 6>& Camera::covPrior() {
    return this->covPrior_;
}

inline std::pair<Eigen::Vector2d, std::shared_ptr<FeaturePoint> > Camera::fp() {
    return this->fp_;
}
