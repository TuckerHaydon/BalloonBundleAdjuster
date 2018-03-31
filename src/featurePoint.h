/* Author: Tucker Haydon */

#pragma once

#include <Eigen/Core>

class FeaturePoint {

public:
    FeaturePoint();
    FeaturePoint(const Eigen::Vector3d prior);
    Eigen::Vector3d& pos();

private:
    Eigen::Vector3d pos_;
};


inline FeaturePoint::FeaturePoint(const Eigen::Vector3d prior) 
    : pos_(prior)
{}

inline FeaturePoint::FeaturePoint() 
    : pos_(Eigen::Vector3d(0, 0, 0))
{}

inline Eigen::Vector3d& FeaturePoint::pos() {
    return this->pos_;
}

