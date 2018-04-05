/* Author: Tucker Haydon */

#pragma once

#include <Eigen/Core>

class Feature {

public:
    Feature();
    Feature(const Eigen::Vector3d prior);
    Eigen::Vector3d& pos();

private:
    Eigen::Vector3d pos_;
};


inline Feature::Feature(const Eigen::Vector3d prior) 
    : pos_(prior)
{}

inline Feature::Feature() 
    : pos_(Eigen::Vector3d(0, 0, 0))
{}

inline Eigen::Vector3d& Feature::pos() {
    return this->pos_;
}

