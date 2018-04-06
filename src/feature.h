/* Author: Tucker Haydon */

#pragma once

#include <Eigen/Core>

class Feature {

public:
    // Constructor
    Feature();

    // Constructor with prior
    Feature(const Eigen::Vector3d& prior);

    // Expose a reference to the position
    Eigen::Vector3d& Pos();

    // Set the prior
    void SetPrior(const Eigen::Vector3d& prior);

private:
    // Position of feature point in inertial frame
    Eigen::Vector3d pos_;
};


inline Feature::Feature(const Eigen::Vector3d& prior) 
    : pos_(prior)
{}

inline Feature::Feature() 
    : pos_(Eigen::Vector3d(0, 0, 0))
{}

inline Eigen::Vector3d& Feature::Pos() {
    return pos_;
}

inline void Feature::SetPrior(const Eigen::Vector3d& prior) {
    pos_ = prior;
}
