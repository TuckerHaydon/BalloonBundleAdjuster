/* Author: Tucker Haydon */

#pragma once

#include <Eigen/Core>
#include <memory>
#include "feature.h"

class Observation {
public:
    Observation(const std::shared_ptr<Feature> feature, const Eigen::Vector2d& measurement);

    Eigen::Vector2d Measurement();
    std::shared_ptr<Feature> GetFeature();

private:
    // 2D pixel observation
    Eigen::Vector2d measurement_;

    // Feature point corresponding to the observation
    std::shared_ptr<Feature> feature_;
};

inline Observation::Observation(
    const std::shared_ptr<Feature> feature,
    const Eigen::Vector2d& measurement)
    : feature_(feature),
      measurement_(measurement)
{}

inline Eigen::Vector2d Observation::Measurement() {
    return measurement_;
}

inline std::shared_ptr<Feature> Observation::GetFeature() {
    return feature_;
}
