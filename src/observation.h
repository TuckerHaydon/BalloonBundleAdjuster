/* Author: Tucker Haydon */

#pragma once

#include <Eigen/Core>
#include <memory>
#include "feature.h"

class Observation {
public:
    Observation(const std::shared_ptr<Feature> feature, const Eigen::Vector2d& feature_pos);

    Eigen::Vector2d Measurement();

    std::shared_ptr<Feature> GetFeature();

private:
    Eigen::Vector2d feature_pos_;
    std::shared_ptr<Feature> feature_;
};

inline Observation::Observation(
    const std::shared_ptr<Feature> feature,
    const Eigen::Vector2d& feature_pos)
    : feature_(feature),
      feature_pos_(feature_pos)
{}

inline Eigen::Vector2d Observation::Measurement() {
    return feature_pos_;
}

inline std::shared_ptr<Feature> Observation::GetFeature() {
    return feature_;
}
