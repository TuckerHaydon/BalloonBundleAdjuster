/* Author: Tucker Haydon */

#pragma once

#include <vector>
#include <string>
#include <experimental/filesystem>

#include "io.h"
#include "types.h"
#include "camera.h"

class FeatureExtractor {
public:
    // Constructor
    FeatureExtractor(PoseMap pose_map, FeaturePtr red_feature, FeaturePtr blue_feature);

    // Extract red and blue feature points from a single image and package them
    // as a camera
    Camera ExtractFeaturesFromImage(const std::string& image_path);

    // Extract red and blue feature points from a list of images and package
    // them into cameras
    std::vector<Camera> ExtractFeaturesFromImageDirectory(const std::vector<std::string> image_paths);
private:
    PoseMap pose_map_;
    FeaturePtr red_feature_, blue_feature_;
}

inline FeatureExtractor::FeatureExtractor(
    PoseMap pose_map,
    FeaturePtr red_feature,
    FeaturePtr blue_feature)
    : pose_map_(pose_map),
      red_feature_(red_feature),
      blue_feature_(blue_feature)
    {}

