/* Author: Tucker Haydon */

#pragma once

#include <vector>
#include <string>
#include <map>
#include <memory>

#include "io.h"
#include "camera.h"
#include "feature.h"

class FeatureExtractor {
public:
    // Constructor
    FeatureExtractor(std::map<std::string, PoseInfo> pose_map,
                     std::shared_ptr<Feature> red_feature,
                     std::shared_ptr<Feature> blue_feature);

    // Extract red and blue feature points from a single image and package them
    // as a camera
    std::shared_ptr<Camera> ExtractFeaturesFromImage(const std::string& image_path, const PoseInfo& pose_info);

    // Extract red and blue feature points from a list of images and package
    // them into cameras
    std::vector< std::shared_ptr<Camera> > ExtractFeaturesFromImageDirectory(const std::string& directory_path);

private:
    std::map<std::string, PoseInfo> pose_map_;
    std::shared_ptr<Feature> red_feature_, blue_feature_;
};

inline FeatureExtractor::FeatureExtractor(
    std::map<std::string, PoseInfo> pose_map,
    std::shared_ptr<Feature> red_feature,
    std::shared_ptr<Feature> blue_feature)
    : pose_map_(pose_map),
      red_feature_(red_feature),
      blue_feature_(blue_feature)
{}

