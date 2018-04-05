/* Author: Tucker Haydon */

#pragma once 

#include <vector>

#include "camera.h"
#include "feature.h"


class Reconstruction {
public:

    // Constructor
    Reconstruction();

    // Add a feature to the reconstruction
    void AddFeature(const std::shared_ptr<Feature> feature);
    void AddFeatures(const std::vector<std::shared_ptr<Feature>> features);

    // Add a camera to the reconstruction
    void AddCamera(const std::shared_ptr<Camera> camera);
    void AddCameras(const std::vector<std::shared_ptr<Camera>> cameras);
    
    // Returns the list of feature points
    std::vector<std::shared_ptr<Feature>> Features();

    // Returns the list of cameras
    std::vector<std::shared_ptr<Camera>> Cameras();


private: 
    std::vector<std::shared_ptr<Feature>> features_;
    std::vector<std::shared_ptr<Camera>> cameras_;

};

inline Reconstruction::Reconstruction() {}


inline void Reconstruction::AddFeature(const std::shared_ptr<Feature> feature) {
    features_.push_back(feature);
}

inline void Reconstruction::AddFeatures(const std::vector<std::shared_ptr<Feature>> features) {
    features_.insert(features_.end(), features.begin(), features.end());
}

inline void Reconstruction::AddCamera(const std::shared_ptr<Camera> camera) {
    cameras_.push_back(camera);
}

inline void Reconstruction::AddCameras(const std::vector<std::shared_ptr<Camera>> cameras) {
    cameras_.insert(cameras_.end(), cameras.begin(), cameras.end());
}

inline std::vector<std::shared_ptr<Feature>> Reconstruction::Features() {
    return features_;
}

inline std::vector<std::shared_ptr<Camera>> Reconstruction::Cameras() {
    return cameras_;
}
