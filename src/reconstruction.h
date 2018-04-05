/* Author: Tucker Haydon */

#pragma once 

#include <vector>

#include "types.h"


class Reconstruction {
public:

    // Constructor
    Reconstruction();

    // Add a feature to the reconstruction
    void AddFeature(const FeaturePtr feature);

    // Add a camera to the reconstruction
    void AddCamera(const Camera camera);
    
    // Returns the list of feature points
    std::vector<FeaturePtr> Features();

    // Returns the list of cameras
    std::vector<Camera> Cameras();


private: 
    std::vector<FeaturePtr> features_;
    std::vector<Camera> cameras_;

}

inline Reconstruction::Reconstruction() {};


inline void Reconstruction::AddFeature(const FeaturePtr feature) {
    features_.push_back(feature);
}

inline void Reconstruction::AddCamera(const Camera camera) {
    cameras_.push_back(camera);
}

inline std::vector<FeaturePtr> Features() {
    return features_;
}

inline std::vector<Camera> Cameras() {
    return cameras_;
}
