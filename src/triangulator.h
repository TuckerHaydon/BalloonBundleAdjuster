/* Author: Tucker Haydon */

#pragma once

#include <vector>
#include <memory>

#include "feature.h"
#include "camera.h"

class Triangulator {
public:
    // Constructor
    Triangulator(std::vector< std::shared_ptr<Feature> > features, std::vector< std::shared_ptr<Camera> > cameras);

    // Traingulates the position of feature points and sets the feature points' priors
    void Solve();

private:
    // List of feature points to triangulate
    std::vector< std::shared_ptr<Feature> > features_;

    // List of cameras
    std::vector< std::shared_ptr<Camera> > cameras_;

};

inline Triangulator::Triangulator(
    std::vector< std::shared_ptr<Feature> > features, 
    std::vector< std::shared_ptr<Camera> > cameras)
    : features_(features),
      cameras_(cameras)
{}

