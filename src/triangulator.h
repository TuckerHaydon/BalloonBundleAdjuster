/* Author: Tucker Haydon */

#pragma once

#include <vector>
#include <memory>

#include "feature.h"
#include "camera.h"

class Triangulator {
public:
    Triangulator(std::vector< std::shared_ptr<Feature> > features, std::vector< std::shared_ptr<Camera> > cameras);
    void Solve();

private:
    std::vector< std::shared_ptr<Feature> > features_;
    std::vector< std::shared_ptr<Camera> > cameras_;

};

inline Triangulator::Triangulator(
    std::vector< std::shared_ptr<Feature> > features, 
    std::vector< std::shared_ptr<Camera> > cameras)
    : features_(features),
      cameras_(cameras)
{}

