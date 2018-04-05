/* Author: Tucker Haydon */

#pragma once

#include <memory>

#include "reconstruction.h"

class BundleAdjuster {
public:
    // Constructor
    BundleAdjuster(std::shared_ptr<Reconstruction> reconstruction);

    // Performs BA over the reconstruction and saves the data to the reconstruction.
    void Solve();


private:
    std::shared_ptr<Reconstruction> reconstruction_;
}

inline BundleAdjuster::BundleAdjuster(std::shared_ptr<Reconstruction> reconstruction) {
    reconstruction_ = reconstruction;
}
