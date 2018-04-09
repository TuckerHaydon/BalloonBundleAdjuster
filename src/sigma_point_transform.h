/* Author: Tucker Haydon */

#pragma once

#include <Eigen/Core>

namespace Eigen {
    typedef Matrix<double, 6, 6> Matrix6d;
    typedef Matrix<double, 6, 1> Vector6d;
}

typedef struct {
    Eigen::Vector3d rciC;       // Vector from camera origin to ENU origin in camera frame 
    Eigen::Matrix3d RIC;        // Rotation of ENU away from camera
    Eigen::Matrix6d P;          // Covaraince. First 3 terms are for rciC, last 3 are for attitude error.
} TransformedMeasurement;

class SigmaPointTransform {
public:
    SigmaPointTransform();
    SigmaPointTransform(const double& alpha, const double& beta, const double& kappa);
    TransformedMeasurement TransformMeasurement(const MeasurementInfo& measurement);

private:
    double alpha_;
    double beta_;
    double kappa_;
    double nx_;
    double ns_;
    double lambda_;
    double c_;
    double Ws0_;
    double Wsi_;
    double Wc0_;
    double Wci_;
};

inline SigmaPointTransform::SigmaPointTransform() : SigmaPointTransform(1e-3, 2, 0) {
}

inline SigmaPointTransform::SigmaPointTransform(
    const double& alpha, 
    const double& beta, 
    const double& kappa)
    : alpha_(alpha),
      beta_(beta),
      kappa_(kappa),
      nx_(6) {
        ns_     = 2*nx_ + 1;
        lambda_ = std::pow(alpha_, 2) * (kappa_ + nx_) - nx_;
        c_      = std::sqrt(nx_ + lambda_);
        Ws0_    = lambda_ / (nx_ + lambda_);
        Wsi_    = 1 / (2 * (nx_ + lambda_));
        Wc0_    = Ws0_ + 1 - std::pow(alpha_, 2) + beta_;
        Wci_    = 1 / (2 * (nx_ + lambda_));
}

