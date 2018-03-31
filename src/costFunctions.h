/* Author: Tucker Haydon */

#pragma once

#include <Eigen/Core>
#include <ceres/ceres.h>
#include <ceres/rotation.h>

class CameraPoseCostFunction {

public:
   CameraPoseCostFunction(const Eigen::Vector4d& qVec,
                          const Eigen::Vector3d& tVec,
                          const Eigen::Matrix<double, 6, 6>& cov)
       : qVec_(qVec),
         tVec_(tVec),
         cov_(cov) 
     {}
 
    static ceres::CostFunction* Create(const Eigen::Vector4d& qVec,
                                       const Eigen::Vector3d& tVec,
                                       const Eigen::Matrix<double, 6, 6>& cov) {
        return (new ceres::AutoDiffCostFunction<CameraPoseCostFunction, 6, 4, 3>(
            new CameraPoseCostFunction(qVec, tVec, cov)));
    }
 
    template <typename T>
    bool operator()(const T* const qVec, const T* const tVec, 
                   T* residuals) const {

        // Typedefs
        typedef Eigen::Matrix<T, 4, 1> QVEC;
        typedef Eigen::Matrix<T, 3, 1> TVEC;
        typedef Eigen::Matrix<T, 6, 6> COV;
    
        // Measurements
        const QVEC qVec_meas = qVec_.cast<T>();
        const TVEC tVec_meas = tVec_.cast<T>();
    
        // Square root of information matrix
        const Eigen::LLT<Eigen::Matrix<double, 6, 6> > chol(cov_);
        const Eigen::Matrix<double, 6, 6> lower = chol.matrixL();
        const COV sqrtInfo = lower.inverse().cast<T>();
    
        // Estimates
        const QVEC qVec_est(qVec[0], qVec[1], qVec[2], qVec[3]);
        const TVEC tVec_est(tVec[0], tVec[1], tVec[2]);
    
        // Conjugate/invert estimated quaternion
        // Conjugate is inverse when quaternion is unit
        const QVEC qVec_meas_inv(qVec_meas(0), T(-1)*qVec_meas(1), T(-1)*qVec_meas(2), T(-1)*qVec_meas(3));
    
        // Calculate quaternion error
        T dq[4];
        ceres::QuaternionProduct(qVec_est.data(), qVec_meas_inv.data(), dq);
    
        // Normalize quaternion error
        const T norm = sqrt(dq[0]*dq[0] + dq[1]*dq[1] + dq[2]*dq[2] + dq[3]*dq[3]);
        dq[0] /= norm;
        dq[1] /= norm;
        dq[2] /= norm;
        dq[3] /= norm;
    
        // Convert quaternion error to euler angle error
        // http://www.sedris.org/wg8home/Documents/WG80485.pdf
        // Page 39
        T re[3] = {
            atan2((dq[0]*dq[1] + dq[2]*dq[3]), T(0.5) - (dq[1]*dq[1] + dq[2]*dq[2])),
            asin(T(2)*(dq[0]*dq[2] - dq[3]*dq[1])),
            atan2((dq[0]*dq[3] + dq[1]*dq[2]), T(0.5) - (dq[2]*dq[2] + dq[3]*dq[3]))
        };
    
        // tvec residual
        const TVEC rt = tVec_est - tVec_meas;
    
        // Combined residuals
        const Eigen::Matrix<T, 6, 1> r = (Eigen::Matrix<T, 6, 1>() << rt(0), rt(1), rt(2), re[0], re[1], re[2]).finished();
    
        // Scale by square root info
        const Eigen::Matrix<T, 6, 1> rr = sqrtInfo * r; 
    
        // Output
        residuals[0] = rr(0);
        residuals[1] = rr(1);
        residuals[2] = rr(2);
        residuals[3] = rr(3);
        residuals[4] = rr(4);
        residuals[5] = rr(5);

        return true;
    }
 
private:
    const Eigen::Vector4d qVec_;
    const Eigen::Vector3d tVec_;
    const Eigen::Matrix<double, 6, 6> cov_;
};

// Reprojection cost function for standard bundle adjustment
class ReprojectionCostFunction {

public:
    ReprojectionCostFunction(const Eigen::Vector2d& feature2D)
        : feature2D_(feature2D)
    {}

    static ceres::CostFunction* Create(const Eigen::Vector2d& feature2D) {
        return (new ceres::AutoDiffCostFunction<ReprojectionCostFunction, 2, 4, 3, 3>(
            new ReprojectionCostFunction(feature2D)));
    }
  
    template <typename T>
    bool operator()(const T* const qVec, const T* const tVec,
                    const T* const feature3D, T* residuals) const {

 
        // Rotate and translate 3D feature point into camera frame
        T projection[3];
        ceres::UnitQuaternionRotatePoint(qVec, feature3D, projection);
        projection[0] += tVec[0];
        projection[1] += tVec[1];
        projection[2] += tVec[2];

        // Project to image plane.
        projection[0] /= projection[2];
        projection[1] /= projection[2];
        projection[2] /= projection[2];

        // Transform from meters to pixels
        const double pixelSize = 2e-6; // meters per pixel
        projection[0] /= pixelSize;
        projection[1] /= pixelSize;

        // Re-projection error.
        residuals[0] = projection[0] - T(feature2D_(0));
        residuals[1] = projection[1] - T(feature2D_(1));

        // Covariance of pixels. Assumed to be independent. Divide by standard deviation.
        const T sig = T(5.0);
        residuals[0] /= sig;
        residuals[1] /= sig;

        return true;
    }

private:
    const Eigen::Vector2d feature2D_;
};
