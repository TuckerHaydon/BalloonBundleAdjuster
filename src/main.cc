/* Author: Tucker Haydon */

#include <cstdlib>
#include <memory>
#include <Eigen/Core>
#include <utility>
#include <ceres/ceres.h>
#include <glog/logging.h>

#include "featurePoint.h"
#include "camera.h"
#include "costFunctions.h"


// Returns quaternion representation of inertial frame away from camera frame
// Takes 3-2-1 euler angle rotation of camera away from inertial
Eigen::Vector4d QuaternionInertialAwayFromCamera(const double roll,     // R 
                                                 const double pitch, 
                                                 const double yaw) {
    const Eigen::Quaterniond q(
        Eigen::AngleAxisd(yaw,   Eigen::Vector3d::UnitZ()) *
        Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
        Eigen::AngleAxisd(roll,  Eigen::Vector3d::UnitX()));

    return Eigen::Vector4d(q.w(), -q.x(), -q.y(), -q.z());
}

int main(int argc, char** argv) {
    google::InitGoogleLogging(argv[0]);

    
    // Create a feature point
    auto fp = std::make_shared<FeaturePoint>(Eigen::Vector3d(0,0,0));

    // Intialize dummy covariance
    const Eigen::Matrix<double, 6, 6> cov = (Eigen::Matrix<double, 6, 1>() << 0.03, 0.03, 0.03, 1e9, 1e9, 1e9).finished().asDiagonal();

    // Initialize some cameras
    Camera cam1(std::make_pair(Eigen::Vector2d(0.1, -0.03), fp),
                Eigen::Vector3d(0, 0, 1),
                QuaternionInertialAwayFromCamera(M_PI/2, M_PI/4, -M_PI/6),
                cov);

    Camera cam2(std::make_pair(Eigen::Vector2d(0.001, 0), fp),
                Eigen::Vector3d(0, 0, 1),
                QuaternionInertialAwayFromCamera(0, 0, 0),
                cov);

    Camera cam3(std::make_pair(Eigen::Vector2d(-0.05, 0.03), fp),
                Eigen::Vector3d(0, 0, 1),
                QuaternionInertialAwayFromCamera(M_PI, 0, M_PI/2),
                cov);

    const std::vector<Camera*> cams = {&cam1, &cam2, &cam3};

    // Create a ceres problem
    ceres::Problem problem;

    // Add blocks to cost function
    for (Camera* cam: cams) {
        // Camera Pose
        {
            ceres::CostFunction* cost_function = 
                CameraPoseCostFunction::Create(
                    cam->qVecPrior(), cam->tVecPrior(), cam->covPrior()
                );
    
            problem.AddResidualBlock(cost_function,
                                     NULL, /* squared loss */
                                     cam->qVec().data(),
                                     cam->tVec().data());
        }

        // Feature point
        {
            ceres::CostFunction* cost_function =
                ReprojectionCostFunction::Create(
                    cam->fp().first
                );  

            problem.AddResidualBlock(cost_function,
                                     NULL,
                                     cam->qVec().data(),
                                     cam->tVec().data(),
                                     cam->fp().second->pos().data()
                                     );
        }

        // Quaternion parameterization of SO3 with 4 parameters
        ceres::LocalParameterization* quaternion_parameterization =
            new ceres::QuaternionParameterization;
                problem.SetParameterization(
                    cam->qVec().data(), 
                    quaternion_parameterization);
    }

    // Solve problem
    ceres::Solver::Options options;
    options.minimizer_progress_to_stdout = true;
    options.max_num_iterations = 500;
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);
    std::cout << summary.FullReport() << std::endl;

    std::cout << fp->pos().transpose() << std::endl;

    return EXIT_SUCCESS;
}
