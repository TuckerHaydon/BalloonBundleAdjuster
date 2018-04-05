/* Author: Tucker Haydon */

#include <ceres/ceres.h>

#include "bundle_adjuster.h"
#include "camera.h"

void BundleAdjuster::Solve() {
    // Create a ceres problem
    ceres::Problem problem;

    // Add blocks to cost function
    for (std::shared_ptr<Camera> cam: cams) {
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
    options.max_num_iterations = 10000;
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);
    std::cout << summary.FullReport() << std::endl;

    std::cout << fp->pos().transpose() << std::endl;

    return;
}
