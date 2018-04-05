/* Author: Tucker Haydon */

#include <ceres/ceres.h>

#include "bundle_adjuster.h"
#include "cost_functions.h"
#include "camera.h"

void BundleAdjuster::Solve() {
    // Create a ceres problem
    ceres::Problem problem;

    // Add blocks to cost function
    for (std::shared_ptr<Camera> cam: reconstruction_->Cameras()) {
        // Camera Pose
        {
            ceres::CostFunction* cost_function = 
                CameraPoseCostFunction::Create(
                    cam->QVecPrior(), cam->TVecPrior(), cam->CovPrior()
                );
    
            problem.AddResidualBlock(cost_function,
                                     NULL, /* squared loss */
                                     cam->QVec().data(),
                                     cam->TVec().data());
        }

        // Feature points
        for(Observation observation: cam->Observations())
        {
            ceres::CostFunction* cost_function =
                ReprojectionCostFunction::Create(
                    observation.Measurement()
                );  

            problem.AddResidualBlock(cost_function,
                                     NULL,
                                     cam->QVec().data(),
                                     cam->TVec().data(),
                                     observation.GetFeature()->Pos().data()
                                     );
        }

        // Quaternion parameterization of SO3 with 4 parameters
        ceres::LocalParameterization* quaternion_parameterization =
            new ceres::QuaternionParameterization;
                problem.SetParameterization(
                    cam->QVec().data(), 
                    quaternion_parameterization);
    }

    // Solve problem
    ceres::Solver::Options options;
    options.minimizer_progress_to_stdout = true;
    options.max_num_iterations = 10000;
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);
    std::cout << summary.FullReport() << std::endl;

    return;
}
