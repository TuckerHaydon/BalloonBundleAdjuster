/* Author: Tucker Haydon */

#include <cstdlib>
#include <memory>
#include <Eigen/Core>
#include <utility>
#include <ceres/ceres.h>
#include <glog/logging.h>
#include <map>
#include <fstream>

#include "featurePoint.h"
#include "camera.h"
#include "costFunctions.h"
#include "utility.h"
#include "triangulate.h"
#include "io.h"
#include "tests.h"



int main(int argc, char** argv) {
    google::InitGoogleLogging(argv[0]);
    testTriangulate();
    return EXIT_SUCCESS;

    // Parse info from text files
    std::map<std::string, Eigen::Vector2d> balloonInfoMap = parseBalloonInfo("../input/balloon_positions.txt");
    std::map<std::string, PoseInfo> poseInfoMap = parsePoseInfo("../input/image_poses.txt");
 
    // Create the balloon feature point
    auto fp = std::make_shared<FeaturePoint>(Eigen::Vector3d(-3, 0, 0));

    // Initialize cameras
    std::vector< std::tuple<Eigen::Vector4d, Eigen::Vector3d, Eigen::Vector2d> > camInfoVec;
    std::vector< std::shared_ptr<Camera> > cams;
    for (auto const& it: balloonInfoMap) {
        const std::string imageName = it.first;
        const Eigen::Vector2d feature = it.second;
   
        // If the pose isn't logged, continue 
        if(poseInfoMap.find(imageName) == poseInfoMap.end()) { continue; }

        PoseInfo poseInfo = poseInfoMap.at(imageName);

        std::shared_ptr<Camera> cam = std::make_shared<Camera>(
            std::make_pair(feature, fp),
            poseInfo.rciC,
            poseInfo.qIC,
            poseInfo.P
        );
        cams.push_back(cam);
        camInfoVec.push_back(std::make_tuple(poseInfo.qIC, poseInfo.rciC, feature));
    }


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
    std::cout << triangulate(camInfoVec).transpose() << std::endl;

    return EXIT_SUCCESS;
}
