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



typedef struct {
    Eigen::Vector3d ricI;
    Eigen::Vector3d rciC;
    Eigen::Vector4d qIC;
    Eigen::Matrix<double, 6, 6> P;
} PoseInfo;

const double imageWidth = 3840;
const double imageHeight = 2176;

std::map<std::string, Eigen::Vector2d> parseBalloonInfo(const std::string& filename) {
    
    std::map<std::string, Eigen::Vector2d> balloonInfoMap;

    std::ifstream f(filename);
    std::string line;
    while (getline(f, line)) {
        std::stringstream ss(line);
        std::string imageName;
        Eigen::Vector2d pos;
        
        ss >> imageName;
        ss >> pos(0);
        ss >> pos(1); 

        balloonInfoMap[imageName] = pos;
    }

    return balloonInfoMap;
}

std::map<std::string, PoseInfo> parsePoseInfo(const std::string& filename) {

    std::map<std::string, PoseInfo> poseInfoMap;

    std::ifstream f(filename);
    std::string line;
    while (getline(f, line)) {
        std::stringstream ss(line);
        std::string imageName;
        Eigen::Vector3d ricI;
        Eigen::Vector3d rciC;
        Eigen::Matrix<double, 3, 3> RIC;
        Eigen::Vector4d qIC;
        Eigen::Matrix<double, 6, 6> P;
        
        ss >> imageName;
        ss >> ricI(0) >> ricI(1) >> ricI(2);
        ss >> rciC(0) >> rciC(1) >> rciC(2);
    
        // Rotation
        for(int i = 0; i < 3; i ++) {
            for(int j = 0; j < 3; j++) {
                ss >> RIC(j, i);
            }
        }

        // Covariance
        for(int i = 0; i < 6; i++) {
            for(int j = 0; j < 6; j++) {
                ss >> P(j, i);
            }
        }

        PoseInfo poseInfo;
        poseInfo.ricI = ricI;
        poseInfo.rciC = rciC;
        poseInfo.qIC = Rot2Quat(RIC);
        poseInfo.P = P;

        poseInfoMap[imageName] = poseInfo;
    }

    return poseInfoMap;
}

int main(int argc, char** argv) {
    google::InitGoogleLogging(argv[0]);

    // Parse balloon info
    std::map<std::string, Eigen::Vector2d> balloonInfoMap = parseBalloonInfo("../input/balloon_positions.txt");

    // Parse pose info
    std::map<std::string, PoseInfo> poseInfoMap = parsePoseInfo("../input/image_poses.txt");
    
    // Create the balloon feature point
    auto fp = std::make_shared<FeaturePoint>(Eigen::Vector3d(-3, 0, 0));

    // Initialize cameras
    std::vector< std::tuple<Eigen::Vector4d, Eigen::Vector3d, Eigen::Vector2d> > camInfoVec;
    std::vector< std::shared_ptr<Camera> > cams;
    for (auto const& it: balloonInfoMap) {
        const std::string imageName = it.first;
        const Eigen::Vector2d featureGraphics = it.second;
        const Eigen::Vector2d feature = Eigen::Vector2d(featureGraphics(0) - imageWidth/2, -1*(featureGraphics(1) - imageHeight/2));
   
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
