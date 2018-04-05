/* Author: Tucker Haydon */
#pragma once 

typedef struct {
    Eigen::Vector3d ricI;
    Eigen::Vector3d rciC;
    Eigen::Vector4d qIC;
    Eigen::Matrix<double, 6, 6> P;
} PoseInfo;


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

