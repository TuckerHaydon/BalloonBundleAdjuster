/* Author: Tucker Haydon */

#include <map>
#include <string>
#include <Eigen/Core>
#include <memory>

#include "types.h"
#include "reconstruction.h"
#include "feature.h"

/* Extern Variables */
SensorParams sensorParams;

int main(int argc, char** argv) {
    google::InitGoogleLogging(argv[0]);

    // Parse info from text files
    std::map<std::string, Eigen::Vector2d> balloonInfoMap = parseBalloonInfo("../input/balloon_positions.txt");
    std::map<std::string, PoseInfo> poseInfoMap = parsePoseInfo("../input/image_poses.txt");
 
    // Create the balloon feature point
    FeaturePtr red_feature = std::make_shared<Feature>(Eigen::Vector3d(0, 0, 0));

    Reconstruction reconstruction;

    reconstruction.AddFeature(red_feature);
    reconstruction.AddFeature(blue_feature);



    return EXIT_SUCCESS;
}
