/* Author: Tucker Haydon */

#include <map>
#include <string>
#include <Eigen/Core>
#include <memory>
#include <glog/logging.h>

#include "reconstruction.h"
#include "feature.h"
#include "sensor_params.h"
#include "feature_extractor.h"
#include "bundle_adjuster.h"

/* Extern Variables */
SensorParams sensorParams;

int main(int argc, char** argv) {
    google::InitGoogleLogging(argv[0]);

    // Parse info from text files
    const std::map<std::string, PoseInfo> pose_map = parsePoseInfo("../images/image_poses.txt");

    // Create the balloon feature points
    std::shared_ptr<Feature> red_feature  = std::make_shared<Feature>(Eigen::Vector3d(0, 0, 0));
    std::shared_ptr<Feature> blue_feature = std::make_shared<Feature>(Eigen::Vector3d(0, 0, 0));

    // Extract feature from images and generate cameras
    FeatureExtractor feature_extractor(pose_map, red_feature, blue_feature);
    std::vector< std::shared_ptr<Camera> > cameras =  feature_extractor.ExtractFeaturesFromImageDirectory("../images");
    
    // Create a reconstruction, add cameras and features
    Reconstruction reconstruction;
    reconstruction.AddCameras(cameras);
    reconstruction.AddFeatures({red_feature, blue_feature});

    // Triangulate the feature points
    // TODO

    // Create a bundle adjuster
    BundleAdjuster bundle_adjuster(std::make_shared<Reconstruction>(reconstruction));
    bundle_adjuster.Solve();

    std::cout << red_feature->Pos().transpose() << std::endl;

    return EXIT_SUCCESS;
}
