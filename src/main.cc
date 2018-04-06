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
#include "triangulate.h"

/* Extern Variables */
SensorParams sensor_params;

int main(int argc, char** argv) {
    google::InitGoogleLogging(argv[0]);

    // Load params
    sensor_params.image_width = 3840.0;
    sensor_params.image_height = 2176.0;

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
    std::vector< std::tuple<Eigen::Vector4d, Eigen::Vector3d, Eigen::Vector2d> > cam_info_vec;
    for(std::shared_ptr<Camera> cam: cameras) {
        if(cam->Observations().size() > 0) {
            cam_info_vec.push_back(
                std::make_tuple(
                    cam->QVec(),
                    cam->TVec(),
                    cam->Observations()[0].Measurement()
                )
            );
        }
    }
    std::cout << triangulate(cam_info_vec).transpose() << std::endl;
    return EXIT_SUCCESS;

    // Create a bundle adjuster
    BundleAdjuster bundle_adjuster(std::make_shared<Reconstruction>(reconstruction));
    bundle_adjuster.Solve();

    std::cout << red_feature->Pos().transpose() << std::endl;

    return EXIT_SUCCESS;
}
