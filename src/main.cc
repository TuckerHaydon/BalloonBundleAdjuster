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
#include "triangulator.h"
#include "transform.h"
#include "measurement_converter.h"
#include "sigma_point_transform.h"

/* Extern Variables */
SensorParams sensor_params;

int main(int argc, char** argv) {
    google::InitGoogleLogging(argv[0]);

    // Load params
    sensor_params.image_width   = 3840;
    sensor_params.image_height  = 2160;
    sensor_params.fx            = 1691.0;
    sensor_params.fy            = 1697.0;
    sensor_params.cx            = 1914.0;
    sensor_params.cy            = 1074.0;
    sensor_params.stdev_feature = 50.0;
    sensor_params.RCB           = Eul2Rot321(-120.0 * M_PI/180.0, 0, -M_PI/2);
    sensor_params.rcB           = Eigen::Vector3d(0.017, 0, -0.09);
    sensor_params.riG           = Eigen::Vector3d(-742018.3187986395, -5462218.0363918105, 3198014.2988005267);
    sensor_params.sig_roll      = 5 * M_PI/180;
    sensor_params.camera_matrix = (cv::Mat1d(3, 3) << sensor_params.fx, 0, sensor_params.cx, 0, sensor_params.fy, sensor_params.cy, 0, 0, 1);
    sensor_params.dist_coeffs = (cv::Mat1d(1, 5) << -0.06117476, 0.11208021, -0.00043455, -0.00232441, -0.06783447);

    // Parse info from text files
    std::map<std::string, MeasurementInfo> measurement_map = ParseMeasurementFile("../images/image_data_raw.txt");
    std::map<std::string, PoseInfo> pose_map;

    // Convert measurement to map
    for(const auto& it: measurement_map) {
        const std::string image_name = it.first;
        const MeasurementInfo meas_info = it.second;

        const TransformedMeasurement meas = SigmaPointTransform().TransformMeasurement(meas_info);
        pose_map[image_name] = PoseInfo{Eigen::Vector3d(0, 0, 0), meas.rciC, Rot2Quat(meas.RIC), meas.P};
    }

    // Create the balloon feature points
    std::shared_ptr<Feature> red_feature  = std::make_shared<Feature>();
    std::shared_ptr<Feature> blue_feature = std::make_shared<Feature>();

    // Extract feature from images and generate cameras
    std::cout << "Detecting balloons..." << std::endl;
    FeatureExtractor feature_extractor(pose_map, red_feature, blue_feature);
    std::vector< std::shared_ptr<Camera> > cameras =  feature_extractor.ExtractFeaturesFromImageDirectory("../images");
    
    // Create a reconstruction, add cameras and features
    Reconstruction reconstruction;
    reconstruction.AddCameras(cameras);
    reconstruction.AddFeatures({red_feature, blue_feature});

    // Triangulate the feature points
    Triangulator triangulator({red_feature, blue_feature}, cameras);
    triangulator.Solve();

    std::cout << "Triangulating features..." << std::endl;
    std::cout << red_feature->Pos().transpose() << std::endl;
    std::cout << blue_feature->Pos().transpose() << std::endl;

    // Create a bundle adjuster
    BundleAdjuster bundle_adjuster(std::make_shared<Reconstruction>(reconstruction));
    bundle_adjuster.Solve();

    std::cout << "Bundle Adjustment..." << std::endl;
    std::cout << red_feature->Pos().transpose() << std::endl;
    std::cout << blue_feature->Pos().transpose() << std::endl;

    return EXIT_SUCCESS;
}
