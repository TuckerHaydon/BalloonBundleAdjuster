/* Author: Tucker Haydon */

#include <cstdlib>
#include <string>
#include <iostream>
#include <experimental/filesystem>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <map>

#include "callback.h"
#include "feature_extractor.h"

typedef std::shared_ptr<Camera> CameraPtr;

CameraPtr FeatureExtractor::ExtractFeaturesFromImage(const std::string& image_path, const PoseInfo& pose_info) {

    // Load image
    const cv::Mat image = cv::imread(image_path, CV_LOAD_IMAGE_COLOR);

    // Find balloons
    const std::vector<BalloonInfo> balloon_info_vec = processImage(image);

    // Initialize camera
    CameraPtr camera = std::make_shared<Camera>(pose_info.rciC, pose_info.qIC, pose_info.P);

    for(const auto& balloon_info: balloon_info_vec) {
        const Eigen::Vector3d pos = balloon_info.balloonLocation;
        switch(balloon_info.color) {
            red: 
                camera->AddObservation(Observation(red_feature_, Eigen::Vector2d(pos(0), pos(1))));
                break;
            blue:
                camera->AddObservation(Observation(blue_feature_, Eigen::Vector2d(pos(0), pos(1))));
                break;
            default: 
                std::cerr << "Not red nor blue??" << std::endl;
                exit(EXIT_FAILURE);
            break;
        }
    }

    return camera;

    // // Draw circle around located balloons
    // for(BalloonInfo& info: balloonInfoVec) {
    //     // Draw outer circle
    //     cv::circle(inputImg, 
    //         cv::Point2d(info.balloonLocation(0), info.balloonLocation(1)),
    //         info.balloonRadius,
    //         cv::Scalar(0, 0, 0),
    //         5);

    //     // Draw inner circle
    //     cv::circle(inputImg, 
    //         cv::Point2d(info.balloonLocation(0), info.balloonLocation(1)),
    //         5,
    //         cv::Scalar(0, 0, 0),
    //         5);
    // }

    // // Display image
    // if(balloonInfoVec.size() > 1) {
    //     std::cout << balloonInfoVec.size() << std::endl;
    //     cv::Mat displayImg = cv::Mat(960, 1280, 3);
    //     cv::resize(inputImg, displayImg, displayImg.size());
    //     cv::imshow("Image", displayImg); 
    //     cv::waitKey(0);  
    // }

}

std::vector<CameraPtr> FeatureExtractor::ExtractFeaturesFromImageDirectory(const std::string& directory_path) {
    namespace fs = std::experimental::filesystem;

    std::vector<CameraPtr> cameras;

    /* Process images */
    for(const auto& de: fs::directory_iterator(directory_path)){
        const std::string image_path = de.path().string();
        const std::string image_name = de.path().filename().string();

        // Only pull in files with extension ".jpg"
        if(image_name.find(".jpg") == std::string::npos) { continue; }

        // If the pose isn't logged, continue 
        if(pose_map_.find(image_name) == pose_map_.end()) { continue; }
        
        const PoseInfo pose_info = pose_map_.at(image_name);

        // Extract features from image and push onto vector
        const CameraPtr camera = ExtractFeaturesFromImage(image_path, pose_info);
        cameras.push_back(camera);
    }

    return cameras;
}
