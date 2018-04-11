/* Author: Tucker Haydon */

#include "triangulator.h"
#include "sensor_params.h"
#include "utility.h"

#include <map>
#include <iostream>
#include <vector>
#include <random>
#include <set>

typedef std::shared_ptr<Feature> FeaturePtr;

typedef struct {
    Eigen::Vector4d qIC;
    Eigen::Vector3d rciC;
    Eigen::Vector2d pix;
} FeatureInfo;

void Triangulator::Solve() {

    // Intrinsic matrix
    Eigen::Matrix<double, 3, 4> k = Eigen::Matrix<double, 3, 4>().setZero(); 
    k.topLeftCorner<3,3>().diagonal() = Eigen::Vector3d(sensor_params.fx, sensor_params.fy, 1);

    std::map<FeaturePtr, std::vector<FeatureInfo> > feature_info_map;
    for(auto camera: cameras_) {
        for(auto obs: camera->Observations()) {
            const FeatureInfo info { camera->QVec(), camera->TVec(), obs.Measurement()};

            if(feature_info_map.find(obs.GetFeature()) == feature_info_map.end()) {
                feature_info_map[obs.GetFeature()] = std::vector<FeatureInfo>();
            }

            feature_info_map[obs.GetFeature()].push_back(info);
        }
    }

    for(const auto& item: feature_info_map) {
        FeaturePtr feature = item.first;
        std::vector<FeatureInfo> feature_info_vec = item.second;

        const size_t nz = feature_info_vec.size();
        Eigen::MatrixXd H(2*nz, 4);


        for(size_t i = 0; i < nz; i++) {
            
            const Eigen::Vector4d qIC  = feature_info_vec[i].qIC;
            const Eigen::Vector3d rciC = feature_info_vec[i].rciC;
            const Eigen::Vector2d pix  = feature_info_vec[i].pix;

            Eigen::Matrix4d P_ = Eigen::Matrix4d().setZero();
            P_.topLeftCorner<3,3>() = Quat2Rot(qIC);
            P_.topRightCorner<3,1>() = rciC;
            P_(3,3) = 1;

            const Eigen::Matrix<double, 3, 4> P = k * P_;
            const Eigen::Vector4d P1 = P.row(0);
            const Eigen::Vector4d P2 = P.row(1);
            const Eigen::Vector4d P3 = P.row(2);

            const double x_tilde = pix(0);
            const double y_tilde = pix(1);

            H.row(2*i)   = x_tilde * P3 - P1;
            H.row(2*i+1) = y_tilde * P3 - P2;
        }

        const Eigen::MatrixXd H_r = H.leftCols(3);
        const Eigen::MatrixXd z   = -1 * H.rightCols(1);
        const Eigen::Vector3d x   = (H_r.transpose() * H_r).inverse() * H_r.transpose() * z;

        feature->SetPrior(x);
    }
}

void Triangulator::SolveRobust() {

    // Intrinsic matrix
    Eigen::Matrix<double, 3, 4> k_ = Eigen::Matrix<double, 3, 4>().setZero(); 
    k_.topLeftCorner<3,3>().diagonal() = Eigen::Vector3d(sensor_params.fx, sensor_params.fy, 1);

    // Map of feature to observation information
    std::map<FeaturePtr, std::vector<FeatureInfo> > feature_info_map;
    for(auto camera: cameras_) {
        for(auto obs: camera->Observations()) {
            const FeatureInfo info { camera->QVec(), camera->TVec(), obs.Measurement()};

            if(feature_info_map.find(obs.GetFeature()) == feature_info_map.end()) {
                feature_info_map[obs.GetFeature()] = std::vector<FeatureInfo>();
            }

            feature_info_map[obs.GetFeature()].push_back(info);
        }
    }


    for(const auto& item: feature_info_map) {
        FeaturePtr feature = item.first;
        std::vector<FeatureInfo> feature_info_vec = item.second;

        const int num_iterations = 30;
        const double max_error = 0.5; // meters
        const int num_samples = 5;
        const double num_data = feature_info_vec.size();
        const Eigen::Vector3d best_model(0, 0, 0);

        for(size_t k = 0; k < num_iterations; k++) {

            Eigen::MatrixXd H(2*num_samples, 4);

            std::set<int> sample_indexes;
            while(sample_indexes.size() < num_samples) {
                std::random_device rd;     // only used once to initialise (seed) engine
                std::mt19937 rng(rd());    // random-number engine used (Mersenne-Twister in this case)
                std::uniform_int_distribution<int> uni(0, num_data-1); // guaranteed unbiased
                auto random_integer = uni(rng);
                sample_indexes.insert(random_integer);
            }

            int counter = 0;
            for(int i: sample_indexes) { 
                const Eigen::Vector4d qIC  = feature_info_vec[i].qIC;
                const Eigen::Vector3d rciC = feature_info_vec[i].rciC;
                const Eigen::Vector2d pix  = feature_info_vec[i].pix;

                Eigen::Matrix4d P_ = Eigen::Matrix4d().setZero();
                P_.topLeftCorner<3,3>() = Quat2Rot(qIC);
                P_.topRightCorner<3,1>() = rciC;
                P_(3,3) = 1;

                const Eigen::Matrix<double, 3, 4> P = k_ * P_;
                const Eigen::Vector4d P1 = P.row(0);
                const Eigen::Vector4d P2 = P.row(1);
                const Eigen::Vector4d P3 = P.row(2);

                const double x_tilde = pix(0);
                const double y_tilde = pix(1);

                H.row(2*counter)   = x_tilde * P3 - P1;
                H.row(2*counter+1) = y_tilde * P3 - P2;
                counter++;
            }

            const Eigen::MatrixXd H_r = H.leftCols(3);
            const Eigen::MatrixXd z   = -1 * H.rightCols(1);
            const Eigen::Vector3d x   = (H_r.transpose() * H_r).inverse() * H_r.transpose() * z;
        }

        // feature->SetPrior(x);
    }
}
