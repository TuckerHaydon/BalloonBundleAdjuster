/* Author: Tucker Haydon */

#include "triangulator.h"
#include "sensor_params.h"
#include "utility.h"

#include <map>
#include <iostream>
#include <vector>

typedef std::shared_ptr<Feature> FeaturePtr;

typedef struct {
    Eigen::Vector4d qIC;
    Eigen::Vector3d rciC;
    Eigen::Vector2d pix;
} FeatureInfo;

void Triangulator::Solve() {

    // Intrinsic matrix
    Eigen::Matrix<double, 3, 4> k = Eigen::Matrix<double, 3, 4>().setZero(); 
    k.topLeftCorner<3,3>().diagonal() = Eigen::Vector3d(sensor_params.f, sensor_params.f, 1);

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

            const double x_tilde = pix(0) * sensor_params.pixel_size;
            const double y_tilde = pix(1) * sensor_params.pixel_size;

            H.row(2*i)   = x_tilde * P3 - P1;
            H.row(2*i+1) = y_tilde * P3 - P2;
        }

        const Eigen::MatrixXd H_r = H.leftCols(3);
        const Eigen::MatrixXd z   = -1 * H.rightCols(1);
        const Eigen::Vector3d x   = (H_r.transpose() * H_r).inverse() * H_r.transpose() * z;

        feature->Prior(x);
    }
}
