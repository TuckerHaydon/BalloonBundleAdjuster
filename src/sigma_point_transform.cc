
#include <vector>

#include "utility.h"
#include "io.h"
#include "measurement_converter.h"
#include "sigma_point_transform.h"


TransformedMeasurement SigmaPointTransform::TransformMeasurement(
    const MeasurementInfo& measurement) {

        // Extract state
        const double az           = measurement.az;
        const double el           = measurement.el;
        const double roll         = measurement.roll;
        const double az_sigma     = measurement.az_sigma;
        const double el_sigma     = measurement.el_sigma;
        const double roll_sigma   = measurement.roll_sigma;
        const Eigen::Vector3d rpG = measurement.rpG;
        const Eigen::Matrix3d RpG = measurement.RpG;
        const Eigen::Matrix3d RbG = measurement.RbG;

        const Eigen::Vector6d measurement_mean = (Eigen::Vector6d() << rpG(0), rpG(1), rpG(2), az, el, roll).finished();

        // Sqrt of covariance
        const Eigen::Matrix3d sqrt_R_att = Eigen::Vector3d(az_sigma, el_sigma, roll_sigma).asDiagonal();
        // const Eigen::Matrix3d R_att = Eigen::Vector3d(az_sigma*az_sigma, el_sigma*el_sigma, roll_sigma*roll_sigma).asDiagonal();
        const Eigen::Matrix3d L = Eigen::LLT<Eigen::Matrix3d>(RpG).matrixL(); 
        Eigen::Matrix6d S = Eigen::Matrix6d().Zero();
        S.topLeftCorner(3,3) = L;
        S.bottomRightCorner(3,3) = sqrt_R_att;

        // std::cout << RpG << std::endl << std::endl;
        // std::cout << L * L.transpose() << std::endl << std::endl;
        // Eigen::Matrix6d S = Eigen::Matrix6d().Zero();

        // Sigma points
        std::vector<Eigen::Vector6d> sigma_points;
        sigma_points.reserve(ns_);

        // Mean sigma point
        sigma_points.push_back(measurement_mean);
        
        // Upper sigma points
        for(size_t i = 0; i < nx_; i++) {
            sigma_points.push_back(measurement_mean + c_ * S.col(i));
        }

        // Lower sigma points
        for(size_t i = nx_; i < 2*nx_; i++) {
            sigma_points.push_back(measurement_mean - c_ * S.col(i - nx_));
        }

        // Push sigma points through the transform function 
        std::vector<Eigen::Vector6d> transformed_sigma_points;  
        Eigen::Vector3d rciC_mean;
        Eigen::Matrix3d RIC_mean;
        {
            const ProcessedMeasurement transformed_measurement = TransformRawMeasurement(RawMeasurement{az, el, roll, rpG});
            rciC_mean = transformed_measurement.rciC;
            RIC_mean = transformed_measurement.RIC;
        }

        for(size_t i = 0; i < ns_; i++) {
            const RawMeasurement raw_measurement{az, el, roll, rpG};
            const ProcessedMeasurement transformed_measurement = TransformRawMeasurement(raw_measurement);

            const Eigen::Vector3d rciC = transformed_measurement.rciC;
            const Eigen::Matrix3d RIC = transformed_measurement.RIC;

            const Eigen::Matrix3d dR = RIC * RIC_mean.transpose();
            const Eigen::Vector3d de = Rot2Eul321(dR);

            transformed_sigma_points.push_back((Eigen::Vector6d() << rciC(0), rciC(1), rciC(2), de(0), de(1), de(2)).finished());
        }

        // Recombine sigma points
        // Mean
        Eigen::Vector6d transformed_sigma_points_mean = Ws0_ 
            * transformed_sigma_points[0];

        for(size_t i = 1; i < ns_; i++) {
            transformed_sigma_points_mean = transformed_sigma_points_mean 
                +  Wsi_ 
                * transformed_sigma_points[i];
        }

        // Covaraince
        Eigen::Matrix6d transformed_sigma_points_cov = Wc0_ 
            * (transformed_sigma_points[0] - transformed_sigma_points_mean) 
            * (transformed_sigma_points[0] - transformed_sigma_points_mean).transpose();

        for(size_t i = 0; i < ns_; i++) {
            transformed_sigma_points_cov = transformed_sigma_points_cov 
                +  Wci_ 
                * (transformed_sigma_points[i] - transformed_sigma_points_mean) 
                * (transformed_sigma_points[i] - transformed_sigma_points_mean).transpose();
        }

        return TransformedMeasurement{rciC_mean, RIC_mean, transformed_sigma_points_cov};
}

