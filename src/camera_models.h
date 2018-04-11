/* Author: Tucker Haydon */
#pragma once
#include "sensor_params.h"

template <typename T>
class OpenCVCameraModel {
public:
    OpenCVCameraModel();
    void WorldToImage(const T& x_prime, const T& y_prime, T* u, T* v);
private:
    const T fx_, fy_, cx_, cy_, k1_, k2_, k3_, p1_, p2_;
};

template <typename T>
inline OpenCVCameraModel<T>::OpenCVCameraModel() 
    : fx_(T(sensor_params.fx)),
      fy_(T(sensor_params.fy)),
      cx_(T(sensor_params.cx)),
      cy_(T(sensor_params.cy)),
      k1_(T(sensor_params.dist_coeffs.at<double>(0, 0))),
      k2_(T(sensor_params.dist_coeffs.at<double>(0, 1))),
      p1_(T(sensor_params.dist_coeffs.at<double>(0, 2))),
      p2_(T(sensor_params.dist_coeffs.at<double>(0, 3))),
      k3_(T(sensor_params.dist_coeffs.at<double>(0, 4)))
{}

// https://docs.opencv.org/3.0-beta/modules/calib3d/doc/camera_calibration_and_3d_reconstruction.html
template <typename T>
inline void OpenCVCameraModel<T>::WorldToImage(const T& x_prime, const T& y_prime, T* u, T* v) {
    
    const T r2 = x_prime * x_prime + y_prime * y_prime;
    const T r4 = r2 * r2;
    const T r6 = r2 * r2 * r2;
    const T y_prime2 = y_prime * y_prime;
    const T x_prime2 = x_prime * x_prime;
    const T xy_prime = x_prime * y_prime;

    const T x_prime_prime = x_prime * (T(1) + k1_*r2 + k2_*r4 + k3_*r6) + T(2)*p1_*xy_prime + p2_*(r2 + T(2)*x_prime2);
    const T y_prime_prime = y_prime * (T(1) + k1_*r2 + k2_*r4 + k3_*r6) + T(2)*p2_*xy_prime + p1_*(r2 + T(2)*y_prime2);

    *u = fx_ * x_prime_prime;
    *v = fy_ * y_prime_prime;
}
