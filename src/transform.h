/* Author: Tucker Haydon */

#pragma once

#include <Eigen/Core>

// Converts azimuth, elevation, and roll to 321 roll, pitch, yaw euler angles
// Azimuth - Rotation away from true north with positive towards east
// Elevation - Rotation away from ground with positive towards zenith
inline Eigen::Vector3d AzElRoll2RollPitchYaw(
    const double& az,
    const double& el, 
    const double& roll) {
        return Eigen::Vector3d(roll, -1*el, M_PI/2 - az);
}

inline Eigen::Vector3d Body2ENU(
    const Eigen::Vector3d& rB,      // Vector in body frame
    const Eigen::Matrix3d& RBI,     // Rotation of body away from ENU
    const Eigen::Vector3d& ribI     // Vector from ENU origin to body origin in ENU frame
    ) {
        return ribI + RBI * rB;
}

inline Eigen::Vector3d ENU2Body(
    const Eigen::Vector3d& rI,      // Vector in ENU frame
    const Eigen::Matrix3d& RIB,     // Rotation of ENU away from body
    const Eigen::Vector3d& rbiB     // Vector from body origin to ENU origin in body frame
    ) {
        return rbiB + RIB * rI;
}

inline Eigen::Vector3d ENU2ECEF(
    const Eigen::Vector3d& rI,      // Vector in ENU frame
    const Eigen::Matrix3d& RIG,     // Rotation of ENU away from ECEF
    const Eigen::Vector3d& rgiG     // Vector from ECEF origin to ENU origin in ECEF frame
    ) {
        return rgiG + RIG * rI;
}

inline Eigen::Vector3d ECEF2ENU(
    const Eigen::Vector3d& rG,      // Vector in ECEF frame
    const Eigen::Matrix3d& RGI,     // Rotation of ECEF away from ENU
    const Eigen::Vector3d& rigI     // Vector from ENU origin to ECEF origin in ENU frame
    ) {
        return rigI + RGI * rG;
}

inline Eigen::Vector3d Camera2Body(
    const Eigen::Vector3d& rC,      // Vector in camera frame
    const Eigen::Matrix3d& RCB,     // Rotation of camera frame away from body frame
    const Eigen::Vector3d& rbcB     // Vector from body origin to camera origin in body frame
    ) {
        return rbcB + RCB * rC;
}

inline Eigen::Vector3d Body2Camera(
    const Eigen::Vector3d& rB,      // Vector in body frame
    const Eigen::Matrix3d& RBC,     // Rotation of body away from camera
    const Eigen::Vector3d& rcbC     // Vector from camera origin to body origin in camera frame
    ) {
        return rcbC + RBC * rB;
}

inline Eigen::Vector3d Sphere2Cart(
    const double& az,               // Azimuth in radians
    const double& el,               // Elevation in radians
    const double& r                 // Radius
    ) {
        const double x = r * std::cos(el) * std::cos(az);
        const double y = r * std::cos(el) * std::sin(az);
        const double z = r * std::sin(el);
        return Eigen::Vector3d(x, y, z);
}

// Adopted from Todd Humphreys
inline Eigen::Vector3d ECEF2LLA(
    const Eigen::Vector3d& r        // Vector in ECEF frame
    ) {

        // WGS-84 ellipsoid semi-major and semi-minor axes, meters.
        constexpr double AA = 6378137.00000;             
        constexpr double BB = 6356752.3142518;

        constexpr double e = std::sqrt((AA*AA - BB*BB)/(AA*AA));
        constexpr double ep = std::sqrt((AA*AA - BB*BB)/(BB*BB));

        const double x = r(0);
        const double y = r(1);
        const double z = r(2);

        const double lon = std::atan2(y, x);
        const double p = std::sqrt(x*x + y*y);
        const double theta = atan2(z*AA, p*BB);
        const double lat = std::atan2(z + ep*ep*BB*std::pow(std::sin(theta), 3), p - e*e*AA*std::pow(std::cos(theta),3));
        const double N = AA / std::sqrt(1 - e*e*std::sin(lat)*std::sin(lat));
        const double alt = p / std::cos(lat) - N;

        return Eigen::Vector3d(lat, lon, alt);
}

// Adopted from Todd Humphreys
// Reference: Misra and Enge appendix to chapter 4.
inline Eigen::Matrix3d DECEF2ENU(
    const Eigen::Vector3d& riG      // Vector from ECEF origin to ENU origin in ECEF frame
    ) {
            const Eigen::Vector3d lat_lon_alt = ECEF2LLA(riG);
            const Eigen::Vector3d xyz = Sphere2Cart(lat_lon_alt(1), lat_lon_alt(0), 1);

            const Eigen::Vector3d v_vert = xyz;
            const Eigen::Vector3d v_east = Eigen::Vector3d(0, 0, 1).cross(v_vert).normalized();
            const Eigen::Vector3d v_north = v_vert.cross(v_east).normalized();

            return (Eigen::Matrix3d() << v_east, v_north, v_vert).finished().transpose();
}
