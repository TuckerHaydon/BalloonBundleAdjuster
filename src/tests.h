/* Author: Tucker Haydon */
#pragma once 

#include <Eigen/Core>
#include <iostream>
#include <utility>

#include "triangulate.h"

void testTriangulate() {
    // Feature point at (1, 0, 0)
    std::vector< std::tuple<Eigen::Vector4d, Eigen::Vector3d, Eigen::Vector2d> > camInfoVec;

    // Cam 1 at (0, 0, 0) looking forward
    {
        const Eigen::Vector4d qCI = Eul2Quat321(-M_PI/2, 0, -M_PI/2);
        const Eigen::Vector4d qIC = InvertQuat(qCI);

        const Eigen::Vector3d ricI = Eigen::Vector3d(0, 0, 0);
        const Eigen::Vector3d rciC = QuatRotatePoint(qIC, -1 * ricI);

        const Eigen::Vector2d f = Eigen::Vector2d(0, 0);

        camInfoVec.push_back(std::make_tuple(qIC, rciC, f));
    }

    // Cam 1 at (2, 0, 0) looking backwards
    {
        const Eigen::Vector4d qCI = Eul2Quat321(-M_PI/2, 0, M_PI/2);
        const Eigen::Vector4d qIC = InvertQuat(qCI);

        const Eigen::Vector3d ricI = Eigen::Vector3d(2, 0, 0);
        const Eigen::Vector3d rciC = QuatRotatePoint(qIC, -1 * ricI);

        const Eigen::Vector2d f = Eigen::Vector2d(0, 0);

        camInfoVec.push_back(std::make_tuple(qIC, rciC, f));
    }

    // Cam 1 at (1, 0, 1) looking down
    {
        const Eigen::Vector4d qCI = Eul2Quat321(0, M_PI, 0);
        const Eigen::Vector4d qIC = InvertQuat(qCI);

        const Eigen::Vector3d ricI = Eigen::Vector3d(1, 0, 1);
        const Eigen::Vector3d rciC = QuatRotatePoint(qIC, -1 * ricI);

        const Eigen::Vector2d f = Eigen::Vector2d(0, 0);

        camInfoVec.push_back(std::make_tuple(qIC, rciC, f));
    }

    std::cout << triangulate(camInfoVec).transpose() << std::endl;
}
