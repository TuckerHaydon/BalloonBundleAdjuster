cmake_minimum_required(VERSION 3.5.0)
project(BalloonBundleAdjuster)
set(TARGET ${PROJECT_NAME})
set(CMAKE_BUILD_TYPE Release)

set(CMAKE_C_COMPILER "gcc-7")
set(CMAKE_CXX_COMPILER "g++-7")
set(CMAKE_CXX_FLAGS "-std=c++1z ${CMAKE_CXX_FLAGS}")

find_package(Ceres REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)

include_directories(
	${CERES_INCLUDE_DIRS}
	${EIGEN_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
)

include_directories(
    ${CMAKE_SOURCE_DIR}/src/base
    ${CMAKE_SOURCE_DIR}/src/bundle_adjuster
    ${CMAKE_SOURCE_DIR}/src/exe
    ${CMAKE_SOURCE_DIR}/src/feature
    ${CMAKE_SOURCE_DIR}/src/misc
)

file(GLOB_RECURSE SRC_FILES "src/*.cc")
file(GLOB_RECURSE INC_FILES "src/*.h")

# add the ROS executables
add_executable(${TARGET} ${SRC_FILES} ${INC_FILES})
target_link_libraries(${TARGET} ${CERES_LIBRARIES} ${EIGEN_LIBRARIES} ${OpenCV_LIBS})

