cmake_minimum_required(VERSION 3.5.0)
project(BalloonBundleAdjustment)
set(TARGET ${PROJECT_NAME})
set(CMAKE_BUILD_TYPE Release)

set(CMAKE_CXX_FLAGS "-std=c++17 ${CMAKE_CXX_FLAGS}")

find_package(Ceres REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(
	${CERES_INCLUDE_DIRS}
	${EIGEN_INCLUDE_DIRS}
)

file(GLOB_RECURSE SRC_FILES "src/*.cc")
file(GLOB_RECURSE INC_FILES "src/*.h")

# add the ROS executables
add_executable(${TARGET} ${SRC_FILES} ${INC_FILES})
target_link_libraries(${TARGET} ${CERES_LIBRARIES} ${EIGEN_LIBRARIES})

