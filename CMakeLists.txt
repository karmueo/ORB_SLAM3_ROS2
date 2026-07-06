cmake_minimum_required(VERSION 3.5)
project(orbslam3)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}/CMakeModules)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(message_filters REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Eigen REQUIRED)
find_package(Sophus REQUIRED)
find_package(Pangolin REQUIRED)
find_package(ORB_SLAM3 REQUIRED)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)
endif()

include_directories(
  include
  ${ORB_SLAM3_ROOT_DIR}/include
  ${ORB_SLAM3_ROOT_DIR}/include/CameraModels
  ${ORB_SLAM3_ROOT_DIR}/Thirdparty/Sophus
  ${EIGEN_INCLUDE_DIR}
)

link_directories(
  include
)

add_executable(mono
  src/monocular/mono.cpp
  src/monocular/monocular-slam-node.cpp
)
ament_target_dependencies(mono rclcpp sensor_msgs cv_bridge ORB_SLAM3 Pangolin OpenCV)

add_executable(monocular-inertial
  src/monocular-inertial/monocular-inertial.cpp
  src/monocular-inertial/monocular-inertial-node.cpp
  src/monocular-inertial/monocular-inertial-pose.cpp
  src/monocular-inertial/monocular-inertial-sync.cpp
)
target_include_directories(monocular-inertial PRIVATE src/monocular-inertial)
ament_target_dependencies(monocular-inertial rclcpp sensor_msgs geometry_msgs nav_msgs cv_bridge ORB_SLAM3 Pangolin OpenCV)

add_executable(rgbd
  src/rgbd/rgbd.cpp
  src/rgbd/rgbd-slam-node.cpp
)
ament_target_dependencies(rgbd rclcpp sensor_msgs cv_bridge message_filters ORB_SLAM3 Pangolin OpenCV)

add_executable(rgbd-raw-data-converter
  src/rgbd/rgbd-raw-data-converter.cpp
  src/rgbd/rgbd-raw-data-converter-node.cpp
)
target_include_directories(rgbd-raw-data-converter PRIVATE src/rgbd)
ament_target_dependencies(rgbd-raw-data-converter rclcpp sensor_msgs)

add_executable(stereo
  src/stereo/stereo.cpp
  src/stereo/stereo-slam-node.cpp
)
ament_target_dependencies(stereo rclcpp sensor_msgs cv_bridge message_filters ORB_SLAM3 Pangolin OpenCV)

add_executable(stereo-inertial
  src/stereo-inertial/stereo-inertial.cpp
  src/stereo-inertial/stereo-inertial-node.cpp
)
ament_target_dependencies(stereo-inertial rclcpp sensor_msgs cv_bridge ORB_SLAM3 Pangolin OpenCV)

install(TARGETS mono monocular-inertial rgbd stereo stereo-inertial DESTINATION lib/${PROJECT_NAME})
install(TARGETS rgbd-raw-data-converter DESTINATION lib/${PROJECT_NAME})

install(PROGRAMS scripts/evaluate_trajectory.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME evaluate_trajectory
)

if(BUILD_TESTING)
  ament_add_pytest_test(evaluate_trajectory test/test_evaluate_trajectory.py)
  ament_add_gtest(test_rgbd_raw_data_converter test/test_rgbd_raw_data_converter.cpp src/rgbd/rgbd-raw-data-converter.cpp)
  if(TARGET test_rgbd_raw_data_converter)
    target_include_directories(test_rgbd_raw_data_converter PRIVATE src/rgbd)
    ament_target_dependencies(test_rgbd_raw_data_converter sensor_msgs)
  endif()
  ament_add_gtest(test_monocular_inertial_pose test/test_monocular_inertial_pose.cpp src/monocular-inertial/monocular-inertial-pose.cpp)
  if(TARGET test_monocular_inertial_pose)
    target_include_directories(test_monocular_inertial_pose PRIVATE src/monocular-inertial)
    ament_target_dependencies(test_monocular_inertial_pose ORB_SLAM3 OpenCV)
  endif()
  ament_add_gtest(test_monocular_inertial_sync test/test_monocular_inertial_sync.cpp src/monocular-inertial/monocular-inertial-sync.cpp)
  if(TARGET test_monocular_inertial_sync)
    target_include_directories(test_monocular_inertial_sync PRIVATE src/monocular-inertial)
    ament_target_dependencies(test_monocular_inertial_sync rclcpp sensor_msgs ORB_SLAM3 OpenCV)
  endif()
endif()

# Install launch files.
#install(DIRECTORY launch config vocabulary
#  DESTINATION share/${PROJECT_NAME}/)

ament_package()
