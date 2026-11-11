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

# The fixed ORB_SLAM3 core is built with its CMakeLists' -march=native.
# Eigen/g2o class layouts depend on that target, so wrapper and tests must use
# the same ISA flags when exchanging ORB_SLAM3 objects and IMU::Point vectors.
add_compile_options(-march=native)

# Use DT_RPATH (instead of DT_RUNPATH) so locally built Pangolin dependencies
# remain resolvable transitively without requiring a global runner setup.
if(CMAKE_VERSION VERSION_GREATER_EQUAL "3.13")
  add_link_options(-Wl,--disable-new-dtags)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(message_filters REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Eigen REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(rosbag2_storage REQUIRED)
find_package(Sophus REQUIRED)
find_package(Pangolin REQUIRED)
find_package(ORB_SLAM3 REQUIRED)

# 从仓库跟踪的压缩包生成运行时需要的 ORB-SLAM3 文本词典。
set(ORB_VOCABULARY_ARCHIVE "${CMAKE_CURRENT_SOURCE_DIR}/vocabulary/ORBvoc.txt.tar.gz")
set(ORB_VOCABULARY_BUILD_DIR "${CMAKE_CURRENT_BINARY_DIR}/vocabulary")
set(ORB_VOCABULARY_FILE "${ORB_VOCABULARY_BUILD_DIR}/ORBvoc.txt")
file(MAKE_DIRECTORY "${ORB_VOCABULARY_BUILD_DIR}")
add_custom_command(
  OUTPUT "${ORB_VOCABULARY_FILE}"
  COMMAND "${CMAKE_COMMAND}" -E tar xzf "${ORB_VOCABULARY_ARCHIVE}"
  DEPENDS "${ORB_VOCABULARY_ARCHIVE}"
  WORKING_DIRECTORY "${ORB_VOCABULARY_BUILD_DIR}"
  COMMENT "Extracting ORB-SLAM3 vocabulary"
  VERBATIM
)
add_custom_target(extract_orb_vocabulary ALL DEPENDS "${ORB_VOCABULARY_FILE}")

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
  src/monocular/monocular-feature-mask.cpp
  src/monocular/monocular-pose.cpp
  src/monocular/monocular-slam-node.cpp
)
ament_target_dependencies(mono rclcpp sensor_msgs geometry_msgs nav_msgs cv_bridge tf2_ros ORB_SLAM3 Pangolin OpenCV)

add_executable(monocular-inertial
  src/monocular/monocular-feature-mask.cpp
  src/monocular-inertial/monocular-inertial.cpp
  src/monocular-inertial/monocular-inertial-node.cpp
  src/monocular-inertial/monocular-inertial-pose.cpp
  src/monocular-inertial/monocular-inertial-sync.cpp
)
target_include_directories(monocular-inertial PRIVATE src/monocular src/monocular-inertial)
ament_target_dependencies(monocular-inertial rclcpp sensor_msgs geometry_msgs nav_msgs cv_bridge tf2_ros ORB_SLAM3 Pangolin OpenCV)

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
  src/stereo-inertial/offline-bag-reader.cpp
  src/stereo-inertial/offline-cli.cpp
  src/stereo-inertial/offline-output.cpp
  src/stereo-inertial/offline-runner.cpp
  src/stereo-inertial/offline-slam-sink.cpp
  src/stereo-inertial/offline-sync.cpp
)
target_include_directories(stereo-inertial PRIVATE src/stereo-inertial)
ament_target_dependencies(stereo-inertial rclcpp sensor_msgs cv_bridge rosbag2_cpp rosbag2_storage ORB_SLAM3 Pangolin OpenCV)

install(TARGETS mono monocular-inertial rgbd stereo stereo-inertial DESTINATION lib/${PROJECT_NAME})
install(TARGETS rgbd-raw-data-converter DESTINATION lib/${PROJECT_NAME})

install(PROGRAMS scripts/evaluate_trajectory.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME evaluate_trajectory
)
install(PROGRAMS scripts/validate_xv_0709_bag.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME validate_xv_0709_bag
)
install(PROGRAMS scripts/prepare_ego_stereo_inertial_config.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME prepare-ego-stereo-inertial-config
)
install(PROGRAMS scripts/align_ego_stereo_inertial_time.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME align-ego-stereo-inertial-time
)
install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)
install(FILES "${ORB_VOCABULARY_ARCHIVE}" "${ORB_VOCABULARY_FILE}"
  DESTINATION share/${PROJECT_NAME}/vocabulary
)
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE
  PATTERN "*.pyc" EXCLUDE
)
install(DIRECTORY rviz
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  ament_add_pytest_test(evaluate_trajectory test/test_evaluate_trajectory.py)
  ament_add_pytest_test(validate_xv_0709_bag test/test_validate_xv_0709_bag.py)
  ament_add_pytest_test(xv_rgb_fisheye_config test/test_xv_rgb_fisheye_config.py)
  ament_add_pytest_test(xv_rgb_fisheye_monocular_config test/test_xv_rgb_fisheye_monocular_config.py)
  add_executable(prepare_ego_config_settings_probe test/prepare_ego_config_settings_probe.cpp)
  ament_target_dependencies(prepare_ego_config_settings_probe ORB_SLAM3 Pangolin OpenCV)
  ament_add_pytest_test(prepare_ego_stereo_inertial_config test/test_prepare_ego_stereo_inertial_config.py
    ENV "EGO_CONFIG_SETTINGS_PROBE=$<TARGET_FILE:prepare_ego_config_settings_probe>")
  ament_add_pytest_test(prepare_offline_core_patch_stack test/test_prepare_offline_core_patch_stack.py)
  ament_add_pytest_test(align_ego_stereo_inertial_time test/test_align_ego_stereo_inertial_time.py)
  ament_add_gtest(test_monocular_feature_mask test/test_monocular_feature_mask.cpp src/monocular/monocular-feature-mask.cpp)
  if(TARGET test_monocular_feature_mask)
    target_include_directories(test_monocular_feature_mask PRIVATE src/monocular)
    target_compile_definitions(test_monocular_feature_mask PRIVATE ORBSLAM3_TEST_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}")
    ament_target_dependencies(test_monocular_feature_mask ORB_SLAM3 Pangolin OpenCV)
  endif()
  ament_add_gtest(test_core_fisheye_settings test/test_core_fisheye_settings.cpp)
  if(TARGET test_core_fisheye_settings)
    ament_target_dependencies(test_core_fisheye_settings ORB_SLAM3 Pangolin OpenCV)
  endif()
  ament_add_gtest(test_monocular_pose test/test_monocular_pose.cpp src/monocular/monocular-pose.cpp)
  if(TARGET test_monocular_pose)
    target_include_directories(test_monocular_pose PRIVATE src/monocular)
    ament_target_dependencies(test_monocular_pose geometry_msgs nav_msgs)
  endif()

  ament_add_gtest(test_rgbd_raw_data_converter test/test_rgbd_raw_data_converter.cpp src/rgbd/rgbd-raw-data-converter.cpp)
  if(TARGET test_rgbd_raw_data_converter)
    target_include_directories(test_rgbd_raw_data_converter PRIVATE src/rgbd)
    ament_target_dependencies(test_rgbd_raw_data_converter sensor_msgs)
  endif()
  ament_add_gtest(test_monocular_inertial_pose test/test_monocular_inertial_pose.cpp src/monocular-inertial/monocular-inertial-pose.cpp)
  if(TARGET test_monocular_inertial_pose)
    target_include_directories(test_monocular_inertial_pose PRIVATE src/monocular-inertial)
    ament_target_dependencies(test_monocular_inertial_pose geometry_msgs nav_msgs ORB_SLAM3 OpenCV)
  endif()
  ament_add_gtest(test_monocular_inertial_sync test/test_monocular_inertial_sync.cpp src/monocular-inertial/monocular-inertial-sync.cpp)
  if(TARGET test_monocular_inertial_sync)
    target_include_directories(test_monocular_inertial_sync PRIVATE src/monocular-inertial)
    ament_target_dependencies(test_monocular_inertial_sync rclcpp sensor_msgs ORB_SLAM3 OpenCV)
  endif()

  ament_add_gtest(test_offline_cli test/test_offline_cli.cpp src/stereo-inertial/offline-cli.cpp)
  if(TARGET test_offline_cli)
    target_include_directories(test_offline_cli PRIVATE src/stereo-inertial)
    ament_target_dependencies(test_offline_cli OpenCV)
  endif()

  ament_add_gtest(test_offline_sync test/test_offline_sync.cpp src/stereo-inertial/offline-sync.cpp)
  if(TARGET test_offline_sync)
    target_include_directories(test_offline_sync PRIVATE src/stereo-inertial)
    ament_target_dependencies(test_offline_sync ORB_SLAM3 OpenCV)
  endif()

  ament_add_gtest(test_offline_output test/test_offline_output.cpp
    src/stereo-inertial/offline-output.cpp
    src/stereo-inertial/offline-sync.cpp)
  if(TARGET test_offline_output)
    target_include_directories(test_offline_output PRIVATE src/stereo-inertial)
    ament_target_dependencies(test_offline_output ORB_SLAM3 Pangolin OpenCV)
  endif()

  ament_add_gtest(test_offline_bag_reader test/test_offline_bag_reader.cpp
    src/stereo-inertial/offline-bag-reader.cpp
    src/stereo-inertial/offline-sync.cpp)
  if(TARGET test_offline_bag_reader)
    target_include_directories(test_offline_bag_reader PRIVATE src/stereo-inertial)
    ament_target_dependencies(test_offline_bag_reader rclcpp sensor_msgs rosbag2_cpp rosbag2_storage OpenCV)
  endif()

  ament_add_gtest(test_offline_runner test/test_offline_runner.cpp
    src/stereo-inertial/offline-runner.cpp
    src/stereo-inertial/offline-output.cpp
    src/stereo-inertial/offline-sync.cpp
    src/stereo-inertial/offline-slam-sink.cpp
    src/stereo-inertial/offline-bag-reader.cpp)
  if(TARGET test_offline_runner)
    target_include_directories(test_offline_runner PRIVATE src/stereo-inertial)
    ament_target_dependencies(test_offline_runner rclcpp sensor_msgs rosbag2_cpp rosbag2_storage ORB_SLAM3 Pangolin OpenCV)
  endif()

  ament_add_gtest(test_offline_core_lifecycle test/test_offline_core_lifecycle.cpp)
  if(TARGET test_offline_core_lifecycle)
    target_compile_definitions(test_offline_core_lifecycle PRIVATE
      OFFLINE_TEST_VOCABULARY="${CMAKE_CURRENT_BINARY_DIR}/vocabulary/ORBvoc.txt")
    # Test-only access to existing core internals: no production debug API is
    # added just so gtest can seed a LoopClosing queue or Tracking record.
    if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
      target_compile_options(test_offline_core_lifecycle PRIVATE -fno-access-control)
    endif()
    ament_target_dependencies(test_offline_core_lifecycle ORB_SLAM3 Pangolin OpenCV)
  endif()
endif()


ament_package()
