/**
 * @file monocular-inertial-pose.hpp
 * @brief 声明单目惯性节点使用的位姿转换与标定读取辅助函数。
 */

#ifndef __MONOCULAR_INERTIAL_POSE_HPP__
#define __MONOCULAR_INERTIAL_POSE_HPP__

#include "sophus/se3.hpp"

#include <string>

/**
 * @brief 根据 ORB_SLAM3 相机位姿和机体到相机外参计算世界到机体系位姿。
 * @param Tcw 世界系到相机系位姿。
 * @param Tbc 机体系到相机系外参，对应 ORB_SLAM3 配置中的 IMU.T_b_c1 或 Tbc。
 * @return 世界系到机体系位姿。
 */
Sophus::SE3f ComputeBodyPoseFromCameraPose(const Sophus::SE3f& Tcw, const Sophus::SE3f& Tbc);

/**
 * @brief 从 ORB_SLAM3 配置文件读取机体系到相机系外参。
 * @param settingsFile ORB_SLAM3 YAML 配置文件路径。
 * @param Tbc 输出的机体系到相机系外参。
 * @param errorMessage 读取失败时输出错误说明；可为空。
 * @return 读取成功返回 true，否则返回 false。
 */
bool LoadBodyToCameraExtrinsic(
    const std::string& settingsFile,
    Sophus::SE3f& Tbc,
    std::string* errorMessage);

#endif
