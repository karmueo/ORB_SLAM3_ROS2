/**
 * @file monocular-pose.hpp
 * @brief 声明纯单目相机位姿转换、ROS 消息构造和有界轨迹维护接口。
 */

#ifndef ORBSLAM3_ROS2_MONOCULAR_POSE_HPP_
#define ORBSLAM3_ROS2_MONOCULAR_POSE_HPP_

#include <cstddef>
#include <cstdint>
#include <string>

#include <builtin_interfaces/msg/time.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <sophus/se3.hpp>

/**
 * @brief 记录纯单目跟踪丢失后是否需要重置 ROS 轨迹。
 */
class CameraPathResetState
{
public:
  /** @brief 标记 ORB-SLAM3 已进入 LOST 状态。 */
  void MarkTrackingLost();

  /**
   * @brief 读取并清除待处理的轨迹重置请求。
   * @return 存在待处理重置请求时返回 true。
   */
  bool ConsumeResetRequest();

private:
  /** @brief 下一次有效位姿发布前是否需要清空旧轨迹。 */
  bool reset_pending_{false};
};

/**
 * @brief 将 ORB-SLAM3 的世界到相机位姿转换为世界系下的相机位姿。
 * @param Tcw 世界系到相机系位姿。
 * @return 相机系到世界系位姿。
 */
Sophus::SE3f ComputeCameraPoseInWorld(const Sophus::SE3f& Tcw);

/**
 * @brief 将 ORB 光学坐标表示的相机位姿换基为 ROS 机体坐标表示。
 * @param Twc ORB 世界系下的当前相机光学位姿。
 * @return ROS map 系下的 camera_link 位姿，其中 x 前、y 左、z 上。
 */
Sophus::SE3f ConvertOrbCameraPoseToRos(const Sophus::SE3f& Twc);

/**
 * @brief 获取 camera_optical_frame 在 camera_link 中的固定姿态。
 * @return 零平移固定变换，满足光学系 x 右、y 下、z 前。
 */
Sophus::SE3f ComputeCameraOpticalPoseInCameraLink();

/**
 * @brief 构造指定参考系下的 ROS 位姿消息。
 * @param pose_in_parent 参考系下的目标位姿。
 * @param stamp 当前输入图像时间戳。
 * @param frame_id 世界坐标系名称。
 * @return 四元数已归一化的 ROS 位姿消息。
 */
geometry_msgs::msg::PoseStamped CreateCameraPoseMessage(
  const Sophus::SE3f& pose_in_parent,
  const builtin_interfaces::msg::Time& stamp,
  const std::string& frame_id);

/**
 * @brief 根据相机位姿消息构造等价的动态 TF 消息。
 * @param pose_message 世界系下的相机位姿消息。
 * @param child_frame_id 子坐标系名称。
 * @return 与输入位姿具有相同时间戳、平移和旋转的 TF 消息。
 */
geometry_msgs::msg::TransformStamped CreateCameraTransformMessage(
  const geometry_msgs::msg::PoseStamped& pose_message,
  const std::string& child_frame_id);

/**
 * @brief 将有效位姿加入轨迹，并按需重置或删除最旧位姿。
 * @param pose_message 要加入轨迹的当前相机位姿。
 * @param max_path_length 最大轨迹长度；0 表示无限累计。
 * @param reset_path 加入当前位姿前是否清空旧轨迹。
 * @param path_message 待更新的轨迹消息，不能为空。
 * @throws std::invalid_argument path_message 为空时抛出。
 */
void UpdateCameraPath(
  const geometry_msgs::msg::PoseStamped& pose_message,
  std::size_t max_path_length,
  bool reset_path,
  nav_msgs::msg::Path* path_message);

/**
 * @brief 校验并转换 max_path_length ROS 参数。
 * @param configured_length ROS 参数中的有符号整数。
 * @return 可用于轨迹裁剪的无符号长度。
 * @throws std::invalid_argument configured_length 小于 0 时抛出。
 */
std::size_t ValidateMaxPathLength(std::int64_t configured_length);

#endif  // ORBSLAM3_ROS2_MONOCULAR_POSE_HPP_
