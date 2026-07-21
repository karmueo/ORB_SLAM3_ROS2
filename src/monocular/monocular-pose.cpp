/**
 * @file monocular-pose.cpp
 * @brief 实现纯单目相机位姿转换、ROS 消息构造和有界轨迹维护逻辑。
 */

#include "monocular-pose.hpp"

#include <stdexcept>

namespace
{
/**
 * @brief 构造将 ROS camera_link 坐标向量转换到光学坐标的固定基变换。
 * @return 零平移变换，满足机体系 x 前、y 左、z 上和光学系 x 右、y 下、z 前。
 */
Sophus::SE3f ComputeOpticalFromCameraLinkBasis()
{
  /** @brief 将 ROS 机体坐标向量转换到 ORB 光学坐标的固定旋转。 */
  Eigen::Matrix3f optical_from_link_rotation;
  optical_from_link_rotation <<
      0.0F, -1.0F, 0.0F,
      0.0F, 0.0F, -1.0F,
      1.0F, 0.0F, 0.0F;
  return Sophus::SE3f(
      optical_from_link_rotation, Eigen::Vector3f::Zero());
}
}  // namespace

/** @brief 标记下一次有效位姿发布前需要重置轨迹。 */
void CameraPathResetState::MarkTrackingLost()
{
  reset_pending_ = true;
}

/** @brief 读取并清除轨迹重置请求。 */
bool CameraPathResetState::ConsumeResetRequest()
{
  /** @brief 本次调用需要返回的重置状态。 */
  const bool should_reset = reset_pending_;
  reset_pending_ = false;
  return should_reset;
}

/** @brief 将世界到相机位姿取逆为世界系下的相机位姿。 */
Sophus::SE3f ComputeCameraPoseInWorld(const Sophus::SE3f& Tcw)
{
  return Tcw.inverse();
}

/** @brief 将 ORB 光学坐标表示的相机位姿换基为 ROS 机体坐标表示。 */
Sophus::SE3f ConvertOrbCameraPoseToRos(const Sophus::SE3f& Twc)
{
  /** @brief ROS 机体系到 ORB 光学系的零平移固定变换。 */
  const Sophus::SE3f optical_from_link =
      ComputeOpticalFromCameraLinkBasis();
  return optical_from_link.inverse() * Twc * optical_from_link;
}

/** @brief 获取 camera_optical_frame 在 camera_link 中的固定姿态。 */
Sophus::SE3f ComputeCameraOpticalPoseInCameraLink()
{
  return ComputeOpticalFromCameraLinkBasis().inverse();
}

/** @brief 构造四元数归一化后的 ROS 位姿消息。 */
geometry_msgs::msg::PoseStamped CreateCameraPoseMessage(
  const Sophus::SE3f& pose_in_parent,
  const builtin_interfaces::msg::Time& stamp,
  const std::string& frame_id)
{
  /** @brief 目标姿态对应的归一化四元数。 */
  Eigen::Quaternionf quaternion = pose_in_parent.unit_quaternion();
  quaternion.normalize();
  /** @brief 目标在参考系下的平移。 */
  const Eigen::Vector3f translation = pose_in_parent.translation();
  /** @brief 待返回的 ROS 位姿消息。 */
  geometry_msgs::msg::PoseStamped pose_message;
  pose_message.header.stamp = stamp;
  pose_message.header.frame_id = frame_id;
  pose_message.pose.position.x = translation.x();
  pose_message.pose.position.y = translation.y();
  pose_message.pose.position.z = translation.z();
  pose_message.pose.orientation.x = quaternion.x();
  pose_message.pose.orientation.y = quaternion.y();
  pose_message.pose.orientation.z = quaternion.z();
  pose_message.pose.orientation.w = quaternion.w();
  return pose_message;
}

/** @brief 从 ROS 相机位姿构造具有相同变换的动态 TF 消息。 */
geometry_msgs::msg::TransformStamped CreateCameraTransformMessage(
  const geometry_msgs::msg::PoseStamped& pose_message,
  const std::string& child_frame_id)
{
  /** @brief 待返回的相机动态 TF 消息。 */
  geometry_msgs::msg::TransformStamped transform_message;
  transform_message.header = pose_message.header;
  transform_message.child_frame_id = child_frame_id;
  transform_message.transform.translation.x = pose_message.pose.position.x;
  transform_message.transform.translation.y = pose_message.pose.position.y;
  transform_message.transform.translation.z = pose_message.pose.position.z;
  transform_message.transform.rotation = pose_message.pose.orientation;
  return transform_message;
}

/** @brief 更新累计轨迹，并保证轨迹长度不超过配置上限。 */
void UpdateCameraPath(
  const geometry_msgs::msg::PoseStamped& pose_message,
  std::size_t max_path_length,
  bool reset_path,
  nav_msgs::msg::Path* path_message)
{
  if (path_message == nullptr)
  {
    throw std::invalid_argument("相机轨迹消息指针不能为空");
  }
  if (reset_path)
  {
    path_message->poses.clear();
  }

  path_message->header = pose_message.header;
  path_message->poses.push_back(pose_message);
  if (max_path_length > 0U && path_message->poses.size() > max_path_length)
  {
    /** @brief 超出配置上限、需要从轨迹开头删除的位姿数量。 */
    const std::size_t excess_count = path_message->poses.size() - max_path_length;
    path_message->poses.erase(
      path_message->poses.begin(),
      path_message->poses.begin() + excess_count);
  }
}

/** @brief 拒绝负数并将合法轨迹长度转换为无符号整数。 */
std::size_t ValidateMaxPathLength(std::int64_t configured_length)
{
  if (configured_length < 0)
  {
    throw std::invalid_argument("max_path_length 不能为负数");
  }
  return static_cast<std::size_t>(configured_length);
}
