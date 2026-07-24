/**
 * @file test_monocular_pose.cpp
 * @brief 测试纯单目相机位姿转换、ROS 消息、动态 TF 和有界轨迹维护。
 */

#include "monocular-pose.hpp"

#include <cstdint>
#include <stdexcept>

#include "gtest/gtest.h"

namespace
{
/** @brief 浮点比较容差。 */
constexpr float kTolerance = 1e-5F;
/** @brief 测试使用的 90 度弧度值。 */
constexpr float kHalfPi = 1.57079632679489661923F;

/**
 * @brief 创建带有指定 x 坐标和秒级时间戳的测试位姿。
 * @param x_position 世界系下的 x 坐标。
 * @param stamp_seconds 消息时间戳的秒字段。
 * @return 用于轨迹测试的位姿消息。
 */
geometry_msgs::msg::PoseStamped CreateTestPose(
    double x_position,
    std::int32_t stamp_seconds)
{
  /** @brief 待返回的测试位姿消息。 */
  geometry_msgs::msg::PoseStamped pose_message;
  pose_message.header.frame_id = "camera_start";
  pose_message.header.stamp.sec = stamp_seconds;
  pose_message.pose.position.x = x_position;
  pose_message.pose.orientation.w = 1.0;
  return pose_message;
}
}  // namespace

/** @brief 验证世界到相机位姿的平移会被正确取逆。 */
TEST(MonocularPose, InvertsTranslation)
{
  /** @brief 世界系到相机系位姿。 */
  const Sophus::SE3f Tcw(
      Eigen::Matrix3f::Identity(),
      Eigen::Vector3f(1.0F, -2.0F, 3.0F));
  /** @brief 转换得到的世界系下相机位姿。 */
  const Sophus::SE3f Twc = ComputeCameraPoseInWorld(Tcw);

  EXPECT_NEAR(Twc.translation().x(), -1.0F, kTolerance);
  EXPECT_NEAR(Twc.translation().y(), 2.0F, kTolerance);
  EXPECT_NEAR(Twc.translation().z(), -3.0F, kTolerance);
}

/** @brief 验证世界到相机姿态会被正确取逆且四元数保持归一化。 */
TEST(MonocularPose, InvertsRotationAndKeepsQuaternionNormalized)
{
  /** @brief 绕 z 轴旋转 90 度的世界到相机姿态。 */
  const Eigen::AngleAxisf camera_rotation(
      kHalfPi, Eigen::Vector3f::UnitZ());
  /** @brief 世界系到相机系位姿。 */
  const Sophus::SE3f Tcw(
      camera_rotation.toRotationMatrix(), Eigen::Vector3f::Zero());
  /** @brief 转换得到的世界系下相机位姿。 */
  const Sophus::SE3f Twc = ComputeCameraPoseInWorld(Tcw);
  /** @brief 转换位姿对应的姿态四元数。 */
  const Eigen::Quaternionf quaternion = Twc.unit_quaternion();

  EXPECT_NEAR(quaternion.norm(), 1.0F, kTolerance);
  EXPECT_TRUE(
      Twc.rotationMatrix().isApprox(
          camera_rotation.toRotationMatrix().transpose(),
          kTolerance));
}

/** @brief 验证单位光学位姿换基后得到单位 ROS 机体位姿。 */
TEST(MonocularPose, ConvertsIdentityPoseToRosCameraLink)
{
  /** @brief ORB 世界系下的单位相机光学位姿。 */
  const Sophus::SE3f Twc;
  /** @brief ROS camera_start 系下转换后的 camera_link 位姿。 */
  const Sophus::SE3f camera_start_from_camera_link =
      ConvertOrbCameraPoseToRos(Twc);

  EXPECT_TRUE(
      camera_start_from_camera_link.matrix().isApprox(
          Sophus::SE3f().matrix(), kTolerance));
}

/** @brief 验证光学系平移轴映射为 ROS 的 x 前、y 左、z 上。 */
TEST(MonocularPose, MapsOpticalTranslationAxesToRosCameraStart)
{
  /** @brief 光学系向前移动一个单位后的相机位姿。 */
  const Sophus::SE3f optical_forward_pose(
      Eigen::Matrix3f::Identity(), Eigen::Vector3f::UnitZ());
  /** @brief 光学系向右移动一个单位后的相机位姿。 */
  const Sophus::SE3f optical_right_pose(
      Eigen::Matrix3f::Identity(), Eigen::Vector3f::UnitX());
  /** @brief 光学系向下移动一个单位后的相机位姿。 */
  const Sophus::SE3f optical_down_pose(
      Eigen::Matrix3f::Identity(), Eigen::Vector3f::UnitY());
  /** @brief ROS camera_start 系下的向前平移。 */
  const Sophus::SE3f ros_forward_pose =
      ConvertOrbCameraPoseToRos(optical_forward_pose);
  /** @brief ROS camera_start 系下的向右平移。 */
  const Sophus::SE3f ros_right_pose =
      ConvertOrbCameraPoseToRos(optical_right_pose);
  /** @brief ROS camera_start 系下的向下平移。 */
  const Sophus::SE3f ros_down_pose =
      ConvertOrbCameraPoseToRos(optical_down_pose);

  EXPECT_TRUE(
      ros_forward_pose.translation().isApprox(
          Eigen::Vector3f::UnitX(), kTolerance));
  EXPECT_TRUE(
      ros_right_pose.translation().isApprox(
          -Eigen::Vector3f::UnitY(), kTolerance));
  EXPECT_TRUE(
      ros_down_pose.translation().isApprox(
          -Eigen::Vector3f::UnitZ(), kTolerance));
}

/** @brief 验证绕光学系负 y 轴旋转会转换为绕机体系正 z 轴旋转。 */
TEST(MonocularPose, ConvertsOpticalRotationToRosBodyRotation)
{
  /** @brief 绕光学系负 y 轴旋转 90 度的相机位姿。 */
  const Sophus::SE3f optical_pose(
      Eigen::AngleAxisf(
          kHalfPi, -Eigen::Vector3f::UnitY()).toRotationMatrix(),
      Eigen::Vector3f::Zero());
  /** @brief 换基后的 ROS camera_link 位姿。 */
  const Sophus::SE3f ros_pose = ConvertOrbCameraPoseToRos(optical_pose);
  /** @brief 期望的绕 ROS 正 z 轴旋转 90 度姿态。 */
  const Eigen::Matrix3f expected_rotation =
      Eigen::AngleAxisf(
          kHalfPi, Eigen::Vector3f::UnitZ()).toRotationMatrix();

  EXPECT_TRUE(
      ros_pose.rotationMatrix().isApprox(expected_rotation, kTolerance));
}

/** @brief 验证机体到光学静态 TF 的轴映射和动态 TF 链组合。 */
TEST(MonocularPose, ComposesCameraLinkAndOpticalTransforms)
{
  /** @brief camera_optical_frame 在 camera_link 中的固定变换。 */
  const Sophus::SE3f link_from_optical =
      ComputeCameraOpticalPoseInCameraLink();
  EXPECT_TRUE(
      (link_from_optical * Eigen::Vector3f::UnitX()).isApprox(
          -Eigen::Vector3f::UnitY(), kTolerance));
  EXPECT_TRUE(
      (link_from_optical * Eigen::Vector3f::UnitY()).isApprox(
          -Eigen::Vector3f::UnitZ(), kTolerance));
  EXPECT_TRUE(
      (link_from_optical * Eigen::Vector3f::UnitZ()).isApprox(
          Eigen::Vector3f::UnitX(), kTolerance));

  /** @brief 用于验证 TF 链组合的 ORB 相机光学位姿。 */
  const Sophus::SE3f Twc(
      Eigen::AngleAxisf(0.4F, Eigen::Vector3f::UnitX()).toRotationMatrix(),
      Eigen::Vector3f(1.0F, 2.0F, 3.0F));
  /** @brief ROS camera_start 到 camera_link 的动态变换。 */
  const Sophus::SE3f camera_start_from_camera_link =
      ConvertOrbCameraPoseToRos(Twc);
  /** @brief 动态 TF 与静态 TF 组合得到的光学位姿。 */
  const Sophus::SE3f composed_camera_start_from_optical =
      camera_start_from_camera_link * link_from_optical;
  /** @brief 直接将 ORB 世界基准旋转到 ROS camera_start 后的期望光学位姿。 */
  const Sophus::SE3f expected_camera_start_from_optical =
      link_from_optical * Twc;

  EXPECT_TRUE(
      composed_camera_start_from_optical.matrix().isApprox(
          expected_camera_start_from_optical.matrix(), kTolerance));
}

/** @brief 验证 camera_link 的 PoseStamped 和动态 TF 保持一致。 */
TEST(MonocularPose, CreatesConsistentPoseAndTransformMessages)
{
  /** @brief 世界系下用于消息转换测试的相机位姿。 */
  const Sophus::SE3f Twc(
      Eigen::AngleAxisf(0.4F, Eigen::Vector3f::UnitY()).toRotationMatrix(),
      Eigen::Vector3f(1.0F, 2.0F, 3.0F));
  /** @brief 输入图像时间戳。 */
  builtin_interfaces::msg::Time stamp;
  stamp.sec = 42;
  stamp.nanosec = 123456789U;
  /** @brief 转换得到的 ROS 相机位姿消息。 */
  const geometry_msgs::msg::PoseStamped pose_message =
      CreateCameraPoseMessage(Twc, stamp, "camera_start");
  /** @brief 根据位姿消息生成的动态 TF。 */
  const geometry_msgs::msg::TransformStamped transform_message =
      CreateCameraTransformMessage(
          pose_message, "camera_link");
  /** @brief 位姿消息中四元数的模长平方。 */
  const double quaternion_norm_squared =
      pose_message.pose.orientation.x * pose_message.pose.orientation.x +
      pose_message.pose.orientation.y * pose_message.pose.orientation.y +
      pose_message.pose.orientation.z * pose_message.pose.orientation.z +
      pose_message.pose.orientation.w * pose_message.pose.orientation.w;

  EXPECT_EQ(pose_message.header.frame_id, "camera_start");
  EXPECT_EQ(pose_message.header.stamp, stamp);
  EXPECT_NEAR(quaternion_norm_squared, 1.0, kTolerance);
  EXPECT_EQ(transform_message.header, pose_message.header);
  EXPECT_EQ(transform_message.child_frame_id, "camera_link");
  EXPECT_DOUBLE_EQ(
      transform_message.transform.translation.x,
      pose_message.pose.position.x);
  EXPECT_DOUBLE_EQ(
      transform_message.transform.translation.y,
      pose_message.pose.position.y);
  EXPECT_DOUBLE_EQ(
      transform_message.transform.translation.z,
      pose_message.pose.position.z);
  EXPECT_EQ(
      transform_message.transform.rotation,
      pose_message.pose.orientation);
}

/** @brief 验证有限轨迹只保留最新的配置数量位姿。 */
TEST(MonocularPose, BoundsPathAndDropsOldestPoses)
{
  /** @brief 接收有限长度位姿的轨迹消息。 */
  nav_msgs::msg::Path path_message;
  for (std::int32_t index = 1; index <= 4; ++index)
  {
    UpdateCameraPath(
        CreateTestPose(static_cast<double>(index), index),
        3U,
        false,
        &path_message);
  }

  ASSERT_EQ(path_message.poses.size(), 3U);
  EXPECT_EQ(path_message.header.frame_id, "camera_start");
  EXPECT_DOUBLE_EQ(path_message.poses[0].pose.position.x, 2.0);
  EXPECT_DOUBLE_EQ(path_message.poses[1].pose.position.x, 3.0);
  EXPECT_DOUBLE_EQ(path_message.poses[2].pose.position.x, 4.0);
  EXPECT_EQ(path_message.header.stamp.sec, 4);
}

/** @brief 验证长度 0 表示无限累计且显式重置后只保留当前位姿。 */
TEST(MonocularPose, SupportsUnlimitedAndResetPaths)
{
  /** @brief 接收无限累计位姿的轨迹消息。 */
  nav_msgs::msg::Path path_message;
  for (std::int32_t index = 1; index <= 5; ++index)
  {
    UpdateCameraPath(
        CreateTestPose(static_cast<double>(index), index),
        0U,
        false,
        &path_message);
  }
  ASSERT_EQ(path_message.poses.size(), 5U);

  UpdateCameraPath(CreateTestPose(9.0, 9), 0U, true, &path_message);
  ASSERT_EQ(path_message.poses.size(), 1U);
  EXPECT_DOUBLE_EQ(path_message.poses.front().pose.position.x, 9.0);
  EXPECT_THROW(
      UpdateCameraPath(CreateTestPose(10.0, 10), 0U, false, nullptr),
      std::invalid_argument);
}

/** @brief 验证 LOST 重置请求只会在恢复后的第一次有效发布时消费。 */
TEST(MonocularPose, ConsumesLostResetRequestOnce)
{
  /** @brief 被测 LOST 后轨迹重置状态。 */
  CameraPathResetState reset_state;

  EXPECT_FALSE(reset_state.ConsumeResetRequest());
  reset_state.MarkTrackingLost();
  EXPECT_TRUE(reset_state.ConsumeResetRequest());
  EXPECT_FALSE(reset_state.ConsumeResetRequest());
}

/** @brief 验证轨迹长度参数拒绝负数并接受 0 与默认值。 */
TEST(MonocularPose, ValidatesMaximumPathLength)
{
  EXPECT_EQ(ValidateMaxPathLength(0), 0U);
  EXPECT_EQ(ValidateMaxPathLength(10000), 10000U);
  EXPECT_THROW(ValidateMaxPathLength(-1), std::invalid_argument);
}
