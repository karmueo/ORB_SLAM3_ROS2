/**
 * @file test_monocular_inertial_pose.cpp
 * @brief 测试单目惯性节点中相机位姿到机体系位姿的转换逻辑。
 */

#include "monocular-inertial-pose.hpp"

#include <cmath>
#include <stdexcept>

#include "gtest/gtest.h"

namespace
{
/**
 * @brief 浮点比较容差。
 */
constexpr float kTolerance = 1e-5F;

/**
 * @brief 创建带指定 x 坐标和秒级时间戳的测试位姿。
 * @param xPosition 世界系下的 x 坐标。
 * @param stampSeconds 消息时间戳的秒字段。
 * @return 用于轨迹测试的位姿消息。
 */
geometry_msgs::msg::PoseStamped CreateTestBodyPose(
    const double xPosition,
    const std::int32_t stampSeconds)
{
  /** @brief 待返回的测试机体位姿。 */
  geometry_msgs::msg::PoseStamped poseMessage;
  poseMessage.header.frame_id = "map";
  poseMessage.header.stamp.sec = stampSeconds;
  poseMessage.pose.position.x = xPosition;
  poseMessage.pose.orientation.w = 1.0;
  return poseMessage;
}
}  // namespace

TEST(MonocularInertialPose, ConvertsCameraPoseToBodyPoseWithTranslationExtrinsic)
{
  /** @brief 世界到相机位姿，使用单位变换便于验证外参方向。 */
  const Sophus::SE3f Tcw;
  /** @brief 相机系到机体系外参，表示相机位于机体系 x 轴正方向 1 米处。 */
  const Sophus::SE3f Tbc(Eigen::Matrix3f::Identity(), Eigen::Vector3f(1.0F, 0.0F, 0.0F));

  /** @brief 转换得到的机体系到世界系位姿。 */
  const Sophus::SE3f Twb = ComputeBodyPoseFromCameraPose(Tcw, Tbc);

  EXPECT_NEAR(Twb.translation().x(), -1.0F, kTolerance);
  EXPECT_NEAR(Twb.translation().y(), 0.0F, kTolerance);
  EXPECT_NEAR(Twb.translation().z(), 0.0F, kTolerance);
}

TEST(MonocularInertialPose, PublishesOnlyAfterBa1RemainsStable)
{
  /** @brief BA1 首次被观察为完成的时间戳。 */
  double ba1InitializedSinceSec = -1.0;

  EXPECT_FALSE(UpdateInertialBa1PublishGate(false, 9.0, 1.0, &ba1InitializedSinceSec));
  EXPECT_DOUBLE_EQ(ba1InitializedSinceSec, -1.0);
  EXPECT_FALSE(UpdateInertialBa1PublishGate(true, 10.0, 1.0, &ba1InitializedSinceSec));
  EXPECT_DOUBLE_EQ(ba1InitializedSinceSec, 10.0);
  EXPECT_FALSE(UpdateInertialBa1PublishGate(true, 10.99, 1.0, &ba1InitializedSinceSec));
  EXPECT_TRUE(UpdateInertialBa1PublishGate(true, 11.0, 1.0, &ba1InitializedSinceSec));

  EXPECT_FALSE(UpdateInertialBa1PublishGate(false, 11.1, 1.0, &ba1InitializedSinceSec));
  EXPECT_DOUBLE_EQ(ba1InitializedSinceSec, -1.0);
  EXPECT_FALSE(UpdateInertialBa1PublishGate(true, 20.0, 1.0, &ba1InitializedSinceSec));
}

TEST(MonocularInertialPose, RestartsBa1SettlingAfterTimestampRollback)
{
  /** @brief BA1 首次被观察为完成的时间戳。 */
  double ba1InitializedSinceSec = 10.0;

  EXPECT_FALSE(UpdateInertialBa1PublishGate(true, 9.0, 1.0, &ba1InitializedSinceSec));
  EXPECT_DOUBLE_EQ(ba1InitializedSinceSec, 9.0);
}

TEST(MonocularInertialPose, ConvertsRotationAndKeepsQuaternionNormalized)
{
  /** @brief 绕 z 轴旋转 90 度的世界到相机姿态。 */
  const Eigen::AngleAxisf camera_rotation(static_cast<float>(M_PI_2), Eigen::Vector3f::UnitZ());
  /** @brief 世界到相机位姿，用于验证旋转会正确取逆到世界位姿。 */
  const Sophus::SE3f Tcw(camera_rotation.toRotationMatrix(), Eigen::Vector3f::Zero());
  /** @brief 单位相机系到机体系外参。 */
  const Sophus::SE3f Tbc;

  /** @brief 转换得到的机体系到世界系位姿。 */
  const Sophus::SE3f Twb = ComputeBodyPoseFromCameraPose(Tcw, Tbc);
  /** @brief 转换位姿对应的四元数。 */
  const Eigen::Quaternionf quaternion = Twb.unit_quaternion();

  EXPECT_NEAR(quaternion.norm(), 1.0F, kTolerance);
  EXPECT_TRUE(Twb.rotationMatrix().isApprox(camera_rotation.toRotationMatrix().transpose(), kTolerance));
}


TEST(MonocularInertialPose, CreatesConsistentPoseAndDynamicTransform)
{
  /** @brief 用于消息转换测试的世界系下机体位姿。 */
  const Sophus::SE3f Twb(
      Eigen::AngleAxisf(0.4F, Eigen::Vector3f::UnitY()).toRotationMatrix(),
      Eigen::Vector3f(1.0F, 2.0F, 3.0F));
  /** @brief 输入图像时间戳。 */
  builtin_interfaces::msg::Time stamp;
  stamp.sec = 42;
  stamp.nanosec = 123456789U;
  /** @brief 转换得到的 ROS 机体位姿消息。 */
  const geometry_msgs::msg::PoseStamped poseMessage =
      CreateBodyPoseMessage(Twb, stamp, "map");
  /** @brief 根据机体位姿生成的动态 TF 消息。 */
  const geometry_msgs::msg::TransformStamped transformMessage =
      CreateBodyTransformMessage(poseMessage, "body_link");
  /** @brief 位姿消息中四元数的模长平方。 */
  const double quaternionNormSquared =
      poseMessage.pose.orientation.x * poseMessage.pose.orientation.x +
      poseMessage.pose.orientation.y * poseMessage.pose.orientation.y +
      poseMessage.pose.orientation.z * poseMessage.pose.orientation.z +
      poseMessage.pose.orientation.w * poseMessage.pose.orientation.w;

  EXPECT_EQ(poseMessage.header.frame_id, "map");
  EXPECT_EQ(poseMessage.header.stamp, stamp);
  EXPECT_NEAR(quaternionNormSquared, 1.0, kTolerance);
  EXPECT_EQ(transformMessage.header, poseMessage.header);
  EXPECT_EQ(transformMessage.child_frame_id, "body_link");
  EXPECT_DOUBLE_EQ(
      transformMessage.transform.translation.x,
      poseMessage.pose.position.x);
  EXPECT_EQ(
      transformMessage.transform.rotation,
      poseMessage.pose.orientation);
}

TEST(MonocularInertialPose, CreatesBodyToCameraTransformFromTbc)
{
  /** @brief 相机光学系在机体系中的标定外参。 */
  const Sophus::SE3f Tbc(
      Eigen::Matrix3f::Identity(),
      Eigen::Vector3f(0.1F, -0.2F, 0.3F));
  /** @brief 静态 TF 使用的时间戳。 */
  builtin_interfaces::msg::Time stamp;
  stamp.sec = 7;
  /** @brief 相机光学系在 body_link 中的位姿消息。 */
  const geometry_msgs::msg::PoseStamped cameraPose =
      CreateBodyPoseMessage(Tbc, stamp, "body_link");
  /** @brief body_link 到图像 frame 的静态 TF 消息。 */
  const geometry_msgs::msg::TransformStamped cameraTransform =
      CreateBodyTransformMessage(cameraPose, "rgb_optical_frame");

  EXPECT_EQ(cameraTransform.header.frame_id, "body_link");
  EXPECT_EQ(cameraTransform.child_frame_id, "rgb_optical_frame");
  EXPECT_NEAR(cameraTransform.transform.translation.x, 0.1, kTolerance);
  EXPECT_NEAR(cameraTransform.transform.translation.y, -0.2, kTolerance);
  EXPECT_NEAR(cameraTransform.transform.translation.z, 0.3, kTolerance);
}

TEST(MonocularInertialPose, BoundsPathAndDropsOldestPoses)
{
  /** @brief 接收有限长度位姿的轨迹消息。 */
  nav_msgs::msg::Path pathMessage;
  for (std::int32_t index = 1; index <= 4; ++index)
  {
    UpdateBodyPath(
        CreateTestBodyPose(static_cast<double>(index), index),
        3U,
        false,
        &pathMessage);
  }

  ASSERT_EQ(pathMessage.poses.size(), 3U);
  EXPECT_DOUBLE_EQ(pathMessage.poses[0].pose.position.x, 2.0);
  EXPECT_DOUBLE_EQ(pathMessage.poses[2].pose.position.x, 4.0);
  EXPECT_EQ(pathMessage.header.stamp.sec, 4);
}

TEST(MonocularInertialPose, SupportsUnlimitedPathAndLostRecoveryReset)
{
  /** @brief 接收无限累计位姿的轨迹消息。 */
  nav_msgs::msg::Path pathMessage;
  for (std::int32_t index = 1; index <= 5; ++index)
  {
    UpdateBodyPath(
        CreateTestBodyPose(static_cast<double>(index), index),
        0U,
        false,
        &pathMessage);
  }
  ASSERT_EQ(pathMessage.poses.size(), 5U);

  /** @brief 模拟跟踪 LOST 后的轨迹重置状态。 */
  BodyPathResetState resetState;
  resetState.MarkTrackingLost();
  EXPECT_TRUE(resetState.ConsumeResetRequest());
  EXPECT_FALSE(resetState.ConsumeResetRequest());

  UpdateBodyPath(CreateTestBodyPose(9.0, 9), 0U, true, &pathMessage);
  ASSERT_EQ(pathMessage.poses.size(), 1U);
  EXPECT_DOUBLE_EQ(pathMessage.poses.front().pose.position.x, 9.0);
}

TEST(MonocularInertialPose, RejectsInvalidPathArguments)
{
  /** @brief 用于空指针检查的测试位姿。 */
  const geometry_msgs::msg::PoseStamped poseMessage =
      CreateTestBodyPose(1.0, 1);

  EXPECT_THROW(
      UpdateBodyPath(poseMessage, 10U, false, nullptr),
      std::invalid_argument);
  EXPECT_EQ(ValidateBodyMaxPathLength(0), 0U);
  EXPECT_EQ(ValidateBodyMaxPathLength(10000), 10000U);
  EXPECT_THROW(ValidateBodyMaxPathLength(-1), std::invalid_argument);
}
