/**
 * @file test_monocular_inertial_pose.cpp
 * @brief 测试单目惯性节点中相机位姿到机体系位姿的转换逻辑。
 */

#include "monocular-inertial-pose.hpp"

#include <cmath>

#include "gtest/gtest.h"

namespace
{
/**
 * @brief 浮点比较容差。
 */
constexpr float kTolerance = 1e-5F;
}  // namespace

TEST(MonocularInertialPose, ConvertsCameraPoseToBodyPoseWithTranslationExtrinsic)
{
  /** @brief 世界到相机位姿，使用单位变换便于验证外参方向。 */
  const Sophus::SE3f Tcw;
  /** @brief 机体系到相机系外参，表示相机位于机体系 x 轴正方向 1 米处。 */
  const Sophus::SE3f Tbc(Eigen::Matrix3f::Identity(), Eigen::Vector3f(1.0F, 0.0F, 0.0F));

  /** @brief 转换得到的世界到机体系位姿。 */
  const Sophus::SE3f Twb = ComputeBodyPoseFromCameraPose(Tcw, Tbc);

  EXPECT_NEAR(Twb.translation().x(), -1.0F, kTolerance);
  EXPECT_NEAR(Twb.translation().y(), 0.0F, kTolerance);
  EXPECT_NEAR(Twb.translation().z(), 0.0F, kTolerance);
}

TEST(MonocularInertialPose, ConvertsRotationAndKeepsQuaternionNormalized)
{
  /** @brief 绕 z 轴旋转 90 度的世界到相机姿态。 */
  const Eigen::AngleAxisf camera_rotation(static_cast<float>(M_PI_2), Eigen::Vector3f::UnitZ());
  /** @brief 世界到相机位姿，用于验证旋转会正确取逆到世界位姿。 */
  const Sophus::SE3f Tcw(camera_rotation.toRotationMatrix(), Eigen::Vector3f::Zero());
  /** @brief 单位机体系到相机系外参。 */
  const Sophus::SE3f Tbc;

  /** @brief 转换得到的世界到机体系位姿。 */
  const Sophus::SE3f Twb = ComputeBodyPoseFromCameraPose(Tcw, Tbc);
  /** @brief 转换位姿对应的四元数。 */
  const Eigen::Quaternionf quaternion = Twb.unit_quaternion();

  EXPECT_NEAR(quaternion.norm(), 1.0F, kTolerance);
  EXPECT_TRUE(Twb.rotationMatrix().isApprox(camera_rotation.toRotationMatrix().transpose(), kTolerance));
}
