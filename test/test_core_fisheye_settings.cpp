/**
 * @file test_core_fisheye_settings.cpp
 * @brief 验证上游核心读取左右独立鱼眼参数和离线兼容开关。
 */

#include <cmath>
#include <fstream>
#include <string>

#include <gtest/gtest.h>

#include "CameraModels/KannalaBrandt8.h"
#include "Settings.h"
#include "System.h"

namespace
{

/**
 * @brief 写出仅包含 Settings 必需字段的测试配置。
 * @param path 输出 YAML 路径。
 * @param camera1_k1 左相机 k1。
 * @param camera2_k1 右相机 k1。
 * @param defer_low_motion IMU.DeferLowMotionReset 数值。
 * @param defer_pre_ba2 IMU.DeferPreBA2TrackingReset 数值。
 */
void WriteSettingsFile(
    const std::string& path,
    float camera1_k1,
    float camera2_k1,
    int defer_low_motion,
    int defer_pre_ba2)
{
  std::ofstream output(path);
  ASSERT_TRUE(output.is_open());

  output << "%YAML:1.0\n"
         << "File.version: \"1.0\"\n"
         << "Camera.type: \"KannalaBrandt8\"\n"
         << "Camera1.fx: 300.0\n"
         << "Camera1.fy: 301.0\n"
         << "Camera1.cx: 160.0\n"
         << "Camera1.cy: 120.0\n"
         << "Camera1.k1: " << camera1_k1 << "\n"
         << "Camera1.k2: 0.02\n"
         << "Camera1.k3: -0.03\n"
         << "Camera1.k4: 0.04\n"
         << "Camera1.overlappingBegin: 0\n"
         << "Camera1.overlappingEnd: 319\n"
         << "Camera2.fx: 310.0\n"
         << "Camera2.fy: 311.0\n"
         << "Camera2.cx: 165.0\n"
         << "Camera2.cy: 118.0\n"
         << "Camera2.k1: " << camera2_k1 << "\n"
         << "Camera2.k2: 0.12\n"
         << "Camera2.k3: -0.13\n"
         << "Camera2.k4: 0.14\n"
         << "Camera2.overlappingBegin: 0\n"
         << "Camera2.overlappingEnd: 319\n"
         << "Camera.width: 320\n"
         << "Camera.height: 240\n"
         << "Camera.fps: 10\n"
         << "Camera.RGB: 0\n"
         << "Stereo.T_c1_c2: !!opencv-matrix\n"
         << "   rows: 4\n"
         << "   cols: 4\n"
         << "   dt: f\n"
         << "   data: [1.0, 0.0, 0.0, 0.100,\n"
         << "          0.0, 1.0, 0.0, 0.001,\n"
         << "          0.0, 0.0, 1.0, 0.002,\n"
         << "          0.0, 0.0, 0.0, 1.0]\n"
         << "Stereo.ThDepth: 40.0\n"
         << "IMU.NoiseGyro: 0.003\n"
         << "IMU.NoiseAcc: 0.001\n"
         << "IMU.GyroWalk: 0.004\n"
         << "IMU.AccWalk: 0.002\n"
         << "IMU.Frequency: 50.0\n"
         << "IMU.T_b_c1: !!opencv-matrix\n"
         << "   rows: 4\n"
         << "   cols: 4\n"
         << "   dt: f\n"
         << "   data: [1.0, 0.0, 0.0, 0.05,\n"
         << "          0.0, 1.0, 0.0, 0.02,\n"
         << "          0.0, 0.0, 1.0, -0.03,\n"
         << "          0.0, 0.0, 0.0, 1.0]\n"
         << "IMU.InsertKFsWhenLost: 0\n"
         << "IMU.DeferLowMotionReset: " << defer_low_motion << "\n"
         << "IMU.DeferPreBA2TrackingReset: " << defer_pre_ba2 << "\n"
         << "ORBextractor.nFeatures: 500\n"
         << "ORBextractor.scaleFactor: 1.2\n"
         << "ORBextractor.nLevels: 8\n"
         << "ORBextractor.iniThFAST: 20\n"
         << "ORBextractor.minThFAST: 7\n"
         << "Viewer.KeyFrameSize: 0.05\n"
         << "Viewer.KeyFrameLineWidth: 1.0\n"
         << "Viewer.GraphLineWidth: 0.9\n"
         << "Viewer.PointSize: 2.0\n"
         << "Viewer.CameraSize: 0.08\n"
         << "Viewer.CameraLineWidth: 3.0\n"
         << "Viewer.ViewpointX: 0.0\n"
         << "Viewer.ViewpointY: -0.7\n"
         << "Viewer.ViewpointZ: -3.5\n"
         << "Viewer.ViewpointF: 500.0\n";
  output.close();
}

/**
 * @brief readCamera2 必须读取 Camera2.k*，不能复用 Camera1.k*。
 */
TEST(CoreFisheyeSettingsTest, ReadsIndependentCamera2Distortion)
{
  const std::string path = testing::TempDir() + "/orb_core_fisheye_settings.yaml";
  WriteSettingsFile(path, 0.11F, 0.77F, 1, 1);

  ORB_SLAM3::Settings settings(path, ORB_SLAM3::System::IMU_STEREO);
  auto* camera1 =
      dynamic_cast<ORB_SLAM3::KannalaBrandt8*>(settings.camera1());
  auto* camera2 =
      dynamic_cast<ORB_SLAM3::KannalaBrandt8*>(settings.camera2());

  ASSERT_NE(camera1, nullptr);
  ASSERT_NE(camera2, nullptr);
  EXPECT_NEAR(camera1->getParameter(4), 0.11F, 1e-6F);
  EXPECT_NEAR(camera2->getParameter(4), 0.77F, 1e-6F);
  EXPECT_NEAR(camera1->getParameter(5), 0.02F, 1e-6F);
  EXPECT_NEAR(camera2->getParameter(5), 0.12F, 1e-6F);
  EXPECT_NEAR(camera1->getParameter(6), -0.03F, 1e-6F);
  EXPECT_NEAR(camera2->getParameter(6), -0.13F, 1e-6F);
  EXPECT_NEAR(camera1->getParameter(7), 0.04F, 1e-6F);
  EXPECT_NEAR(camera2->getParameter(7), 0.14F, 1e-6F);
  EXPECT_FALSE(settings.needToRectify());
}

/**
 * @brief 离线兼容开关默认关闭，但配置为 1 时由 Settings 返回开启。
 */
TEST(CoreFisheyeSettingsTest, ReadsDeferredResetSwitches)
{
  const std::string enabled_path = testing::TempDir() + "/orb_core_defer_enabled.yaml";
  const std::string disabled_path = testing::TempDir() + "/orb_core_defer_disabled.yaml";
  WriteSettingsFile(enabled_path, 0.11F, 0.77F, 1, 1);
  WriteSettingsFile(disabled_path, 0.11F, 0.77F, 0, 0);

  ORB_SLAM3::Settings enabled(enabled_path, ORB_SLAM3::System::IMU_STEREO);
  ORB_SLAM3::Settings disabled(disabled_path, ORB_SLAM3::System::IMU_STEREO);

  EXPECT_TRUE(enabled.deferLowMotionReset());
  EXPECT_TRUE(enabled.deferPreBA2TrackingReset());
  EXPECT_FALSE(disabled.deferLowMotionReset());
  EXPECT_FALSE(disabled.deferPreBA2TrackingReset());
}

}  // namespace
