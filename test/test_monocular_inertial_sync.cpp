/**
 * @file test_monocular_inertial_sync.cpp
 * @brief 测试单目惯性节点图像/IMU 同步队列的边界行为。
 */

#include "monocular-inertial-sync.hpp"

#include "utility.hpp"

#include <queue>
#include <vector>

#include "gtest/gtest.h"

namespace
{
/**
 * @brief 创建指定时间戳的图像消息。
 * @param timestamp 图像时间戳，单位为秒。
 * @return 带指定时间戳的图像消息。
 */
MonocularSyncImageMsg::SharedPtr MakeImage(const double timestamp)
{
  /** @brief 待返回的图像消息。 */
  MonocularSyncImageMsg::SharedPtr msg = std::make_shared<MonocularSyncImageMsg>();
  /** @brief 时间戳整数秒部分。 */
  const int32_t sec = static_cast<int32_t>(timestamp);
  /** @brief 时间戳纳秒部分。 */
  const uint32_t nanosec = static_cast<uint32_t>((timestamp - static_cast<double>(sec)) * 1e9 + 0.5);
  msg->header.stamp.sec = sec;
  msg->header.stamp.nanosec = nanosec;
  return msg;
}

/**
 * @brief 创建指定时间戳的 IMU 消息。
 * @param timestamp IMU 时间戳，单位为秒。
 * @return 带指定时间戳的 IMU 消息。
 */
MonocularSyncImuMsg::SharedPtr MakeImu(const double timestamp)
{
  /** @brief 待返回的 IMU 消息。 */
  MonocularSyncImuMsg::SharedPtr msg = std::make_shared<MonocularSyncImuMsg>();
  /** @brief 时间戳整数秒部分。 */
  const int32_t sec = static_cast<int32_t>(timestamp);
  /** @brief 时间戳纳秒部分。 */
  const uint32_t nanosec = static_cast<uint32_t>((timestamp - static_cast<double>(sec)) * 1e9 + 0.5);
  msg->header.stamp.sec = sec;
  msg->header.stamp.nanosec = nanosec;
  msg->linear_acceleration.x = timestamp;
  msg->angular_velocity.z = timestamp;
  return msg;
}
}  // namespace

TEST(MonocularInertialSync, WaitsWhenLatestImuDoesNotCoverImage)
{
  /** @brief 待测试的 IMU 队列。 */
  std::queue<MonocularSyncImuMsg::SharedPtr> imuQueue;
  imuQueue.push(MakeImu(1.00));
  imuQueue.push(MakeImu(1.01));

  EXPECT_FALSE(MonocularInertialSync::HasImuCoverageForImage(imuQueue, 1.02));
}

TEST(MonocularInertialSync, ExtractsImuUpToImageTimestampInOrder)
{
  /** @brief 待测试的 IMU 队列。 */
  std::queue<MonocularSyncImuMsg::SharedPtr> imuQueue;
  imuQueue.push(MakeImu(1.00));
  imuQueue.push(MakeImu(1.01));
  imuQueue.push(MakeImu(1.02));
  imuQueue.push(MakeImu(1.03));

  /** @brief 图像时间戳之前的 IMU 测量。 */
  const std::vector<ORB_SLAM3::IMU::Point> measurements =
      MonocularInertialSync::ExtractImuMeasurementsUntil(imuQueue, 1.02);

  ASSERT_EQ(measurements.size(), 3U);
  EXPECT_DOUBLE_EQ(measurements[0].t, 1.00);
  EXPECT_DOUBLE_EQ(measurements[1].t, 1.01);
  EXPECT_DOUBLE_EQ(measurements[2].t, 1.02);
  ASSERT_FALSE(imuQueue.empty());
  EXPECT_DOUBLE_EQ(Utility::StampToSec(imuQueue.front()->header.stamp), 1.03);
}

TEST(MonocularInertialSync, FiltersNonMonotonicImuSamples)
{
  /** @brief 待测试的 IMU 队列。 */
  std::queue<MonocularSyncImuMsg::SharedPtr> imuQueue;
  /** @brief 最近一次接收的 IMU 时间戳。 */
  double lastImuTimestamp = -1.0;
  /** @brief 被过滤的 IMU 样本数量。 */
  std::uint64_t droppedImuCount = 0;

  EXPECT_TRUE(MonocularInertialSync::PushMonotonicImu(imuQueue, MakeImu(1.00), &lastImuTimestamp, &droppedImuCount));
  EXPECT_FALSE(MonocularInertialSync::PushMonotonicImu(imuQueue, MakeImu(0.99), &lastImuTimestamp, &droppedImuCount));
  EXPECT_FALSE(MonocularInertialSync::PushMonotonicImu(imuQueue, MakeImu(1.00), &lastImuTimestamp, &droppedImuCount));
  EXPECT_TRUE(MonocularInertialSync::PushMonotonicImu(imuQueue, MakeImu(1.01), &lastImuTimestamp, &droppedImuCount));

  EXPECT_EQ(droppedImuCount, 2U);
  EXPECT_DOUBLE_EQ(lastImuTimestamp, 1.01);

  /** @brief 图像时间戳之前的 IMU 测量。 */
  const std::vector<ORB_SLAM3::IMU::Point> measurements =
      MonocularInertialSync::ExtractImuMeasurementsUntil(imuQueue, 1.01);

  ASSERT_EQ(measurements.size(), 2U);
  EXPECT_LT(measurements[0].t, measurements[1].t);
}

TEST(MonocularInertialSync, DropsOldestImageWhenQueueLimitIsExceeded)
{
  /** @brief 待测试的图像队列。 */
  std::queue<MonocularSyncImageMsg::SharedPtr> imageQueue;
  /** @brief 被丢弃的图像数量。 */
  std::uint64_t droppedImageCount = 0;

  MonocularInertialSync::PushBoundedImage(imageQueue, MakeImage(1.00), 3U, &droppedImageCount);
  MonocularInertialSync::PushBoundedImage(imageQueue, MakeImage(1.01), 3U, &droppedImageCount);
  MonocularInertialSync::PushBoundedImage(imageQueue, MakeImage(1.02), 3U, &droppedImageCount);
  MonocularInertialSync::PushBoundedImage(imageQueue, MakeImage(1.03), 3U, &droppedImageCount);

  ASSERT_EQ(imageQueue.size(), 3U);
  EXPECT_EQ(droppedImageCount, 1U);
  EXPECT_DOUBLE_EQ(Utility::StampToSec(imageQueue.front()->header.stamp), 1.01);
  imageQueue.pop();
  EXPECT_DOUBLE_EQ(Utility::StampToSec(imageQueue.front()->header.stamp), 1.02);
  imageQueue.pop();
  EXPECT_DOUBLE_EQ(Utility::StampToSec(imageQueue.front()->header.stamp), 1.03);
}

TEST(MonocularInertialSync, RequiresAtLeastTwoImuSamplesForPreintegration)
{
  /** @brief 没有 IMU 样本时的测量序列。 */
  std::vector<ORB_SLAM3::IMU::Point> noMeasurements;
  /** @brief 只有一个 IMU 样本时的测量序列。 */
  std::vector<ORB_SLAM3::IMU::Point> oneMeasurement;
  oneMeasurement.push_back(ORB_SLAM3::IMU::Point(cv::Point3f(), cv::Point3f(), 1.0));
  /** @brief 至少两个 IMU 样本时的测量序列。 */
  std::vector<ORB_SLAM3::IMU::Point> twoMeasurements = oneMeasurement;
  twoMeasurements.push_back(ORB_SLAM3::IMU::Point(cv::Point3f(), cv::Point3f(), 1.01));

  EXPECT_FALSE(MonocularInertialSync::HasEnoughMeasurementsForPreintegration(noMeasurements));
  EXPECT_FALSE(MonocularInertialSync::HasEnoughMeasurementsForPreintegration(oneMeasurement));
  EXPECT_TRUE(MonocularInertialSync::HasEnoughMeasurementsForPreintegration(twoMeasurements));
}
