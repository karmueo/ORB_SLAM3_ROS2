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

TEST(MonocularInertialSync, AppliesImuTimeOffsetBeforeExtraction)
{
  /** @brief 待测试的 IMU 队列。 */
  std::queue<MonocularSyncImuMsg::SharedPtr> imuQueue;
  /** @brief 最近一次接收的 IMU 时间戳。 */
  double lastImuTimestamp = -1.0;
  /** @brief 被过滤的 IMU 样本数量。 */
  std::uint64_t droppedImuCount = 0;

  EXPECT_TRUE(MonocularInertialSync::PushMonotonicImu(
      imuQueue,
      MakeImu(1.00),
      0.02,
      &lastImuTimestamp,
      &droppedImuCount));
  EXPECT_TRUE(MonocularInertialSync::PushMonotonicImu(
      imuQueue,
      MakeImu(1.01),
      0.02,
      &lastImuTimestamp,
      &droppedImuCount));

  /** @brief 图像时间戳之前的 IMU 测量。 */
  const std::vector<ORB_SLAM3::IMU::Point> measurements =
      MonocularInertialSync::ExtractImuMeasurementsUntil(imuQueue, 1.03);

  ASSERT_EQ(measurements.size(), 2U);
  EXPECT_DOUBLE_EQ(measurements[0].t, 1.02);
  EXPECT_DOUBLE_EQ(measurements[1].t, 1.03);
  EXPECT_DOUBLE_EQ(lastImuTimestamp, 1.03);
  EXPECT_EQ(droppedImuCount, 0U);
}

TEST(MonocularInertialSync, ReordersImuSamplesInsideConfiguredWindow)
{
  /** @brief 待测试的 IMU 队列。 */
  std::queue<MonocularSyncImuMsg::SharedPtr> imuQueue;
  /** @brief IMU 小窗口重排状态。 */
  MonocularImuReorderState reorderState;
  /** @brief 最近一次输出到主 IMU 队列的时间戳。 */
  double lastQueuedImuTimestamp = -1.0;
  /** @brief 被过滤的 IMU 样本数量。 */
  std::uint64_t droppedImuCount = 0;

  MonocularInertialSync::PushReorderedImu(
      imuQueue,
      MakeImu(1.000),
      0.0,
      0.010,
      &reorderState,
      &lastQueuedImuTimestamp,
      &droppedImuCount);
  MonocularInertialSync::PushReorderedImu(
      imuQueue,
      MakeImu(1.006),
      0.0,
      0.010,
      &reorderState,
      &lastQueuedImuTimestamp,
      &droppedImuCount);
  MonocularInertialSync::PushReorderedImu(
      imuQueue,
      MakeImu(1.004),
      0.0,
      0.010,
      &reorderState,
      &lastQueuedImuTimestamp,
      &droppedImuCount);
  /** @brief 远端样本会推动窗口内 1.000、1.004、1.006 三个样本按序输出。 */
  MonocularInertialSync::PushReorderedImu(
      imuQueue,
      MakeImu(1.020),
      0.0,
      0.010,
      &reorderState,
      &lastQueuedImuTimestamp,
      &droppedImuCount);

  ASSERT_EQ(imuQueue.size(), 3U);
  EXPECT_DOUBLE_EQ(Utility::StampToSec(imuQueue.front()->header.stamp), 1.000);
  imuQueue.pop();
  EXPECT_DOUBLE_EQ(Utility::StampToSec(imuQueue.front()->header.stamp), 1.004);
  imuQueue.pop();
  EXPECT_DOUBLE_EQ(Utility::StampToSec(imuQueue.front()->header.stamp), 1.006);
  EXPECT_EQ(droppedImuCount, 0U);
}

TEST(MonocularInertialSync, FlushesPendingReorderedImuUntilImageIsCoveredWhenInputPauses)
{
  /** @brief 待测试的 IMU 队列。 */
  std::queue<MonocularSyncImuMsg::SharedPtr> imuQueue;
  /** @brief IMU 小窗口重排状态。 */
  MonocularImuReorderState reorderState;
  /** @brief 最近一次输出到主 IMU 队列的时间戳。 */
  double lastQueuedImuTimestamp = -1.0;
  /** @brief 被过滤的 IMU 样本数量。 */
  std::uint64_t droppedImuCount = 0;

  MonocularInertialSync::PushReorderedImu(
      imuQueue,
      MakeImu(1.000),
      0.0,
      0.010,
      &reorderState,
      &lastQueuedImuTimestamp,
      &droppedImuCount);
  MonocularInertialSync::PushReorderedImu(
      imuQueue,
      MakeImu(1.004),
      0.0,
      0.010,
      &reorderState,
      &lastQueuedImuTimestamp,
      &droppedImuCount);
  MonocularInertialSync::PushReorderedImu(
      imuQueue,
      MakeImu(1.006),
      0.0,
      0.010,
      &reorderState,
      &lastQueuedImuTimestamp,
      &droppedImuCount);

  ASSERT_TRUE(imuQueue.empty());
  ASSERT_EQ(reorderState.pendingImuMsgs.size(), 3U);
  EXPECT_FALSE(MonocularInertialSync::HasImuCoverageForImage(imuQueue, 1.004));

  /** @brief 输入暂停后，为覆盖当前图像而从 pending 缓存刷出的 IMU 样本统计。 */
  const MonocularImuQueuePushResult flushResult =
      MonocularInertialSync::FlushPendingReorderedImuUntilCovered(
          imuQueue,
          &reorderState,
          1.004,
          &lastQueuedImuTimestamp,
          &droppedImuCount);

  EXPECT_EQ(flushResult.acceptedCount, 2U);
  EXPECT_EQ(flushResult.droppedCount, 0U);
  EXPECT_EQ(flushResult.pendingCount, 1U);
  EXPECT_TRUE(MonocularInertialSync::HasImuCoverageForImage(imuQueue, 1.004));
  ASSERT_EQ(imuQueue.size(), 2U);
  EXPECT_DOUBLE_EQ(Utility::StampToSec(imuQueue.front()->header.stamp), 1.000);
  imuQueue.pop();
  EXPECT_DOUBLE_EQ(Utility::StampToSec(imuQueue.front()->header.stamp), 1.004);
  EXPECT_DOUBLE_EQ(lastQueuedImuTimestamp, 1.004);
  EXPECT_EQ(droppedImuCount, 0U);
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

TEST(MonocularInertialSync, AcceptsAlignedImageAndImuTimeBases)
{
  /** @brief 实测校正图像时间戳，单位为秒。 */
  const double imageTimestamp = 288763.403175746;
  /** @brief 同一时刻附近的实测 IMU 时间戳，单位为秒。 */
  const double latestImuTimestamp = 288763.410210499;

  EXPECT_TRUE(MonocularInertialSync::AreTimeBasesAligned(
      imageTimestamp,
      latestImuTimestamp,
      1.0));
}

TEST(MonocularInertialSync, RejectsDifferentImageAndImuTimeBases)
{
  /** @brief 旧实时校正图像所在的设备时间基准，单位为秒。 */
  const double imageTimestamp = 2668.036934;
  /** @brief 旧 IMU 使用的主机 steady clock 时间基准，单位为秒。 */
  const double latestImuTimestamp = 286132.143124;

  EXPECT_FALSE(MonocularInertialSync::AreTimeBasesAligned(
      imageTimestamp,
      latestImuTimestamp,
      1.0));
}

TEST(MonocularInertialSync, RejectsImageWhenTargetFpsIntervalHasNotElapsed)
{
  /** @brief 待测试的图像队列。 */
  std::queue<MonocularSyncImageMsg::SharedPtr> imageQueue;
  /** @brief 图像平均帧率限制状态。 */
  MonocularImageRateLimitState rateLimitState{-1.0, 0.0, -1.0};
  /** @brief 因限帧被丢弃的图像数量。 */
  std::uint64_t rateLimitedImageCount = 0;
  /** @brief 因队列溢出被丢弃的图像数量。 */
  std::uint64_t overflowDroppedImageCount = 0;

  MonocularInertialSync::PushRateLimitedBoundedImage(
      imageQueue,
      MakeImage(1.000),
      2U,
      30.0,
      &rateLimitState,
      &rateLimitedImageCount,
      &overflowDroppedImageCount);
  /** @brief 30Hz 平均限帧下，首帧后 20ms 的图像应被主动限帧丢弃。 */
  const MonocularImageQueuePushResult result = MonocularInertialSync::PushRateLimitedBoundedImage(
      imageQueue,
      MakeImage(1.020),
      2U,
      30.0,
      &rateLimitState,
      &rateLimitedImageCount,
      &overflowDroppedImageCount);

  EXPECT_FALSE(result.accepted);
  EXPECT_TRUE(result.droppedByRateLimit);
  EXPECT_FALSE(result.droppedOldestImage);
  ASSERT_EQ(imageQueue.size(), 1U);
  EXPECT_DOUBLE_EQ(Utility::StampToSec(imageQueue.front()->header.stamp), 1.000);
  EXPECT_DOUBLE_EQ(rateLimitState.lastAcceptedImageTimestamp, 1.000);
  EXPECT_EQ(rateLimitedImageCount, 1U);
  EXPECT_EQ(overflowDroppedImageCount, 0U);
}

TEST(MonocularInertialSync, AcceptsImageWhenTargetFpsIntervalHasElapsed)
{
  /** @brief 待测试的图像队列。 */
  std::queue<MonocularSyncImageMsg::SharedPtr> imageQueue;
  /** @brief 图像平均帧率限制状态。 */
  MonocularImageRateLimitState rateLimitState{-1.0, 0.0, -1.0};
  /** @brief 因限帧被丢弃的图像数量。 */
  std::uint64_t rateLimitedImageCount = 0;
  /** @brief 因队列溢出被丢弃的图像数量。 */
  std::uint64_t overflowDroppedImageCount = 0;

  MonocularInertialSync::PushRateLimitedBoundedImage(
      imageQueue,
      MakeImage(1.000),
      2U,
      30.0,
      &rateLimitState,
      &rateLimitedImageCount,
      &overflowDroppedImageCount);
  /** @brief 1.034 秒距离上一帧约 34ms，达到 30Hz 平均限帧额度后应正常入队。 */
  const MonocularImageQueuePushResult result = MonocularInertialSync::PushRateLimitedBoundedImage(
      imageQueue,
      MakeImage(1.034),
      2U,
      30.0,
      &rateLimitState,
      &rateLimitedImageCount,
      &overflowDroppedImageCount);

  EXPECT_TRUE(result.accepted);
  EXPECT_FALSE(result.droppedByRateLimit);
  EXPECT_FALSE(result.droppedOldestImage);
  ASSERT_EQ(imageQueue.size(), 2U);
  EXPECT_DOUBLE_EQ(rateLimitState.lastAcceptedImageTimestamp, 1.034);
  EXPECT_EQ(rateLimitedImageCount, 0U);
  EXPECT_EQ(overflowDroppedImageCount, 0U);
}

TEST(MonocularInertialSync, KeepsAverageTargetFpsForSlightlyOverSixtyHertzInput)
{
  /** @brief 待测试的图像队列。 */
  std::queue<MonocularSyncImageMsg::SharedPtr> imageQueue;
  /** @brief 图像平均帧率限制状态。 */
  MonocularImageRateLimitState rateLimitState{-1.0, 0.0, -1.0};
  /** @brief 因限帧被丢弃的图像数量。 */
  std::uint64_t rateLimitedImageCount = 0;
  /** @brief 因队列溢出被丢弃的图像数量。 */
  std::uint64_t overflowDroppedImageCount = 0;
  /** @brief 成功入队的图像数量。 */
  std::uint64_t acceptedImageCount = 0;
  /** @brief 模拟输入图像帧率，略高于 60Hz。 */
  const double inputFps = 60.6;
  /** @brief 模拟输入持续时间，单位为秒。 */
  const double durationSeconds = 2.0;
  /** @brief 模拟输入图像数量。 */
  const int imageCount = static_cast<int>(inputFps * durationSeconds);

  for (int imageIndex = 0; imageIndex < imageCount; ++imageIndex)
  {
    /** @brief 当前模拟图像时间戳，单位为秒。 */
    const double timestamp = 1.0 + static_cast<double>(imageIndex) / inputFps;
    /** @brief 当前图像入队结果。 */
    const MonocularImageQueuePushResult result = MonocularInertialSync::PushRateLimitedBoundedImage(
        imageQueue,
        MakeImage(timestamp),
        200U,
        30.0,
        &rateLimitState,
        &rateLimitedImageCount,
        &overflowDroppedImageCount);
    if (result.accepted)
    {
      ++acceptedImageCount;
    }
  }

  EXPECT_GE(acceptedImageCount, 58U);
  EXPECT_LE(acceptedImageCount, 62U);
  EXPECT_NEAR(static_cast<double>(acceptedImageCount), 60.0, 2.0);
  EXPECT_EQ(acceptedImageCount + rateLimitedImageCount, static_cast<std::uint64_t>(imageCount));
  EXPECT_EQ(overflowDroppedImageCount, 0U);
}

TEST(MonocularInertialSync, KeepsNewestImagesWhenRateLimitedQueueLimitIsExceeded)
{
  /** @brief 待测试的图像队列。 */
  std::queue<MonocularSyncImageMsg::SharedPtr> imageQueue;
  /** @brief 图像平均帧率限制状态。 */
  MonocularImageRateLimitState rateLimitState{-1.0, 0.0, -1.0};
  /** @brief 因限帧被丢弃的图像数量。 */
  std::uint64_t rateLimitedImageCount = 0;
  /** @brief 因队列溢出被丢弃的图像数量。 */
  std::uint64_t overflowDroppedImageCount = 0;

  MonocularInertialSync::PushRateLimitedBoundedImage(
      imageQueue, MakeImage(1.00), 2U, 0.0, &rateLimitState, &rateLimitedImageCount, &overflowDroppedImageCount);
  MonocularInertialSync::PushRateLimitedBoundedImage(
      imageQueue, MakeImage(1.01), 2U, 0.0, &rateLimitState, &rateLimitedImageCount, &overflowDroppedImageCount);
  const MonocularImageQueuePushResult result = MonocularInertialSync::PushRateLimitedBoundedImage(
      imageQueue, MakeImage(1.02), 2U, 0.0, &rateLimitState, &rateLimitedImageCount, &overflowDroppedImageCount);

  EXPECT_TRUE(result.accepted);
  EXPECT_FALSE(result.droppedByRateLimit);
  EXPECT_TRUE(result.droppedOldestImage);
  ASSERT_EQ(imageQueue.size(), 2U);
  EXPECT_EQ(overflowDroppedImageCount, 1U);
  EXPECT_DOUBLE_EQ(Utility::StampToSec(imageQueue.front()->header.stamp), 1.01);
  imageQueue.pop();
  EXPECT_DOUBLE_EQ(Utility::StampToSec(imageQueue.front()->header.stamp), 1.02);
}

TEST(MonocularInertialSync, KeepsExistingPushBehaviorWhenTargetFpsIsDisabled)
{
  /** @brief 待测试的图像队列。 */
  std::queue<MonocularSyncImageMsg::SharedPtr> imageQueue;
  /** @brief 图像平均帧率限制状态。 */
  MonocularImageRateLimitState rateLimitState{-1.0, 0.0, -1.0};
  /** @brief 因限帧被丢弃的图像数量。 */
  std::uint64_t rateLimitedImageCount = 0;
  /** @brief 因队列溢出被丢弃的图像数量。 */
  std::uint64_t overflowDroppedImageCount = 0;

  EXPECT_TRUE(MonocularInertialSync::PushRateLimitedBoundedImage(
                  imageQueue, MakeImage(1.000), 100U, 0.0, &rateLimitState, &rateLimitedImageCount, &overflowDroppedImageCount)
                  .accepted);
  EXPECT_TRUE(MonocularInertialSync::PushRateLimitedBoundedImage(
                  imageQueue, MakeImage(1.001), 100U, 0.0, &rateLimitState, &rateLimitedImageCount, &overflowDroppedImageCount)
                  .accepted);

  ASSERT_EQ(imageQueue.size(), 2U);
  EXPECT_EQ(rateLimitedImageCount, 0U);
  EXPECT_EQ(overflowDroppedImageCount, 0U);
  EXPECT_DOUBLE_EQ(rateLimitState.lastAcceptedImageTimestamp, 1.001);
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
