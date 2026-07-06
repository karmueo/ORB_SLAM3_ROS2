/**
 * @file monocular-inertial-sync.cpp
 * @brief 实现单目惯性图像/IMU 同步队列辅助函数。
 */

#include "monocular-inertial-sync.hpp"

#include "utility.hpp"

#include <opencv2/core/core.hpp>

/**
 * @brief 判断 IMU 队列最新样本是否覆盖指定图像时间戳。
 * @param imuQueue IMU 消息队列。
 * @param imageTimestamp 图像时间戳，单位为秒。
 * @return 最新 IMU 时间戳大于等于图像时间戳时返回 true。
 */
bool MonocularInertialSync::HasImuCoverageForImage(
    const std::queue<MonocularSyncImuMsg::SharedPtr>& imuQueue,
    const double imageTimestamp)
{
  if (imuQueue.empty())
  {
    return false;
  }

  /** @brief IMU 队列最新样本时间戳，单位为秒。 */
  const double newestImuTimestamp = Utility::StampToSec(imuQueue.back()->header.stamp);
  return newestImuTimestamp >= imageTimestamp;
}

/**
 * @brief 将图像压入有界 FIFO 队列，溢出时丢弃最旧图像。
 * @param imageQueue 图像消息队列。
 * @param imageMsg 待入队图像消息。
 * @param maxQueueSize 队列最大容量。
 * @param droppedImageCount 被丢弃图像累计数量，可为空。
 * @return 发生溢出丢弃时返回 true。
 */
bool MonocularInertialSync::PushBoundedImage(
    std::queue<MonocularSyncImageMsg::SharedPtr>& imageQueue,
    const MonocularSyncImageMsg::SharedPtr& imageMsg,
    const std::size_t maxQueueSize,
    std::uint64_t* const droppedImageCount)
{
  /** @brief 当前入队操作是否丢弃了旧图像。 */
  bool droppedOldestImage = false;
  if (maxQueueSize > 0U && imageQueue.size() >= maxQueueSize)
  {
    imageQueue.pop();
    droppedOldestImage = true;
    if (droppedImageCount)
    {
      ++(*droppedImageCount);
    }
  }

  imageQueue.push(imageMsg);
  return droppedOldestImage;
}

/**
 * @brief 将 IMU 样本按严格递增时间戳入队。
 * @param imuQueue IMU 消息队列。
 * @param imuMsg 待入队 IMU 消息。
 * @param lastImuTimestamp 最近一次接收的 IMU 时间戳。
 * @param droppedImuCount 被过滤 IMU 累计数量，可为空。
 * @return 样本成功入队时返回 true，回跳或重复时间戳返回 false。
 */
bool MonocularInertialSync::PushMonotonicImu(
    std::queue<MonocularSyncImuMsg::SharedPtr>& imuQueue,
    const MonocularSyncImuMsg::SharedPtr& imuMsg,
    double* const lastImuTimestamp,
    std::uint64_t* const droppedImuCount)
{
  /** @brief 当前 IMU 样本时间戳，单位为秒。 */
  const double imuTimestamp = Utility::StampToSec(imuMsg->header.stamp);
  if (lastImuTimestamp && *lastImuTimestamp >= 0.0 && imuTimestamp <= *lastImuTimestamp)
  {
    if (droppedImuCount)
    {
      ++(*droppedImuCount);
    }
    return false;
  }

  imuQueue.push(imuMsg);
  if (lastImuTimestamp)
  {
    *lastImuTimestamp = imuTimestamp;
  }
  return true;
}

/**
 * @brief 从 IMU 队列取出时间戳不晚于图像时间戳的测量。
 * @param imuQueue IMU 消息队列，已取出的样本会被弹出。
 * @param imageTimestamp 图像时间戳，单位为秒。
 * @return 按时间顺序排列的 ORB_SLAM3 IMU 测量。
 */
std::vector<ORB_SLAM3::IMU::Point> MonocularInertialSync::ExtractImuMeasurementsUntil(
    std::queue<MonocularSyncImuMsg::SharedPtr>& imuQueue,
    const double imageTimestamp)
{
  /** @brief 当前图像帧之前的 IMU 测量序列。 */
  std::vector<ORB_SLAM3::IMU::Point> measurements;
  while (!imuQueue.empty() && Utility::StampToSec(imuQueue.front()->header.stamp) <= imageTimestamp)
  {
    /** @brief 当前 IMU 消息。 */
    const MonocularSyncImuMsg::SharedPtr imuMsg = imuQueue.front();
    /** @brief 当前 IMU 测量时间戳，单位为秒。 */
    const double timestamp = Utility::StampToSec(imuMsg->header.stamp);
    /** @brief 当前 IMU 线加速度测量，单位沿用 ROS 消息。 */
    const cv::Point3f acc(
        imuMsg->linear_acceleration.x,
        imuMsg->linear_acceleration.y,
        imuMsg->linear_acceleration.z);
    /** @brief 当前 IMU 角速度测量，单位沿用 ROS 消息。 */
    const cv::Point3f gyr(
        imuMsg->angular_velocity.x,
        imuMsg->angular_velocity.y,
        imuMsg->angular_velocity.z);
    measurements.push_back(ORB_SLAM3::IMU::Point(acc, gyr, timestamp));
    imuQueue.pop();
  }

  return measurements;
}

/**
 * @brief 判断 IMU 测量数量是否足够 ORB_SLAM3 做相邻帧预积分。
 * @param measurements 当前图像帧对应的 IMU 测量序列。
 * @return 至少包含两个 IMU 样本时返回 true。
 */
bool MonocularInertialSync::HasEnoughMeasurementsForPreintegration(
    const std::vector<ORB_SLAM3::IMU::Point>& measurements)
{
  return measurements.size() >= 2U;
}
