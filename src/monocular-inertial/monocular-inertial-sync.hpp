/**
 * @file monocular-inertial-sync.hpp
 * @brief 声明单目惯性图像/IMU 同步队列辅助函数。
 */

#ifndef __MONOCULAR_INERTIAL_SYNC_HPP__
#define __MONOCULAR_INERTIAL_SYNC_HPP__

#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/imu.hpp"

#include "ImuTypes.h"

#include <cstdint>
#include <queue>
#include <vector>

/** @brief ROS 2 IMU 消息类型别名。 */
using MonocularSyncImuMsg = sensor_msgs::msg::Imu;
/** @brief ROS 2 图像消息类型别名。 */
using MonocularSyncImageMsg = sensor_msgs::msg::Image;

/**
 * @brief 图像入队辅助函数的结果。
 */
struct MonocularImageQueuePushResult
{
  /** @brief 当前图像是否成功进入待处理队列。 */
  bool accepted;
  /** @brief 当前图像是否因限帧策略被主动丢弃。 */
  bool droppedByRateLimit;
  /** @brief 当前入队是否导致最旧图像被丢弃。 */
  bool droppedOldestImage;
};

/**
 * @brief 图像平均帧率限制状态。
 */
struct MonocularImageRateLimitState
{
  /** @brief 最近一次看到的图像时间戳，单位为秒。 */
  double lastSeenImageTimestamp;
  /** @brief 当前累计的帧率令牌额度，达到 1.0 时可接受一帧。 */
  double imageRateCredit;
  /** @brief 最近一次通过限帧并成功入队的图像时间戳，单位为秒。 */
  double lastAcceptedImageTimestamp;
};

/**
 * @brief IMU 小窗口重排状态。
 */
struct MonocularImuReorderState
{
  /** @brief 当前重排窗口内观察到的最新 IMU 时间戳，单位为秒。 */
  double newestObservedImuTimestamp = -1.0;
  /** @brief 尚未超过重排窗口的候选 IMU 消息。 */
  std::vector<MonocularSyncImuMsg::SharedPtr> pendingImuMsgs;
};

/**
 * @brief IMU 入队辅助函数的结果。
 */
struct MonocularImuQueuePushResult
{
  /** @brief 本次调用输出到主 IMU 队列的样本数量。 */
  std::size_t acceptedCount;
  /** @brief 本次调用因时间戳回跳或重复被过滤的样本数量。 */
  std::size_t droppedCount;
  /** @brief 本次调用结束后仍保留在重排窗口内的样本数量。 */
  std::size_t pendingCount;
};

/**
 * @brief 单目惯性同步队列辅助函数集合。
 *
 * 该类只处理消息队列和时间戳顺序，不依赖 ROS 节点生命周期，便于单元测试。
 */
class MonocularInertialSync
{
public:
  /**
   * @brief 判断图像与最近 IMU 时间戳是否处于同一时间基准。
   * @param imageTimestamp 当前图像原始时间戳，单位为秒。
   * @param latestImuTimestamp 当前图像到达时最近的原始 IMU 时间戳，单位为秒。
   * @param maxDifferenceSec 允许的最大绝对差值，单位为秒。
   * @return 两个时间戳有限、阈值合法且绝对差不超过阈值时返回 true。
   */
  static bool AreTimeBasesAligned(
      double imageTimestamp,
      double latestImuTimestamp,
      double maxDifferenceSec);

  /**
   * @brief 判断 IMU 队列最新样本是否覆盖指定图像时间戳。
   * @param imuQueue IMU 消息队列。
   * @param imageTimestamp 图像时间戳，单位为秒。
   * @return 最新 IMU 时间戳大于等于图像时间戳时返回 true。
   */
  static bool HasImuCoverageForImage(
      const std::queue<MonocularSyncImuMsg::SharedPtr>& imuQueue,
      double imageTimestamp);

  /**
   * @brief 将图像压入有界 FIFO 队列，溢出时丢弃最旧图像。
   * @param imageQueue 图像消息队列。
   * @param imageMsg 待入队图像消息。
   * @param maxQueueSize 队列最大容量。
   * @param droppedImageCount 被丢弃图像累计数量，可为空。
   * @return 发生溢出丢弃时返回 true。
   */
  static bool PushBoundedImage(
      std::queue<MonocularSyncImageMsg::SharedPtr>& imageQueue,
      const MonocularSyncImageMsg::SharedPtr& imageMsg,
      std::size_t maxQueueSize,
      std::uint64_t* droppedImageCount);

  /**
   * @brief 按目标帧率和队列容量缓存图像，优先保留较新的图像帧。
   * @param imageQueue 图像消息队列。
   * @param imageMsg 待入队图像消息。
   * @param maxQueueSize 队列最大容量。
   * @param targetFps 目标入队帧率，单位 Hz；小于等于 0 时关闭限帧。
   * @param rateLimitState 图像平均帧率限制状态，可为空。
   * @param rateLimitedImageCount 因限帧被主动丢弃的图像累计数量，可为空。
   * @param overflowDroppedImageCount 因队列溢出被丢弃的图像累计数量，可为空。
   * @return 当前图像入队结果。
   */
  static MonocularImageQueuePushResult PushRateLimitedBoundedImage(
      std::queue<MonocularSyncImageMsg::SharedPtr>& imageQueue,
      const MonocularSyncImageMsg::SharedPtr& imageMsg,
      std::size_t maxQueueSize,
      double targetFps,
      MonocularImageRateLimitState* rateLimitState,
      std::uint64_t* rateLimitedImageCount,
      std::uint64_t* overflowDroppedImageCount);

  /**
   * @brief 将 IMU 样本按严格递增时间戳入队。
   * @param imuQueue IMU 消息队列。
   * @param imuMsg 待入队 IMU 消息。
   * @param lastImuTimestamp 最近一次接收的 IMU 时间戳。
   * @param droppedImuCount 被过滤 IMU 累计数量，可为空。
   * @return 样本成功入队时返回 true，回跳或重复时间戳返回 false。
   */
  static bool PushMonotonicImu(
      std::queue<MonocularSyncImuMsg::SharedPtr>& imuQueue,
      const MonocularSyncImuMsg::SharedPtr& imuMsg,
      double* lastImuTimestamp,
      std::uint64_t* droppedImuCount);

  /**
   * @brief 将 IMU 样本应用时间偏移后按严格递增时间戳入队。
   * @param imuQueue IMU 消息队列。
   * @param imuMsg 待入队 IMU 消息。
   * @param imuTimeOffsetSec 应用于 IMU 时间戳的偏移量，单位为秒。
   * @param lastImuTimestamp 最近一次接收的 IMU 时间戳。
   * @param droppedImuCount 被过滤 IMU 累计数量，可为空。
   * @return 样本成功入队时返回 true，回跳或重复时间戳返回 false。
   */
  static bool PushMonotonicImu(
      std::queue<MonocularSyncImuMsg::SharedPtr>& imuQueue,
      const MonocularSyncImuMsg::SharedPtr& imuMsg,
      double imuTimeOffsetSec,
      double* lastImuTimestamp,
      std::uint64_t* droppedImuCount);

  /**
   * @brief 将 IMU 样本应用时间偏移后放入小窗口重排，再按递增时间戳输出到主队列。
   * @param imuQueue IMU 消息队列。
   * @param imuMsg 待处理 IMU 消息。
   * @param imuTimeOffsetSec 应用于 IMU 时间戳的偏移量，单位为秒。
   * @param reorderWindowSec 重排窗口长度，单位为秒；小于等于 0 时退化为单调入队。
   * @param reorderState IMU 小窗口重排状态。
   * @param lastImuTimestamp 最近一次输出到主队列的 IMU 时间戳。
   * @param droppedImuCount 被过滤 IMU 累计数量，可为空。
   * @return 本次调用的入队、过滤和待排样本统计。
   */
  static MonocularImuQueuePushResult PushReorderedImu(
      std::queue<MonocularSyncImuMsg::SharedPtr>& imuQueue,
      const MonocularSyncImuMsg::SharedPtr& imuMsg,
      double imuTimeOffsetSec,
      double reorderWindowSec,
      MonocularImuReorderState* reorderState,
      double* lastImuTimestamp,
      std::uint64_t* droppedImuCount);

  /**
   * @brief 从 IMU 重排缓存刷新样本，直到主队列覆盖指定图像时间戳。
   * @param imuQueue IMU 主队列。
   * @param reorderState IMU 小窗口重排状态。
   * @param imageTimestamp 当前等待处理的图像时间戳，单位为秒。
   * @param lastImuTimestamp 最近一次输出到主队列的 IMU 时间戳。
   * @param droppedImuCount 被过滤 IMU 累计数量，可为空。
   * @return 本次刷新输出、过滤和剩余待排样本统计。
   */
  static MonocularImuQueuePushResult FlushPendingReorderedImuUntilCovered(
      std::queue<MonocularSyncImuMsg::SharedPtr>& imuQueue,
      MonocularImuReorderState* reorderState,
      double imageTimestamp,
      double* lastImuTimestamp,
      std::uint64_t* droppedImuCount);

  /**
   * @brief 从 IMU 队列取出时间戳不晚于图像时间戳的测量。
   * @param imuQueue IMU 消息队列，已取出的样本会被弹出。
   * @param imageTimestamp 图像时间戳，单位为秒。
   * @return 按时间顺序排列的 ORB_SLAM3 IMU 测量。
   */
  static std::vector<ORB_SLAM3::IMU::Point> ExtractImuMeasurementsUntil(
      std::queue<MonocularSyncImuMsg::SharedPtr>& imuQueue,
      double imageTimestamp);

  /**
   * @brief 判断 IMU 测量数量是否足够 ORB_SLAM3 做相邻帧预积分。
   * @param measurements 当前图像帧对应的 IMU 测量序列。
   * @return 至少包含两个 IMU 样本时返回 true。
   */
  static bool HasEnoughMeasurementsForPreintegration(
      const std::vector<ORB_SLAM3::IMU::Point>& measurements);
};

#endif
