/**
 * @file monocular-inertial-sync.cpp
 * @brief 实现单目惯性图像/IMU 同步队列辅助函数。
 */

#include "monocular-inertial-sync.hpp"

#include "utility.hpp"

#include <opencv2/core/core.hpp>

#include <algorithm>
#include <cmath>

namespace
{
/**
 * @brief 将浮点秒数写入 ROS 时间戳。
 * @param timestampSec 浮点时间戳，单位为秒。
 * @param stamp 输出 ROS 时间戳。
 */
void SetStampFromSec(const double timestampSec, builtin_interfaces::msg::Time* const stamp)
{
  /** @brief 时间戳整数秒部分，使用 floor 兼容负偏移后的边界。 */
  int32_t sec = static_cast<int32_t>(std::floor(timestampSec));
  /** @brief 时间戳纳秒部分，四舍五入到整数纳秒。 */
  int64_t nanosec = static_cast<int64_t>(std::llround((timestampSec - static_cast<double>(sec)) * 1e9));
  if (nanosec >= 1000000000LL)
  {
    ++sec;
    nanosec -= 1000000000LL;
  }
  if (nanosec < 0LL)
  {
    --sec;
    nanosec += 1000000000LL;
  }

  stamp->sec = sec;
  stamp->nanosec = static_cast<uint32_t>(nanosec);
}

/**
 * @brief 复制 IMU 消息并应用时间偏移。
 * @param imuMsg 原始 IMU 消息。
 * @param imuTimeOffsetSec 应用于 IMU 时间戳的偏移量，单位为秒。
 * @return 带偏移后时间戳的 IMU 消息副本。
 */
MonocularSyncImuMsg::SharedPtr CloneImuWithTimeOffset(
    const MonocularSyncImuMsg::SharedPtr& imuMsg,
    const double imuTimeOffsetSec)
{
  /** @brief 应用偏移后的 IMU 消息副本。 */
  MonocularSyncImuMsg::SharedPtr shiftedMsg = std::make_shared<MonocularSyncImuMsg>(*imuMsg);
  /** @brief 应用偏移后的 IMU 时间戳，单位为秒。 */
  const double shiftedTimestamp = Utility::StampToSec(imuMsg->header.stamp) + imuTimeOffsetSec;
  SetStampFromSec(shiftedTimestamp, &shiftedMsg->header.stamp);
  return shiftedMsg;
}

/**
 * @brief 读取 IMU 消息时间戳。
 * @param imuMsg IMU 消息。
 * @return IMU 时间戳，单位为秒。
 */
double ReadImuTimestamp(const MonocularSyncImuMsg::SharedPtr& imuMsg)
{
  return Utility::StampToSec(imuMsg->header.stamp);
}
}  // namespace

/**
 * @brief 判断图像与最近 IMU 时间戳是否处于同一时间基准。
 * @param imageTimestamp 当前图像原始时间戳，单位为秒。
 * @param latestImuTimestamp 当前图像到达时最近的原始 IMU 时间戳，单位为秒。
 * @param maxDifferenceSec 允许的最大绝对差值，单位为秒。
 * @return 两个时间戳有限、阈值合法且绝对差不超过阈值时返回 true。
 */
bool MonocularInertialSync::AreTimeBasesAligned(
    const double imageTimestamp,
    const double latestImuTimestamp,
    const double maxDifferenceSec)
{
  return std::isfinite(imageTimestamp) &&
         std::isfinite(latestImuTimestamp) &&
         std::isfinite(maxDifferenceSec) &&
         maxDifferenceSec >= 0.0 &&
         std::abs(imageTimestamp - latestImuTimestamp) <= maxDifferenceSec;
}

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
MonocularImageQueuePushResult MonocularInertialSync::PushRateLimitedBoundedImage(
    std::queue<MonocularSyncImageMsg::SharedPtr>& imageQueue,
    const MonocularSyncImageMsg::SharedPtr& imageMsg,
    const std::size_t maxQueueSize,
    const double targetFps,
    MonocularImageRateLimitState* const rateLimitState,
    std::uint64_t* const rateLimitedImageCount,
    std::uint64_t* const overflowDroppedImageCount)
{
  /** @brief 当前图像时间戳，单位为秒。 */
  const double imageTimestamp = Utility::StampToSec(imageMsg->header.stamp);
  /** @brief 限帧策略是否启用。 */
  const bool rateLimitEnabled = targetFps > 0.0 && rateLimitState != nullptr;
  if (rateLimitEnabled)
  {
    if (rateLimitState->lastAcceptedImageTimestamp >= 0.0 &&
        imageTimestamp <= rateLimitState->lastAcceptedImageTimestamp)
    {
      if (rateLimitedImageCount)
      {
        ++(*rateLimitedImageCount);
      }
      return MonocularImageQueuePushResult{false, true, false};
    }

    if (rateLimitState->lastSeenImageTimestamp < 0.0)
    {
      rateLimitState->imageRateCredit = 1.0;
    }
    else
    {
      /** @brief 当前图像与上一张输入图像的时间间隔，单位为秒。 */
      const double imageInterval = imageTimestamp - rateLimitState->lastSeenImageTimestamp;
      if (imageInterval >= 0.0)
      {
        /** @brief 长时间停顿后允许保留的最大令牌额度，保留跨阈值余量且避免恢复时连续补发过多旧帧。 */
        const double maxImageRateCredit = 2.0;
        rateLimitState->imageRateCredit += imageInterval * targetFps;
        if (rateLimitState->imageRateCredit > maxImageRateCredit)
        {
          rateLimitState->imageRateCredit = maxImageRateCredit;
        }
      }
      else
      {
        rateLimitState->imageRateCredit = 0.0;
      }
    }
    rateLimitState->lastSeenImageTimestamp = imageTimestamp;

    if (rateLimitState->imageRateCredit < 1.0)
    {
      if (rateLimitedImageCount)
      {
        ++(*rateLimitedImageCount);
      }
      return MonocularImageQueuePushResult{false, true, false};
    }
  }

  /** @brief 当前入队是否导致最旧图像被丢弃。 */
  const bool droppedOldestImage = PushBoundedImage(
      imageQueue,
      imageMsg,
      maxQueueSize,
      overflowDroppedImageCount);
  if (rateLimitState)
  {
    rateLimitState->lastAcceptedImageTimestamp = imageTimestamp;
    if (rateLimitEnabled)
    {
      rateLimitState->imageRateCredit -= 1.0;
    }
  }

  return MonocularImageQueuePushResult{true, false, droppedOldestImage};
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
  return PushMonotonicImu(imuQueue, imuMsg, 0.0, lastImuTimestamp, droppedImuCount);
}

/**
 * @brief 将 IMU 样本应用时间偏移后按严格递增时间戳入队。
 * @param imuQueue IMU 消息队列。
 * @param imuMsg 待入队 IMU 消息。
 * @param imuTimeOffsetSec 应用于 IMU 时间戳的偏移量，单位为秒。
 * @param lastImuTimestamp 最近一次接收的 IMU 时间戳。
 * @param droppedImuCount 被过滤 IMU 累计数量，可为空。
 * @return 样本成功入队时返回 true，回跳或重复时间戳返回 false。
 */
bool MonocularInertialSync::PushMonotonicImu(
    std::queue<MonocularSyncImuMsg::SharedPtr>& imuQueue,
    const MonocularSyncImuMsg::SharedPtr& imuMsg,
    const double imuTimeOffsetSec,
    double* const lastImuTimestamp,
    std::uint64_t* const droppedImuCount)
{
  /** @brief 当前 IMU 样本时间戳，单位为秒。 */
  const double imuTimestamp = Utility::StampToSec(imuMsg->header.stamp) + imuTimeOffsetSec;
  if (lastImuTimestamp && *lastImuTimestamp >= 0.0 && imuTimestamp <= *lastImuTimestamp)
  {
    if (droppedImuCount)
    {
      ++(*droppedImuCount);
    }
    return false;
  }

  /** @brief 应用时间偏移后的 IMU 消息。 */
  const MonocularSyncImuMsg::SharedPtr shiftedMsg = CloneImuWithTimeOffset(imuMsg, imuTimeOffsetSec);
  imuQueue.push(shiftedMsg);
  if (lastImuTimestamp)
  {
    *lastImuTimestamp = imuTimestamp;
  }
  return true;
}

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
MonocularImuQueuePushResult MonocularInertialSync::PushReorderedImu(
    std::queue<MonocularSyncImuMsg::SharedPtr>& imuQueue,
    const MonocularSyncImuMsg::SharedPtr& imuMsg,
    const double imuTimeOffsetSec,
    const double reorderWindowSec,
    MonocularImuReorderState* const reorderState,
    double* const lastImuTimestamp,
    std::uint64_t* const droppedImuCount)
{
  if (reorderWindowSec <= 0.0 || reorderState == nullptr)
  {
    /** @brief 不启用重排时当前样本是否直接入队。 */
    const bool accepted = PushMonotonicImu(imuQueue, imuMsg, imuTimeOffsetSec, lastImuTimestamp, droppedImuCount);
    return MonocularImuQueuePushResult{accepted ? 1U : 0U, accepted ? 0U : 1U, 0U};
  }

  /** @brief 应用时间偏移后的 IMU 消息。 */
  MonocularSyncImuMsg::SharedPtr shiftedMsg = CloneImuWithTimeOffset(imuMsg, imuTimeOffsetSec);
  /** @brief 应用偏移后的 IMU 时间戳，单位为秒。 */
  const double shiftedTimestamp = ReadImuTimestamp(shiftedMsg);
  if (reorderState->newestObservedImuTimestamp < 0.0 ||
      shiftedTimestamp > reorderState->newestObservedImuTimestamp)
  {
    reorderState->newestObservedImuTimestamp = shiftedTimestamp;
  }

  reorderState->pendingImuMsgs.push_back(shiftedMsg);
  std::sort(
      reorderState->pendingImuMsgs.begin(),
      reorderState->pendingImuMsgs.end(),
      [](const MonocularSyncImuMsg::SharedPtr& left, const MonocularSyncImuMsg::SharedPtr& right) {
        return ReadImuTimestamp(left) < ReadImuTimestamp(right);
      });

  /** @brief 可安全输出的最晚 IMU 时间戳，单位为秒。 */
  const double flushTimestamp = reorderState->newestObservedImuTimestamp - reorderWindowSec;
  /** @brief 当前调用输出到主 IMU 队列的样本数量。 */
  std::size_t acceptedCount = 0U;
  /** @brief 当前调用过滤掉的 IMU 样本数量。 */
  std::size_t droppedCount = 0U;
  /** @brief 当前重排缓存的遍历迭代器。 */
  std::vector<MonocularSyncImuMsg::SharedPtr>::iterator imuIt = reorderState->pendingImuMsgs.begin();
  while (imuIt != reorderState->pendingImuMsgs.end())
  {
    /** @brief 当前候选 IMU 时间戳，单位为秒。 */
    const double candidateTimestamp = ReadImuTimestamp(*imuIt);
    if (candidateTimestamp > flushTimestamp)
    {
      break;
    }

    /** @brief 当前候选样本是否输出到主 IMU 队列。 */
    const bool accepted = PushMonotonicImu(imuQueue, *imuIt, 0.0, lastImuTimestamp, droppedImuCount);
    if (accepted)
    {
      ++acceptedCount;
    }
    else
    {
      ++droppedCount;
    }
    imuIt = reorderState->pendingImuMsgs.erase(imuIt);
  }

  return MonocularImuQueuePushResult{acceptedCount, droppedCount, reorderState->pendingImuMsgs.size()};
}

/**
 * @brief 从 IMU 重排缓存刷新样本，直到主队列覆盖指定图像时间戳。
 * @param imuQueue IMU 主队列。
 * @param reorderState IMU 小窗口重排状态。
 * @param imageTimestamp 当前等待处理的图像时间戳，单位为秒。
 * @param lastImuTimestamp 最近一次输出到主队列的 IMU 时间戳。
 * @param droppedImuCount 被过滤 IMU 累计数量，可为空。
 * @return 本次刷新输出、过滤和剩余待排样本统计。
 */
MonocularImuQueuePushResult MonocularInertialSync::FlushPendingReorderedImuUntilCovered(
    std::queue<MonocularSyncImuMsg::SharedPtr>& imuQueue,
    MonocularImuReorderState* const reorderState,
    const double imageTimestamp,
    double* const lastImuTimestamp,
    std::uint64_t* const droppedImuCount)
{
  if (reorderState == nullptr || reorderState->pendingImuMsgs.empty())
  {
    return MonocularImuQueuePushResult{0U, 0U, 0U};
  }

  std::sort(
      reorderState->pendingImuMsgs.begin(),
      reorderState->pendingImuMsgs.end(),
      [](const MonocularSyncImuMsg::SharedPtr& left, const MonocularSyncImuMsg::SharedPtr& right) {
        return ReadImuTimestamp(left) < ReadImuTimestamp(right);
      });

  /** @brief 当前调用输出到主 IMU 队列的样本数量。 */
  std::size_t acceptedCount = 0U;
  /** @brief 当前调用过滤掉的 IMU 样本数量。 */
  std::size_t droppedCount = 0U;
  /** @brief 当前重排缓存的遍历迭代器。 */
  std::vector<MonocularSyncImuMsg::SharedPtr>::iterator imuIt = reorderState->pendingImuMsgs.begin();
  while (imuIt != reorderState->pendingImuMsgs.end())
  {
    /** @brief 当前候选样本是否输出到主 IMU 队列。 */
    const bool accepted = PushMonotonicImu(imuQueue, *imuIt, 0.0, lastImuTimestamp, droppedImuCount);
    if (accepted)
    {
      ++acceptedCount;
    }
    else
    {
      ++droppedCount;
    }
    imuIt = reorderState->pendingImuMsgs.erase(imuIt);

    if (HasImuCoverageForImage(imuQueue, imageTimestamp))
    {
      break;
    }
  }

  return MonocularImuQueuePushResult{acceptedCount, droppedCount, reorderState->pendingImuMsgs.size()};
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
