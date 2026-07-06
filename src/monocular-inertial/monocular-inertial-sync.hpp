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
 * @brief 单目惯性同步队列辅助函数集合。
 *
 * 该类只处理消息队列和时间戳顺序，不依赖 ROS 节点生命周期，便于单元测试。
 */
class MonocularInertialSync
{
public:
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
