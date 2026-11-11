/**
 * @file offline-sync.hpp
 * @brief 声明离线双目惯性时间覆盖判定与单向前置 IMU 游标。
 */

#ifndef __STEREO_INERTIAL_OFFLINE_SYNC_HPP__
#define __STEREO_INERTIAL_OFFLINE_SYNC_HPP__

#include "ImuTypes.h"

#include <cstddef>
#include <cstdint>
#include <limits>
#include <string>
#include <vector>

/**
 * @brief 第一遍扫描缓存的单条 IMU 样本。
 */
struct OfflineImuSample
{
    /** @brief 原始 header int64 纳秒时间戳。 */
    std::int64_t raw_timestamp_ns = 0;
    /**
     * @brief 已应用偏移的 int64 纳秒时间戳。
     *
     * 默认哨兵值表示调用方未缓存该字段，提取游标时按
     * raw_timestamp_ns + offset_ns 现场计算，避免退回 epoch double。
     */
    std::int64_t corrected_timestamp_ns =
        std::numeric_limits<std::int64_t>::min();
    /** @brief 相对首帧并应用时间偏移后的 double 秒，仅用于核心积分。 */
    double corrected_relative_sec = 0.0;
    /** @brief 已扣除固定 bias 的线加速度。 */
    double acceleration[3] = {0.0, 0.0, 0.0};
    /** @brief 已扣除固定 bias 的角速度。 */
    double angular_velocity[3] = {0.0, 0.0, 0.0};
};

/**
 * @brief 单向前置 IMU 游标。
 */
struct OfflineImuCursor
{
    /** @brief 下一条尚未提交给 ORB_SLAM3 的样本索引。 */
    std::size_t next_index = 0;
};

/**
 * @brief 判断图像是否位于修正后 IMU 首尾覆盖区间内。
 * @param image_timestamp_ns 图像原始 int64 纳秒时间戳。
 * @param first_imu_timestamp_ns 第一条原始 IMU 纳秒时间戳。
 * @param last_imu_timestamp_ns 最后一条原始 IMU 纳秒时间戳。
 * @param offset_ns 以纳秒表示的 IMU 时间偏移。
 * @return 图像时间位于闭区间内返回 true。
 */
bool IsImageCoveredByImu(
    std::int64_t image_timestamp_ns,
    std::int64_t first_imu_timestamp_ns,
    std::int64_t last_imu_timestamp_ns,
    std::int64_t offset_ns);

/**
 * @brief 将秒偏移转换为截断到整纳秒的偏移。
 * @param offset_seconds 秒偏移。
 * @return 截断后的 int64 纳秒偏移。
 */
std::int64_t TimeOffsetToNanoseconds(double offset_seconds);

/**
 * @brief 将连续 IMU 样本提交到首条修正后 t>=image_t 的样本（含该条）。
 * @param samples 按时间升序缓存的 IMU 样本。
 * @param image_timestamp_ns 当前图像原始 int64 纳秒时间戳。
 * @param offset_ns 以整纳秒表示的 IMU 时间偏移。
 * @param cursor 单向前置游标。
 * @param measurements 输出给 ORB_SLAM3 的 IMU 测量。
 * @param error_message 失败时的诊断文本。
 * @return 找到包含端点且至少提交一条样本时返回 true。
 */
bool ExtractImuUntilImage(
    const std::vector<OfflineImuSample>& samples,
    std::int64_t image_timestamp_ns,
    std::int64_t offset_ns,
    OfflineImuCursor* cursor,
    std::vector<ORB_SLAM3::IMU::Point>* measurements,
    std::string* error_message);

/**
 * @brief 判断当前提交批次是否满足最少预积分样本数。
 * @param measurements 当前批次。
 * @param is_first_frame 是否为整段输入的图像首帧。
 * @return 首帧至少 1 条、后续帧至少 2 条时返回 true。
 */
bool HasEnoughImuSamples(
    const std::vector<ORB_SLAM3::IMU::Point>& measurements,
    bool is_first_frame);

/**
 * @brief 将源纳秒时间戳格式化为整秒加 9 位小数的 TUM 时间。
 * @param timestamp_ns 原始 int64 纳秒时间戳。
 * @return 精确还原秒和 9 位纳秒余数的文本。
 */
std::string FormatTimestampSeconds(std::int64_t timestamp_ns);

#endif
