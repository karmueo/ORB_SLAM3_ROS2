/**
 * @file offline-bag-reader.hpp
 * @brief 声明两遍式 MCAP/rosbag2 离线读取接口。
 */

#ifndef __STEREO_INERTIAL_OFFLINE_BAG_READER_HPP__
#define __STEREO_INERTIAL_OFFLINE_BAG_READER_HPP__

#include "offline-sync.hpp"

#include <opencv2/core/core.hpp>

#include <array>
#include <cstdint>
#include <functional>
#include <string>
#include <vector>

/**
 * @brief 离线读取参数。
 */
struct OfflineBagOptions
{
    /** @brief bag 文件或目录路径。 */
    std::string bag_path;
    /** @brief 拼接 CompressedImage topic。 */
    std::string stereo_topic = "/stereo/image_raw";
    /** @brief sensor_msgs/msg/Imu topic。 */
    std::string imu_topic = "/imu";
    /** @brief IMU 时间偏移，单位为秒。 */
    double imu_time_offset_sec = 0.0;
    /** @brief 加速度计固定 bias。 */
    std::array<double, 3> accelerometer_bias{{0.0, 0.0, 0.0}};
    /** @brief 陀螺仪固定 bias，单位为 rad/s。 */
    std::array<double, 3> gyroscope_bias{{0.0, 0.0, 0.0}};
    /** @brief 期望的拼接图像宽度；小于等于 0 时不校验。 */
    int expected_image_width = 0;
    /** @brief 期望的拼接图像高度；小于等于 0 时不校验。 */
    int expected_image_height = 0;
};

/**
 * @brief 第一遍扫描得到的只读 bag 信息和缓存 IMU。
 */
struct OfflineBagInfo
{
    /** @brief 全部图像原始 int64 纳秒时间戳，严格递增。 */
    std::vector<std::int64_t> image_stamps_ns;
    /** @brief 全部 IMU 样本，原始时间严格递增。 */
    std::vector<OfflineImuSample> imu_samples;
    /** @brief 修正后的首条 IMU 纳秒时间戳。 */
    std::int64_t first_corrected_imu_ns = 0;
    /** @brief 修正后的末条 IMU 纳秒时间戳。 */
    std::int64_t last_corrected_imu_ns = 0;
    /** @brief 拼接图像宽度。 */
    int image_width = 0;
    /** @brief 拼接图像高度。 */
    int image_height = 0;
    /** @brief 单目半幅宽度。 */
    int camera_width = 0;
    /** @brief CompressedImage.format 字段。 */
    std::string image_format;
};

/**
 * @brief 第二遍顺序解码后的一帧左右灰度图像。
 */
struct OfflineImageFrame
{
    /** @brief 原始 int64 纳秒时间戳。 */
    std::int64_t timestamp_ns = 0;
    /** @brief 左相机灰度图。 */
    cv::Mat left;
    /** @brief 右相机灰度图。 */
    cv::Mat right;
};

/**
 * @brief 可注入的离线图像源接口，便于测试 reader/runner 边界。
 */
class OfflineFrameSource
{
public:
    virtual ~OfflineFrameSource() = default;

    /**
     * @brief 第一遍扫描类型、时间、数值和尺寸，并缓存 IMU。
     */
    virtual bool Inspect(
        const OfflineBagOptions& options,
        OfflineBagInfo* info,
        std::string* error_message) = 0;

    /**
     * @brief 第二遍按时间顺序解码并回调每一帧。
     */
    virtual bool ForEachImage(
        const OfflineBagOptions& options,
        const OfflineBagInfo& info,
        const std::function<bool(const OfflineImageFrame&, std::string*)>& callback,
        std::string* error_message) = 0;
};

/**
 * @brief 基于 rosbag2_cpp 的 MCAP/rosbag2 离线读取器。
 */
class OfflineBagReader : public OfflineFrameSource
{
public:
    bool Inspect(
        const OfflineBagOptions& options,
        OfflineBagInfo* info,
        std::string* error_message) override;

    bool ForEachImage(
        const OfflineBagOptions& options,
        const OfflineBagInfo& info,
        const std::function<bool(const OfflineImageFrame&, std::string*)>& callback,
        std::string* error_message) override;
};

#endif
