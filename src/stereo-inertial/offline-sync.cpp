/**
 * @file offline-sync.cpp
 * @brief 实现离线时间覆盖判断与单向前置 IMU 游标。
 */

#include "offline-sync.hpp"

#include <cmath>
#include <cstdio>
#include <limits>

bool IsImageCoveredByImu(
    const std::int64_t image_timestamp_ns,
    const std::int64_t first_imu_timestamp_ns,
    const std::int64_t last_imu_timestamp_ns,
    const std::int64_t offset_ns)
{
    const std::int64_t first_corrected_ns = first_imu_timestamp_ns + offset_ns;
    const std::int64_t last_corrected_ns = last_imu_timestamp_ns + offset_ns;
    return image_timestamp_ns >= first_corrected_ns &&
           image_timestamp_ns <= last_corrected_ns;
}

std::int64_t TimeOffsetToNanoseconds(const double offset_seconds)
{
    if (!std::isfinite(offset_seconds))
    {
        return 0;
    }
    const long double nanoseconds =
        static_cast<long double>(offset_seconds) * 1000000000.0L;
    if (nanoseconds >= static_cast<long double>(std::numeric_limits<std::int64_t>::max()))
    {
        return std::numeric_limits<std::int64_t>::max();
    }
    if (nanoseconds <= static_cast<long double>(std::numeric_limits<std::int64_t>::min()))
    {
        return std::numeric_limits<std::int64_t>::min();
    }
    return static_cast<std::int64_t>(nanoseconds);
}

bool ExtractImuUntilImage(
    const std::vector<OfflineImuSample>& samples,
    const std::int64_t image_timestamp_ns,
    const std::int64_t offset_ns,
    OfflineImuCursor* cursor,
    std::vector<ORB_SLAM3::IMU::Point>* measurements,
    std::string* error_message)
{
    if (!cursor || !measurements)
    {
        if (error_message)
        {
            *error_message = "IMU 游标、输出或图像时间戳无效";
        }
        return false;
    }
    measurements->clear();

    const std::int64_t kUnset = std::numeric_limits<std::int64_t>::min();
    while (cursor->next_index < samples.size())
    {
        const OfflineImuSample& sample = samples[cursor->next_index];
        ++cursor->next_index;
        const std::int64_t corrected_timestamp_ns =
            sample.corrected_timestamp_ns == kUnset
                ? sample.raw_timestamp_ns + offset_ns
                : sample.corrected_timestamp_ns;
        const cv::Point3f acceleration(
            static_cast<float>(sample.acceleration[0]),
            static_cast<float>(sample.acceleration[1]),
            static_cast<float>(sample.acceleration[2]));
        const cv::Point3f angular_velocity(
            static_cast<float>(sample.angular_velocity[0]),
            static_cast<float>(sample.angular_velocity[1]),
            static_cast<float>(sample.angular_velocity[2]));
        measurements->push_back(ORB_SLAM3::IMU::Point(
            acceleration, angular_velocity, sample.corrected_relative_sec));

        if (corrected_timestamp_ns >= image_timestamp_ns)
        {
            if (error_message)
            {
                error_message->clear();
            }
            return true;
        }
    }

    if (error_message)
    {
        *error_message = "IMU 数据在图像时间戳前结束，缺少闭区间端点";
    }
    return false;
}

bool HasEnoughImuSamples(
    const std::vector<ORB_SLAM3::IMU::Point>& measurements,
    const bool is_first_frame)
{
    return measurements.size() >= (is_first_frame ? 1U : 2U);
}

std::string FormatTimestampSeconds(const std::int64_t timestamp_ns)
{
    const bool negative = timestamp_ns < 0;
    const std::uint64_t absolute =
        negative
            ? static_cast<std::uint64_t>(-(timestamp_ns + 1)) + 1U
            : static_cast<std::uint64_t>(timestamp_ns);
    const std::uint64_t seconds = absolute / 1000000000ULL;
    const std::uint64_t nanoseconds = absolute % 1000000000ULL;

    char buffer[64];
    std::snprintf(
        buffer, sizeof(buffer), "%s%llu.%09llu",
        negative ? "-" : "",
        static_cast<unsigned long long>(seconds),
        static_cast<unsigned long long>(nanoseconds));
    return std::string(buffer);
}
