/**
 * @file test_offline_sync.cpp
 * @brief 测试离线 IMU 覆盖、整数时间边界、单向前置游标和时间格式化。
 */

#include "offline-sync.hpp"

#include "gtest/gtest.h"

#include <cstdint>
#include <vector>

namespace
{
OfflineImuSample MakeSample(const std::int64_t stamp_ns, const double relative_sec)
{
    OfflineImuSample sample;
    sample.raw_timestamp_ns = stamp_ns;
    sample.corrected_relative_sec = relative_sec;
    sample.acceleration[0] = 1.0;
    sample.acceleration[1] = 2.0;
    sample.acceleration[2] = 3.0;
    sample.angular_velocity[0] = 0.1;
    sample.angular_velocity[1] = 0.2;
    sample.angular_velocity[2] = 0.3;
    return sample;
}
}  // namespace

TEST(OfflineSync, CoverageUsesClosedCorrectedImuRange)
{
    EXPECT_TRUE(IsImageCoveredByImu(1000, 900, 1100, 100));
    EXPECT_TRUE(IsImageCoveredByImu(1000, 1000, 1000, 0));
    EXPECT_FALSE(IsImageCoveredByImu(999, 900, 1100, 100));
    EXPECT_FALSE(IsImageCoveredByImu(1201, 900, 1100, 100));
}

TEST(OfflineSync, OffsetNanosecondsTruncateTowardZero)
{
    EXPECT_EQ(TimeOffsetToNanoseconds(0.0000000014), 1);
    EXPECT_EQ(TimeOffsetToNanoseconds(-0.0000000014), -1);
    EXPECT_EQ(TimeOffsetToNanoseconds(-0.005089461604383672), -5089461);
}

TEST(OfflineSync, CursorSubmitsInclusiveEndpointOnceWithoutRefeed)
{
    const std::vector<OfflineImuSample> samples = {
        MakeSample(100, 0.000), MakeSample(101, 0.001),
        MakeSample(102, 0.002), MakeSample(103, 0.003),
        MakeSample(104, 0.004)};
    OfflineImuCursor cursor;
    std::vector<ORB_SLAM3::IMU::Point> first;
    std::string error;
    ASSERT_TRUE(ExtractImuUntilImage(samples, 102, 0, &cursor, &first, &error))
        << error;
    ASSERT_EQ(first.size(), 3U);
    EXPECT_DOUBLE_EQ(first.front().t, 0.000);
    EXPECT_DOUBLE_EQ(first.back().t, 0.002);
    EXPECT_EQ(cursor.next_index, 3U);

    std::vector<ORB_SLAM3::IMU::Point> second;
    ASSERT_TRUE(ExtractImuUntilImage(samples, 104, 0, &cursor, &second, &error))
        << error;
    ASSERT_EQ(second.size(), 2U);
    EXPECT_DOUBLE_EQ(second.front().t, 0.003);
    EXPECT_DOUBLE_EQ(second.back().t, 0.004);
    EXPECT_EQ(cursor.next_index, 5U);
}

TEST(OfflineSync, AlreadyCommittedFutureEndpointIsNeverSubmittedTwice)
{
    const std::vector<OfflineImuSample> samples = {
        MakeSample(1000, 0.000), MakeSample(1001, 0.001),
        MakeSample(1002, 0.002)};
    OfflineImuCursor cursor;
    std::vector<ORB_SLAM3::IMU::Point> first;
    std::string error;
    ASSERT_TRUE(ExtractImuUntilImage(samples, 1001, 0, &cursor, &first, &error))
        << error;
    ASSERT_EQ(first.size(), 2U);

    // The next image has the same timestamp as the already consumed future
    // endpoint; the endpoint must not be fed again, only newer samples may be.
    std::vector<ORB_SLAM3::IMU::Point> second;
    ASSERT_TRUE(ExtractImuUntilImage(samples, 1001, 0, &cursor, &second, &error))
        << error;
    ASSERT_EQ(second.size(), 1U);
    EXPECT_DOUBLE_EQ(second.front().t, 0.002);
    EXPECT_EQ(cursor.next_index, 3U);
}

TEST(OfflineSync, MissingEndpointIsExplicitFailure)
{
    const std::vector<OfflineImuSample> samples = {
        MakeSample(100, 0.000), MakeSample(101, 0.001)};
    OfflineImuCursor cursor;
    std::vector<ORB_SLAM3::IMU::Point> measurements;
    std::string error;
    EXPECT_FALSE(ExtractImuUntilImage(samples, 102, 0, &cursor, &measurements, &error));
    EXPECT_NE(error.find("端点"), std::string::npos);
}

TEST(OfflineSync, FractionalOffsetUsesExactIntegerNanosecondBoundary)
{
    // A negative fractional offset truncates to -1 ns; the corrected endpoint
    // is exactly 99 ns and must be included for an image at 99 ns.
    const std::int64_t offset_ns = TimeOffsetToNanoseconds(-0.0000000014);
    ASSERT_EQ(offset_ns, -1);
    const std::vector<OfflineImuSample> samples = {
        MakeSample(90, -0.000000010), MakeSample(100, 0.000)};
    OfflineImuCursor cursor;
    std::vector<ORB_SLAM3::IMU::Point> measurements;
    std::string error;
    ASSERT_TRUE(ExtractImuUntilImage(
        samples, 99, offset_ns, &cursor, &measurements, &error)) << error;
    ASSERT_EQ(measurements.size(), 2U);

    std::vector<ORB_SLAM3::IMU::Point> beyond;
    OfflineImuCursor beyond_cursor;
    EXPECT_FALSE(ExtractImuUntilImage(
        samples, 100, offset_ns, &beyond_cursor, &beyond, &error));
}

TEST(OfflineSync, UsesCachedCorrectedTimestampWhenPresent)
{
    std::vector<OfflineImuSample> samples = {
        MakeSample(100, 0.000), MakeSample(200, 0.001)};
    samples[0].corrected_timestamp_ns = 150;
    samples[1].corrected_timestamp_ns = 250;
    OfflineImuCursor cursor;
    std::vector<ORB_SLAM3::IMU::Point> measurements;
    std::string error;
    // raw+offset would place both before 200, but the cached corrected times
    // make the second sample the inclusive endpoint.
    ASSERT_TRUE(ExtractImuUntilImage(samples, 200, 0, &cursor, &measurements, &error))
        << error;
    EXPECT_EQ(measurements.size(), 2U);
}

TEST(OfflineSync, EnforcesTwoSamplesAfterFirstFrame)
{
    std::vector<ORB_SLAM3::IMU::Point> one;
    one.push_back(ORB_SLAM3::IMU::Point(cv::Point3f(0, 0, 0), cv::Point3f(0, 0, 0), 0.0));
    EXPECT_TRUE(HasEnoughImuSamples(one, true));
    EXPECT_FALSE(HasEnoughImuSamples(one, false));
    one.push_back(ORB_SLAM3::IMU::Point(cv::Point3f(0, 0, 0), cv::Point3f(0, 0, 0), 0.001));
    EXPECT_TRUE(HasEnoughImuSamples(one, false));
}

TEST(OfflineSync, FormatsOriginalNanosecondTimestampExactly)
{
    EXPECT_EQ(FormatTimestampSeconds(1793850304571406667LL),
              "1793850304.571406667");
    EXPECT_EQ(FormatTimestampSeconds(1000000001LL), "1.000000001");
    EXPECT_EQ(FormatTimestampSeconds(-1LL), "-0.000000001");
}
