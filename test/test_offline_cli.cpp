/**
 * @file test_offline_cli.cpp
 * @brief 测试 stereo-inertial 离线参数解析和严格诊断。
 */

#include "offline-cli.hpp"

#include "gtest/gtest.h"

#include <vector>

TEST(OfflineCli, ParsesOnlinePositionalArgumentsUnchanged)
{
    const std::vector<std::string> arguments = {
        "vocab.txt", "settings.yaml", "false", "true", "false"};
    StereoInertialCliOptions options;
    std::string error;
    ASSERT_TRUE(ParseStereoInertialArguments(arguments, &options, &error)) << error;
    EXPECT_FALSE(options.offline);
    EXPECT_FALSE(options.do_rectify);
    EXPECT_TRUE(options.do_equalize);
    EXPECT_FALSE(options.use_viewer);
    EXPECT_EQ(options.vocabulary_file, "vocab.txt");
}

TEST(OfflineCli, OfflineDefaultsDisableViewerAndEqualize)
{
    const std::vector<std::string> arguments = {
        "vocab.txt", "settings.yaml", "false", "false", "false",
        "--bag", "job.mcap", "--output", "trajectory.txt"};
    StereoInertialCliOptions options;
    std::string error;
    ASSERT_TRUE(ParseStereoInertialArguments(arguments, &options, &error)) << error;
    EXPECT_TRUE(options.offline);
    EXPECT_FALSE(options.do_equalize);
    EXPECT_FALSE(options.use_viewer);
    EXPECT_EQ(options.bag_path, "job.mcap");
    EXPECT_EQ(options.output_path, "trajectory.txt");
    EXPECT_EQ(options.stereo_topic, "/stereo/image_raw");
    EXPECT_EQ(options.imu_topic, "/imu");
    EXPECT_FALSE(options.has_imu_time_offset);
}

TEST(OfflineCli, ParsesOffsetOverrideAndOverwrite)
{
    const std::vector<std::string> arguments = {
        "vocab.txt", "settings.yaml", "false",
        "--bag", "job.mcap", "--output", "out.txt",
        "--stereo-topic", "/stereo", "--imu-topic", "/imu2",
        "--imu-time-offset-sec", "0.025", "--overwrite"};
    StereoInertialCliOptions options;
    std::string error;
    ASSERT_TRUE(ParseStereoInertialArguments(arguments, &options, &error)) << error;
    EXPECT_DOUBLE_EQ(options.imu_time_offset_sec, 0.025);
    EXPECT_TRUE(options.has_imu_time_offset);
    EXPECT_TRUE(options.overwrite);
    EXPECT_EQ(options.stereo_topic, "/stereo");
    EXPECT_EQ(options.imu_topic, "/imu2");
}

TEST(OfflineCli, RejectsNonStrictBoolean)
{
    StereoInertialCliOptions options;
    std::string error;
    EXPECT_FALSE(ParseStereoInertialArguments(
        {"vocab", "settings", "False"}, &options, &error));
    EXPECT_NE(error.find("DO_RECTIFY"), std::string::npos);
    EXPECT_FALSE(ParseStrictBool("1", &options.do_rectify));
}

TEST(OfflineCli, RejectsMissingOrUnknownOfflineOptions)
{
    StereoInertialCliOptions options;
    std::string error;
    EXPECT_FALSE(ParseStereoInertialArguments(
        {"vocab", "settings", "false", "--bag", "job.mcap"}, &options, &error));
    EXPECT_NE(error.find("--output"), std::string::npos);
    EXPECT_FALSE(ParseStereoInertialArguments(
        {"vocab", "settings", "false", "--bag", "job.mcap", "--output", "o",
         "--unknown"}, &options, &error));
    EXPECT_NE(error.find("--unknown"), std::string::npos);
}

TEST(OfflineCli, RejectsInvalidOffset)
{
    StereoInertialCliOptions options;
    std::string error;
    EXPECT_FALSE(ParseStereoInertialArguments(
        {"vocab", "settings", "false", "--bag", "job.mcap", "--output", "o",
         "--imu-time-offset-sec", "nan"}, &options, &error));
    EXPECT_NE(error.find("--imu-time-offset-sec"), std::string::npos);
}
