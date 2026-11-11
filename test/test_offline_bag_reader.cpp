/**
 * @file test_offline_bag_reader.cpp
 * @brief 使用小型合成 MCAP 测试离线 reader 的类型、解码和数值边界。
 */

#include "offline-bag-reader.hpp"

#include "rclcpp/time.hpp"
#include "rosbag2_cpp/writer.hpp"
#include "rosbag2_storage/storage_options.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/imu.hpp"

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include "gtest/gtest.h"

#include <unistd.h>

#include <cmath>
#include <limits>
#include <cstdint>
#include <string>
#include <vector>

namespace
{
const char* const kImageTopic = "/stereo/image_raw";
const char* const kImuTopic = "/imu";

struct SyntheticBag
{
    std::vector<std::int64_t> image_stamps;
    std::vector<std::int64_t> imu_stamps;
    bool corrupt_second_image = false;
    bool duplicate_image_stamp = false;
    bool nonfinite_imu = false;
    bool wrong_image_type = false;
    bool only_one_imu = false;
};

std::string MakeBagDirectory()
{
    char pattern[] = "/tmp/orbslam3_offline_reader_XXXXXX";
    char* directory = ::mkdtemp(pattern);
    EXPECT_NE(directory, nullptr);
    return directory ? std::string(directory) : std::string();
}

sensor_msgs::msg::CompressedImage MakeCompressedImage(const std::int64_t stamp_ns)
{
    sensor_msgs::msg::CompressedImage message;
    message.header.stamp.sec = static_cast<std::int32_t>(stamp_ns / 1000000000LL);
    message.header.stamp.nanosec = static_cast<std::uint32_t>(stamp_ns % 1000000000LL);
    message.format = "jpeg";
    cv::Mat image(3, 8, CV_8UC3, cv::Scalar(0, 0, 0));
    image.colRange(0, 4).setTo(cv::Scalar(0, 0, 255));
    image.colRange(4, 8).setTo(cv::Scalar(255, 0, 0));
    std::vector<uchar> encoded;
    EXPECT_TRUE(cv::imencode(".jpg", image, encoded));
    message.data.assign(encoded.begin(), encoded.end());
    return message;
}

sensor_msgs::msg::Imu MakeImu(
    const std::int64_t stamp_ns,
    const double acceleration,
    const bool nonfinite)
{
    sensor_msgs::msg::Imu message;
    message.header.stamp.sec = static_cast<std::int32_t>(stamp_ns / 1000000000LL);
    message.header.stamp.nanosec = static_cast<std::uint32_t>(stamp_ns % 1000000000LL);
    message.linear_acceleration.x = nonfinite
        ? std::numeric_limits<double>::quiet_NaN() : acceleration;
    message.linear_acceleration.y = acceleration;
    message.linear_acceleration.z = acceleration;
    message.angular_velocity.x = 0.1;
    message.angular_velocity.y = 0.2;
    message.angular_velocity.z = 0.3;
    return message;
}

std::string WriteSyntheticBag(const SyntheticBag& specification)
{
    const std::string directory = MakeBagDirectory();
    const std::string path = directory + "/synthetic.mcap";
    rosbag2_cpp::Writer writer;
    rosbag2_storage::StorageOptions storage;
    storage.uri = path;
    storage.storage_id = "mcap";
    writer.open(storage, rosbag2_cpp::ConverterOptions{"cdr", "cdr"});

    rosbag2_storage::TopicMetadata image_topic;
    image_topic.name = kImageTopic;
    image_topic.type = specification.wrong_image_type
        ? "sensor_msgs/msg/Image"
        : "sensor_msgs/msg/CompressedImage";
    image_topic.serialization_format = "cdr";
    writer.create_topic(image_topic);

    rosbag2_storage::TopicMetadata imu_topic;
    imu_topic.name = kImuTopic;
    imu_topic.type = "sensor_msgs/msg/Imu";
    imu_topic.serialization_format = "cdr";
    writer.create_topic(imu_topic);

    std::size_t imu_count = specification.only_one_imu ? 1U : 5U;
    if (specification.imu_stamps.size() < imu_count)
    {
        imu_count = specification.imu_stamps.size();
    }
    for (std::size_t i = 0; i < imu_count; ++i)
    {
        writer.write(
            MakeImu(specification.imu_stamps[i], static_cast<double>(i),
                    specification.nonfinite_imu && i == 1U),
            kImuTopic, rclcpp::Time(specification.imu_stamps[i]));
    }

    for (std::size_t i = 0; i < specification.image_stamps.size(); ++i)
    {
        const std::int64_t stamp = specification.duplicate_image_stamp && i == 1U
            ? specification.image_stamps[0]
            : specification.image_stamps[i];
        if (specification.wrong_image_type)
        {
            sensor_msgs::msg::Image image;
            image.header.stamp.sec = static_cast<std::int32_t>(stamp / 1000000000LL);
            image.header.stamp.nanosec = static_cast<std::uint32_t>(stamp % 1000000000LL);
            writer.write(image, kImageTopic, rclcpp::Time(stamp));
        }
        else
        {
            sensor_msgs::msg::CompressedImage image = MakeCompressedImage(stamp);
            if (specification.corrupt_second_image && i == 1U)
            {
                image.data = {0xFF, 0xD8, 0x00, 0x01, 0x02};
            }
            writer.write(image, kImageTopic, rclcpp::Time(stamp));
        }
    }
    writer.close();
    return path;
}

SyntheticBag NormalSpecification()
{
    SyntheticBag bag;
    bag.image_stamps = {1000000000LL, 1010000000LL};
    bag.imu_stamps = {
        999000000LL, 999500000LL, 1000000000LL,
        1000500000LL, 1011000000LL};
    return bag;
}

OfflineBagOptions MakeOptions(const std::string& path)
{
    OfflineBagOptions options;
    options.bag_path = path;
    options.stereo_topic = kImageTopic;
    options.imu_topic = kImuTopic;
    options.expected_image_width = 8;
    options.expected_image_height = 3;
    options.accelerometer_bias = {{0.5, 0.0, 0.0}};
    options.gyroscope_bias = {{0.0, 0.0, 0.0}};
    return options;
}
}  // namespace

TEST(OfflineBagReader, InspectsTypesStampsAndSubtractsBias)
{
    const std::string path = WriteSyntheticBag(NormalSpecification());
    OfflineBagReader reader;
    OfflineBagInfo info;
    std::string error;
    ASSERT_TRUE(reader.Inspect(MakeOptions(path), &info, &error)) << error;
    ASSERT_EQ(info.image_stamps_ns.size(), 2U);
    ASSERT_EQ(info.imu_samples.size(), 5U);
    EXPECT_EQ(info.image_width, 8);
    EXPECT_EQ(info.camera_width, 4);
    EXPECT_EQ(info.image_height, 3);
    EXPECT_DOUBLE_EQ(info.imu_samples[0].acceleration[0], -0.5);
    EXPECT_DOUBLE_EQ(info.imu_samples[0].corrected_relative_sec, -0.001);
    EXPECT_EQ(info.first_corrected_imu_ns, 999000000LL);
    EXPECT_EQ(info.last_corrected_imu_ns, 1011000000LL);
}

TEST(OfflineBagReader, StreamsSplitGrayscaleImages)
{
    const std::string path = WriteSyntheticBag(NormalSpecification());
    OfflineBagReader reader;
    OfflineBagInfo info;
    std::string error;
    const OfflineBagOptions options = MakeOptions(path);
    ASSERT_TRUE(reader.Inspect(options, &info, &error)) << error;
    std::vector<std::int64_t> seen;
    ASSERT_TRUE(reader.ForEachImage(
        options, info,
        [&](const OfflineImageFrame& frame, std::string*) {
            seen.push_back(frame.timestamp_ns);
            EXPECT_EQ(frame.left.cols, 4);
            EXPECT_EQ(frame.left.rows, 3);
            EXPECT_EQ(frame.right.cols, 4);
            EXPECT_GT(cv::mean(frame.left)[0], cv::mean(frame.right)[0]);
            return true;
        },
        &error)) << error;
    EXPECT_EQ(seen, info.image_stamps_ns);
}

TEST(OfflineBagReader, RejectsWrongTopicType)
{
    SyntheticBag specification = NormalSpecification();
    specification.wrong_image_type = true;
    const std::string path = WriteSyntheticBag(specification);
    OfflineBagReader reader;
    OfflineBagInfo info;
    std::string error;
    EXPECT_FALSE(reader.Inspect(MakeOptions(path), &info, &error));
    EXPECT_NE(error.find("CompressedImage"), std::string::npos);
}

TEST(OfflineBagReader, RejectsNonFiniteImu)
{
    SyntheticBag specification = NormalSpecification();
    specification.nonfinite_imu = true;
    const std::string path = WriteSyntheticBag(specification);
    OfflineBagReader reader;
    OfflineBagInfo info;
    std::string error;
    EXPECT_FALSE(reader.Inspect(MakeOptions(path), &info, &error));
    EXPECT_NE(error.find("非有限"), std::string::npos);
}

TEST(OfflineBagReader, RejectsDuplicateImageStamp)
{
    SyntheticBag specification = NormalSpecification();
    specification.duplicate_image_stamp = true;
    const std::string path = WriteSyntheticBag(specification);
    OfflineBagReader reader;
    OfflineBagInfo info;
    std::string error;
    EXPECT_FALSE(reader.Inspect(MakeOptions(path), &info, &error));
    EXPECT_NE(error.find("严格递增"), std::string::npos);
}

TEST(OfflineBagReader, RejectsFewerThanTwoImuSamples)
{
    SyntheticBag specification = NormalSpecification();
    specification.only_one_imu = true;
    const std::string path = WriteSyntheticBag(specification);
    OfflineBagReader reader;
    OfflineBagInfo info;
    std::string error;
    EXPECT_FALSE(reader.Inspect(MakeOptions(path), &info, &error));
    EXPECT_NE(error.find("少于 2"), std::string::npos);
}

TEST(OfflineBagReader, FailsMidStreamOnCorruptSecondImage)
{
    SyntheticBag specification = NormalSpecification();
    specification.corrupt_second_image = true;
    const std::string path = WriteSyntheticBag(specification);
    OfflineBagReader reader;
    OfflineBagInfo info;
    std::string error;
    const OfflineBagOptions options = MakeOptions(path);
    ASSERT_TRUE(reader.Inspect(options, &info, &error)) << error;
    std::size_t callbacks = 0U;
    EXPECT_FALSE(reader.ForEachImage(
        options, info,
        [&](const OfflineImageFrame&, std::string*) {
            ++callbacks;
            return true;
        },
        &error));
    EXPECT_EQ(callbacks, 1U);
    EXPECT_NE(error.find("解码"), std::string::npos);
}
