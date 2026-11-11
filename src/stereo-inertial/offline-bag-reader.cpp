/**
 * @file offline-bag-reader.cpp
 * @brief 实现两遍式 MCAP/rosbag2 离线读取、解码与 IMU 缓存。
 */

#include "offline-bag-reader.hpp"

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include "rclcpp/serialization.hpp"
#include "rclcpp/serialized_message.hpp"
#include "rosbag2_cpp/reader.hpp"
#include "rosbag2_storage/serialized_bag_message.hpp"
#include "rosbag2_storage/storage_filter.hpp"
#include "rosbag2_storage/storage_options.hpp"
#include "rosbag2_storage/topic_metadata.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"
#include "sensor_msgs/msg/imu.hpp"

#include <sys/stat.h>

#include <cerrno>
#include <cmath>
#include <cstring>
#include <fstream>
#include <sstream>

namespace
{
const char* const kCompressedImageType = "sensor_msgs/msg/CompressedImage";
const char* const kImuType = "sensor_msgs/msg/Imu";

/**
 * @brief 判断路径是否存在。
 */
bool PathExists(const std::string& path)
{
    struct stat info;
    return ::stat(path.c_str(), &info) == 0;
}

/**
 * @brief 判断路径是否目录。
 */
bool PathIsDirectory(const std::string& path)
{
    struct stat info;
    return ::stat(path.c_str(), &info) == 0 && S_ISDIR(info.st_mode);
}

/**
 * @brief 根据文件扩展名或目录 metadata 推断 rosbag2 storage id。
 */
std::string InferStorageId(const std::string& bag_path)
{
    std::string path = bag_path;
    while (path.size() > 1U && path.back() == '/')
    {
        path.pop_back();
    }
    if (path.size() >= 5U && path.compare(path.size() - 5U, 5U, ".mcap") == 0)
    {
        return "mcap";
    }
    if (path.size() >= 4U && path.compare(path.size() - 4U, 4U, ".db3") == 0)
    {
        return "sqlite3";
    }
    if (PathIsDirectory(path))
    {
        const std::string metadata = path + "/metadata.yaml";
        std::ifstream input(metadata.c_str());
        if (input.good())
        {
            std::string line;
            while (std::getline(input, line))
            {
                const std::size_t colon = line.find("storage_identifier:");
                if (colon != std::string::npos)
                {
                    std::string value = line.substr(colon + 19U);
                    const std::size_t first = value.find_first_not_of(" \t\"");
                    const std::size_t last = value.find_last_not_of(" \t\"");
                    if (first != std::string::npos && last >= first)
                    {
                        return value.substr(first, last - first + 1U);
                    }
                }
            }
        }
    }
    return "mcap";
}

/**
 * @brief 读取 builtin_interfaces/Time 为 int64 纳秒。
 */
bool ReadStampNs(
    const builtin_interfaces::msg::Time& stamp,
    std::int64_t* timestamp_ns,
    std::string* error_message)
{
    if (stamp.nanosec >= 1000000000U)
    {
        *error_message = "header.stamp.nanosec 超出 [0, 1e9) 范围";
        return false;
    }
    *timestamp_ns =
        static_cast<std::int64_t>(stamp.sec) * 1000000000LL +
        static_cast<std::int64_t>(stamp.nanosec);
    return true;
}

/**
 * @brief 打开 rosbag2 Reader。
 */
bool OpenReader(
    const std::string& bag_path,
    rosbag2_cpp::Reader* reader,
    std::string* error_message)
{
    if (!PathExists(bag_path))
    {
        *error_message = "bag 路径不存在: " + bag_path;
        return false;
    }
    rosbag2_storage::StorageOptions storage_options;
    storage_options.uri = bag_path;
    storage_options.storage_id = InferStorageId(bag_path);
    rosbag2_cpp::ConverterOptions converter_options{"cdr", "cdr"};
    try
    {
        reader->open(storage_options, converter_options);
    }
    catch (const std::exception& exception)
    {
        *error_message = "无法打开 bag " + bag_path + ": " + exception.what();
        return false;
    }
    return true;
}

/**
 * @brief 校验 topic 类型并返回 topic 映射。
 */
bool ValidateTopics(
    const rosbag2_cpp::Reader& reader,
    const OfflineBagOptions& options,
    std::string* error_message)
{
    std::string image_type;
    std::string imu_type;
    for (const rosbag2_storage::TopicMetadata& topic :
         reader.get_all_topics_and_types())
    {
        if (topic.name == options.stereo_topic)
        {
            image_type = topic.type;
        }
        else if (topic.name == options.imu_topic)
        {
            imu_type = topic.type;
        }
    }
    if (image_type != kCompressedImageType)
    {
        *error_message = "topic " + options.stereo_topic + " 必须是 " +
                         kCompressedImageType + "，实际为: " +
                         (image_type.empty() ? "<缺失>" : image_type);
        return false;
    }
    if (imu_type != kImuType)
    {
        *error_message = "topic " + options.imu_topic + " 必须是 " + kImuType +
                         "，实际为: " + (imu_type.empty() ? "<缺失>" : imu_type);
        return false;
    }
    return true;
}

/**
 * @brief 校验图像原始时间严格递增。
 */
bool CheckStrictlyIncreasing(
    const std::int64_t timestamp_ns,
    std::int64_t* previous_ns,
    const std::string& name,
    std::string* error_message)
{
    if (*previous_ns >= 0 && timestamp_ns <= *previous_ns)
    {
        std::ostringstream out;
        out << name << " header 时间戳不是严格递增: " << *previous_ns
            << " -> " << timestamp_ns;
        *error_message = out.str();
        return false;
    }
    *previous_ns = timestamp_ns;
    return true;
}

/**
 * @brief 将压缩图像解码为 BGR 图像。
 */
bool DecodeCompressedImage(
    const sensor_msgs::msg::CompressedImage& message,
    cv::Mat* image,
    std::string* error_message)
{
    if (message.data.empty())
    {
        *error_message = "CompressedImage.data 为空";
        return false;
    }
    cv::Mat encoded(
        1, static_cast<int>(message.data.size()), CV_8UC1,
        const_cast<unsigned char*>(message.data.data()));
    try
    {
        *image = cv::imdecode(encoded, cv::IMREAD_COLOR);
    }
    catch (const cv::Exception& exception)
    {
        *error_message = "JPEG/压缩图像解码异常: " + std::string(exception.what());
        return false;
    }
    if (image->empty())
    {
        *error_message = "JPEG/压缩图像解码失败";
        return false;
    }
    if (image->channels() != 3)
    {
        *error_message = "压缩图像解码结果不是三通道";
        return false;
    }
    return true;
}

/**
 * @brief 来自 IMU 消息的有限性校验。
 */
bool ValidateImuFinite(
    const sensor_msgs::msg::Imu& message,
    std::string* error_message)
{
    const double values[] = {
        message.linear_acceleration.x, message.linear_acceleration.y,
        message.linear_acceleration.z, message.angular_velocity.x,
        message.angular_velocity.y, message.angular_velocity.z};
    for (double value : values)
    {
        if (!std::isfinite(value))
        {
            *error_message = "IMU 消息包含非有限测量";
            return false;
        }
    }
    return true;
}
}  // namespace

bool OfflineBagReader::Inspect(
    const OfflineBagOptions& options,
    OfflineBagInfo* info,
    std::string* error_message)
{
    if (!info || !error_message)
    {
        return false;
    }
    *info = OfflineBagInfo();
    error_message->clear();
    if (!std::isfinite(options.imu_time_offset_sec))
    {
        *error_message = "IMU 时间偏移必须是有限数值";
        return false;
    }

    rosbag2_cpp::Reader reader;
    if (!OpenReader(options.bag_path, &reader, error_message) ||
        !ValidateTopics(reader, options, error_message))
    {
        return false;
    }

    rosbag2_storage::StorageFilter filter;
    filter.topics = {options.stereo_topic, options.imu_topic};
    reader.set_filter(filter);

    rclcpp::Serialization<sensor_msgs::msg::CompressedImage> image_serializer;
    rclcpp::Serialization<sensor_msgs::msg::Imu> imu_serializer;
    std::int64_t previous_image_ns = -1;
    std::int64_t previous_imu_ns = -1;
    bool first_image_seen = false;

    try
    {
        while (reader.has_next())
        {
            std::shared_ptr<rosbag2_storage::SerializedBagMessage> message =
                reader.read_next();
            if (!message)
            {
                *error_message = "rosbag2 返回空消息";
                return false;
            }
            rclcpp::SerializedMessage serialized(*message->serialized_data);

            if (message->topic_name == options.stereo_topic)
            {
                sensor_msgs::msg::CompressedImage image_message;
                image_serializer.deserialize_message(&serialized, &image_message);
                std::int64_t timestamp_ns = 0;
                if (!ReadStampNs(image_message.header.stamp, &timestamp_ns, error_message) ||
                    !CheckStrictlyIncreasing(
                        timestamp_ns, &previous_image_ns, "图像", error_message))
                {
                    return false;
                }
                info->image_stamps_ns.push_back(timestamp_ns);

                if (!first_image_seen)
                {
                    cv::Mat image;
                    if (!DecodeCompressedImage(image_message, &image, error_message))
                    {
                        return false;
                    }
                    info->image_width = image.cols;
                    info->image_height = image.rows;
                    info->image_format = image_message.format;
                    first_image_seen = true;
                }
            }
            else if (message->topic_name == options.imu_topic)
            {
                sensor_msgs::msg::Imu imu_message;
                imu_serializer.deserialize_message(&serialized, &imu_message);
                std::int64_t timestamp_ns = 0;
                if (!ReadStampNs(imu_message.header.stamp, &timestamp_ns, error_message) ||
                    !CheckStrictlyIncreasing(
                        timestamp_ns, &previous_imu_ns, "IMU", error_message) ||
                    !ValidateImuFinite(imu_message, error_message))
                {
                    return false;
                }
                OfflineImuSample sample;
                sample.raw_timestamp_ns = timestamp_ns;
                for (int axis = 0; axis < 3; ++axis)
                {
                    const double acceleration =
                        axis == 0 ? imu_message.linear_acceleration.x
                                  : axis == 1 ? imu_message.linear_acceleration.y
                                              : imu_message.linear_acceleration.z;
                    const double angular_velocity =
                        axis == 0 ? imu_message.angular_velocity.x
                                  : axis == 1 ? imu_message.angular_velocity.y
                                              : imu_message.angular_velocity.z;
                    sample.acceleration[axis] =
                        acceleration - options.accelerometer_bias[static_cast<std::size_t>(axis)];
                    sample.angular_velocity[axis] =
                        angular_velocity - options.gyroscope_bias[static_cast<std::size_t>(axis)];
                }
                info->imu_samples.push_back(sample);
            }
        }
    }
    catch (const std::exception& exception)
    {
        *error_message = "读取 bag 消息失败: " + std::string(exception.what());
        return false;
    }

    if (!first_image_seen || info->image_stamps_ns.empty())
    {
        *error_message = "bag 中没有可解码的 " + options.stereo_topic + " 图像";
        return false;
    }
    if (info->imu_samples.size() < 2U)
    {
        *error_message = "IMU 样本少于 2 条，无法支撑预积分";
        return false;
    }
    if (info->image_width <= 0 || info->image_height <= 0 ||
        (info->image_width % 2) != 0)
    {
        *error_message = "拼接图像宽度必须为偶数正整数";
        return false;
    }
    if (options.expected_image_width > 0 &&
        info->image_width != options.expected_image_width)
    {
        std::ostringstream out;
        out << "拼接图像宽度与配置不一致: 实际 " << info->image_width
            << "，期望 " << options.expected_image_width;
        *error_message = out.str();
        return false;
    }
    if (options.expected_image_height > 0 &&
        info->image_height != options.expected_image_height)
    {
        std::ostringstream out;
        out << "拼接图像高度与配置不一致: 实际 " << info->image_height
            << "，期望 " << options.expected_image_height;
        *error_message = out.str();
        return false;
    }
    info->camera_width = info->image_width / 2;

    const std::int64_t origin_ns = info->image_stamps_ns.front();
    const std::int64_t offset_ns =
        TimeOffsetToNanoseconds(options.imu_time_offset_sec);
    for (OfflineImuSample& sample : info->imu_samples)
    {
        sample.corrected_timestamp_ns = sample.raw_timestamp_ns + offset_ns;
        sample.corrected_relative_sec =
            static_cast<double>(sample.raw_timestamp_ns - origin_ns) * 1e-9 +
            options.imu_time_offset_sec;
    }
    info->first_corrected_imu_ns = info->imu_samples.front().corrected_timestamp_ns;
    info->last_corrected_imu_ns = info->imu_samples.back().corrected_timestamp_ns;
    return true;
}

bool OfflineBagReader::ForEachImage(
    const OfflineBagOptions& options,
    const OfflineBagInfo& inspected,
    const std::function<bool(const OfflineImageFrame&, std::string*)>& callback,
    std::string* error_message)
{
    if (!callback || !error_message)
    {
        return false;
    }
    error_message->clear();

    rosbag2_cpp::Reader reader;
    if (!OpenReader(options.bag_path, &reader, error_message) ||
        !ValidateTopics(reader, options, error_message))
    {
        return false;
    }
    rosbag2_storage::StorageFilter filter;
    filter.topics = {options.stereo_topic};
    reader.set_filter(filter);

    rclcpp::Serialization<sensor_msgs::msg::CompressedImage> serializer;
    std::size_t image_index = 0U;
    try
    {
        while (reader.has_next())
        {
            std::shared_ptr<rosbag2_storage::SerializedBagMessage> message =
                reader.read_next();
            if (!message || message->topic_name != options.stereo_topic)
            {
                continue;
            }
            sensor_msgs::msg::CompressedImage image_message;
            rclcpp::SerializedMessage serialized(*message->serialized_data);
            serializer.deserialize_message(&serialized, &image_message);

            std::int64_t timestamp_ns = 0;
            if (!ReadStampNs(image_message.header.stamp, &timestamp_ns, error_message))
            {
                return false;
            }
            if (image_index >= inspected.image_stamps_ns.size() ||
                timestamp_ns != inspected.image_stamps_ns[image_index])
            {
                *error_message = "第二遍图像时间戳与第一遍缓存不一致";
                return false;
            }

            cv::Mat image;
            if (!DecodeCompressedImage(image_message, &image, error_message))
            {
                std::ostringstream out;
                out << "第 " << image_index << " 帧(" << timestamp_ns << "ns): "
                    << *error_message;
                *error_message = out.str();
                return false;
            }
            if (image.cols != inspected.image_width ||
                image.rows != inspected.image_height)
            {
                std::ostringstream out;
                out << "第 " << image_index << " 帧尺寸不一致: " << image.cols
                    << "x" << image.rows << "，期望 " << inspected.image_width
                    << "x" << inspected.image_height;
                *error_message = out.str();
                return false;
            }

            OfflineImageFrame frame;
            frame.timestamp_ns = timestamp_ns;
            const int half_width = image.cols / 2;
            cv::Mat left_bgr = image.colRange(0, half_width);
            cv::Mat right_bgr = image.colRange(half_width, image.cols);
            cv::cvtColor(left_bgr, frame.left, cv::COLOR_BGR2GRAY);
            cv::cvtColor(right_bgr, frame.right, cv::COLOR_BGR2GRAY);

            ++image_index;
            if (!callback(frame, error_message))
            {
                if (error_message->empty())
                {
                    *error_message = "图像回调请求停止";
                }
                return false;
            }
        }
    }
    catch (const std::exception& exception)
    {
        *error_message = "第二遍读取图像失败: " + std::string(exception.what());
        return false;
    }

    if (image_index != inspected.image_stamps_ns.size())
    {
        std::ostringstream out;
        out << "第二遍解码图像数量不一致: " << image_index << " / "
            << inspected.image_stamps_ns.size();
        *error_message = out.str();
        return false;
    }
    return true;
}
