/**
 * @file offline-cli.cpp
 * @brief 实现 stereo-inertial 在线/离线参数解析与离线配置读取。
 */

#include "offline-cli.hpp"

#include <opencv2/core/core.hpp>

#include <cerrno>
#include <cmath>
#include <cstdlib>
#include <sstream>

namespace
{
/**
 * @brief 判断 token 是否为离线选项。
 * @param token 命令行 token。
 * @return 任一支持的离线选项返回 true。
 */
bool IsOfflineOption(const std::string& token)
{
    return token == "--bag" || token == "--output" || token == "--stereo-topic" ||
           token == "--imu-topic" || token == "--imu-time-offset-sec" ||
           token == "--overwrite";
}

/**
 * @brief 读取选项值并前移索引。
 * @param arguments 参数列表。
 * @param index 当前选项索引，成功时前移到值之后。
 * @param option 选项名称，用于诊断。
 * @param value 输出值。
 * @param error_message 失败诊断。
 * @return 成功返回 true。
 */
bool ReadOptionValue(
    const std::vector<std::string>& arguments,
    std::size_t* index,
    const std::string& option,
    std::string* value,
    std::string* error_message)
{
    if (*index + 1U >= arguments.size())
    {
        *error_message = "选项 " + option + " 缺少取值";
        return false;
    }
    *value = arguments[*index + 1U];
    *index += 2U;
    return true;
}

/**
 * @brief 解析有限浮点数。
 * @param text 输入文本。
 * @param value 输出值。
 * @return 完全消费且有限时返回 true。
 */
bool ParseFiniteDouble(const std::string& text, double* value)
{
    char* end = nullptr;
    errno = 0;
    const double parsed = std::strtod(text.c_str(), &end);
    if (end == text.c_str() || *end != '\0' || errno == ERANGE || !std::isfinite(parsed))
    {
        return false;
    }
    *value = parsed;
    return true;
}

/**
 * @brief 从 OpenCV FileStorage 读取三元素向量。
 * @param node 输入节点。
 * @param name 字段名。
 * @param output 输出数组。
 * @param error_message 失败诊断。
 * @return 成功返回 true。
 */
bool ReadThreeVector(
    const cv::FileNode& node,
    const std::string& name,
    std::array<double, 3>* output,
    std::string* error_message)
{
    if (node.empty() || !node.isSeq() || node.size() != 3)
    {
        *error_message = "配置字段 " + name + " 需要 3 个数值";
        return false;
    }
    for (int i = 0; i < 3; ++i)
    {
        const double value = static_cast<double>(node[static_cast<int>(i)]);
        if (!std::isfinite(value))
        {
            *error_message = "配置字段 " + name + " 包含非有限数值";
            return false;
        }
        (*output)[static_cast<std::size_t>(i)] = value;
    }
    return true;
}
}  // namespace

bool ParseStrictBool(const std::string& value, bool* parsed)
{
    if (!parsed)
    {
        return false;
    }
    if (value == "true")
    {
        *parsed = true;
        return true;
    }
    if (value == "false")
    {
        *parsed = false;
        return true;
    }
    return false;
}

std::string StereoInertialUsage()
{
    return
        "Usage (online): ros2 run orbslam3 stereo-inertial VOCAB ORB_CONFIG "
        "DO_RECTIFY [DO_EQUALIZE] [USE_VIEWER]\n"
        "Usage (offline): ros2 run orbslam3 stereo-inertial VOCAB ORB_CONFIG "
        "false false false --bag MCAP --output trajectory.txt "
        "[--stereo-topic /stereo/image_raw] [--imu-topic /imu] "
        "[--imu-time-offset-sec SEC] [--overwrite]\n";
}

bool ParseStereoInertialArguments(
    const std::vector<std::string>& arguments,
    StereoInertialCliOptions* options,
    std::string* error_message)
{
    if (!options || !error_message)
    {
        return false;
    }
    *options = StereoInertialCliOptions();
    error_message->clear();

    if (arguments.size() == 1U && (arguments[0] == "--help" || arguments[0] == "-h"))
    {
        options->show_help = true;
        return true;
    }
    if (arguments.size() < 3U)
    {
        *error_message = "至少需要 VOCAB、ORB_CONFIG 和 DO_RECTIFY 三个位置参数";
        return false;
    }

    options->vocabulary_file = arguments[0];
    options->settings_file = arguments[1];
    if (!ParseStrictBool(arguments[2], &options->do_rectify))
    {
        *error_message = "DO_RECTIFY 必须是严格布尔值 true 或 false，实际为: " + arguments[2];
        return false;
    }
    if (options->vocabulary_file.empty() || options->settings_file.empty())
    {
        *error_message = "VOCAB 和 ORB_CONFIG 不能为空";
        return false;
    }

    bool offline_requested = false;
    for (const std::string& token : arguments)
    {
        if (IsOfflineOption(token))
        {
            offline_requested = true;
            break;
        }
    }

    std::size_t index = 3U;
    bool equalize_seen = false;
    bool viewer_seen = false;
    if (index < arguments.size() && !arguments[index].empty() &&
        arguments[index][0] != '-')
    {
        if (!ParseStrictBool(arguments[index], &options->do_equalize))
        {
            *error_message = "DO_EQUALIZE 必须是严格布尔值 true 或 false，实际为: " +
                             arguments[index];
            return false;
        }
        equalize_seen = true;
        ++index;
    }
    if (index < arguments.size() && !arguments[index].empty() &&
        arguments[index][0] != '-')
    {
        if (!ParseStrictBool(arguments[index], &options->use_viewer))
        {
            *error_message = "USE_VIEWER 必须是严格布尔值 true 或 false，实际为: " +
                             arguments[index];
            return false;
        }
        viewer_seen = true;
        ++index;
    }

    if (!offline_requested)
    {
        if (!equalize_seen)
        {
            options->do_equalize = false;
        }
        if (!viewer_seen)
        {
            options->use_viewer = true;
        }
        if (index != arguments.size())
        {
            *error_message = "无法识别的多余参数: " + arguments[index];
            return false;
        }
        options->offline = false;
        return true;
    }

    options->offline = true;
    options->do_equalize = equalize_seen ? options->do_equalize : false;
    options->use_viewer = viewer_seen ? options->use_viewer : false;

    bool bag_seen = false;
    bool output_seen = false;
    while (index < arguments.size())
    {
        const std::string token = arguments[index];
        if (token == "--help" || token == "-h")
        {
            options->show_help = true;
            ++index;
            continue;
        }
        if (token == "--overwrite")
        {
            if (options->overwrite)
            {
                *error_message = "选项 --overwrite 不能重复";
                return false;
            }
            options->overwrite = true;
            ++index;
            continue;
        }
        if (token == "--bag")
        {
            std::string value;
            if (!ReadOptionValue(arguments, &index, token, &value, error_message))
            {
                return false;
            }
            if (bag_seen)
            {
                *error_message = "选项 --bag 不能重复";
                return false;
            }
            options->bag_path = value;
            bag_seen = true;
            continue;
        }
        if (token == "--output")
        {
            std::string value;
            if (!ReadOptionValue(arguments, &index, token, &value, error_message))
            {
                return false;
            }
            if (output_seen)
            {
                *error_message = "选项 --output 不能重复";
                return false;
            }
            options->output_path = value;
            output_seen = true;
            continue;
        }
        if (token == "--stereo-topic" || token == "--imu-topic")
        {
            std::string value;
            if (!ReadOptionValue(arguments, &index, token, &value, error_message))
            {
                return false;
            }
            if (value.empty())
            {
                *error_message = "选项 " + token + " 不能为空";
                return false;
            }
            if (token == "--stereo-topic")
            {
                options->stereo_topic = value;
            }
            else
            {
                options->imu_topic = value;
            }
            continue;
        }
        if (token == "--imu-time-offset-sec")
        {
            std::string value;
            if (!ReadOptionValue(arguments, &index, token, &value, error_message))
            {
                return false;
            }
            if (options->has_imu_time_offset)
            {
                *error_message = "选项 --imu-time-offset-sec 不能重复";
                return false;
            }
            if (!ParseFiniteDouble(value, &options->imu_time_offset_sec))
            {
                *error_message = "选项 --imu-time-offset-sec 需要有限数值，实际为: " + value;
                return false;
            }
            options->has_imu_time_offset = true;
            continue;
        }
        *error_message = "离线模式无法识别的选项或参数: " + token;
        return false;
    }

    if (!bag_seen || options->bag_path.empty())
    {
        *error_message = "离线模式必须提供 --bag MCAP";
        return false;
    }
    if (!output_seen || options->output_path.empty())
    {
        *error_message = "离线模式必须提供 --output trajectory.txt";
        return false;
    }
    if (options->use_viewer)
    {
        *error_message = "离线模式不支持启用 viewer；请使用 false 或省略 USE_VIEWER";
        return false;
    }
    return true;
}

bool LoadOfflineStereoConfig(
    const std::string& settings_file,
    OfflineStereoConfig* config,
    std::string* error_message)
{
    if (!config || !error_message)
    {
        return false;
    }
    error_message->clear();
    cv::FileStorage storage(settings_file, cv::FileStorage::READ);
    if (!storage.isOpened())
    {
        *error_message = "无法打开 ORB_SLAM3 配置文件: " + settings_file;
        return false;
    }

    const cv::FileNode camera_type = storage["Camera.type"];
    if (camera_type.empty() || !camera_type.isString())
    {
        *error_message = "配置缺少字符串字段 Camera.type";
        return false;
    }
    config->camera_type = camera_type.string();

    const cv::FileNode width = storage["Camera.width"];
    const cv::FileNode height = storage["Camera.height"];
    if (width.empty() || height.empty())
    {
        *error_message = "配置缺少 Camera.width/Camera.height";
        return false;
    }
    config->camera_width = static_cast<int>(width);
    config->camera_height = static_cast<int>(height);
    if (config->camera_width <= 0 || config->camera_height <= 0)
    {
        *error_message = "Camera.width/Camera.height 必须为正整数";
        return false;
    }

    const cv::FileNode offset = storage["ROS.ImuTimeOffsetSec"];
    if (!offset.empty())
    {
        const double parsed = static_cast<double>(offset);
        if (!std::isfinite(parsed))
        {
            *error_message = "ROS.ImuTimeOffsetSec 必须是有限数值";
            return false;
        }
        config->configured_imu_time_offset_sec = parsed;
    }

    if (!ReadThreeVector(
            storage["IMU.AccBias"], "IMU.AccBias", &config->accelerometer_bias,
            error_message))
    {
        return false;
    }
    if (!ReadThreeVector(
            storage["IMU.GyroBias"], "IMU.GyroBias", &config->gyroscope_bias,
            error_message))
    {
        return false;
    }
    return true;
}
