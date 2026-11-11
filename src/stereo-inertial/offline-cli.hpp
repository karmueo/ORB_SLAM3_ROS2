/**
 * @file offline-cli.hpp
 * @brief 声明 stereo-inertial 在线/离线命令行解析与配置读取辅助。
 */

#ifndef __STEREO_INERTIAL_OFFLINE_CLI_HPP__
#define __STEREO_INERTIAL_OFFLINE_CLI_HPP__

#include <array>
#include <string>
#include <vector>

/**
 * @brief stereo-inertial 可执行入口解析后的参数。
 */
struct StereoInertialCliOptions
{
    /** @brief ORB 词典路径。 */
    std::string vocabulary_file;
    /** @brief ORB_SLAM3 配置路径。 */
    std::string settings_file;
    /** @brief 是否对图像做矫正/去畸变。 */
    bool do_rectify = false;
    /** @brief 是否对图像做 CLAHE 均衡化。 */
    bool do_equalize = false;
    /** @brief 是否启用 viewer。 */
    bool use_viewer = true;

    /** @brief 是否选择离线 bag 模式。 */
    bool offline = false;
    /** @brief 输入 MCAP 或 rosbag2 目录。 */
    std::string bag_path;
    /** @brief 主轨迹输出路径。 */
    std::string output_path;
    /** @brief 拼接压缩图像 topic。 */
    std::string stereo_topic = "/stereo/image_raw";
    /** @brief IMU topic。 */
    std::string imu_topic = "/imu";
    /** @brief 是否显式覆盖标定中的 IMU 时间偏移。 */
    bool has_imu_time_offset = false;
    /** @brief 命令行 IMU 时间偏移，单位为秒。 */
    double imu_time_offset_sec = 0.0;
    /** @brief 是否允许覆盖本次命名范围内的旧输出。 */
    bool overwrite = false;
    /** @brief 是否请求打印帮助。 */
    bool show_help = false;
};

/**
 * @brief 离线模式所需的标定配置字段。
 */
struct OfflineStereoConfig
{
    /** @brief 单目相机宽度。 */
    int camera_width = 0;
    /** @brief 单目相机高度。 */
    int camera_height = 0;
    /** @brief ORB_SLAM3 Camera.type。 */
    std::string camera_type;
    /** @brief 配置中的 ROS.ImuTimeOffsetSec。 */
    double configured_imu_time_offset_sec = 0.0;
    /** @brief IMU 加速度计固定 bias。 */
    std::array<double, 3> accelerometer_bias{{0.0, 0.0, 0.0}};
    /** @brief IMU 陀螺仪固定 bias，单位为 rad/s。 */
    std::array<double, 3> gyroscope_bias{{0.0, 0.0, 0.0}};
};

/**
 * @brief 严格解析 true/false 布尔字面量。
 * @param value 输入文本。
 * @param parsed 解析结果。
 * @return 文本恰好为 true 或 false 时返回 true。
 */
bool ParseStrictBool(const std::string& value, bool* parsed);

/**
 * @brief 解析 stereo-inertial 的非 ROS 参数。
 * @param arguments 已剥离 ROS 参数、且不含 argv[0] 的 token。
 * @param options 输出参数。
 * @param error_message 失败时的诊断文本。
 * @return 解析成功返回 true。
 */
bool ParseStereoInertialArguments(
    const std::vector<std::string>& arguments,
    StereoInertialCliOptions* options,
    std::string* error_message);

/**
 * @brief 返回 stereo-inertial 用法文本。
 * @return 多行用法说明。
 */
std::string StereoInertialUsage();

/**
 * @brief 读取离线模式需要的相机和 IMU 配置字段。
 * @param settings_file ORB_SLAM3 配置路径。
 * @param config 输出配置。
 * @param error_message 失败时的诊断文本。
 * @return 读取成功返回 true。
 */
bool LoadOfflineStereoConfig(
    const std::string& settings_file,
    OfflineStereoConfig* config,
    std::string* error_message);

#endif
