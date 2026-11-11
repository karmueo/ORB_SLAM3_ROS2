/**
 * @file stereo-inertial.cpp
 * @brief 启动双目惯性 ORB_SLAM3 在线节点或离线 MCAP 处理入口。
 */

#include <opencv2/core/core.hpp>

#include <atomic>
#include <csignal>
#include <exception>
#include <fstream>
#include <iostream>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "stereo-inertial-node.hpp"

#include "offline-bag-reader.hpp"
#include "offline-cli.hpp"
#include "offline-runner.hpp"
#include "offline-slam-sink.hpp"

#include "System.h"

namespace
{
/** @brief 离线运行收到 SIGINT/SIGTERM 后置位的取消标志。 */
std::atomic<bool> g_offline_cancel{false};

/**
 * @brief 离线信号处理函数，仅设置取消标志。
 */
void HandleOfflineSignal(int)
{
    g_offline_cancel.store(true);
}

/**
 * @brief 判断普通文件存在且可读。
 */
bool IsReadableFile(const std::string& path)
{
    return !path.empty() && std::ifstream(path.c_str()).good();
}

/**
 * @brief 执行离线模式并在失败时清理 ORB_SLAM3 后台线程。
 */
int RunOffline(
    const StereoInertialCliOptions& cli,
    const OfflineStereoConfig& config)
{
    if (cli.do_rectify)
    {
        if (config.camera_type == "KannalaBrandt8")
        {
            std::cerr << "ERROR: 离线原生鱼眼模式不支持 DO_RECTIFY=true；"
                         "请保持 BOOL_RECTIFY=false 并传入原始拼接图像。"
                      << std::endl;
        }
        else
        {
            std::cerr << "ERROR: 离线模式当前不支持 DO_RECTIFY=true。" << std::endl;
        }
        return 2;
    }

    std::signal(SIGINT, HandleOfflineSignal);
    std::signal(SIGTERM, HandleOfflineSignal);

    OfflineRunOptions options;
    options.bag.bag_path = cli.bag_path;
    options.bag.stereo_topic = cli.stereo_topic;
    options.bag.imu_topic = cli.imu_topic;
    options.bag.imu_time_offset_sec = cli.has_imu_time_offset
        ? cli.imu_time_offset_sec
        : config.configured_imu_time_offset_sec;
    options.bag.accelerometer_bias = config.accelerometer_bias;
    options.bag.gyroscope_bias = config.gyroscope_bias;
    options.bag.expected_image_width = config.camera_width * 2;
    options.bag.expected_image_height = config.camera_height;
    options.output_path = cli.output_path;
    options.vocabulary_file = cli.vocabulary_file;
    options.settings_file = cli.settings_file;
    options.overwrite = cli.overwrite;
    options.do_equalize = cli.do_equalize;
    options.configured_imu_time_offset_sec = config.configured_imu_time_offset_sec;
    options.imu_time_offset_overridden = cli.has_imu_time_offset;
    options.cancel = &g_offline_cancel;
    options.orb_slam3_commit = "f8ac79153b103dd44f7ea3cf2a1a4aea10b18483";
    options.opencv_version = CV_VERSION;

    ORB_SLAM3::System slam(
        cli.vocabulary_file, cli.settings_file,
        ORB_SLAM3::System::IMU_STEREO, false);
    OrbSlamSystemSink sink(&slam, &g_offline_cancel);
    OfflineBagReader source;
    OfflineRunReport report;
    std::string error_message;
    const bool pipeline_ok =
        RunOfflinePipeline(&source, &sink, options, &report, &error_message);

    if (!pipeline_ok && !error_message.empty())
    {
        std::cerr << "ERROR: " << error_message << std::endl;
    }
    std::cout << "Offline status: " << report.status << std::endl;
    if (report.summary_written)
    {
        std::cout << "Offline summary: "
                  << BuildOfflineOutputPaths(cli.output_path).summary << std::endl;
    }

    if (report.status == "interrupted")
    {
        return 130;
    }
    return pipeline_ok ? 0 : 2;
}
}  // namespace

/**
 * @brief stereo-inertial 可执行入口。
 */
int main(int argc, char **argv)
{
    try
    {
        rclcpp::init(argc, argv);
    }
    catch (const std::exception& exception)
    {
        std::cerr << "ERROR: ROS 参数初始化失败: " << exception.what() << std::endl;
        return 1;
    }

    std::vector<std::string> arguments;
    try
    {
        arguments = rclcpp::remove_ros_arguments(argc, argv);
        if (!arguments.empty())
        {
            arguments.erase(arguments.begin());
        }
    }
    catch (const std::exception& exception)
    {
        std::cerr << "ERROR: 剥离 ROS 参数失败: " << exception.what() << std::endl;
        if (rclcpp::ok())
        {
            rclcpp::shutdown();
        }
        return 1;
    }

    StereoInertialCliOptions cli;
    std::string parse_error;
    if (!ParseStereoInertialArguments(arguments, &cli, &parse_error))
    {
        std::cerr << "ERROR: " << parse_error << std::endl;
        std::cerr << StereoInertialUsage();
        if (rclcpp::ok())
        {
            rclcpp::shutdown();
        }
        return 1;
    }
    if (cli.show_help)
    {
        std::cout << StereoInertialUsage();
        if (rclcpp::ok())
        {
            rclcpp::shutdown();
        }
        return 0;
    }

    if (!IsReadableFile(cli.vocabulary_file))
    {
        std::cerr << "ERROR: 词典文件不存在或不可读: " << cli.vocabulary_file
                  << std::endl;
        if (rclcpp::ok())
        {
            rclcpp::shutdown();
        }
        return 1;
    }
    if (!IsReadableFile(cli.settings_file))
    {
        std::cerr << "ERROR: 配置文件不存在或不可读: " << cli.settings_file
                  << std::endl;
        if (rclcpp::ok())
        {
            rclcpp::shutdown();
        }
        return 1;
    }

    if (cli.offline)
    {
        OfflineStereoConfig config;
        std::string config_error;
        if (!LoadOfflineStereoConfig(cli.settings_file, &config, &config_error))
        {
            std::cerr << "ERROR: " << config_error << std::endl;
            if (rclcpp::ok())
            {
                rclcpp::shutdown();
            }
            return 1;
        }
        const int exit_code = RunOffline(cli, config);
        if (rclcpp::ok())
        {
            rclcpp::shutdown();
        }
        return exit_code;
    }

    /** @brief ORB_SLAM3 双目惯性系统实例。 */
    ORB_SLAM3::System pSLAM(
        cli.vocabulary_file, cli.settings_file,
        ORB_SLAM3::System::IMU_STEREO, cli.use_viewer);

    {
        /** @brief ROS 2 双目惯性节点，作用域结束时先于 ROS shutdown 释放。 */
        auto node = std::make_shared<StereoInertialNode>(
            &pSLAM, cli.settings_file,
            cli.do_rectify ? "true" : "false",
            cli.do_equalize ? "true" : "false");
        std::cout << "============================" << std::endl;
        rclcpp::spin(node);
    }

    if (rclcpp::ok())
    {
        rclcpp::shutdown();
    }
    return 0;
}
