/**
 * @file monocular-inertial.cpp
 * @brief 启动单目惯性 ORB_SLAM3 ROS 2 节点，并管理 ROS 与 SLAM 系统生命周期。
 */

#include <iostream>
#include <fstream>
#include <sstream>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "monocular-inertial-node.hpp"

#include "System.h"

/**
 * @brief monocular-inertial 可执行入口。
 * @param argc 命令行参数数量。
 * @param argv 命令行参数数组，依次包含词典、配置文件和可选的 viewer 开关。
 * @return 进程退出码，0 表示正常退出，非 0 表示参数错误或运行失败。
 */
int main(int argc, char** argv)
{
    if (argc < 3)
    {
        std::cerr << "\nUsage: ros2 run orbslam3 monocular-inertial path_to_vocabulary path_to_settings [use_viewer]" << std::endl;
        return 1;
    }

    /** @brief ORB_SLAM3 词典文件路径，需在 ROS 参数解析前复制，避免 argv 被后续流程影响。 */
    const std::string vocabularyFile = argv[1] ? argv[1] : "";
    /** @brief ORB_SLAM3 配置文件路径，需在 ROS 参数解析前复制，避免空路径传入 OpenCV。 */
    const std::string settingsFile = argv[2] ? argv[2] : "";
    if (vocabularyFile.empty() || settingsFile.empty())
    {
        std::cerr << "Error: vocabulary path and settings path must not be empty." << std::endl;
        std::cerr << "Usage: ros2 run orbslam3 monocular-inertial path_to_vocabulary path_to_settings [use_viewer]" << std::endl;
        return 1;
    }
    if (!std::ifstream(vocabularyFile).good())
    {
        std::cerr << "Error: vocabulary file does not exist or is not readable: " << vocabularyFile << std::endl;
        return 1;
    }
    if (!std::ifstream(settingsFile).good())
    {
        std::cerr << "Error: settings file does not exist or is not readable: " << settingsFile << std::endl;
        return 1;
    }

    /** @brief ORB_SLAM3 viewer 开关解析流。 */
    std::stringstream ssViewer((argc > 3) ? argv[3] : "true");
    /** @brief 是否启用 ORB_SLAM3 可视化窗口。 */
    bool visualization = true;
    ssViewer >> std::boolalpha >> visualization;

    rclcpp::init(argc, argv);

    /** @brief ORB_SLAM3 单目惯性系统实例，负责跟踪、建图和轨迹保存。 */
    ORB_SLAM3::System SLAM(vocabularyFile, settingsFile, ORB_SLAM3::System::IMU_MONOCULAR, visualization);

    {
        /** @brief ROS 2 单目惯性节点，作用域结束时先于 ROS shutdown 释放。 */
        auto node = std::make_shared<MonocularInertialNode>(&SLAM, settingsFile);
        std::cout << "============================" << std::endl;
        std::cout << "Monocular-Inertial" << std::endl;

        rclcpp::spin(node);
    }

    if (rclcpp::ok())
    {
        rclcpp::shutdown();
    }

    return 0;
}
