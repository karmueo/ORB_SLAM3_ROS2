/**
 * @file mono.cpp
 * @brief 启动纯单目 ORB-SLAM3 ROS 2 节点，并解析可选的 viewer 开关。
 */

#include <exception>
#include <memory>
#include <fstream>
#include <iostream>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "monocular-slam-node.hpp"

#include "System.h"

namespace
{
/** @brief 纯单目节点命令行用法。 */
constexpr const char* kUsage =
    "ros2 run orbslam3 mono path_to_vocabulary path_to_settings [use_viewer]";

/**
 * @brief 解析命令行中的 viewer 布尔开关。
 * @param argument 待解析字符串，支持 true、false、1 和 0。
 * @param visualization 输出 viewer 是否启用。
 * @return 参数合法且成功解析时返回 true。
 */
bool ParseViewerArgument(const char* const argument, bool* const visualization)
{
    if (!argument || !visualization)
    {
        return false;
    }

    /** @brief viewer 参数文本。 */
    const std::string viewerValue(argument);
    if (viewerValue == "true" || viewerValue == "1")
    {
        *visualization = true;
        return true;
    }
    if (viewerValue == "false" || viewerValue == "0")
    {
        *visualization = false;
        return true;
    }
    return false;
}
}  // namespace

/**
 * @brief 纯单目 ORB-SLAM3 ROS 2 可执行入口。
 * @param argc 命令行参数数量。
 * @param argv 命令行参数数组，依次包含词典、配置文件和可选的 viewer 开关。
 * @return 进程退出码，0 表示正常退出，非 0 表示参数错误。
 */
int main(int argc, char** argv)
{
    if (argc < 3)
    {
        std::cerr << "\nUsage: " << kUsage << std::endl;
        return 1;
    }

    /** @brief ORB-SLAM3 词典文件路径。 */
    const std::string vocabularyFile = argv[1] ? argv[1] : "";
    /** @brief ORB-SLAM3 配置文件路径。 */
    const std::string settingsFile = argv[2] ? argv[2] : "";
    if (vocabularyFile.empty() || settingsFile.empty())
    {
        std::cerr << "Error: vocabulary path and settings path must not be empty." << std::endl;
        std::cerr << "Usage: " << kUsage << std::endl;
        return 1;
    }
    if (!std::ifstream(vocabularyFile).good())
    {
        std::cerr << "Error: vocabulary file does not exist or is not readable: "
                  << vocabularyFile << std::endl;
        return 1;
    }
    if (!std::ifstream(settingsFile).good())
    {
        std::cerr << "Error: settings file does not exist or is not readable: "
                  << settingsFile << std::endl;
        return 1;
    }

    /** @brief 是否启用 ORB-SLAM3 Pangolin viewer，默认启用。 */
    bool visualization = true;
    /** @brief 第三个非 ROS 参数是否为显式 viewer 开关。 */
    const bool hasViewerArgument = argc > 3 && argv[3] && std::string(argv[3]) != "--ros-args";
    if (hasViewerArgument && !ParseViewerArgument(argv[3], &visualization))
    {
        std::cerr << "Error: use_viewer must be true, false, 1, or 0; got: "
                  << argv[3] << std::endl;
        std::cerr << "Usage: " << kUsage << std::endl;
        return 1;
    }

    rclcpp::init(argc, argv);

    /** @brief ORB-SLAM3 纯单目系统实例，负责跟踪、建图和轨迹保存。 */
    ORB_SLAM3::System SLAM(
        vocabularyFile,
        settingsFile,
        ORB_SLAM3::System::MONOCULAR,
        visualization);

    {
        /** @brief ROS 2 纯单目节点，负责加载可选掩膜并转发相机帧。 */
        std::shared_ptr<MonocularSlamNode> node;
        try
        {
            node = std::make_shared<MonocularSlamNode>(&SLAM);
        }
        catch (const std::exception& exception)
        {
            std::cerr << "Error: failed to initialize monocular node: "
                      << exception.what() << std::endl;
            SLAM.Shutdown();
            if (rclcpp::ok())
            {
                rclcpp::shutdown();
            }
            return 1;
        }
        std::cout << "============================" << std::endl;
        std::cout << "Monocular" << std::endl;

        rclcpp::spin(node);
    }

    if (rclcpp::ok())
    {
        rclcpp::shutdown();
    }

    return 0;
}
