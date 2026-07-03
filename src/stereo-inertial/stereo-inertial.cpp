/**
 * @file stereo-inertial.cpp
 * @brief 启动双目惯性 ORB_SLAM3 ROS 2 节点，并管理 ROS 与 SLAM 系统生命周期。
 */

#include <iostream>
#include <algorithm>
#include <fstream>
#include <chrono>
#include <sstream>

#include "rclcpp/rclcpp.hpp"
#include "stereo-inertial-node.hpp"

#include "System.h"

/**
 * @brief stereo-inertial 可执行入口。
 * @param argc 命令行参数数量。
 * @param argv 命令行参数数组，依次包含词典、配置文件、是否矫正、可选的是否均衡化和可选的是否启用 viewer。
 * @return 进程退出码，0 表示正常退出，非 0 表示参数错误或运行失败。
 */
int main(int argc, char **argv)
{
    if(argc < 4)
    {
        std::cerr << "\nUsage: ros2 run orbslam stereo path_to_vocabulary path_to_settings do_rectify [do_equalize] [use_viewer]" << std::endl;
        return 1;
    }

    /** @brief 图像均衡化开关，未传入时默认关闭。 */
    const std::string doEqual = (argc > 4) ? argv[4] : "false";

    /** @brief ORB_SLAM3 viewer 开关解析流。 */
    std::stringstream ss_viewer((argc > 5) ? argv[5] : "true");
    /** @brief 是否启用 ORB_SLAM3 可视化窗口。 */
    bool visualization = true;
    ss_viewer >> std::boolalpha >> visualization;

    rclcpp::init(argc, argv);

    /** @brief ORB_SLAM3 双目惯性系统实例，负责跟踪、建图和轨迹保存。 */
    ORB_SLAM3::System pSLAM(argv[1], argv[2], ORB_SLAM3::System::IMU_STEREO, visualization);

    {
        /** @brief ROS 2 双目惯性节点，作用域结束时先于 ROS shutdown 释放。 */
        auto node = std::make_shared<StereoInertialNode>(&pSLAM, argv[2], argv[3], doEqual);
        std::cout << "============================" << std::endl;

        rclcpp::spin(node);
    }

    if (rclcpp::ok())
    {
        rclcpp::shutdown();
    }

    return 0;
}
