/**
 * @file monocular-inertial-node.hpp
 * @brief 声明单目惯性 ORB_SLAM3 ROS 2 节点，负责订阅单目图像和 IMU 并驱动跟踪。
 */

#ifndef __MONOCULAR_INERTIAL_NODE_HPP__
#define __MONOCULAR_INERTIAL_NODE_HPP__

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/image_encodings.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/imu.hpp"

#include <cv_bridge/cv_bridge.hpp>

#include "Frame.h"
#include "Map.h"
#include "System.h"
#include "Tracking.h"

#include "monocular-inertial-pose.hpp"
#include "monocular-inertial-sync.hpp"
#include "utility.hpp"

#include <atomic>
#include <cstdint>
#include <mutex>
#include <queue>
#include <thread>
#include <vector>

/** @brief ROS 2 IMU 消息类型别名。 */
using ImuMsg = sensor_msgs::msg::Imu;
/** @brief ROS 2 图像消息类型别名。 */
using ImageMsg = sensor_msgs::msg::Image;
/** @brief ROS 2 位姿消息类型别名。 */
using PoseStampedMsg = geometry_msgs::msg::PoseStamped;
/** @brief ROS 2 轨迹消息类型别名。 */
using PathMsg = nav_msgs::msg::Path;

/**
 * @brief 单目惯性 SLAM 节点。
 *
 * 节点订阅单目图像和 IMU 数据，在后台线程中按图像时间戳取出 IMU 测量，
 * 并调用 ORB_SLAM3 的 IMU_MONOCULAR 跟踪入口。
 */
class MonocularInertialNode : public rclcpp::Node
{
public:
    /**
     * @brief 创建单目惯性 SLAM 节点并启动图像/IMU 同步线程。
     * @param pSLAM ORB_SLAM3 系统实例指针，生命周期由调用方管理。
     * @param settingsFile ORB_SLAM3 配置文件路径，用于读取 IMU 到相机外参。
     */
    MonocularInertialNode(ORB_SLAM3::System* pSLAM, const std::string& settingsFile);

    /**
     * @brief 停止同步线程，关闭 ORB_SLAM3 并保存关键帧轨迹。
     */
    ~MonocularInertialNode();

private:
    /**
     * @brief 缓存一帧 IMU 消息。
     * @param msg IMU 消息共享指针。
     */
    void GrabImu(const ImuMsg::SharedPtr msg);

    /**
     * @brief 缓存一帧单目图像消息。
     * @param msg 图像消息共享指针。
     */
    void GrabImage(const ImageMsg::SharedPtr msg);

    /**
     * @brief 将 ROS 图像消息转换为 OpenCV 灰度图。
     * @param msg 图像消息共享指针。
     * @return 复制出的 OpenCV 图像矩阵；转换失败时返回空矩阵。
     */
    cv::Mat GetImage(const ImageMsg::SharedPtr msg);

    /**
     * @brief 后台按图像时间戳取出 IMU 测量，并调用 ORB_SLAM3 跟踪。
     */
    void SyncWithImu();

    /**
     * @brief 在跟踪状态有效时发布当前机体系位姿和累计轨迹。
     * @param Tcw 世界系到相机系位姿。
     * @param stamp 当前图像时间戳。
     */
    void PublishBodyPose(const Sophus::SE3f& Tcw, const builtin_interfaces::msg::Time& stamp);

    /** @brief IMU 订阅器。 */
    rclcpp::Subscription<ImuMsg>::SharedPtr subImu_;
    /** @brief 单目图像订阅器。 */
    rclcpp::Subscription<ImageMsg>::SharedPtr subImg_;
    /** @brief 机体系实时位姿发布器。 */
    rclcpp::Publisher<PoseStampedMsg>::SharedPtr posePub_;
    /** @brief 机体系累计轨迹发布器。 */
    rclcpp::Publisher<PathMsg>::SharedPtr pathPub_;

    /** @brief ORB_SLAM3 系统实例指针，节点不拥有其生命周期。 */
    ORB_SLAM3::System* SLAM_;
    /** @brief 图像和 IMU 同步线程。 */
    std::thread syncThread_;
    /** @brief 同步线程停止标志，用于析构时安全退出。 */
    std::atomic<bool> stopSync_;

    /** @brief IMU 消息缓存队列。 */
    std::queue<ImuMsg::SharedPtr> imuBuf_;
    /** @brief IMU 缓存队列互斥锁。 */
    std::mutex imuMutex_;

    /** @brief 图像消息缓存队列。 */
    std::queue<ImageMsg::SharedPtr> imgBuf_;
    /** @brief 图像缓存队列互斥锁。 */
    std::mutex imgMutex_;
    /** @brief 图像缓存队列最大容量，防止跟踪线程落后时内存持续增长。 */
    std::size_t maxImageQueueSize_;
    /** @brief 最近一次处理的图像时间戳，用于丢弃回跳帧。 */
    double lastImageTimestamp_;
    /** @brief 最近一次接收的 IMU 时间戳，用于过滤回跳样本。 */
    double lastImuTimestamp_;
    /** @brief 已接收图像帧数量。 */
    std::atomic<std::uint64_t> receivedImageCount_;
    /** @brief 已送入 ORB_SLAM3 跟踪的图像帧数量。 */
    std::atomic<std::uint64_t> trackedImageCount_;
    /** @brief 因队列溢出、时间戳回跳或转换失败丢弃的图像帧数量。 */
    std::atomic<std::uint64_t> droppedImageCount_;
    /** @brief 因 IMU 时间戳回跳或重复被过滤的 IMU 样本数量。 */
    std::atomic<std::uint64_t> droppedImuCount_;
    /** @brief 因 IMU 尚未覆盖图像时间戳而等待的次数。 */
    std::atomic<std::uint64_t> imuWaitCount_;
    /** @brief 机体系到相机系外参，对应 ORB_SLAM3 配置中的 IMU.T_b_c1 或 Tbc。 */
    Sophus::SE3f Tbc_;
    /** @brief 累计发布的机体系轨迹。 */
    PathMsg pathMsg_;
};

#endif
