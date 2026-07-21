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

    /**
     * @brief 查询节点是否因不可恢复的运行时错误停止。
     * @return 发生图像/IMU 时基不一致等致命错误时返回 true。
     */
    bool HasFatalError() const;

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
     * @return 当前帧成功发布机体系位姿时返回 true。
     */
    bool PublishBodyPose(const Sophus::SE3f& Tcw, const builtin_interfaces::msg::Time& stamp);

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
    /** @brief ROS wrapper 主动送入 SLAM 的目标图像帧率，单位 Hz；小于等于 0 时关闭限帧。 */
    double targetImageFps_;
    /** @brief 图像平均帧率限制状态，用于避免固定间隔限帧在略高输入帧率下退化。 */
    MonocularImageRateLimitState imageRateLimitState_;
    /** @brief 最近一次处理的图像时间戳，用于丢弃回跳帧。 */
    double lastImageTimestamp_;
    /** @brief 最近一次接收的 IMU 时间戳，用于过滤回跳样本。 */
    double lastImuTimestamp_;
    /** @brief 应用于 IMU 消息时间戳的固定偏移量，单位为秒。 */
    double imuTimeOffsetSec_;
    /** @brief 已观察到的最新有限原始 IMU 时间戳，单位为秒；回跳样本不会覆盖该值。 */
    double latestRawImuTimestamp_;
    /** @brief 是否已确认图像与 IMU 的原始时间戳使用同一时间基准。 */
    std::atomic<bool> timeBaseValidated_;
    /** @brief 是否发生需要终止整个进程的运行时错误。 */
    std::atomic<bool> fatalError_;
    /** @brief 图像与最近 IMU 时间戳允许的最大启动差值，单位为秒。 */
    static constexpr double kMaxTimeBaseDifferenceSec = 1.0;
    /** @brief IMU 小窗口重排长度，单位为秒；小于等于 0 时关闭重排。 */
    double imuReorderWindowSec_;
    /** @brief IMU 小窗口重排状态，用于缓存尚未越过窗口的样本。 */
    MonocularImuReorderState imuReorderState_;
    /** @brief 已接收图像帧数量。 */
    std::atomic<std::uint64_t> receivedImageCount_;
    /** @brief 已送入 ORB_SLAM3 跟踪的图像帧数量。 */
    std::atomic<std::uint64_t> trackedImageCount_;
    /** @brief 因队列溢出、时间戳回跳或转换失败丢弃的图像帧数量。 */
    std::atomic<std::uint64_t> droppedImageCount_;
    /** @brief 因 ROS wrapper 主动限帧被丢弃的图像帧数量。 */
    std::atomic<std::uint64_t> rateLimitedImageCount_;
    /** @brief 因图像队列达到容量上限被丢弃的最旧图像帧数量。 */
    std::atomic<std::uint64_t> queueOverflowDroppedImageCount_;
    /** @brief 因 IMU 时间戳回跳或重复被过滤的 IMU 样本数量。 */
    std::atomic<std::uint64_t> droppedImuCount_;
    /** @brief 经过时间偏移和小窗口重排后进入主 IMU 队列的样本数量。 */
    std::atomic<std::uint64_t> queuedImuCount_;
    /** @brief 因 IMU 尚未覆盖图像时间戳而等待的次数。 */
    std::atomic<std::uint64_t> imuWaitCount_;
    /** @brief 已成功发布的机体系位姿数量。 */
    std::atomic<std::uint64_t> posePublishCount_;
    /** @brief 机体系到相机系外参，对应 ORB_SLAM3 配置中的 IMU.T_b_c1 或 Tbc。 */
    Sophus::SE3f Tbc_;
    /** @brief 累计发布的机体系轨迹。 */
    PathMsg pathMsg_;
};

#endif
