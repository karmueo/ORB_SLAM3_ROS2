/**
 * @file stereo-inertial-node.hpp
 * @brief 声明双目惯性 ORB_SLAM3 ROS 2 节点，负责订阅图像/IMU 并驱动跟踪线程。
 */

#ifndef __STEREO_INERTIAL_NODE_HPP__
#define __STEREO_INERTIAL_NODE_HPP__

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/image_encodings.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/imu.hpp"

#include <cv_bridge/cv_bridge.hpp>

#include "System.h"
#include "Frame.h"
#include "Map.h"
#include "Tracking.h"

#include "utility.hpp"

#include <atomic>

/** @brief ROS 2 IMU 消息类型别名。 */
using ImuMsg = sensor_msgs::msg::Imu;
/** @brief ROS 2 图像消息类型别名。 */
using ImageMsg = sensor_msgs::msg::Image;

/**
 * @brief 双目惯性 SLAM 节点。
 *
 * 节点订阅左右目图像和 IMU 数据，在后台线程中按时间戳同步后调用 ORB_SLAM3。
 */
class StereoInertialNode : public rclcpp::Node
{
public:
    /**
     * @brief 创建双目惯性 SLAM 节点并启动同步线程。
     * @param pSLAM ORB_SLAM3 系统实例指针，生命周期由调用方管理。
     * @param strSettingsFile 相机和 IMU 配置文件路径。
     * @param strDoRectify 是否对双目图像做去畸变矫正的字符串开关。
     * @param strDoEqual 是否对图像做 CLAHE 均衡化的字符串开关。
     */
    StereoInertialNode(ORB_SLAM3::System* pSLAM, const string &strSettingsFile, const string &strDoRectify, const string &strDoEqual);

    /**
     * @brief 停止同步线程，关闭 ORB_SLAM3 并保存关键帧轨迹。
     */
    ~StereoInertialNode();

private:
    /**
     * @brief 缓存一帧 IMU 消息。
     * @param msg IMU 消息共享指针。
     */
    void GrabImu(const ImuMsg::SharedPtr msg);

    /**
     * @brief 缓存一帧左目图像消息。
     * @param msgLeft 左目图像消息共享指针。
     */
    void GrabImageLeft(const ImageMsg::SharedPtr msgLeft);

    /**
     * @brief 缓存一帧右目图像消息。
     * @param msgRight 右目图像消息共享指针。
     */
    void GrabImageRight(const ImageMsg::SharedPtr msgRight);

    /**
     * @brief 将 ROS 图像消息转换为 OpenCV 灰度图。
     * @param msg 图像消息共享指针。
     * @return 复制出的 OpenCV 图像矩阵。
     */
    cv::Mat GetImage(const ImageMsg::SharedPtr msg);

    /**
     * @brief 后台同步左右目图像和 IMU 数据，并调用 ORB_SLAM3 跟踪。
     */
    void SyncWithImu();

    /** @brief IMU 订阅器。 */
    rclcpp::Subscription<ImuMsg>::SharedPtr   subImu_;
    /** @brief 左目图像订阅器。 */
    rclcpp::Subscription<ImageMsg>::SharedPtr subImgLeft_;
    /** @brief 右目图像订阅器。 */
    rclcpp::Subscription<ImageMsg>::SharedPtr subImgRight_;

    /** @brief ORB_SLAM3 系统实例指针，节点不拥有其生命周期。 */
    ORB_SLAM3::System *SLAM_;
    /** @brief 图像和 IMU 同步线程指针。 */
    std::thread *syncThread_;
    /** @brief 同步线程停止标志，用于析构时安全退出。 */
    std::atomic<bool> stopSync_;

    /** @brief IMU 消息缓存队列。 */
    queue<ImuMsg::SharedPtr> imuBuf_;
    /** @brief IMU 缓存队列互斥锁。 */
    std::mutex bufMutex_;

    /** @brief 左右目图像消息缓存队列。 */
    queue<ImageMsg::SharedPtr> imgLeftBuf_, imgRightBuf_;
    /** @brief 左右目图像缓存队列互斥锁。 */
    std::mutex bufMutexLeft_, bufMutexRight_;

    /** @brief 是否对双目图像做去畸变矫正。 */
    bool doRectify_;
    /** @brief 是否对图像做均衡化。 */
    bool doEqual_;
    /** @brief 左右目去畸变和矫正映射矩阵。 */
    cv::Mat M1l_, M2l_, M1r_, M2r_;

    /** @brief 是否启用 CLAHE 均衡化。 */
    bool bClahe_;
    /** @brief CLAHE 均衡化处理器。 */
    cv::Ptr<cv::CLAHE> clahe_ = cv::createCLAHE(3.0, cv::Size(8, 8));
};

#endif
