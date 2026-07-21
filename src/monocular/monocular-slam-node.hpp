/**
 * @file monocular-slam-node.hpp
 * @brief 声明支持特征掩膜、位姿、轨迹和 TF 输出的 ORB-SLAM3 ROS 2 纯单目节点。
 */

#ifndef ORBSLAM3_ROS2_MONOCULAR_SLAM_NODE_HPP_
#define ORBSLAM3_ROS2_MONOCULAR_SLAM_NODE_HPP_

#include <cstddef>
#include <memory>
#include <string>

#include <cv_bridge/cv_bridge.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <opencv2/core/mat.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>

#include "System.h"

#include "monocular-pose.hpp"
#include "utility.hpp"

/**
 * @brief 订阅单目图像，清除掩膜排除区域，并发布相机位姿、轨迹和 TF。
 */
class MonocularSlamNode : public rclcpp::Node
{
public:
    /**
     * @brief 创建纯单目 SLAM 节点，加载特征掩膜并初始化定位输出。
     * @param pSLAM 已初始化的 ORB-SLAM3 系统，生命周期必须覆盖本节点。
     * @throws std::runtime_error 掩膜文件无法读取时抛出。
     * @throws std::invalid_argument SLAM 指针、掩膜或 max_path_length 无效时抛出。
     */
    explicit MonocularSlamNode(ORB_SLAM3::System* pSLAM);

    /** @brief 停止 SLAM 后端并保存关键帧轨迹。 */
    ~MonocularSlamNode() override;

private:
    /** @brief ROS 2 图像消息类型别名。 */
    using ImageMsg = sensor_msgs::msg::Image;
    /** @brief ROS 2 相机位姿消息类型别名。 */
    using PoseStampedMsg = geometry_msgs::msg::PoseStamped;
    /** @brief ROS 2 累计轨迹消息类型别名。 */
    using PathMsg = nav_msgs::msg::Path;

    /**
     * @brief 接收图像、校验掩膜尺寸、执行跟踪并按状态发布定位结果。
     * @param msg 输入图像消息。
     */
    void GrabImage(const ImageMsg::SharedPtr msg);

    /**
     * @brief 发布 ROS map 系下的 camera_link 位姿、轨迹和动态 TF。
     * @param Tcw ORB-SLAM3 返回的世界到相机位姿。
     * @param stamp 当前输入图像时间戳。
     * @param reset_path 发布前是否清空历史轨迹。
     */
    void PublishCameraPose(
        const Sophus::SE3f& Tcw,
        const builtin_interfaces::msg::Time& stamp,
        bool reset_path);

    /** @brief ORB-SLAM3 系统指针，不拥有其生命周期。 */
    ORB_SLAM3::System* m_SLAM;

    /** @brief 当前 ROS 图像转换得到的 OpenCV 图像。 */
    cv_bridge::CvImagePtr m_cvImPtr;

    /** @brief 原始输入分辨率下的静态二值特征掩膜。 */
    cv::Mat m_feature_mask;

    /** @brief 是否已使用首帧完成掩膜尺寸校验。 */
    bool m_feature_mask_size_validated;

    /** @brief ROS 轨迹允许保留的最大有效位姿数量，0 表示无限累计。 */
    std::size_t m_max_path_length;

    /** @brief 相机图像订阅器。 */
    rclcpp::Subscription<ImageMsg>::SharedPtr m_image_subscriber;

    /** @brief ROS map 系下实时 camera_link 位姿发布器。 */
    rclcpp::Publisher<PoseStampedMsg>::SharedPtr m_pose_publisher;

    /** @brief 世界系下有界相机轨迹发布器。 */
    rclcpp::Publisher<PathMsg>::SharedPtr m_path_publisher;

    /** @brief map 到 camera_link 的动态 TF 广播器。 */
    std::unique_ptr<tf2_ros::TransformBroadcaster> m_dynamic_tf_broadcaster;

    /** @brief camera_link 到 camera_optical_frame 的静态 TF 广播器。 */
    std::unique_ptr<tf2_ros::StaticTransformBroadcaster> m_static_tf_broadcaster;

    /** @brief 进程内累计的相机轨迹消息。 */
    PathMsg m_camera_path;

    /** @brief LOST 后下一次有效位姿是否需要清空轨迹的状态。 */
    CameraPathResetState m_path_reset_state;
};

#endif  // ORBSLAM3_ROS2_MONOCULAR_SLAM_NODE_HPP_
