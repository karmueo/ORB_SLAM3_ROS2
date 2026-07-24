/**
 * @file monocular-slam-node.cpp
 * @brief 实现支持特征掩膜、位姿、轨迹和 TF 输出的 ORB-SLAM3 ROS 2 纯单目节点。
 */

#include "monocular-slam-node.hpp"

#include <cstdint>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>

#include <opencv2/core/core.hpp>

#include "Tracking.h"
#include "monocular-feature-mask.hpp"

using std::placeholders::_1;

namespace
{
/** @brief 与 ORB-SLAM3 初始化首帧相机坐标系重合的 ROS 参考 frame 名称。 */
constexpr const char* kCameraStartFrameId = "camera_start";
/** @brief ROS 标准相机机体坐标系名称。 */
constexpr const char* kCameraLinkFrameId = "camera_link";
/** @brief ORB/OpenCV 相机光学坐标系对应的 ROS frame 名称。 */
constexpr const char* kCameraOpticalFrameId = "camera_optical_frame";
/** @brief 默认保留的有效相机位姿数量。 */
constexpr std::int64_t kDefaultMaxPathLength = 10000;
}  // namespace

/**
 * @brief 创建纯单目 SLAM 节点，加载特征掩膜并初始化定位输出。
 * @param pSLAM 已初始化的 ORB-SLAM3 系统，生命周期必须覆盖本节点。
 * @throws std::runtime_error 掩膜文件无法读取时抛出。
 * @throws std::invalid_argument SLAM 指针、掩膜或 max_path_length 无效时抛出。
 */
MonocularSlamNode::MonocularSlamNode(ORB_SLAM3::System* pSLAM)
: Node("ORB_SLAM3_ROS2"),
  m_SLAM(pSLAM),
  m_feature_mask_size_validated(false),
  m_publish_ros_pose(true),
  m_max_path_length(0U)
{
    if (m_SLAM == nullptr)
    {
        throw std::invalid_argument("ORB-SLAM3 system pointer must not be null");
    }

    /** @brief 静态特征掩膜文件路径，空路径表示禁用。 */
    const std::string feature_mask_path =
        this->declare_parameter<std::string>("feature_mask_path", "");
    m_feature_mask = orbslam3_ros2::LoadFeatureMask(feature_mask_path);
    m_publish_ros_pose =
        this->declare_parameter<bool>("publish_ros_pose", true);

    /** @brief ROS 参数中配置的最大轨迹长度。 */
    const std::int64_t configured_max_path_length =
        this->declare_parameter<std::int64_t>(
            "max_path_length", kDefaultMaxPathLength);
    m_max_path_length = ValidateMaxPathLength(configured_max_path_length);

    if (m_feature_mask.empty())
    {
        RCLCPP_INFO(this->get_logger(), "Monocular feature mask is disabled");
    }
    else
    {
        /** @brief 掩膜排除区域的百分比。 */
        const double excluded_percent =
            orbslam3_ros2::CalculateExcludedRatio(m_feature_mask) * 100.0;
        RCLCPP_INFO(
            this->get_logger(),
            "Loaded monocular feature mask: path=%s, size=%dx%d, excluded=%.2f%%",
            feature_mask_path.c_str(),
            m_feature_mask.cols,
            m_feature_mask.rows,
            excluded_percent);
    }

    if (m_publish_ros_pose)
    {
        m_pose_publisher = this->create_publisher<PoseStampedMsg>(
            "/orbslam3/camera_pose", 10);
        m_path_publisher = this->create_publisher<PathMsg>(
            "/orbslam3/camera_path", rclcpp::QoS(1).reliable());
        m_dynamic_tf_broadcaster =
            std::make_unique<tf2_ros::TransformBroadcaster>(*this);
        m_static_tf_broadcaster =
            std::make_unique<tf2_ros::StaticTransformBroadcaster>(*this);
        m_camera_path.header.frame_id = kCameraStartFrameId;

        /** @brief 静态 TF 使用的节点时钟时间戳。 */
        const builtin_interfaces::msg::Time static_transform_stamp =
            this->now();
        /** @brief camera_optical_frame 在 camera_link 中的固定姿态消息。 */
        const PoseStampedMsg optical_pose_message = CreateCameraPoseMessage(
            ComputeCameraOpticalPoseInCameraLink(),
            static_transform_stamp,
            kCameraLinkFrameId);
        /** @brief camera_link 到 camera_optical_frame 的静态 TF 消息。 */
        const geometry_msgs::msg::TransformStamped optical_transform_message =
            CreateCameraTransformMessage(
                optical_pose_message, kCameraOpticalFrameId);
        m_static_tf_broadcaster->sendTransform(optical_transform_message);

        RCLCPP_INFO(
            this->get_logger(),
            "Monocular ROS pose output is enabled: "
            "pose=/orbslam3/camera_pose, path=/orbslam3/camera_path, "
            "dynamic_tf=%s->%s, static_tf=%s->%s, "
            "axes=x-forward/y-left/z-up, max_path_length=%zu",
            kCameraStartFrameId,
            kCameraLinkFrameId,
            kCameraLinkFrameId,
            kCameraOpticalFrameId,
            m_max_path_length);
    }
    else
    {
        RCLCPP_INFO(
            this->get_logger(),
            "Monocular ROS pose output is disabled by publish_ros_pose=false");
    }

    m_image_subscriber = this->create_subscription<ImageMsg>(
        "camera",
        10,
        std::bind(&MonocularSlamNode::GrabImage, this, _1));
}

/** @brief 停止 SLAM 后端并保存关键帧轨迹。 */
MonocularSlamNode::~MonocularSlamNode()
{
    m_SLAM->Shutdown();
    m_SLAM->SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");
}

/**
 * @brief 接收图像、校验掩膜尺寸、执行跟踪并按状态发布定位结果。
 * @param msg 输入图像消息。
 */
void MonocularSlamNode::GrabImage(const ImageMsg::SharedPtr msg)
{
    try
    {
        m_cvImPtr = cv_bridge::toCvCopy(msg);
    }
    catch (const cv_bridge::Exception& exception)
    {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", exception.what());
        return;
    }

    if (!m_feature_mask.empty() && !m_feature_mask_size_validated)
    {
        /** @brief 首帧掩膜尺寸校验失败时的详细说明。 */
        std::string error_message;
        if (!orbslam3_ros2::ValidateFeatureMaskSize(
                m_feature_mask, m_cvImPtr->image.size(), &error_message))
        {
            RCLCPP_FATAL(this->get_logger(), "%s", error_message.c_str());
            rclcpp::shutdown();
            return;
        }
        m_feature_mask_size_validated = true;
        RCLCPP_INFO(this->get_logger(), "Monocular feature mask size validation passed");
    }

    /** @brief 当前图像时间戳，单位为秒。 */
    const double timestamp_seconds = Utility::StampToSec(msg->header.stamp);
    /** @brief ORB-SLAM3 返回的世界到相机位姿。 */
    Sophus::SE3f Tcw;
    try
    {
        Tcw = m_SLAM->TrackMonocular(
            m_cvImPtr->image, m_feature_mask, timestamp_seconds);
    }
    catch (const cv::Exception& exception)
    {
        RCLCPP_FATAL(this->get_logger(), "ORB-SLAM3 feature mask error: %s", exception.what());
        rclcpp::shutdown();
        return;
    }
    if (!m_publish_ros_pose)
    {
        return;
    }

    /** @brief 当前帧跟踪完成后的 ORB-SLAM3 状态。 */
    const int tracking_state = m_SLAM->GetTrackingState();
    if (tracking_state == ORB_SLAM3::Tracking::LOST)
    {
        m_path_reset_state.MarkTrackingLost();
        return;
    }
    if (tracking_state != ORB_SLAM3::Tracking::OK &&
        tracking_state != ORB_SLAM3::Tracking::OK_KLT)
    {
        return;
    }

    /** @brief LOST 后恢复有效跟踪时是否需要清空旧轨迹。 */
    const bool reset_path = m_path_reset_state.ConsumeResetRequest();
    if (reset_path)
    {
        RCLCPP_WARN(
            this->get_logger(),
            "Monocular tracking recovered after LOST; reset published camera path");
    }
    PublishCameraPose(Tcw, msg->header.stamp, reset_path);
}

/**
 * @brief 发布 ROS camera_start 系下的 camera_link 位姿、轨迹和动态 TF。
 * @param Tcw ORB-SLAM3 返回的世界到相机位姿。
 * @param stamp 当前输入图像时间戳。
 * @param reset_path 发布前是否清空历史轨迹。
 */
void MonocularSlamNode::PublishCameraPose(
    const Sophus::SE3f& Tcw,
    const builtin_interfaces::msg::Time& stamp,
    bool reset_path)
{
    /** @brief ORB 世界系下的当前相机光学位姿。 */
    const Sophus::SE3f Twc = ComputeCameraPoseInWorld(Tcw);
    /** @brief ROS camera_start 系下 x 前、y 左、z 上的 camera_link 位姿。 */
    const Sophus::SE3f camera_start_from_camera_link =
        ConvertOrbCameraPoseToRos(Twc);
    /** @brief 当前帧 ROS camera_link 位姿消息。 */
    const PoseStampedMsg pose_message =
        CreateCameraPoseMessage(
            camera_start_from_camera_link, stamp, kCameraStartFrameId);
    UpdateCameraPath(
        pose_message, m_max_path_length, reset_path, &m_camera_path);
    /** @brief 与当前相机位姿严格一致的动态 TF 消息。 */
    const geometry_msgs::msg::TransformStamped transform_message =
        CreateCameraTransformMessage(pose_message, kCameraLinkFrameId);

    m_pose_publisher->publish(pose_message);
    m_path_publisher->publish(m_camera_path);
    m_dynamic_tf_broadcaster->sendTransform(transform_message);
}
