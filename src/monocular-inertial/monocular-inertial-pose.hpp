/**
 * @file monocular-inertial-pose.hpp
 * @brief 声明单目惯性节点使用的位姿转换与标定读取辅助函数。
 */

#ifndef __MONOCULAR_INERTIAL_POSE_HPP__
#define __MONOCULAR_INERTIAL_POSE_HPP__

#include "builtin_interfaces/msg/time.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "sophus/se3.hpp"

#include <cstddef>
#include <cstdint>
#include <string>

/**
 * @brief 记录单目惯性跟踪丢失后的轨迹重置请求。
 */
class BodyPathResetState
{
public:
    /** @brief 标记跟踪已丢失，下一次有效位姿发布前需要清空旧轨迹。 */
    void MarkTrackingLost();

    /**
     * @brief 读取并清除轨迹重置请求。
     * @return 存在待处理的重置请求时返回 true。
     */
    bool ConsumeResetRequest();

private:
    /** @brief 是否等待在下一次有效位姿发布前清空轨迹。 */
    bool resetPending_ = false;
};

/**
 * @brief 更新惯性 BA1 位姿发布稳定门控。
 * @param ba1Initialized 当前地图是否已经完成惯性 BA1。
 * @param timestampSec 当前图像时间戳，单位为秒。
 * @param settlingDurationSec BA1 需要连续保持完成状态的时长，单位为秒。
 * @param ba1InitializedSinceSec BA1 首次被观察为完成的时间戳；未开始时为负数。
 * @return BA1 已连续完成指定时长时返回 true。
 * @throws std::invalid_argument 时间参数无效或状态指针为空时抛出。
 */
bool UpdateInertialBa1PublishGate(
    bool ba1Initialized,
    double timestampSec,
    double settlingDurationSec,
    double* ba1InitializedSinceSec);

/**
 * @brief 根据 ORB_SLAM3 相机位姿和相机到机体外参计算机体在世界系中的位姿。
 * @param Tcw 世界系到相机系位姿。
 * @param Tbc 相机系到机体系外参，对应 ORB_SLAM3 配置中的 IMU.T_b_c1 或 Tbc。
 * @return 机体系到世界系的位姿。
 */
Sophus::SE3f ComputeBodyPoseFromCameraPose(const Sophus::SE3f& Tcw, const Sophus::SE3f& Tbc);

/**
 * @brief 构造四元数归一化后的 ROS 位姿消息。
 * @param poseInParent 子坐标系在父坐标系中的位姿。
 * @param stamp 消息时间戳。
 * @param frameId 父坐标系名称。
 * @return ROS 位姿消息。
 */
geometry_msgs::msg::PoseStamped CreateBodyPoseMessage(
    const Sophus::SE3f& poseInParent,
    const builtin_interfaces::msg::Time& stamp,
    const std::string& frameId);

/**
 * @brief 从 ROS 位姿构造具有相同变换的 TF 消息。
 * @param poseMessage 父坐标系中的位姿消息。
 * @param childFrameId 子坐标系名称。
 * @return ROS TF 消息。
 */
geometry_msgs::msg::TransformStamped CreateBodyTransformMessage(
    const geometry_msgs::msg::PoseStamped& poseMessage,
    const std::string& childFrameId);

/**
 * @brief 更新累计机体轨迹，并按配置限制轨迹长度。
 * @param poseMessage 当前机体位姿。
 * @param maxPathLength 最大轨迹长度；0 表示无限累计。
 * @param resetPath 是否在加入当前位姿前清空旧轨迹。
 * @param pathMessage 待更新的轨迹消息。
 * @throws std::invalid_argument pathMessage 为空时抛出。
 */
void UpdateBodyPath(
    const geometry_msgs::msg::PoseStamped& poseMessage,
    std::size_t maxPathLength,
    bool resetPath,
    nav_msgs::msg::Path* pathMessage);

/**
 * @brief 校验并转换最大轨迹长度配置。
 * @param configuredLength ROS 参数中的轨迹长度。
 * @return 可用于容器长度比较的无符号值。
 * @throws std::invalid_argument configuredLength 为负数时抛出。
 */
std::size_t ValidateBodyMaxPathLength(std::int64_t configuredLength);

/**
 * @brief 从 ORB_SLAM3 配置文件读取相机系到机体系外参。
 * @param settingsFile ORB_SLAM3 YAML 配置文件路径。
 * @param Tbc 输出的相机系到机体系外参。
 * @param errorMessage 读取失败时输出错误说明；可为空。
 * @return 读取成功返回 true，否则返回 false。
 */
bool LoadBodyToCameraExtrinsic(
    const std::string& settingsFile,
    Sophus::SE3f& Tbc,
    std::string* errorMessage);

#endif
