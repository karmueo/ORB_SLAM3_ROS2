/**
 * @file monocular-inertial-pose.cpp
 * @brief 实现单目惯性节点使用的位姿转换与标定读取辅助函数。
 */

#include "monocular-inertial-pose.hpp"

#include <opencv2/core/core.hpp>

#include <cmath>
#include <stdexcept>

namespace
{
/**
 * @brief 将 OpenCV 4x4 浮点矩阵转换为 Sophus SE3。
 * @param matrix OpenCV 4x4 变换矩阵。
 * @return Sophus SE3 位姿。
 */
Sophus::SE3f CvMatrixToSophus(const cv::Mat& matrix)
{
    /** @brief 按行存储的 Eigen 4x4 变换矩阵，用于匹配 OpenCV 内存布局。 */
    Eigen::Matrix<float, 4, 4, Eigen::RowMajor> eigenMatrix(matrix.ptr<float>(0));
    return Sophus::SE3f(eigenMatrix);
}
}  // namespace

/** @brief 标记跟踪已丢失，下一次有效位姿发布前需要清空旧轨迹。 */
void BodyPathResetState::MarkTrackingLost()
{
    resetPending_ = true;
}

/** @brief 读取并清除轨迹重置请求。 */
bool BodyPathResetState::ConsumeResetRequest()
{
    /** @brief 本次调用需要返回的重置状态。 */
    const bool shouldReset = resetPending_;
    resetPending_ = false;
    return shouldReset;
}

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
    const bool ba1Initialized,
    const double timestampSec,
    const double settlingDurationSec,
    double* const ba1InitializedSinceSec)
{
    if (!ba1InitializedSinceSec)
    {
        throw std::invalid_argument("BA1 initialization timestamp pointer must not be null");
    }
    if (!std::isfinite(timestampSec) ||
        !std::isfinite(settlingDurationSec) ||
        settlingDurationSec < 0.0)
    {
        throw std::invalid_argument("BA1 publish gate time parameters must be finite and non-negative");
    }
    if (!ba1Initialized)
    {
        *ba1InitializedSinceSec = -1.0;
        return false;
    }
    if (*ba1InitializedSinceSec < 0.0 || timestampSec < *ba1InitializedSinceSec)
    {
        *ba1InitializedSinceSec = timestampSec;
    }
    return timestampSec - *ba1InitializedSinceSec >= settlingDurationSec;
}

/**
 * @brief 根据 ORB_SLAM3 相机位姿和相机到机体外参计算机体在世界系中的位姿。
 * @param Tcw 世界系到相机系位姿。
 * @param Tbc 相机系到机体系外参，对应 ORB_SLAM3 配置中的 IMU.T_b_c1 或 Tbc。
 * @return 机体系到世界系的位姿。
 */
Sophus::SE3f ComputeBodyPoseFromCameraPose(const Sophus::SE3f& Tcw, const Sophus::SE3f& Tbc)
{
    /** @brief 世界系到相机系位姿的逆变换，即相机系到世界系位姿。 */
    const Sophus::SE3f Twc = Tcw.inverse();
    /** @brief 相机系到机体系外参的逆变换，即机体系到相机系外参。 */
    const Sophus::SE3f Tcb = Tbc.inverse();
    return Twc * Tcb;
}

/** @brief 构造四元数归一化后的 ROS 位姿消息。 */
geometry_msgs::msg::PoseStamped CreateBodyPoseMessage(
    const Sophus::SE3f& poseInParent,
    const builtin_interfaces::msg::Time& stamp,
    const std::string& frameId)
{
    /** @brief 位姿对应的归一化四元数。 */
    Eigen::Quaternionf quaternion = poseInParent.unit_quaternion();
    quaternion.normalize();
    /** @brief 子坐标系在父坐标系中的平移。 */
    const Eigen::Vector3f translation = poseInParent.translation();
    /** @brief 待返回的 ROS 位姿消息。 */
    geometry_msgs::msg::PoseStamped poseMessage;
    poseMessage.header.stamp = stamp;
    poseMessage.header.frame_id = frameId;
    poseMessage.pose.position.x = translation.x();
    poseMessage.pose.position.y = translation.y();
    poseMessage.pose.position.z = translation.z();
    poseMessage.pose.orientation.x = quaternion.x();
    poseMessage.pose.orientation.y = quaternion.y();
    poseMessage.pose.orientation.z = quaternion.z();
    poseMessage.pose.orientation.w = quaternion.w();
    return poseMessage;
}

/** @brief 从 ROS 位姿构造具有相同变换的 TF 消息。 */
geometry_msgs::msg::TransformStamped CreateBodyTransformMessage(
    const geometry_msgs::msg::PoseStamped& poseMessage,
    const std::string& childFrameId)
{
    /** @brief 待返回的 ROS TF 消息。 */
    geometry_msgs::msg::TransformStamped transformMessage;
    transformMessage.header = poseMessage.header;
    transformMessage.child_frame_id = childFrameId;
    transformMessage.transform.translation.x = poseMessage.pose.position.x;
    transformMessage.transform.translation.y = poseMessage.pose.position.y;
    transformMessage.transform.translation.z = poseMessage.pose.position.z;
    transformMessage.transform.rotation = poseMessage.pose.orientation;
    return transformMessage;
}

/** @brief 更新累计机体轨迹，并按配置限制轨迹长度。 */
void UpdateBodyPath(
    const geometry_msgs::msg::PoseStamped& poseMessage,
    const std::size_t maxPathLength,
    const bool resetPath,
    nav_msgs::msg::Path* const pathMessage)
{
    if (pathMessage == nullptr)
    {
        throw std::invalid_argument("机体轨迹消息指针不能为空");
    }
    if (resetPath)
    {
        pathMessage->poses.clear();
    }

    pathMessage->header = poseMessage.header;
    pathMessage->poses.push_back(poseMessage);
    if (maxPathLength > 0U && pathMessage->poses.size() > maxPathLength)
    {
        /** @brief 超出配置上限、需要从轨迹开头删除的位姿数量。 */
        const std::size_t excessCount = pathMessage->poses.size() - maxPathLength;
        pathMessage->poses.erase(
            pathMessage->poses.begin(),
            pathMessage->poses.begin() + excessCount);
    }
}

/** @brief 校验并转换最大轨迹长度配置。 */
std::size_t ValidateBodyMaxPathLength(const std::int64_t configuredLength)
{
    if (configuredLength < 0)
    {
        throw std::invalid_argument("max_path_length 不能为负数");
    }
    return static_cast<std::size_t>(configuredLength);
}

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
    std::string* errorMessage)
{
    /** @brief ORB_SLAM3 配置文件读取器。 */
    cv::FileStorage fileStorage(settingsFile, cv::FileStorage::READ);
    if (!fileStorage.isOpened())
    {
        if (errorMessage)
        {
            *errorMessage = "cannot open settings file";
        }
        return false;
    }

    /** @brief 新版 ORB_SLAM3 配置中的相机系到机体系外参节点。 */
    cv::FileNode extrinsicNode = fileStorage["IMU.T_b_c1"];
    if (extrinsicNode.empty())
    {
        extrinsicNode = fileStorage["Tbc"];
    }

    if (extrinsicNode.empty())
    {
        if (errorMessage)
        {
            *errorMessage = "missing IMU.T_b_c1 or Tbc";
        }
        return false;
    }

    /** @brief 从 YAML 读取出的相机系到机体系外参矩阵。 */
    const cv::Mat cvTbc = extrinsicNode.mat();
    if (cvTbc.rows != 4 || cvTbc.cols != 4 || cvTbc.type() != CV_32F)
    {
        if (errorMessage)
        {
            *errorMessage = "extrinsic matrix must be 4x4 float";
        }
        return false;
    }

    Tbc = CvMatrixToSophus(cvTbc);
    return true;
}
