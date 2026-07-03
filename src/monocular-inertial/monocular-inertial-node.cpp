/**
 * @file monocular-inertial-node.cpp
 * @brief 实现单目惯性 ORB_SLAM3 ROS 2 节点的数据缓存、同步和退出收尾逻辑。
 */

#include "monocular-inertial-node.hpp"

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <chrono>
#include <iostream>
#include <string>

using std::placeholders::_1;

/**
 * @brief 创建单目惯性 SLAM 节点并启动图像/IMU 同步线程。
 * @param SLAM ORB_SLAM3 系统实例指针，生命周期由调用方管理。
 * @param settingsFile ORB_SLAM3 配置文件路径，用于读取 IMU 到相机外参。
 */
MonocularInertialNode::MonocularInertialNode(ORB_SLAM3::System* SLAM, const std::string& settingsFile) :
    Node("ORB_SLAM3_ROS2"),
    SLAM_(SLAM),
    stopSync_(false),
    lastImageTimestamp_(-1.0),
    Tbc_()
{
    subImu_ = this->create_subscription<ImuMsg>("imu", 1000, std::bind(&MonocularInertialNode::GrabImu, this, _1));
    subImg_ = this->create_subscription<ImageMsg>("camera", 100, std::bind(&MonocularInertialNode::GrabImage, this, _1));
    posePub_ = this->create_publisher<PoseStampedMsg>("/orbslam3/body_pose", 10);
    pathPub_ = this->create_publisher<PathMsg>("/orbslam3/path", 10);
    pathMsg_.header.frame_id = "map";

    /** @brief 外参读取失败原因。 */
    std::string errorMessage;
    if (!LoadBodyToCameraExtrinsic(settingsFile, Tbc_, &errorMessage))
    {
        RCLCPP_WARN(
            this->get_logger(),
            "Use identity body-camera extrinsic because %s: %s",
            settingsFile.c_str(),
            errorMessage.c_str());
    }

    syncThread_ = std::thread(&MonocularInertialNode::SyncWithImu, this);
}

/**
 * @brief 停止同步线程，关闭 ORB_SLAM3 并保存关键帧轨迹。
 */
MonocularInertialNode::~MonocularInertialNode()
{
    stopSync_.store(true);
    if (syncThread_.joinable())
    {
        syncThread_.join();
    }

    SLAM_->Shutdown();

    SLAM_->SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");
}

/**
 * @brief 缓存一帧 IMU 消息。
 * @param msg IMU 消息共享指针。
 */
void MonocularInertialNode::GrabImu(const ImuMsg::SharedPtr msg)
{
    /** @brief IMU 队列锁，保护 IMU 数据入队。 */
    std::lock_guard<std::mutex> lock(imuMutex_);
    imuBuf_.push(msg);
}

/**
 * @brief 缓存一帧单目图像消息。
 * @param msg 图像消息共享指针。
 */
void MonocularInertialNode::GrabImage(const ImageMsg::SharedPtr msg)
{
    /** @brief 图像队列锁，保护图像数据入队。 */
    std::lock_guard<std::mutex> lock(imgMutex_);
    while (!imgBuf_.empty())
    {
        imgBuf_.pop();
    }
    imgBuf_.push(msg);
}

/**
 * @brief 将 ROS 图像消息转换为 OpenCV 灰度图。
 * @param msg 图像消息共享指针。
 * @return 复制出的 OpenCV 图像矩阵；转换失败时返回空矩阵。
 */
cv::Mat MonocularInertialNode::GetImage(const ImageMsg::SharedPtr msg)
{
    /** @brief cv_bridge 转换后的只读图像指针。 */
    cv_bridge::CvImageConstPtr cvPtr;

    try
    {
        cvPtr = cv_bridge::toCvShare(msg);
    }
    catch (cv_bridge::Exception& e)
    {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        return cv::Mat();
    }

    if (cvPtr->image.channels() == 1)
    {
        return cvPtr->image.clone();
    }

    if (cvPtr->image.channels() == 3)
    {
        /** @brief 彩色图像转换后的灰度图。 */
        cv::Mat grayImage;
        cv::cvtColor(cvPtr->image, grayImage, cv::COLOR_BGR2GRAY);
        return grayImage;
    }

    RCLCPP_ERROR(this->get_logger(), "Unsupported image channels: %d", cvPtr->image.channels());
    return cv::Mat();
}

/**
 * @brief 后台按图像时间戳取出 IMU 测量，并调用 ORB_SLAM3 跟踪。
 */
void MonocularInertialNode::SyncWithImu()
{
    while (!stopSync_.load())
    {
        /** @brief 当前待处理的图像消息。 */
        ImageMsg::SharedPtr imageMsg;
        {
            /** @brief 图像队列锁，保护取队首图像和出队操作。 */
            std::lock_guard<std::mutex> lock(imgMutex_);
            if (!imgBuf_.empty())
            {
                imageMsg = imgBuf_.front();
                imgBuf_.pop();
            }
        }

        if (!imageMsg)
        {
            /** @brief 空队列轮询间隔，降低 CPU 占用并允许快速响应停止标志。 */
            const std::chrono::milliseconds tSleep(1);
            std::this_thread::sleep_for(tSleep);
            continue;
        }

        /** @brief 当前图像时间戳，单位为秒。 */
        const double tIm = Utility::StampToSec(imageMsg->header.stamp);
        if (lastImageTimestamp_ >= 0.0 && tIm <= lastImageTimestamp_)
        {
            RCLCPP_WARN(
                this->get_logger(),
                "Drop non-monotonic image timestamp %.9f after %.9f",
                tIm,
                lastImageTimestamp_);
            continue;
        }

        /** @brief 当前图像帧之前的 IMU 测量序列。 */
        std::vector<ORB_SLAM3::IMU::Point> vImuMeas;
        {
            /** @brief IMU 队列锁，保护按图像时间戳取出 IMU 数据。 */
            std::lock_guard<std::mutex> lock(imuMutex_);
            while (!imuBuf_.empty() && Utility::StampToSec(imuBuf_.front()->header.stamp) <= tIm)
            {
                /** @brief 当前 IMU 测量时间戳，单位为秒。 */
                const double t = Utility::StampToSec(imuBuf_.front()->header.stamp);
                /** @brief 当前 IMU 线加速度测量，单位沿用 ROS 消息。 */
                const cv::Point3f acc(
                    imuBuf_.front()->linear_acceleration.x,
                    imuBuf_.front()->linear_acceleration.y,
                    imuBuf_.front()->linear_acceleration.z);
                /** @brief 当前 IMU 角速度测量，单位沿用 ROS 消息。 */
                const cv::Point3f gyr(
                    imuBuf_.front()->angular_velocity.x,
                    imuBuf_.front()->angular_velocity.y,
                    imuBuf_.front()->angular_velocity.z);
                vImuMeas.push_back(ORB_SLAM3::IMU::Point(acc, gyr, t));
                imuBuf_.pop();
            }
        }

        /** @brief 当前待跟踪图像矩阵。 */
        cv::Mat im = GetImage(imageMsg);
        if (im.empty())
        {
            RCLCPP_WARN(this->get_logger(), "Skip empty image at %.9f", tIm);
            continue;
        }

        /** @brief 当前帧世界系到相机系位姿。 */
        const Sophus::SE3f Tcw = SLAM_->TrackMonocular(im, tIm, vImuMeas);
        PublishBodyPose(Tcw, imageMsg->header.stamp);
        lastImageTimestamp_ = tIm;
    }
}

/**
 * @brief 在跟踪状态有效时发布当前机体系位姿和累计轨迹。
 * @param Tcw 世界系到相机系位姿。
 * @param stamp 当前图像时间戳。
 */
void MonocularInertialNode::PublishBodyPose(const Sophus::SE3f& Tcw, const builtin_interfaces::msg::Time& stamp)
{
    /** @brief ORB_SLAM3 当前跟踪状态。 */
    const int trackingState = SLAM_->GetTrackingState();
    if (trackingState != ORB_SLAM3::Tracking::OK && trackingState != ORB_SLAM3::Tracking::OK_KLT)
    {
        return;
    }

    /** @brief 世界系到机体系位姿。 */
    const Sophus::SE3f Twb = ComputeBodyPoseFromCameraPose(Tcw, Tbc_);
    /** @brief 世界系到机体系位姿四元数。 */
    Eigen::Quaternionf quaternion = Twb.unit_quaternion();
    quaternion.normalize();
    /** @brief 世界系到机体系平移。 */
    const Eigen::Vector3f translation = Twb.translation();

    /** @brief 当前帧机体系位姿消息。 */
    PoseStampedMsg poseMsg;
    poseMsg.header.stamp = stamp;
    poseMsg.header.frame_id = "map";
    poseMsg.pose.position.x = translation.x();
    poseMsg.pose.position.y = translation.y();
    poseMsg.pose.position.z = translation.z();
    poseMsg.pose.orientation.x = quaternion.x();
    poseMsg.pose.orientation.y = quaternion.y();
    poseMsg.pose.orientation.z = quaternion.z();
    poseMsg.pose.orientation.w = quaternion.w();

    posePub_->publish(poseMsg);

    pathMsg_.header.stamp = stamp;
    pathMsg_.poses.push_back(poseMsg);
    pathPub_->publish(pathMsg_);
}
