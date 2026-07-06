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
    maxImageQueueSize_(100U),
    lastImageTimestamp_(-1.0),
    lastImuTimestamp_(-1.0),
    receivedImageCount_(0U),
    trackedImageCount_(0U),
    droppedImageCount_(0U),
    droppedImuCount_(0U),
    imuWaitCount_(0U),
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
    /** @brief 当前调用前被过滤的 IMU 样本数量。 */
    std::uint64_t droppedImuCount = droppedImuCount_.load();
    /** @brief 当前 IMU 样本是否成功入队。 */
    const bool accepted = MonocularInertialSync::PushMonotonicImu(
        imuBuf_,
        msg,
        &lastImuTimestamp_,
        &droppedImuCount);
    droppedImuCount_.store(droppedImuCount);
    if (!accepted)
    {
        RCLCPP_WARN_THROTTLE(
            this->get_logger(),
            *this->get_clock(),
            5000,
            "过滤 IMU 回跳或重复时间戳样本，累计过滤=%lu，当前队列长度=%zu",
            droppedImuCount_.load(),
            imuBuf_.size());
    }
}

/**
 * @brief 缓存一帧单目图像消息。
 * @param msg 图像消息共享指针。
 */
void MonocularInertialNode::GrabImage(const ImageMsg::SharedPtr msg)
{
    /** @brief 图像队列锁，保护图像数据入队。 */
    std::lock_guard<std::mutex> lock(imgMutex_);
    ++receivedImageCount_;
    /** @brief 当前调用前被丢弃的图像数量。 */
    std::uint64_t droppedImageCount = droppedImageCount_.load();
    /** @brief 当前图像入队是否导致最旧图像被丢弃。 */
    const bool droppedOldestImage = MonocularInertialSync::PushBoundedImage(
        imgBuf_,
        msg,
        maxImageQueueSize_,
        &droppedImageCount);
    droppedImageCount_.store(droppedImageCount);
    if (droppedOldestImage)
    {
        RCLCPP_WARN_THROTTLE(
            this->get_logger(),
            *this->get_clock(),
            5000,
            "图像队列超过上限，已丢弃最旧帧，累计丢弃=%lu，队列长度=%zu",
            droppedImageCount_.load(),
            imgBuf_.size());
    }
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
        /** @brief 当前图像时间戳，单位为秒。 */
        double tIm = 0.0;
        /** @brief 当前图像队列长度。 */
        std::size_t imageQueueSize = 0U;
        /** @brief 当前 IMU 队列长度。 */
        std::size_t imuQueueSize = 0U;
        {
            /** @brief 图像队列锁，保护读取队首图像。 */
            std::lock_guard<std::mutex> lock(imgMutex_);
            if (!imgBuf_.empty())
            {
                imageMsg = imgBuf_.front();
                tIm = Utility::StampToSec(imageMsg->header.stamp);
                imageQueueSize = imgBuf_.size();
            }
        }

        if (!imageMsg)
        {
            /** @brief 空队列轮询间隔，降低 CPU 占用并允许快速响应停止标志。 */
            const std::chrono::milliseconds tSleep(1);
            std::this_thread::sleep_for(tSleep);
            continue;
        }

        if (lastImageTimestamp_ >= 0.0 && tIm <= lastImageTimestamp_)
        {
            RCLCPP_WARN(
                this->get_logger(),
                "Drop non-monotonic image timestamp %.9f after %.9f",
                tIm,
                lastImageTimestamp_);
            {
                /** @brief 图像队列锁，保护弹出已判定回跳的图像。 */
                std::lock_guard<std::mutex> lock(imgMutex_);
                if (!imgBuf_.empty() && Utility::StampToSec(imgBuf_.front()->header.stamp) == tIm)
                {
                    imgBuf_.pop();
                }
            }
            ++droppedImageCount_;
            continue;
        }

        /** @brief 当前图像是否需要等待更多 IMU 样本覆盖。 */
        bool waitForImu = false;
        {
            /** @brief IMU 队列锁，保护覆盖性判断。 */
            std::lock_guard<std::mutex> lock(imuMutex_);
            imuQueueSize = imuBuf_.size();
            if (!MonocularInertialSync::HasImuCoverageForImage(imuBuf_, tIm))
            {
                ++imuWaitCount_;
                RCLCPP_WARN_THROTTLE(
                    this->get_logger(),
                    *this->get_clock(),
                    5000,
                    "等待 IMU 覆盖图像时间戳 %.9f，等待次数=%lu，图像队列=%zu，IMU队列=%zu",
                    tIm,
                    imuWaitCount_.load(),
                    imageQueueSize,
                    imuQueueSize);
                waitForImu = true;
            }
        }
        if (waitForImu)
        {
            /** @brief 等待 IMU 数据到达的轮询间隔。 */
            const std::chrono::milliseconds tSleep(1);
            std::this_thread::sleep_for(tSleep);
            continue;
        }

        {
            /** @brief 图像队列锁，保护弹出即将处理的图像。 */
            std::lock_guard<std::mutex> lock(imgMutex_);
            if (!imgBuf_.empty() && Utility::StampToSec(imgBuf_.front()->header.stamp) == tIm)
            {
                imgBuf_.pop();
                imageQueueSize = imgBuf_.size();
            }
            else
            {
                continue;
            }
        }

        /** @brief 当前图像帧之前的 IMU 测量序列。 */
        std::vector<ORB_SLAM3::IMU::Point> vImuMeas;
        {
            /** @brief IMU 队列锁，保护按图像时间戳取出 IMU 数据。 */
            std::lock_guard<std::mutex> lock(imuMutex_);
            vImuMeas = MonocularInertialSync::ExtractImuMeasurementsUntil(imuBuf_, tIm);
            imuQueueSize = imuBuf_.size();
        }

        if (!MonocularInertialSync::HasEnoughMeasurementsForPreintegration(vImuMeas))
        {
            ++droppedImageCount_;
            lastImageTimestamp_ = tIm;
            RCLCPP_WARN_THROTTLE(
                this->get_logger(),
                *this->get_clock(),
                5000,
                "跳过 IMU 测量不足的图像帧 %.9f，当前IMU样本数=%zu，累计丢弃图像=%lu",
                tIm,
                vImuMeas.size(),
                droppedImageCount_.load());
            continue;
        }

        /** @brief 当前待跟踪图像矩阵。 */
        cv::Mat im = GetImage(imageMsg);
        if (im.empty())
        {
            ++droppedImageCount_;
            lastImageTimestamp_ = tIm;
            RCLCPP_WARN(this->get_logger(), "Skip empty image at %.9f", tIm);
            continue;
        }

        /** @brief 单帧跟踪开始时间。 */
        const std::chrono::steady_clock::time_point trackStart = std::chrono::steady_clock::now();
        /** @brief 当前帧世界系到相机系位姿。 */
        const Sophus::SE3f Tcw = SLAM_->TrackMonocular(im, tIm, vImuMeas);
        /** @brief 单帧跟踪结束时间。 */
        const std::chrono::steady_clock::time_point trackEnd = std::chrono::steady_clock::now();
        /** @brief 单帧跟踪耗时，单位为毫秒。 */
        const double trackDurationMs = std::chrono::duration<double, std::milli>(trackEnd - trackStart).count();
        PublishBodyPose(Tcw, imageMsg->header.stamp);
        lastImageTimestamp_ = tIm;
        ++trackedImageCount_;
        RCLCPP_INFO_THROTTLE(
            this->get_logger(),
            *this->get_clock(),
            5000,
            "单目惯性统计：接收图像=%lu，跟踪图像=%lu，丢弃图像=%lu，过滤IMU=%lu，等待IMU=%lu，图像队列=%zu，IMU队列=%zu，最近跟踪耗时=%.2fms",
            receivedImageCount_.load(),
            trackedImageCount_.load(),
            droppedImageCount_.load(),
            droppedImuCount_.load(),
            imuWaitCount_.load(),
            imageQueueSize,
            imuQueueSize,
            trackDurationMs);
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
