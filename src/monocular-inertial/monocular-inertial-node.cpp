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
    targetImageFps_(0.0),
    imageRateLimitState_{-1.0, 0.0, -1.0},
    lastImageTimestamp_(-1.0),
    lastImuTimestamp_(-1.0),
    imuTimeOffsetSec_(0.0),
    imuReorderWindowSec_(0.0),
    imuReorderState_(),
    receivedImageCount_(0U),
    trackedImageCount_(0U),
    droppedImageCount_(0U),
    rateLimitedImageCount_(0U),
    queueOverflowDroppedImageCount_(0U),
    droppedImuCount_(0U),
    queuedImuCount_(0U),
    imuWaitCount_(0U),
    posePublishCount_(0U),
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

    /** @brief ORB_SLAM3 配置文件读取器，用于加载 ROS wrapper 可选参数。 */
    cv::FileStorage fsSettings(settingsFile, cv::FileStorage::READ);
    if (fsSettings.isOpened())
    {
        /** @brief ROS wrapper 目标图像帧率配置节点。 */
        const cv::FileNode targetFpsNode = fsSettings["ROS.TargetFps"];
        if (!targetFpsNode.empty())
        {
            targetImageFps_ = static_cast<double>(targetFpsNode);
        }

        /** @brief ROS wrapper 图像队列容量配置节点。 */
        const cv::FileNode maxImageQueueNode = fsSettings["ROS.MaxImageQueueSize"];
        if (!maxImageQueueNode.empty())
        {
            /** @brief 配置文件中读取到的图像队列容量。 */
            const int configuredQueueSize = static_cast<int>(maxImageQueueNode);
            if (configuredQueueSize > 0)
            {
                maxImageQueueSize_ = static_cast<std::size_t>(configuredQueueSize);
            }
            else
            {
                RCLCPP_WARN(
                    this->get_logger(),
                    "Ignore invalid ROS.MaxImageQueueSize=%d, keep default %zu",
                    configuredQueueSize,
                    maxImageQueueSize_);
            }
        }

        /** @brief ROS wrapper IMU 时间戳固定偏移配置节点。 */
        const cv::FileNode imuTimeOffsetNode = fsSettings["ROS.ImuTimeOffsetSec"];
        if (!imuTimeOffsetNode.empty())
        {
            imuTimeOffsetSec_ = static_cast<double>(imuTimeOffsetNode);
        }

        /** @brief ROS wrapper IMU 小窗口重排长度配置节点。 */
        const cv::FileNode imuReorderWindowNode = fsSettings["ROS.ImuReorderWindowSec"];
        if (!imuReorderWindowNode.empty())
        {
            /** @brief 配置文件中读取到的 IMU 小窗口重排长度，单位为秒。 */
            const double configuredReorderWindowSec = static_cast<double>(imuReorderWindowNode);
            if (configuredReorderWindowSec >= 0.0)
            {
                imuReorderWindowSec_ = configuredReorderWindowSec;
            }
            else
            {
                RCLCPP_WARN(
                    this->get_logger(),
                    "Ignore invalid ROS.ImuReorderWindowSec=%.6f, keep default %.6f",
                    configuredReorderWindowSec,
                    imuReorderWindowSec_);
            }
        }
    }
    else
    {
        RCLCPP_WARN(this->get_logger(), "Cannot open settings file for ROS wrapper options: %s", settingsFile.c_str());
    }

    RCLCPP_INFO(
        this->get_logger(),
        "单目惯性 ROS wrapper 配置：TargetFps=%.2f，MaxImageQueueSize=%zu，ImuTimeOffsetSec=%.6f，ImuReorderWindowSec=%.6f",
        targetImageFps_,
        maxImageQueueSize_,
        imuTimeOffsetSec_,
        imuReorderWindowSec_);

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
    /** @brief 当前 IMU 样本入队统计。 */
    const MonocularImuQueuePushResult pushResult = MonocularInertialSync::PushReorderedImu(
        imuBuf_,
        msg,
        imuTimeOffsetSec_,
        imuReorderWindowSec_,
        &imuReorderState_,
        &lastImuTimestamp_,
        &droppedImuCount);
    queuedImuCount_.fetch_add(pushResult.acceptedCount);
    droppedImuCount_.store(droppedImuCount);
    if (pushResult.droppedCount > 0U)
    {
        RCLCPP_WARN_THROTTLE(
            this->get_logger(),
            *this->get_clock(),
            5000,
            "过滤 IMU 回跳或重复时间戳样本，累计过滤=%lu，当前队列长度=%zu，重排等待=%zu",
            droppedImuCount_.load(),
            imuBuf_.size(),
            pushResult.pendingCount);
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
    /** @brief 当前调用前因限帧被丢弃的图像数量。 */
    std::uint64_t rateLimitedImageCount = rateLimitedImageCount_.load();
    /** @brief 当前调用前因队列溢出被丢弃的图像数量。 */
    std::uint64_t queueOverflowDroppedImageCount = queueOverflowDroppedImageCount_.load();
    /** @brief 当前图像入队结果。 */
    const MonocularImageQueuePushResult pushResult = MonocularInertialSync::PushRateLimitedBoundedImage(
        imgBuf_,
        msg,
        maxImageQueueSize_,
        targetImageFps_,
        &imageRateLimitState_,
        &rateLimitedImageCount,
        &queueOverflowDroppedImageCount);
    rateLimitedImageCount_.store(rateLimitedImageCount);
    queueOverflowDroppedImageCount_.store(queueOverflowDroppedImageCount);
    if (pushResult.droppedByRateLimit)
    {
        ++droppedImageCount_;
        RCLCPP_WARN_THROTTLE(
            this->get_logger(),
            *this->get_clock(),
            5000,
            "按目标帧率丢弃图像，累计限帧丢弃=%lu，目标帧率=%.2fHz，队列长度=%zu",
            rateLimitedImageCount_.load(),
            targetImageFps_,
            imgBuf_.size());
    }
    if (pushResult.droppedOldestImage)
    {
        ++droppedImageCount_;
        RCLCPP_WARN_THROTTLE(
            this->get_logger(),
            *this->get_clock(),
            5000,
            "图像队列超过上限，已丢弃最旧帧，累计队列溢出丢弃=%lu，队列长度=%zu",
            queueOverflowDroppedImageCount_.load(),
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
                /** @brief 当前调用前被过滤的 IMU 样本数量。 */
                std::uint64_t droppedImuCount = droppedImuCount_.load();
                /** @brief 为当前图像从 IMU 重排缓存中提前刷出的样本统计。 */
                const MonocularImuQueuePushResult flushResult =
                    MonocularInertialSync::FlushPendingReorderedImuUntilCovered(
                        imuBuf_,
                        &imuReorderState_,
                        tIm,
                        &lastImuTimestamp_,
                        &droppedImuCount);
                queuedImuCount_.fetch_add(flushResult.acceptedCount);
                droppedImuCount_.store(droppedImuCount);
                imuQueueSize = imuBuf_.size();
                if (flushResult.droppedCount > 0U)
                {
                    RCLCPP_WARN_THROTTLE(
                        this->get_logger(),
                        *this->get_clock(),
                        5000,
                        "刷新 IMU 重排缓存时过滤回跳或重复样本，累计过滤=%lu，当前队列长度=%zu，重排等待=%zu",
                        droppedImuCount_.load(),
                        imuQueueSize,
                        flushResult.pendingCount);
                }
            }
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
        /** @brief 当前图像帧对应 IMU 序列的首个时间戳，单位为秒。 */
        double firstImuTimestamp = -1.0;
        /** @brief 当前图像帧对应 IMU 序列的末尾时间戳，单位为秒。 */
        double lastFrameImuTimestamp = -1.0;
        /** @brief 当前 IMU 小窗口重排缓存中的待输出样本数量。 */
        std::size_t imuReorderPendingCount = 0U;
        {
            /** @brief IMU 队列锁，保护按图像时间戳取出 IMU 数据。 */
            std::lock_guard<std::mutex> lock(imuMutex_);
            vImuMeas = MonocularInertialSync::ExtractImuMeasurementsUntil(imuBuf_, tIm);
            imuQueueSize = imuBuf_.size();
            imuReorderPendingCount = imuReorderState_.pendingImuMsgs.size();
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
        firstImuTimestamp = vImuMeas.front().t;
        lastFrameImuTimestamp = vImuMeas.back().t;

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
        /** @brief ORB_SLAM3 当前跟踪状态。 */
        const int trackingState = SLAM_->GetTrackingState();
        /** @brief 当前帧是否发布了机体系位姿。 */
        const bool posePublished = PublishBodyPose(Tcw, imageMsg->header.stamp);
        lastImageTimestamp_ = tIm;
        ++trackedImageCount_;
        /** @brief 当前累计接收的图像数量。 */
        const std::uint64_t receivedImageCount = receivedImageCount_.load();
        /** @brief 当前累计送入跟踪的图像数量。 */
        const std::uint64_t trackedImageCount = trackedImageCount_.load();
        /** @brief 当前累计因限帧丢弃的图像数量。 */
        const std::uint64_t rateLimitedImageCount = rateLimitedImageCount_.load();
        /** @brief 当前累计通过限帧并进入图像队列的图像数量。 */
        const std::uint64_t acceptedAfterRateLimitCount = receivedImageCount - rateLimitedImageCount;
        /** @brief 跟踪图像占接收图像的比例，用于观察实际处理帧率。 */
        const double trackedToReceivedRatio =
            receivedImageCount > 0U ? static_cast<double>(trackedImageCount) / static_cast<double>(receivedImageCount) : 0.0;
        /** @brief 跟踪图像占限帧后入队图像的比例，用于观察队列溢出和等待 IMU 的影响。 */
        const double trackedToAcceptedRatio = acceptedAfterRateLimitCount > 0U
                                                  ? static_cast<double>(trackedImageCount) / static_cast<double>(acceptedAfterRateLimitCount)
                                                  : 0.0;
        /** @brief 当前累计发布的机体系位姿数量。 */
        const std::uint64_t posePublishCount = posePublishCount_.load();
        RCLCPP_INFO(
            this->get_logger(),
            "单目惯性帧诊断：tracking_state=%d，image_t=%.9f，imu_count=%zu，imu_first=%.9f，imu_last=%.9f，pose_published=%d，pose_publish_count=%lu，track_time=%.2fms",
            trackingState,
            tIm,
            vImuMeas.size(),
            firstImuTimestamp,
            lastFrameImuTimestamp,
            posePublished ? 1 : 0,
            posePublishCount,
            trackDurationMs);
        RCLCPP_INFO_THROTTLE(
            this->get_logger(),
            *this->get_clock(),
            5000,
            "单目惯性统计：接收图像=%lu，限帧后入队=%lu，跟踪图像=%lu，跟踪/接收=%.3f，跟踪/限帧后=%.3f，丢弃图像=%lu，限帧丢弃=%lu，队列溢出丢弃=%lu，入队IMU=%lu，过滤IMU=%lu，等待IMU=%lu，图像队列=%zu，IMU队列=%zu，IMU重排等待=%zu，最近跟踪耗时=%.2fms",
            receivedImageCount,
            acceptedAfterRateLimitCount,
            trackedImageCount,
            trackedToReceivedRatio,
            trackedToAcceptedRatio,
            droppedImageCount_.load(),
            rateLimitedImageCount,
            queueOverflowDroppedImageCount_.load(),
            queuedImuCount_.load(),
            droppedImuCount_.load(),
            imuWaitCount_.load(),
            imageQueueSize,
            imuQueueSize,
            imuReorderPendingCount,
            trackDurationMs);
    }
}

/**
 * @brief 在跟踪状态有效时发布当前机体系位姿和累计轨迹。
 * @param Tcw 世界系到相机系位姿。
 * @param stamp 当前图像时间戳。
 * @return 当前帧成功发布机体系位姿时返回 true。
 */
bool MonocularInertialNode::PublishBodyPose(const Sophus::SE3f& Tcw, const builtin_interfaces::msg::Time& stamp)
{
    /** @brief ORB_SLAM3 当前跟踪状态。 */
    const int trackingState = SLAM_->GetTrackingState();
    if (trackingState != ORB_SLAM3::Tracking::OK && trackingState != ORB_SLAM3::Tracking::OK_KLT)
    {
        return false;
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
    ++posePublishCount_;

    pathMsg_.header.stamp = stamp;
    pathMsg_.poses.push_back(poseMsg);
    pathPub_->publish(pathMsg_);
    return true;
}
