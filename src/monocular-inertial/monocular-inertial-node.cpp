/**
 * @file monocular-inertial-node.cpp
 * @brief 实现单目惯性 ORB_SLAM3 ROS 2 节点的数据缓存、同步和退出收尾逻辑。
 */

#include "monocular-inertial-node.hpp"

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <chrono>
#include <cmath>
#include <iostream>
#include <string>

#include "monocular-feature-mask.hpp"

using std::placeholders::_1;

namespace
{
/** @brief ORB_SLAM3 世界坐标系对应的 ROS frame 名称。 */
constexpr const char* kMapFrameId = "map";
/** @brief Kalibr IMU 机体系对应的 ROS frame 名称。 */
constexpr const char* kBodyFrameId = "body_link";
/** @brief 输入图像未声明 frame_id 时使用的相机光学坐标系名称。 */
constexpr const char* kFallbackCameraFrameId = "camera_optical_frame";
/** @brief 默认保留的有效机体位姿数量。 */
constexpr std::int64_t kDefaultMaxPathLength = 10000;
/** @brief 惯性 BA1 完成后继续等待优化收敛的时长，单位为秒。 */
constexpr double kInertialBa1SettlingDurationSec = 1.0;
}  // namespace

/** @brief 为 C++14 提供时基差值阈值常量的存储定义。 */
constexpr double MonocularInertialNode::kMaxTimeBaseDifferenceSec;

/**
 * @brief 创建单目惯性 SLAM 节点并启动图像/IMU 同步线程。
 * @param SLAM ORB_SLAM3 系统实例指针，生命周期由调用方管理。
 * @param settingsFile ORB_SLAM3 配置文件路径，用于读取 IMU 到相机外参。
 */
MonocularInertialNode::MonocularInertialNode(ORB_SLAM3::System* SLAM, const std::string& settingsFile) :
    Node("ORB_SLAM3_ROS2"),
    SLAM_(SLAM),
    featureMask_(),
    featureMaskSizeValidated_(false),
    stopSync_(false),
    maxImageQueueSize_(100U),
    targetImageFps_(0.0),
    maxPathLength_(0U),
    saveKeyframeTrajectory_(false),
    imageRateLimitState_{-1.0, 0.0, -1.0},
    lastImageTimestamp_(-1.0),
    lastImuTimestamp_(-1.0),
    imuTimeOffsetSec_(0.0),
    latestRawImuTimestamp_(-1.0),
    timeBaseValidated_(false),
    fatalError_(false),
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
    ba1InitializedSinceSec_(-1.0),
    Tbc_(),
    pathResetState_(),
    staticCameraTransformPublished_(false),
    cameraFrameId_()
{
    /** @brief 静态特征掩膜文件路径，空路径表示禁用。 */
    const std::string featureMaskPath =
        this->declare_parameter<std::string>("feature_mask_path", "");
    featureMask_ = orbslam3_ros2::LoadFeatureMask(featureMaskPath);

    if (featureMask_.empty())
    {
        RCLCPP_INFO(this->get_logger(), "Monocular-inertial feature mask is disabled");
    }
    else
    {
        /** @brief 掩膜排除区域占输入图像的百分比。 */
        const double excludedPercent =
            orbslam3_ros2::CalculateExcludedRatio(featureMask_) * 100.0;
        RCLCPP_INFO(
            this->get_logger(),
            "Loaded monocular-inertial feature mask: path=%s, size=%dx%d, excluded=%.2f%%",
            featureMaskPath.c_str(),
            featureMask_.cols,
            featureMask_.rows,
            excludedPercent);
    }

    /** @brief ROS 参数中配置的最大轨迹长度。 */
    const std::int64_t configuredMaxPathLength =
        this->declare_parameter<std::int64_t>(
            "max_path_length", kDefaultMaxPathLength);
    maxPathLength_ = ValidateBodyMaxPathLength(configuredMaxPathLength);
    saveKeyframeTrajectory_ =
        this->declare_parameter<bool>("save_keyframe_trajectory", false);

    subImu_ = this->create_subscription<ImuMsg>("imu", 1000, std::bind(&MonocularInertialNode::GrabImu, this, _1));
    subImg_ = this->create_subscription<ImageMsg>("camera", 100, std::bind(&MonocularInertialNode::GrabImage, this, _1));
    posePub_ = this->create_publisher<PoseStampedMsg>("/orbslam3/body_pose", 10);
    pathPub_ = this->create_publisher<PathMsg>("/orbslam3/path", rclcpp::QoS(1).reliable());
    dynamicTfBroadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    staticTfBroadcaster_ = std::make_unique<tf2_ros::StaticTransformBroadcaster>(*this);
    pathMsg_.header.frame_id = kMapFrameId;

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
    RCLCPP_INFO(
        this->get_logger(),
        "单目惯性定位输出：pose=/orbslam3/body_pose，path=/orbslam3/path，"
        "dynamic_tf=%s->%s，发布起点=惯性BA1连续稳定%.1f秒，max_path_length=%zu，save_keyframe_trajectory=%s",
        kMapFrameId,
        kBodyFrameId,
        kInertialBa1SettlingDurationSec,
        maxPathLength_,
        saveKeyframeTrajectory_ ? "true" : "false");

    syncThread_ = std::thread(&MonocularInertialNode::SyncWithImu, this);
}

/**
 * @brief 停止同步线程、关闭 ORB_SLAM3，并按配置保存关键帧轨迹。
 */
MonocularInertialNode::~MonocularInertialNode()
{
    stopSync_.store(true);
    if (syncThread_.joinable())
    {
        syncThread_.join();
    }

    SLAM_->Shutdown();

    if (saveKeyframeTrajectory_)
    {
        SLAM_->SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");
    }
    else
    {
        RCLCPP_INFO(
            this->get_logger(),
            "Skip saving KeyFrameTrajectory.txt because save_keyframe_trajectory=false");
    }
}

/**
 * @brief 查询节点是否因不可恢复的运行时错误停止。
 * @return 发生图像/IMU 时基不一致等致命错误时返回 true。
 */
bool MonocularInertialNode::HasFatalError() const
{
    return fatalError_.load();
}

/**
 * @brief 缓存一帧 IMU 消息。
 * @param msg IMU 消息共享指针。
 */
void MonocularInertialNode::GrabImu(const ImuMsg::SharedPtr msg)
{
    /** @brief IMU 队列锁，保护 IMU 数据入队。 */
    std::lock_guard<std::mutex> lock(imuMutex_);
    /** @brief 当前 IMU 消息中未应用偏移的原始时间戳，单位为秒。 */
    const double rawImuTimestamp = Utility::StampToSec(msg->header.stamp);
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
    if (std::isfinite(rawImuTimestamp) &&
        (latestRawImuTimestamp_ < 0.0 || rawImuTimestamp > latestRawImuTimestamp_))
    {
        latestRawImuTimestamp_ = rawImuTimestamp;
    }
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
    if (!staticCameraTransformPublished_)
    {
        PublishStaticBodyCameraTransform(msg);
    }

    if (!timeBaseValidated_.load())
    {
        /** @brief 当前待校验图像的原始时间戳，单位为秒。 */
        const double imageTimestamp = Utility::StampToSec(msg->header.stamp);
        /** @brief 时基检查期间的 IMU 队列锁，保护最近原始 IMU 时间戳。 */
        std::lock_guard<std::mutex> imuLock(imuMutex_);
        if (latestRawImuTimestamp_ >= 0.0)
        {
            /** @brief 图像与最近原始 IMU 时间戳的绝对差，单位为秒。 */
            const double timeBaseDifferenceSec = std::abs(imageTimestamp - latestRawImuTimestamp_);
            if (!MonocularInertialSync::AreTimeBasesAligned(
                    imageTimestamp,
                    latestRawImuTimestamp_,
                    kMaxTimeBaseDifferenceSec))
            {
                fatalError_.store(true);
                stopSync_.store(true);
                RCLCPP_FATAL(
                    this->get_logger(),
                    "图像与 IMU 时间基准未对齐，程序即将退出：image_t=%.9f，imu_t=%.9f，差值=%.6f秒，允许上限=%.3f秒。请在设备发布端统一 header.stamp，节点不会自动修正时间基准。",
                    imageTimestamp,
                    latestRawImuTimestamp_,
                    timeBaseDifferenceSec,
                    kMaxTimeBaseDifferenceSec);
                rclcpp::shutdown();
                return;
            }
            timeBaseValidated_.store(true);
            RCLCPP_INFO(
                this->get_logger(),
                "图像与 IMU 时间基准检查通过：image_t=%.9f，imu_t=%.9f，差值=%.3fms",
                imageTimestamp,
                latestRawImuTimestamp_,
                timeBaseDifferenceSec * 1000.0);
        }
    }

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
        RCLCPP_INFO_THROTTLE(
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
        if (msg->encoding == sensor_msgs::image_encodings::RGB8)
        {
            cv::cvtColor(cvPtr->image, grayImage, cv::COLOR_RGB2GRAY);
        }
        else if (msg->encoding == sensor_msgs::image_encodings::BGR8)
        {
            cv::cvtColor(cvPtr->image, grayImage, cv::COLOR_BGR2GRAY);
        }
        else
        {
            RCLCPP_ERROR(
                this->get_logger(),
                "Unsupported three-channel image encoding: %s",
                msg->encoding.c_str());
            return cv::Mat();
        }
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
                RCLCPP_DEBUG_THROTTLE(
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

        if (!featureMask_.empty() && !featureMaskSizeValidated_)
        {
            /** @brief 首帧掩膜尺寸校验失败时的详细说明。 */
            std::string errorMessage;
            if (!orbslam3_ros2::ValidateFeatureMaskSize(
                    featureMask_, im.size(), &errorMessage))
            {
                fatalError_.store(true);
                RCLCPP_FATAL(this->get_logger(), "%s", errorMessage.c_str());
                rclcpp::shutdown();
                return;
            }
            featureMaskSizeValidated_ = true;
            RCLCPP_INFO(
                this->get_logger(),
                "Monocular-inertial feature mask size validation passed");
        }

        /** @brief 单帧跟踪开始时间。 */
        const std::chrono::steady_clock::time_point trackStart = std::chrono::steady_clock::now();
        /** @brief 当前帧世界系到相机系位姿。 */
        const Sophus::SE3f Tcw = SLAM_->TrackMonocular(im, featureMask_, tIm, vImuMeas);
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
        RCLCPP_DEBUG(
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
    if (trackingState == ORB_SLAM3::Tracking::LOST)
    {
        pathResetState_.MarkTrackingLost();
        return false;
    }
    if (trackingState != ORB_SLAM3::Tracking::OK && trackingState != ORB_SLAM3::Tracking::OK_KLT)
    {
        return false;
    }
    /** @brief 当前图像时间戳，供 BA1 稳定门控使用。 */
    const double timestampSec = Utility::StampToSec(stamp);
    /** @brief 当前地图是否已经完成 BA1 并持续稳定到允许发布。 */
    const bool ba1PublishReady = UpdateInertialBa1PublishGate(
        SLAM_->isInertialBA1Initialized(),
        timestampSec,
        kInertialBa1SettlingDurationSec,
        &ba1InitializedSinceSec_);
    if (!ba1PublishReady)
    {
        RCLCPP_INFO_THROTTLE(
            this->get_logger(),
            *this->get_clock(),
            5000,
            "等待惯性 BA1 完成并连续稳定 %.1f 秒，暂停发布尚未收敛的 body 位姿",
            kInertialBa1SettlingDurationSec);
        return false;
    }

    /** @brief LOST 后恢复有效跟踪时是否需要清空旧轨迹。 */
    const bool resetPath = pathResetState_.ConsumeResetRequest();
    if (resetPath)
    {
        RCLCPP_WARN(
            this->get_logger(),
            "单目惯性跟踪从 LOST 恢复，清空已发布的旧机体轨迹");
    }

    /** @brief 机体系在世界系中的位姿。 */
    const Sophus::SE3f Twb = ComputeBodyPoseFromCameraPose(Tcw, Tbc_);
    /** @brief 当前帧机体系位姿消息。 */
    const PoseStampedMsg poseMsg =
        CreateBodyPoseMessage(Twb, stamp, kMapFrameId);
    UpdateBodyPath(poseMsg, maxPathLength_, resetPath, &pathMsg_);
    /** @brief 与当前机体位姿严格一致的动态 TF 消息。 */
    const geometry_msgs::msg::TransformStamped transformMsg =
        CreateBodyTransformMessage(poseMsg, kBodyFrameId);

    posePub_->publish(poseMsg);
    pathPub_->publish(pathMsg_);
    dynamicTfBroadcaster_->sendTransform(transformMsg);
    ++posePublishCount_;
    return true;
}

/**
 * @brief 使用首帧图像 frame_id 发布机体到相机光学系的静态 TF。
 * @param msg 首帧输入图像消息。
 */
void MonocularInertialNode::PublishStaticBodyCameraTransform(const ImageMsg::SharedPtr& msg)
{
    cameraFrameId_ = msg->header.frame_id.empty()
                         ? kFallbackCameraFrameId
                         : msg->header.frame_id;
    /** @brief 相机光学系在机体系中的位姿消息，对应配置中的 T_b_c。 */
    const PoseStampedMsg cameraPoseMsg =
        CreateBodyPoseMessage(Tbc_, msg->header.stamp, kBodyFrameId);
    /** @brief 机体系到输入图像光学系的静态 TF。 */
    const geometry_msgs::msg::TransformStamped cameraTransformMsg =
        CreateBodyTransformMessage(cameraPoseMsg, cameraFrameId_);
    staticTfBroadcaster_->sendTransform(cameraTransformMsg);
    staticCameraTransformPublished_ = true;

    RCLCPP_INFO(
        this->get_logger(),
        "发布相机静态 TF：%s->%s，frame_id 来源=%s",
        kBodyFrameId,
        cameraFrameId_.c_str(),
        msg->header.frame_id.empty() ? "fallback" : "image header");
}
