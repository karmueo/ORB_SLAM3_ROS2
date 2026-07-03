/**
 * @file stereo-inertial-node.cpp
 * @brief 实现双目惯性 ORB_SLAM3 ROS 2 节点的数据缓存、同步和退出收尾逻辑。
 */

#include "stereo-inertial-node.hpp"

#include <opencv2/core/core.hpp>

using std::placeholders::_1;

/**
 * @brief 创建双目惯性 SLAM 节点并启动同步线程。
 * @param SLAM ORB_SLAM3 系统实例指针，生命周期由调用方管理。
 * @param strSettingsFile 相机和 IMU 配置文件路径。
 * @param strDoRectify 是否对双目图像做去畸变矫正的字符串开关。
 * @param strDoEqual 是否对图像做 CLAHE 均衡化的字符串开关。
 */
StereoInertialNode::StereoInertialNode(ORB_SLAM3::System *SLAM, const string &strSettingsFile, const string &strDoRectify, const string &strDoEqual) :
    Node("ORB_SLAM3_ROS2"),
    SLAM_(SLAM),
    stopSync_(false)
{
    /** @brief 去畸变矫正开关解析流。 */
    stringstream ss_rec(strDoRectify);
    ss_rec >> boolalpha >> doRectify_;

    /** @brief 均衡化开关解析流。 */
    stringstream ss_eq(strDoEqual);
    ss_eq >> boolalpha >> doEqual_;

    bClahe_ = doEqual_;
    std::cout << "Rectify: " << doRectify_ << std::endl;
    std::cout << "Equal: " << doEqual_ << std::endl;

    if (doRectify_)
    {
        /** @brief 相机配置文件读取器。 */
        cv::FileStorage fsSettings(strSettingsFile, cv::FileStorage::READ);
        if (!fsSettings.isOpened())
        {
            cerr << "ERROR: Wrong path to settings" << endl;
            assert(0);
        }

        /** @brief 左右目内参、投影、旋转和畸变参数矩阵。 */
        cv::Mat K_l, K_r, P_l, P_r, R_l, R_r, D_l, D_r;
        fsSettings["LEFT.K"] >> K_l;
        fsSettings["RIGHT.K"] >> K_r;

        fsSettings["LEFT.P"] >> P_l;
        fsSettings["RIGHT.P"] >> P_r;

        fsSettings["LEFT.R"] >> R_l;
        fsSettings["RIGHT.R"] >> R_r;

        fsSettings["LEFT.D"] >> D_l;
        fsSettings["RIGHT.D"] >> D_r;

        /** @brief 左目图像行数。 */
        int rows_l = fsSettings["LEFT.height"];
        /** @brief 左目图像列数。 */
        int cols_l = fsSettings["LEFT.width"];
        /** @brief 右目图像行数。 */
        int rows_r = fsSettings["RIGHT.height"];
        /** @brief 右目图像列数。 */
        int cols_r = fsSettings["RIGHT.width"];

        if (K_l.empty() || K_r.empty() || P_l.empty() || P_r.empty() || R_l.empty() || R_r.empty() || D_l.empty() || D_r.empty() ||
            rows_l == 0 || rows_r == 0 || cols_l == 0 || cols_r == 0)
        {
            cerr << "ERROR: Calibration parameters to rectify stereo are missing!" << endl;
            assert(0);
        }

        cv::initUndistortRectifyMap(K_l, D_l, R_l, P_l.rowRange(0, 3).colRange(0, 3), cv::Size(cols_l, rows_l), CV_32F, M1l_, M2l_);
        cv::initUndistortRectifyMap(K_r, D_r, R_r, P_r.rowRange(0, 3).colRange(0, 3), cv::Size(cols_r, rows_r), CV_32F, M1r_, M2r_);
    }

    subImu_ = this->create_subscription<ImuMsg>("imu", 1000, std::bind(&StereoInertialNode::GrabImu, this, _1));
    subImgLeft_ = this->create_subscription<ImageMsg>("camera/left", 100, std::bind(&StereoInertialNode::GrabImageLeft, this, _1));
    subImgRight_ = this->create_subscription<ImageMsg>("camera/right", 100, std::bind(&StereoInertialNode::GrabImageRight, this, _1));

    syncThread_ = new std::thread(&StereoInertialNode::SyncWithImu, this);
}

/**
 * @brief 停止同步线程，关闭 ORB_SLAM3 并保存关键帧轨迹。
 */
StereoInertialNode::~StereoInertialNode()
{
    stopSync_.store(true);
    if (syncThread_ && syncThread_->joinable())
    {
        syncThread_->join();
    }
    delete syncThread_;

    SLAM_->Shutdown();

    SLAM_->SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");
}

/**
 * @brief 缓存一帧 IMU 消息。
 * @param msg IMU 消息共享指针。
 */
void StereoInertialNode::GrabImu(const ImuMsg::SharedPtr msg)
{
    bufMutex_.lock();
    imuBuf_.push(msg);
    bufMutex_.unlock();
}

/**
 * @brief 缓存一帧左目图像消息。
 * @param msgLeft 左目图像消息共享指针。
 */
void StereoInertialNode::GrabImageLeft(const ImageMsg::SharedPtr msgLeft)
{
    bufMutexLeft_.lock();

    if (!imgLeftBuf_.empty())
        imgLeftBuf_.pop();
    imgLeftBuf_.push(msgLeft);

    bufMutexLeft_.unlock();
}

/**
 * @brief 缓存一帧右目图像消息。
 * @param msgRight 右目图像消息共享指针。
 */
void StereoInertialNode::GrabImageRight(const ImageMsg::SharedPtr msgRight)
{
    bufMutexRight_.lock();

    if (!imgRightBuf_.empty())
        imgRightBuf_.pop();
    imgRightBuf_.push(msgRight);

    bufMutexRight_.unlock();
}

/**
 * @brief 将 ROS 图像消息转换为 OpenCV 灰度图。
 * @param msg 图像消息共享指针。
 * @return 复制出的 OpenCV 图像矩阵。
 */
cv::Mat StereoInertialNode::GetImage(const ImageMsg::SharedPtr msg)
{
    /** @brief cv_bridge 转换后的只读图像指针。 */
    cv_bridge::CvImageConstPtr cv_ptr;

    try
    {
        cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::MONO8);
    }
    catch (cv_bridge::Exception &e)
    {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
    }

    if (cv_ptr->image.type() == 0)
    {
        return cv_ptr->image.clone();
    }
    else
    {
        std::cerr << "Error image type" << std::endl;
        return cv_ptr->image.clone();
    }
}

/**
 * @brief 后台同步左右目图像和 IMU 数据，并调用 ORB_SLAM3 跟踪。
 */
void StereoInertialNode::SyncWithImu()
{
    /** @brief 左右目图像允许的最大时间差，单位为秒。 */
    const double maxTimeDiff = 0.01;

    while (!stopSync_.load())
    {
        /** @brief 当前待处理的左右目图像。 */
        cv::Mat imLeft, imRight;
        /** @brief 当前待处理左右目图像时间戳，单位为秒。 */
        double tImLeft = 0, tImRight = 0;
        if (!imgLeftBuf_.empty() && !imgRightBuf_.empty() && !imuBuf_.empty())
        {
            tImLeft = Utility::StampToSec(imgLeftBuf_.front()->header.stamp);
            tImRight = Utility::StampToSec(imgRightBuf_.front()->header.stamp);

            bufMutexRight_.lock();
            while ((tImLeft - tImRight) > maxTimeDiff && imgRightBuf_.size() > 1)
            {
                imgRightBuf_.pop();
                tImRight = Utility::StampToSec(imgRightBuf_.front()->header.stamp);
            }
            bufMutexRight_.unlock();

            bufMutexLeft_.lock();
            while ((tImRight - tImLeft) > maxTimeDiff && imgLeftBuf_.size() > 1)
            {
                imgLeftBuf_.pop();
                tImLeft = Utility::StampToSec(imgLeftBuf_.front()->header.stamp);
            }
            bufMutexLeft_.unlock();

            if ((tImLeft - tImRight) > maxTimeDiff || (tImRight - tImLeft) > maxTimeDiff)
            {
                std::cout << "big time difference" << std::endl;
                continue;
            }
            if (tImLeft > Utility::StampToSec(imuBuf_.back()->header.stamp))
                continue;

            bufMutexLeft_.lock();
            imLeft = GetImage(imgLeftBuf_.front());
            imgLeftBuf_.pop();
            bufMutexLeft_.unlock();

            bufMutexRight_.lock();
            imRight = GetImage(imgRightBuf_.front());
            imgRightBuf_.pop();
            bufMutexRight_.unlock();

            /** @brief 当前图像帧之前的 IMU 测量序列。 */
            vector<ORB_SLAM3::IMU::Point> vImuMeas;
            bufMutex_.lock();
            if (!imuBuf_.empty())
            {
                vImuMeas.clear();
                while (!imuBuf_.empty() && Utility::StampToSec(imuBuf_.front()->header.stamp) <= tImLeft)
                {
                    /** @brief 当前 IMU 测量时间戳，单位为秒。 */
                    double t = Utility::StampToSec(imuBuf_.front()->header.stamp);
                    /** @brief 当前 IMU 线加速度测量。 */
                    cv::Point3f acc(imuBuf_.front()->linear_acceleration.x, imuBuf_.front()->linear_acceleration.y, imuBuf_.front()->linear_acceleration.z);
                    /** @brief 当前 IMU 角速度测量。 */
                    cv::Point3f gyr(imuBuf_.front()->angular_velocity.x, imuBuf_.front()->angular_velocity.y, imuBuf_.front()->angular_velocity.z);
                    vImuMeas.push_back(ORB_SLAM3::IMU::Point(acc, gyr, t));
                    imuBuf_.pop();
                }
            }
            bufMutex_.unlock();

            if (bClahe_)
            {
                clahe_->apply(imLeft, imLeft);
                clahe_->apply(imRight, imRight);
            }

            if (doRectify_)
            {
                cv::remap(imLeft, imLeft, M1l_, M2l_, cv::INTER_LINEAR);
                cv::remap(imRight, imRight, M1r_, M2r_, cv::INTER_LINEAR);
            }

            SLAM_->TrackStereo(imLeft, imRight, tImLeft, vImuMeas);
        }

        /** @brief 同步线程轮询间隔，降低空队列时的 CPU 占用并允许快速响应停止标志。 */
        std::chrono::milliseconds tSleep(1);
        std::this_thread::sleep_for(tSleep);
    }
}
