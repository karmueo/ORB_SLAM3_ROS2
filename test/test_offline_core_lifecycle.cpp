/**
 * @file test_offline_core_lifecycle.cpp
 * @brief 对真实 ORB_SLAM3 核心验证离线 readiness/drain/shutdown/snapshot 行为。
 */

#include "System.h"
#include "Atlas.h"
#include "CameraModels/KannalaBrandt8.h"
#include "Frame.h"
#include "ImuTypes.h"
#include "KeyFrameDatabase.h"
#include "LoopClosing.h"
#include "Map.h"
#include "ORBVocabulary.h"
#include "ORBextractor.h"

#include "gtest/gtest.h"

#include <unistd.h>

#include <atomic>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <new>
#include <string>

namespace
{
const char* const kConfigText = R"(%YAML:1.0

File.version: "1.0"
Camera.type: "KannalaBrandt8"
Camera1.fx: 300.0
Camera1.fy: 300.0
Camera1.cx: 320.0
Camera1.cy: 240.0
Camera1.k1: 0.01
Camera1.k2: 0.001
Camera1.k3: 0.0
Camera1.k4: 0.0
Camera2.fx: 300.0
Camera2.fy: 300.0
Camera2.cx: 320.0
Camera2.cy: 240.0
Camera2.k1: 0.01
Camera2.k2: 0.001
Camera2.k3: 0.0
Camera2.k4: 0.0
Camera.width: 640
Camera.height: 480
Camera.fps: 50
Camera.RGB: 0
Camera1.overlappingBegin: 0
Camera1.overlappingEnd: 639
Camera2.overlappingBegin: 0
Camera2.overlappingEnd: 639
Stereo.ThDepth: 40.0
Stereo.T_c1_c2: !!opencv-matrix
   rows: 4
   cols: 4
   dt: f
   data: [1.0, 0.0, 0.0, -0.1,
          0.0, 1.0, 0.0, 0.0,
          0.0, 0.0, 1.0, 0.0,
          0.0, 0.0, 0.0, 1.0]
IMU.T_b_c1: !!opencv-matrix
   rows: 4
   cols: 4
   dt: f
   data: [1.0, 0.0, 0.0, 0.0,
          0.0, 1.0, 0.0, 0.0,
          0.0, 0.0, 1.0, 0.0,
          0.0, 0.0, 0.0, 1.0]
IMU.InsertKFsWhenLost: 0
IMU.NoiseGyro: 0.001
IMU.NoiseAcc: 0.01
IMU.GyroWalk: 0.0001
IMU.AccWalk: 0.0001
IMU.Frequency: 400.0
IMU.AccBias: [0.0, 0.0, 0.0]
IMU.GyroBias: [0.0, 0.0, 0.0]
ROS.ImuTimeOffsetSec: 0.0
ORBextractor.nFeatures: 1200
ORBextractor.scaleFactor: 1.2
ORBextractor.nLevels: 8
ORBextractor.iniThFAST: 20
ORBextractor.minThFAST: 7
Viewer.KeyFrameSize: 0.05
Viewer.KeyFrameLineWidth: 1.0
Viewer.GraphLineWidth: 0.9
Viewer.PointSize: 2.0
Viewer.CameraSize: 0.08
Viewer.CameraLineWidth: 3.0
Viewer.ViewpointX: 0.0
Viewer.ViewpointY: -0.7
Viewer.ViewpointZ: -3.5
Viewer.ViewpointF: 500.0
)";

bool Readable(const std::string& path)
{
    return std::ifstream(path.c_str()).good();
}

std::string WriteConfig()
{
    char pattern[] = "/tmp/orbslam3_offline_core_XXXXXX";
    const int descriptor = ::mkstemp(pattern);
    EXPECT_GE(descriptor, 0);
    if (descriptor < 0)
    {
        return std::string();
    }
    const std::string path(pattern);
    std::ofstream output(path.c_str());
    output << kConfigText;
    output.close();
    ::close(descriptor);
    return path;
}
}  // namespace

TEST(OfflineCoreLifecycle, RealCoreReachesReadyDrainsAndShutsDownWithoutWork)
{
    ASSERT_TRUE(Readable(OFFLINE_TEST_VOCABULARY))
        << "missing built vocabulary: " << OFFLINE_TEST_VOCABULARY;
    const std::string config_path = WriteConfig();
    ASSERT_FALSE(config_path.empty());

    ORB_SLAM3::System system(
        OFFLINE_TEST_VOCABULARY, config_path,
        ORB_SLAM3::System::IMU_STEREO, false);
    system.SetOfflineMode(true);
    system.SetOfflineTimeOriginNs(123456789LL);
    system.SetOfflineWaitTimeouts(5000, 5000);

    std::string reason;
    EXPECT_EQ(
        system.WaitForLocalMappingReady(&reason),
        ORB_SLAM3::OfflineWaitStatus::READY) << reason;

    std::atomic<bool> cancel(true);
    system.SetOfflineCancelFlag(&cancel);
    EXPECT_EQ(
        system.WaitForOfflineDrain(&reason),
        ORB_SLAM3::OfflineWaitStatus::INTERRUPTED);
    cancel.store(false);

    EXPECT_EQ(
        system.WaitForOfflineDrain(&reason),
        ORB_SLAM3::OfflineWaitStatus::COMPLETE) << reason;

    std::vector<ORB_SLAM3::OfflineCameraPose> poses;
    std::vector<ORB_SLAM3::OfflineMapInfo> maps;
    std::string snapshot_error;
    EXPECT_TRUE(system.GetFinalOfflineSnapshot(&poses, &maps, &snapshot_error))
        << snapshot_error;
    EXPECT_TRUE(poses.empty());

    EXPECT_EQ(
        system.ShutdownOffline(&reason),
        ORB_SLAM3::OfflineWaitStatus::COMPLETE) << reason;
    EXPECT_TRUE(system.isShutDown());

    EXPECT_EQ(
        system.ShutdownOffline(&reason),
        ORB_SLAM3::OfflineWaitStatus::COMPLETE);
    EXPECT_FALSE(system.isInertialBA1Initialized());
    std::remove(config_path.c_str());
}

TEST(OfflineCoreLifecycle, GbaWorkersCanBeJoinedOnAllCompletionPaths)
{
    ASSERT_TRUE(Readable(OFFLINE_TEST_VOCABULARY))
        << "missing built vocabulary: " << OFFLINE_TEST_VOCABULARY;
    ORB_SLAM3::ORBVocabulary vocabulary;
    ASSERT_TRUE(vocabulary.loadFromTextFile(OFFLINE_TEST_VOCABULARY));
    ORB_SLAM3::KeyFrameDatabase database(vocabulary);
    ORB_SLAM3::Atlas atlas(0);
    ORB_SLAM3::LoopClosing loop_closing(&atlas, &database, &vocabulary, true, true);
    ORB_SLAM3::Map* map = atlas.GetCurrentMap();
    ASSERT_NE(map, nullptr);

    // Empty maps exercise the early-completion path and a normal join.
    loop_closing.StartGlobalBundleAdjustment(map, 1UL);
    EXPECT_TRUE(loop_closing.HasGlobalBundleAdjustmentWorker());
    loop_closing.RequestStopGlobalBundleAdjustmentAndJoin();
    EXPECT_FALSE(loop_closing.isRunningGBA());
    EXPECT_TRUE(loop_closing.isFinishedGBA());
    EXPECT_FALSE(loop_closing.HasGlobalBundleAdjustmentWorker());

    // A second start after the previous worker completed must not reuse it.
    loop_closing.StartGlobalBundleAdjustment(map, 2UL);
    loop_closing.RequestStopGlobalBundleAdjustmentAndJoin();
    EXPECT_FALSE(loop_closing.isRunningGBA());
    EXPECT_TRUE(loop_closing.isFinishedGBA());
    EXPECT_FALSE(loop_closing.HasGlobalBundleAdjustmentWorker());

    // Replacing a still-registered worker must join the old one before the
    // new worker can share mbStopGBA/map state.
    loop_closing.StartGlobalBundleAdjustment(map, 3UL);
    loop_closing.StartGlobalBundleAdjustment(map, 4UL);
    EXPECT_TRUE(loop_closing.HasGlobalBundleAdjustmentWorker());
    loop_closing.RequestStopGlobalBundleAdjustmentAndJoin();
    EXPECT_FALSE(loop_closing.isRunningGBA());
    EXPECT_TRUE(loop_closing.isFinishedGBA());
    EXPECT_FALSE(loop_closing.HasGlobalBundleAdjustmentWorker());
}

TEST(OfflineCoreLifecycle, FreshLoopClosingInitializesOfflineAtomics)
{
    void* storage = nullptr;
    ASSERT_EQ(posix_memalign(&storage, 64, sizeof(ORB_SLAM3::LoopClosing)), 0);
    std::memset(storage, 0xa5, sizeof(ORB_SLAM3::LoopClosing));
    ORB_SLAM3::LoopClosing* loop_closing = new (storage) ORB_SLAM3::LoopClosing(
        nullptr, nullptr, nullptr, true, true);
    EXPECT_FALSE(loop_closing->IsProcessing());
    EXPECT_EQ(loop_closing->GetWorkGeneration(), 0UL);
    EXPECT_EQ(loop_closing->PendingKeyFrames(), 0);
    EXPECT_FALSE(loop_closing->HasGlobalBundleAdjustmentWorker());
    loop_closing->~LoopClosing();
    std::free(storage);
}

TEST(OfflineCoreLifecycle, EarlyFinishedLoopWorkerWithPendingWorkIsNotComplete)
{
    ASSERT_TRUE(Readable(OFFLINE_TEST_VOCABULARY))
        << "missing built vocabulary: " << OFFLINE_TEST_VOCABULARY;
    const std::string config_path = WriteConfig();
    ASSERT_FALSE(config_path.empty());

    ORB_SLAM3::System system(
        OFFLINE_TEST_VOCABULARY, config_path,
        ORB_SLAM3::System::IMU_STEREO, false);
    system.SetOfflineMode(true);
    system.SetOfflineWaitTimeouts(5000, 1000);
    std::string reason;
    ASSERT_EQ(
        system.WaitForLocalMappingReady(&reason),
        ORB_SLAM3::OfflineWaitStatus::READY) << reason;

    system.mpLoopCloser->RequestFinish();
    for (int i = 0; i < 5000 && !system.mpLoopCloser->isFinished(); ++i)
    {
        usleep(1000);
    }
    ASSERT_TRUE(system.mpLoopCloser->isFinished());

    ORB_SLAM3::KeyFrame* pending = new ORB_SLAM3::KeyFrame();
    pending->mnId = 999UL;
    system.mpLoopCloser->InsertKeyFrame(pending);
    ASSERT_EQ(system.mpLoopCloser->PendingKeyFrames(), 1);

    EXPECT_EQ(
        system.WaitForOfflineDrain(&reason),
        ORB_SLAM3::OfflineWaitStatus::UNAVAILABLE);
    EXPECT_NE(reason.find("finished"), std::string::npos);
    EXPECT_EQ(system.mpLoopCloser->PendingKeyFrames(), 1);

    {
        std::unique_lock<std::mutex> lock(system.mpLoopCloser->mMutexLoopQueue);
        system.mpLoopCloser->mlpLoopKeyFrameQueue.clear();
    }
    EXPECT_EQ(
        system.ShutdownOffline(&reason),
        ORB_SLAM3::OfflineWaitStatus::COMPLETE) << reason;
    EXPECT_TRUE(system.isShutDown());
    EXPECT_EQ(
        system.ShutdownOffline(&reason),
        ORB_SLAM3::OfflineWaitStatus::COMPLETE);
    std::remove(config_path.c_str());
}

TEST(OfflineCoreLifecycle, DrainDoesNotCompleteWhileLocalMappingIsStopped)
{
    ASSERT_TRUE(Readable(OFFLINE_TEST_VOCABULARY))
        << "missing built vocabulary: " << OFFLINE_TEST_VOCABULARY;
    const std::string config_path = WriteConfig();
    ASSERT_FALSE(config_path.empty());

    ORB_SLAM3::System system(
        OFFLINE_TEST_VOCABULARY, config_path,
        ORB_SLAM3::System::IMU_STEREO, false);
    system.SetOfflineMode(true);
    system.SetOfflineWaitTimeouts(5000, 100);
    std::string reason;
    ASSERT_EQ(
        system.WaitForLocalMappingReady(&reason),
        ORB_SLAM3::OfflineWaitStatus::READY) << reason;

    system.mpLocalMapper->RequestStop();
    EXPECT_EQ(
        system.WaitForOfflineDrain(&reason),
        ORB_SLAM3::OfflineWaitStatus::TIMEOUT);
    EXPECT_NE(reason.find("offline drain timed out"), std::string::npos);

    system.mpLocalMapper->Release();
    EXPECT_EQ(
        system.ShutdownOffline(&reason),
        ORB_SLAM3::OfflineWaitStatus::COMPLETE) << reason;
    std::remove(config_path.c_str());
}

TEST(OfflineCoreLifecycle, UpdateFrameImuScalesOfflineFrameRecords)
{
    ASSERT_TRUE(Readable(OFFLINE_TEST_VOCABULARY))
        << "missing built vocabulary: " << OFFLINE_TEST_VOCABULARY;
    const std::string config_path = WriteConfig();
    ASSERT_FALSE(config_path.empty());

    ORB_SLAM3::System system(
        OFFLINE_TEST_VOCABULARY, config_path,
        ORB_SLAM3::System::IMU_STEREO, false);
    ORB_SLAM3::Map* map = system.mpAtlas->GetCurrentMap();
    ASSERT_NE(map, nullptr);

    ORB_SLAM3::KeyFrame* reference = new ORB_SLAM3::KeyFrame();
    reference->mnId = 77UL;
    reference->UpdateMap(map);

    ORB_SLAM3::OfflineFrameRecord record;
    record.timestampNs = 123456789LL;
    record.poseValid = true;
    record.reference = reference;
    record.relativePose = Sophus::SE3f(
        Eigen::Matrix3f::Identity(), Eigen::Vector3f(1.0f, 2.0f, 3.0f));
    system.mpTracker->mlOfflineFrameRecords.push_back(record);

    system.mpTracker->mLastFrame = ORB_SLAM3::Frame();
    system.mpTracker->mLastFrame.mnId = 0UL;
    system.mpTracker->mLastFrame.mpLastKeyFrame = reference;
    system.mpTracker->mCurrentFrame = ORB_SLAM3::Frame();
    system.mpTracker->mCurrentFrame.mnId = 1UL;
    system.mpTracker->mCurrentFrame.setIntegrated();

    system.mpTracker->UpdateFrameIMU(
        2.0f, ORB_SLAM3::IMU::Bias(), reference);

    const Eigen::Vector3f translation =
        system.mpTracker->mlOfflineFrameRecords.front().relativePose.translation();
    EXPECT_NEAR(translation.x(), 2.0f, 1e-6f);
    EXPECT_NEAR(translation.y(), 4.0f, 1e-6f);
    EXPECT_NEAR(translation.z(), 6.0f, 1e-6f);

    EXPECT_EQ(
        system.ShutdownOffline(nullptr),
        ORB_SLAM3::OfflineWaitStatus::COMPLETE);
    std::remove(config_path.c_str());
}

TEST(OfflineCoreLifecycle, FinishedWorkerWithPendingResetIsNotComplete)
{
    ASSERT_TRUE(Readable(OFFLINE_TEST_VOCABULARY))
        << "missing built vocabulary: " << OFFLINE_TEST_VOCABULARY;
    const std::string config_path = WriteConfig();
    ASSERT_FALSE(config_path.empty());

    ORB_SLAM3::System system(
        OFFLINE_TEST_VOCABULARY, config_path,
        ORB_SLAM3::System::IMU_STEREO, false);
    system.SetOfflineMode(true);
    system.SetOfflineWaitTimeouts(5000, 1000);
    std::string reason;
    ASSERT_EQ(
        system.WaitForLocalMappingReady(&reason),
        ORB_SLAM3::OfflineWaitStatus::READY) << reason;

    system.mpLoopCloser->RequestFinish();
    for (int i = 0; i < 5000 && !system.mpLoopCloser->isFinished(); ++i)
    {
        usleep(1000);
    }
    ASSERT_TRUE(system.mpLoopCloser->isFinished());

    // Test-only seed: a finished worker that still reports a pending reset
    // must not be treated as a complete drain.
    system.mpLocalMapper->mbResetRequested = true;
    EXPECT_EQ(
        system.WaitForOfflineDrain(&reason),
        ORB_SLAM3::OfflineWaitStatus::UNAVAILABLE);
    EXPECT_NE(reason.find("finished"), std::string::npos);
    system.mpLocalMapper->mbResetRequested = false;

    EXPECT_EQ(
        system.ShutdownOffline(&reason),
        ORB_SLAM3::OfflineWaitStatus::COMPLETE) << reason;
    std::remove(config_path.c_str());
}

TEST(OfflineCoreLifecycle, ZeroFeatureFramesInitializeImuMutexOnAllConstructors)
{
    ORB_SLAM3::Frame default_frame;
    default_frame.setIntegrated();
    EXPECT_TRUE(default_frame.imuIsPreintegrated());

    ORB_SLAM3::ORBextractor extractor_left(20, 1.2f, 1, 20, 7);
    ORB_SLAM3::ORBextractor extractor_right(20, 1.2f, 1, 20, 7);
    cv::Mat k = cv::Mat::eye(3, 3, CV_32F);
    k.at<float>(0, 0) = 300.0f;
    k.at<float>(1, 1) = 300.0f;
    k.at<float>(0, 2) = 32.0f;
    k.at<float>(1, 2) = 32.0f;
    cv::Mat dist_coef = cv::Mat::zeros(4, 1, CV_32F);
    const cv::Mat blank = cv::Mat::zeros(64, 64, CV_8UC1);
    ORB_SLAM3::IMU::Calib calib;

    ORB_SLAM3::Frame pinhole_stereo(
        blank, blank, 0.0, &extractor_left, &extractor_right, nullptr,
        k, dist_coef, 0.05f, 40.0f, nullptr, nullptr, calib);
    pinhole_stereo.setIntegrated();
    EXPECT_TRUE(pinhole_stereo.imuIsPreintegrated());

    const std::vector<float> fisheye_parameters = {
        300.0f, 300.0f, 32.0f, 32.0f, 0.01f, 0.001f, 0.0f, 0.0f};
    ORB_SLAM3::KannalaBrandt8 camera_left(fisheye_parameters);
    ORB_SLAM3::KannalaBrandt8 camera_right(fisheye_parameters);
    Sophus::SE3f tlr(
        Eigen::Matrix3f::Identity(), Eigen::Vector3f::Zero());
    ORB_SLAM3::Frame fisheye_stereo(
        blank, blank, 0.0, &extractor_left, &extractor_right, nullptr,
        k, dist_coef, 0.05f, 40.0f, &camera_left, &camera_right, tlr,
        nullptr, calib);
    fisheye_stereo.setIntegrated();
    EXPECT_TRUE(fisheye_stereo.imuIsPreintegrated());
}

