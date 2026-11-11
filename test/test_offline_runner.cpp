/**
 * @file test_offline_runner.cpp
 * @brief 用小型 fake source/sink 测试离线流程、覆盖率门槛、失败契约和输出。
 */

#include "offline-runner.hpp"

#include "gtest/gtest.h"

#include <sys/stat.h>
#include <unistd.h>

#include <algorithm>
#include <atomic>
#include <cstdint>
#include <fstream>
#include <iterator>
#include <string>
#include <vector>

namespace
{
std::string MakeTempDirectory()
{
    char pattern[] = "/tmp/orbslam3_offline_runner_XXXXXX";
    char* directory = ::mkdtemp(pattern);
    EXPECT_NE(directory, nullptr);
    return directory ? std::string(directory) : std::string();
}

bool Exists(const std::string& path)
{
    struct stat info;
    return ::stat(path.c_str(), &info) == 0;
}

std::string ReadFile(const std::string& path)
{
    std::ifstream input(path.c_str());
    return std::string(
        (std::istreambuf_iterator<char>(input)),
        std::istreambuf_iterator<char>());
}

ORB_SLAM3::OfflineCameraPose MakePose(
    const std::int64_t stamp_ns,
    const unsigned long map_id)
{
    ORB_SLAM3::OfflineCameraPose pose;
    pose.timestampNs = stamp_ns;
    pose.translation = Eigen::Vector3f(1.0f, 2.0f, 3.0f);
    pose.orientation = Eigen::Quaternionf::Identity();
    pose.mapId = map_id;
    return pose;
}

class FakeSource : public OfflineFrameSource
{
public:
    bool Inspect(
        const OfflineBagOptions&,
        OfflineBagInfo* info,
        std::string* error_message) override
    {
        if (!inspect_ok)
        {
            *error_message = "injected inspect failure";
            return false;
        }
        *info = info_;
        error_message->clear();
        return true;
    }

    bool ForEachImage(
        const OfflineBagOptions&,
        const OfflineBagInfo& info,
        const std::function<bool(const OfflineImageFrame&, std::string*)>& callback,
        std::string* error_message) override
    {
        for (std::size_t i = 0U; i < info.image_stamps_ns.size(); ++i)
        {
            OfflineImageFrame frame;
            frame.timestamp_ns = info.image_stamps_ns[i];
            frame.left = cv::Mat::zeros(2, 2, CV_8UC1);
            frame.right = cv::Mat::zeros(2, 2, CV_8UC1);
            if (!callback(frame, error_message))
            {
                return false;
            }
        }
        expected_callbacks = info.image_stamps_ns.size();
        return true;
    }

    OfflineBagInfo info_;
    bool inspect_ok = true;
    std::size_t expected_callbacks = 0U;
};

class FakeSink : public OfflineSlamSink
{
public:
    bool Configure(std::int64_t origin_ns, std::string*) override
    {
        origin = origin_ns;
        configure_called = true;
        return configure_ok;
    }

    ORB_SLAM3::OfflineWaitStatus WaitReady(std::string*) override
    {
        ++wait_ready_calls;
        return wait_status;
    }

    bool Track(
        const cv::Mat&,
        const cv::Mat&,
        std::int64_t timestamp_ns,
        const std::vector<ORB_SLAM3::IMU::Point>& measurements,
        int* tracking_state,
        std::string*) override
    {
        last_timestamp = timestamp_ns;
        batch_sizes.push_back(measurements.size());
        for (const ORB_SLAM3::IMU::Point& measurement : measurements)
        {
            imu_times.push_back(measurement.t);
        }
        *tracking_state = ORB_SLAM3::Tracking::OK;
        ++track_calls;
        return true;
    }

    ORB_SLAM3::OfflineWaitStatus Finish(std::string*) override
    {
        ++finish_calls;
        return finish_status;
    }

    bool Snapshot(
        std::vector<ORB_SLAM3::OfflineCameraPose>* poses,
        std::vector<ORB_SLAM3::OfflineMapInfo>* maps,
        std::string*) override
    {
        *poses = poses_;
        *maps = maps_;
        return snapshot_ok;
    }

    ORB_SLAM3::OfflineWaitStatus wait_status =
        ORB_SLAM3::OfflineWaitStatus::READY;
    ORB_SLAM3::OfflineWaitStatus finish_status =
        ORB_SLAM3::OfflineWaitStatus::COMPLETE;
    std::vector<ORB_SLAM3::OfflineCameraPose> poses_;
    std::vector<ORB_SLAM3::OfflineMapInfo> maps_;
    bool snapshot_ok = true;
    bool configure_ok = true;
    bool configure_called = false;
    std::int64_t origin = 0;
    std::int64_t last_timestamp = -1;
    std::size_t track_calls = 0U;
    std::size_t finish_calls = 0U;
    std::size_t wait_ready_calls = 0U;
    std::vector<std::size_t> batch_sizes;
    std::vector<double> imu_times;
};

constexpr std::int64_t kFirstImageNs = 1000000000LL;
constexpr std::int64_t kFrameStepNs = 10000000LL;

OfflineBagInfo MakeInfo(const std::size_t frame_count = 3U)
{
    OfflineBagInfo info;
    for (std::size_t i = 0U; i < frame_count; ++i)
    {
        info.image_stamps_ns.push_back(
            kFirstImageNs + static_cast<std::int64_t>(i) * kFrameStepNs);
    }
    const std::int64_t last_image_ns = info.image_stamps_ns.back();
    for (std::int64_t stamp = kFirstImageNs - 10000000LL;
         stamp <= last_image_ns + 10000000LL; stamp += 1000000LL)
    {
        OfflineImuSample sample;
        sample.raw_timestamp_ns = stamp;
        sample.corrected_timestamp_ns = stamp;
        sample.corrected_relative_sec =
            static_cast<double>(stamp - kFirstImageNs) * 1e-9;
        sample.acceleration[0] = 0.0;
        sample.acceleration[1] = 0.0;
        sample.acceleration[2] = 9.81;
        sample.angular_velocity[0] = 0.0;
        sample.angular_velocity[1] = 0.0;
        sample.angular_velocity[2] = 0.0;
        info.imu_samples.push_back(sample);
    }
    info.first_corrected_imu_ns = info.imu_samples.front().corrected_timestamp_ns;
    info.last_corrected_imu_ns = info.imu_samples.back().corrected_timestamp_ns;
    info.image_width = 4;
    info.image_height = 2;
    info.camera_width = 2;
    return info;
}

OfflineBagInfo MakeInfoWithoutCoverage()
{
    OfflineBagInfo info = MakeInfo(3U);
    for (OfflineImuSample& sample : info.imu_samples)
    {
        sample.raw_timestamp_ns += 100000000000LL;
        sample.corrected_timestamp_ns += 100000000000LL;
        sample.corrected_relative_sec += 100.0;
    }
    info.first_corrected_imu_ns = info.imu_samples.front().corrected_timestamp_ns;
    info.last_corrected_imu_ns = info.imu_samples.back().corrected_timestamp_ns;
    return info;
}

OfflineRunOptions MakeOptions(
    const std::string& directory,
    const std::atomic<bool>* cancel)
{
    OfflineRunOptions options;
    options.bag.bag_path = directory + "/input.mcap";
    options.output_path = directory + "/trajectory.txt";
    options.vocabulary_file = directory + "/vocab.txt";
    options.settings_file = directory + "/settings.yaml";
    options.cancel = cancel;
    options.orb_slam3_commit = "test-commit";
    options.opencv_version = "test-opencv";
    return options;
}

std::vector<ORB_SLAM3::OfflineMapInfo> TwoMaps()
{
    std::vector<ORB_SLAM3::OfflineMapInfo> maps(2);
    maps[0].id = 2;
    maps[0].imuInitialized = true;
    maps[1].id = 5;
    maps[1].imuInitialized = true;
    return maps;
}

std::vector<ORB_SLAM3::OfflineCameraPose> MakeFullCoveragePoses()
{
    std::vector<ORB_SLAM3::OfflineCameraPose> poses;
    for (std::size_t i = 0U; i < 20U; ++i)
    {
        poses.push_back(MakePose(
            kFirstImageNs + static_cast<std::int64_t>(i) * kFrameStepNs,
            i == 19U ? 5UL : 2UL));
    }
    return poses;
}

std::vector<ORB_SLAM3::OfflineCameraPose> MakePosesForMaps(
    const std::size_t main_count,
    const std::size_t side_count)
{
    std::vector<ORB_SLAM3::OfflineCameraPose> poses;
    for (std::size_t i = 0U; i < main_count; ++i)
    {
        poses.push_back(MakePose(
            kFirstImageNs + static_cast<std::int64_t>(i) * kFrameStepNs, 2UL));
    }
    for (std::size_t i = 0U; i < side_count; ++i)
    {
        poses.push_back(MakePose(
            kFirstImageNs +
                static_cast<std::int64_t>(main_count + i) * kFrameStepNs,
            5UL));
    }
    return poses;
}

void ExpectNoOldGeneration(const std::string& directory)
{
    EXPECT_FALSE(Exists(directory + "/trajectory.txt"));
    EXPECT_FALSE(Exists(directory + "/trajectory.map-5.txt"));
    EXPECT_FALSE(ReadFile(directory + "/trajectory.manifest.txt").find(
        "file=trajectory.txt") != std::string::npos);
    EXPECT_FALSE(ReadFile(directory + "/trajectory.manifest.txt").find(
        "file=trajectory.map-5.txt") != std::string::npos);
    const std::string summary = ReadFile(directory + "/trajectory.summary.json");
    EXPECT_NE(summary.find("\"status\": \"failed\""), std::string::npos);
    EXPECT_NE(summary.find("\"exit_success\": false"), std::string::npos);
}

enum class EarlyFailure
{
    Inspect,
    Configure,
    ZeroEligible
};

void RunOldGenerationThenEarlyFailure(const EarlyFailure failure)
{
    const std::string directory = MakeTempDirectory();
    const std::atomic<bool> cancel(false);
    FakeSource source;
    source.info_ = MakeInfo(20U);
    FakeSink sink;
    sink.poses_ = MakeFullCoveragePoses();
    sink.maps_ = TwoMaps();

    OfflineRunOptions options = MakeOptions(directory, &cancel);
    OfflineRunReport report;
    std::string error;
    ASSERT_TRUE(RunOfflinePipeline(&source, &sink, options, &report, &error)) << error;
    ASSERT_TRUE(Exists(directory + "/trajectory.txt"));
    ASSERT_TRUE(Exists(directory + "/trajectory.map-5.txt"));

    options.overwrite = true;
    switch (failure)
    {
        case EarlyFailure::Inspect:
            source.inspect_ok = false;
            break;
        case EarlyFailure::Configure:
            sink.configure_ok = false;
            break;
        case EarlyFailure::ZeroEligible:
            source.info_ = MakeInfoWithoutCoverage();
            break;
    }
    EXPECT_FALSE(RunOfflinePipeline(&source, &sink, options, &report, &error));
    EXPECT_EQ(report.status, "failed");
    ExpectNoOldGeneration(directory);

    // A later valid run must not be blocked by leftover files from the older
    // generation that the failed run did not own.
    source.inspect_ok = true;
    sink.configure_ok = true;
    source.info_ = MakeInfo(20U);
    sink.poses_ = MakeFullCoveragePoses();
    sink.maps_ = TwoMaps();
    EXPECT_TRUE(RunOfflinePipeline(&source, &sink, options, &report, &error))
        << error;
    EXPECT_EQ(report.status, "complete");
    EXPECT_TRUE(Exists(directory + "/trajectory.txt"));
    EXPECT_TRUE(Exists(directory + "/trajectory.map-5.txt"));
}
}  // namespace

TEST(OfflineRunner, CompletesPipelineAtThresholdsAndWritesMainPlusSideMap)
{
    const std::string directory = MakeTempDirectory();
    const std::atomic<bool> cancel(false);
    FakeSource source;
    source.info_ = MakeInfo(20U);
    FakeSink sink;
    sink.poses_ = MakeFullCoveragePoses();
    sink.maps_ = TwoMaps();

    OfflineRunOptions options = MakeOptions(directory, &cancel);
    OfflineRunReport report;
    std::string error;
    ASSERT_TRUE(RunOfflinePipeline(&source, &sink, options, &report, &error)) << error;
    EXPECT_EQ(report.status, "complete");
    EXPECT_EQ(report.total_images, 20U);
    EXPECT_EQ(report.eligible_images, 20U);
    EXPECT_EQ(report.submitted_frames, 20U);
    EXPECT_EQ(report.valid_poses, 20U);
    EXPECT_EQ(report.main_map_id, 2U);
    EXPECT_EQ(report.main_map_valid, 19U);
    EXPECT_DOUBLE_EQ(report.total_coverage, 1.0);
    EXPECT_DOUBLE_EQ(report.main_coverage, 0.95);
    EXPECT_EQ(source.expected_callbacks, 20U);
    EXPECT_EQ(sink.track_calls, 20U);
    EXPECT_EQ(sink.finish_calls, 1U);
    EXPECT_TRUE(sink.configure_called);
    EXPECT_FALSE(sink.imu_times.empty());
    for (std::size_t i = 1U; i < sink.imu_times.size(); ++i)
    {
        EXPECT_LT(sink.imu_times[i - 1U], sink.imu_times[i]);
    }
    EXPECT_TRUE(Exists(directory + "/trajectory.txt"));
    EXPECT_TRUE(Exists(directory + "/trajectory.map-5.txt"));
    EXPECT_TRUE(Exists(directory + "/trajectory.summary.json"));
    EXPECT_TRUE(Exists(directory + "/trajectory.manifest.txt"));
    const std::string summary = ReadFile(directory + "/trajectory.summary.json");
    EXPECT_NE(summary.find("\"status\": \"complete\""), std::string::npos);
    EXPECT_NE(summary.find("\"exit_success\": true"), std::string::npos);
    EXPECT_NE(summary.find("\"total_images\": 20"), std::string::npos);
    EXPECT_NE(summary.find("\"main\": 0.9"), std::string::npos);
    const std::string main = ReadFile(directory + "/trajectory.txt");
    EXPECT_EQ(std::count(main.begin(), main.end(), '\n'), 19);
}

TEST(OfflineRunner, AcceptsExactTotalAndMainCoverageThresholds)
{
    const std::string directory = MakeTempDirectory();
    const std::atomic<bool> cancel(false);
    FakeSource source;
    source.info_ = MakeInfo(20U);
    FakeSink sink;
    sink.poses_ = MakePosesForMaps(18U, 1U);
    sink.maps_ = TwoMaps();

    OfflineRunOptions options = MakeOptions(directory, &cancel);
    OfflineRunReport report;
    std::string error;
    ASSERT_TRUE(RunOfflinePipeline(&source, &sink, options, &report, &error)) << error;
    EXPECT_EQ(report.status, "complete");
    EXPECT_DOUBLE_EQ(report.total_coverage, 0.95);
    EXPECT_DOUBLE_EQ(report.main_coverage, 0.90);
    EXPECT_EQ(report.main_map_valid, 18U);
    EXPECT_TRUE(Exists(directory + "/trajectory.txt"));
}

TEST(OfflineRunner, RejectsLowTotalCoverageButKeepsPartialTrajectory)
{
    const std::string directory = MakeTempDirectory();
    const std::atomic<bool> cancel(false);
    FakeSource source;
    source.info_ = MakeInfo(20U);
    FakeSink sink;
    sink.poses_ = MakePosesForMaps(18U, 0U);
    sink.maps_ = TwoMaps();

    OfflineRunOptions options = MakeOptions(directory, &cancel);
    OfflineRunReport report;
    std::string error;
    EXPECT_FALSE(RunOfflinePipeline(&source, &sink, options, &report, &error));
    EXPECT_EQ(report.status, "failed");
    EXPECT_EQ(report.eligible_images, 20U);
    EXPECT_DOUBLE_EQ(report.total_coverage, 0.90);
    EXPECT_NE(report.failure_reason.find("总覆盖率"), std::string::npos);
    EXPECT_TRUE(Exists(directory + "/trajectory.txt"));
    const std::string main = ReadFile(directory + "/trajectory.txt");
    EXPECT_EQ(std::count(main.begin(), main.end(), '\n'), 18);
    const std::string summary = ReadFile(directory + "/trajectory.summary.json");
    EXPECT_NE(summary.find("\"status\": \"failed\""), std::string::npos);
    EXPECT_NE(summary.find("\"exit_success\": false"), std::string::npos);
}

TEST(OfflineRunner, RejectsLowMainCoverage)
{
    const std::string directory = MakeTempDirectory();
    const std::atomic<bool> cancel(false);
    FakeSource source;
    source.info_ = MakeInfo(20U);
    FakeSink sink;
    sink.poses_ = MakePosesForMaps(17U, 2U);
    sink.maps_ = TwoMaps();

    OfflineRunOptions options = MakeOptions(directory, &cancel);
    OfflineRunReport report;
    std::string error;
    EXPECT_FALSE(RunOfflinePipeline(&source, &sink, options, &report, &error));
    EXPECT_EQ(report.status, "failed");
    EXPECT_EQ(report.eligible_images, 20U);
    EXPECT_DOUBLE_EQ(report.total_coverage, 0.95);
    EXPECT_DOUBLE_EQ(report.main_coverage, 0.85);
    EXPECT_NE(report.failure_reason.find("主 map 覆盖率"), std::string::npos);
    EXPECT_TRUE(Exists(directory + "/trajectory.txt"));
}

TEST(OfflineRunner, SparseTailWithoutEndpointFailsWithoutShrinkingEligible)
{
    const std::string directory = MakeTempDirectory();
    const std::atomic<bool> cancel(false);
    FakeSource source;
    source.info_.image_stamps_ns = {kFirstImageNs, kFirstImageNs + 10000000LL};
    for (const std::int64_t delta : {-5000000LL, 15000000LL})
    {
        OfflineImuSample sample;
        sample.raw_timestamp_ns = kFirstImageNs + delta;
        sample.corrected_relative_sec = static_cast<double>(delta) * 1e-9;
        sample.acceleration[0] = 0.0;
        sample.acceleration[1] = 0.0;
        sample.acceleration[2] = 9.81;
        source.info_.imu_samples.push_back(sample);
    }
    source.info_.image_width = 4;
    source.info_.image_height = 2;
    source.info_.camera_width = 2;

    FakeSink sink;
    sink.poses_ = {MakePose(kFirstImageNs, 2UL)};
    sink.maps_ = TwoMaps();

    OfflineRunOptions options = MakeOptions(directory, &cancel);
    OfflineRunReport report;
    std::string error;
    EXPECT_FALSE(RunOfflinePipeline(&source, &sink, options, &report, &error));
    EXPECT_EQ(report.status, "failed");
    EXPECT_EQ(report.total_images, 2U);
    EXPECT_EQ(report.eligible_images, 2U);
    EXPECT_EQ(report.submitted_frames, 1U);
    EXPECT_EQ(report.skipped_after_imu, 0U);
    EXPECT_NE(error.find("IMU"), std::string::npos);
}

TEST(OfflineRunner, OverwriteFailedRunReplacesOldGenerationAndAllowsRerun)
{
    const std::string directory = MakeTempDirectory();
    const std::atomic<bool> cancel(false);
    FakeSource source;
    source.info_ = MakeInfo(20U);
    FakeSink sink;
    sink.poses_ = MakeFullCoveragePoses();
    sink.maps_ = TwoMaps();
    OfflineRunOptions options = MakeOptions(directory, &cancel);
    OfflineRunReport report;
    std::string error;
    ASSERT_TRUE(RunOfflinePipeline(&source, &sink, options, &report, &error)) << error;

    options.overwrite = true;
    sink.poses_.clear();
    EXPECT_FALSE(RunOfflinePipeline(&source, &sink, options, &report, &error));
    EXPECT_EQ(report.status, "failed");
    ExpectNoOldGeneration(directory);
    EXPECT_TRUE(Exists(directory + "/trajectory.summary.json"));
    EXPECT_TRUE(Exists(directory + "/trajectory.manifest.txt"));

    sink.poses_ = MakeFullCoveragePoses();
    EXPECT_TRUE(RunOfflinePipeline(&source, &sink, options, &report, &error)) << error;
    EXPECT_EQ(report.status, "complete");
    EXPECT_TRUE(Exists(directory + "/trajectory.txt"));
    EXPECT_TRUE(Exists(directory + "/trajectory.map-5.txt"));
    EXPECT_NE(ReadFile(directory + "/trajectory.summary.json").find(
        "\"status\": \"complete\""), std::string::npos);
}

TEST(OfflineRunner, OverwriteInspectFailureCleansOldGeneration)
{
    RunOldGenerationThenEarlyFailure(EarlyFailure::Inspect);
}

TEST(OfflineRunner, OverwriteConfigureFailureCleansOldGeneration)
{
    RunOldGenerationThenEarlyFailure(EarlyFailure::Configure);
}

TEST(OfflineRunner, OverwriteZeroEligibleCleansOldGeneration)
{
    RunOldGenerationThenEarlyFailure(EarlyFailure::ZeroEligible);
}

TEST(OfflineRunner, CancellationReturnsInterruptedStatusAndSummary)
{
    const std::string directory = MakeTempDirectory();
    const std::atomic<bool> cancel(true);
    FakeSource source;
    source.info_ = MakeInfo();
    FakeSink sink;
    sink.poses_ = {MakePose(kFirstImageNs, 2)};
    sink.maps_ = TwoMaps();
    OfflineRunOptions options = MakeOptions(directory, &cancel);
    OfflineRunReport report;
    std::string error;
    EXPECT_FALSE(RunOfflinePipeline(&source, &sink, options, &report, &error));
    EXPECT_EQ(report.status, "interrupted");
    EXPECT_TRUE(report.summary_written);
    EXPECT_NE(ReadFile(directory + "/trajectory.summary.json").find(
                  "\"status\": \"interrupted\""), std::string::npos);
}

TEST(OfflineRunner, FailedFinishMarksRunFailedButKeepsDiagnostics)
{
    const std::string directory = MakeTempDirectory();
    const std::atomic<bool> cancel(false);
    FakeSource source;
    source.info_ = MakeInfo();
    FakeSink sink;
    sink.poses_ = {MakePose(kFirstImageNs, 2)};
    sink.maps_ = TwoMaps();
    sink.finish_status = ORB_SLAM3::OfflineWaitStatus::TIMEOUT;
    OfflineRunOptions options = MakeOptions(directory, &cancel);
    OfflineRunReport report;
    std::string error;
    EXPECT_FALSE(RunOfflinePipeline(&source, &sink, options, &report, &error));
    EXPECT_EQ(report.status, "failed");
    EXPECT_TRUE(report.summary_written);
    EXPECT_NE(ReadFile(directory + "/trajectory.summary.json").find(
                  "\"status\": \"failed\""), std::string::npos);
}

TEST(OfflineRunner, WaitTimeoutFailsWithoutTracking)
{
    const std::string directory = MakeTempDirectory();
    const std::atomic<bool> cancel(false);
    FakeSource source;
    source.info_ = MakeInfo();
    FakeSink sink;
    sink.wait_status = ORB_SLAM3::OfflineWaitStatus::TIMEOUT;
    OfflineRunOptions options = MakeOptions(directory, &cancel);
    OfflineRunReport report;
    std::string error;
    EXPECT_FALSE(RunOfflinePipeline(&source, &sink, options, &report, &error));
    EXPECT_EQ(report.status, "failed");
    EXPECT_EQ(sink.track_calls, 0U);
    EXPECT_NE(error.find("not ready"), std::string::npos);
}

TEST(OfflineRunner, NoInitializedMapNeverWritesEmptyTrajectory)
{
    const std::string directory = MakeTempDirectory();
    const std::atomic<bool> cancel(false);
    FakeSource source;
    source.info_ = MakeInfo();
    FakeSink sink;
    sink.poses_ = {MakePose(kFirstImageNs, 2)};
    sink.maps_.resize(1);
    sink.maps_[0].id = 2;
    sink.maps_[0].imuInitialized = false;
    OfflineRunOptions options = MakeOptions(directory, &cancel);
    OfflineRunReport report;
    std::string error;
    EXPECT_FALSE(RunOfflinePipeline(&source, &sink, options, &report, &error));
    EXPECT_EQ(report.status, "failed");
    EXPECT_FALSE(Exists(directory + "/trajectory.txt"));
    EXPECT_TRUE(report.summary_written);
}

TEST(OfflineRunner, ExistingOutputRequiresOverwriteAndIsNotModified)
{
    const std::string directory = MakeTempDirectory();
    const std::atomic<bool> cancel(false);
    FakeSource source;
    source.info_ = MakeInfo();
    FakeSink sink;
    sink.poses_ = {MakePose(kFirstImageNs, 2)};
    sink.maps_ = TwoMaps();
    const std::string main_path = directory + "/trajectory.txt";
    std::ofstream output(main_path.c_str());
    output << "keep-me\n";
    output.close();

    OfflineRunOptions options = MakeOptions(directory, &cancel);
    OfflineRunReport report;
    std::string error;
    EXPECT_FALSE(RunOfflinePipeline(&source, &sink, options, &report, &error));
    EXPECT_EQ(ReadFile(main_path), "keep-me\n");
    EXPECT_EQ(sink.finish_calls, 1U);
}

TEST(OfflineRunner, RejectsCollidingOutputBeforeTrackingOrPublishing)
{
    const std::string directory = MakeTempDirectory();
    const std::atomic<bool> cancel(false);
    FakeSource source;
    source.info_ = MakeInfo(20U);
    FakeSink sink;
    sink.poses_ = MakeFullCoveragePoses();
    sink.maps_ = TwoMaps();
    OfflineRunOptions options = MakeOptions(directory, &cancel);
    options.output_path = directory + "/trajectory.log";
    options.overwrite = true;
    const OfflineOutputPaths paths = BuildOfflineOutputPaths(options.output_path);
    OfflineRunReport report;
    std::string error;

    EXPECT_FALSE(RunOfflinePipeline(&source, &sink, options, &report, &error));
    EXPECT_EQ(report.status, "failed");
    EXPECT_NE(error.find("冲突"), std::string::npos) << error;
    EXPECT_FALSE(sink.configure_called);
    EXPECT_EQ(sink.track_calls, 0U);
    EXPECT_EQ(sink.finish_calls, 1U);
    EXPECT_FALSE(report.summary_written);
    EXPECT_FALSE(Exists(paths.main));
    EXPECT_FALSE(Exists(paths.summary));
    EXPECT_FALSE(Exists(paths.manifest));
    EXPECT_FALSE(Exists(paths.frames));
}

TEST(OfflineRunner, OverwriteRejectsUnownedMainWithoutPublishingNewDiagnostics)
{
    // Both a usable source and an unreadable bag must leave unowned outputs
    // untouched, without publishing new sidecars next to an old trajectory.
    for (const bool inspect_ok : {true, false})
    {
        SCOPED_TRACE(inspect_ok);
        const std::string directory = MakeTempDirectory();
        const std::atomic<bool> cancel(false);
        FakeSource source;
        source.info_ = MakeInfo(20U);
        source.inspect_ok = inspect_ok;
        FakeSink sink;
        sink.poses_ = MakeFullCoveragePoses();
        sink.maps_ = TwoMaps();
        OfflineRunOptions options = MakeOptions(directory, &cancel);
        options.overwrite = true;
        const OfflineOutputPaths paths = BuildOfflineOutputPaths(options.output_path);
        std::ofstream(paths.main) << "old-unowned-trajectory\n";
        OfflineRunReport report;
        std::string error;

        EXPECT_FALSE(RunOfflinePipeline(&source, &sink, options, &report, &error));
        EXPECT_EQ(report.status, "failed");
        EXPECT_NE(error.find("manifest"), std::string::npos) << error;
        EXPECT_FALSE(sink.configure_called);
        EXPECT_EQ(sink.track_calls, 0U);
        EXPECT_EQ(sink.finish_calls, 1U);
        EXPECT_FALSE(report.summary_written);
        EXPECT_EQ(ReadFile(paths.main), "old-unowned-trajectory\n");
        EXPECT_FALSE(Exists(paths.summary));
        EXPECT_FALSE(Exists(paths.log));
        EXPECT_FALSE(Exists(paths.manifest));
        EXPECT_FALSE(Exists(paths.frames));
        EXPECT_FALSE(Exists(directory + "/trajectory.map-5.txt"));
    }
}
