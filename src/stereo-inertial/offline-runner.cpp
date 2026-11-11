/**
 * @file offline-runner.cpp
 * @brief 实现离线 reader/sync/backpressure/drain/snapshot/output 流程。
 */

#include "offline-runner.hpp"

#include "offline-slam-sink.hpp"
#include "offline-sync.hpp"

#include "Tracking.h"

#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <iomanip>
#include <sstream>

namespace
{
/**
 * @brief 构造文本日志内容。
 */
std::string BuildLogText(
    const OfflineRunOptions& options,
    const OfflineRunReport& report)
{
    std::ostringstream out;
    out << "status=" << report.status << '\n'
        << "failure_reason=" << report.failure_reason << '\n'
        << "bag=" << report.bag_path << '\n'
        << "output=" << report.output_path << '\n'
        << "vocabulary=" << options.vocabulary_file << '\n'
        << "settings=" << options.settings_file << '\n'
        << "stereo_topic=" << options.bag.stereo_topic << '\n'
        << "imu_topic=" << options.bag.imu_topic << '\n'
        << std::setprecision(17)
        << "configured_imu_time_offset_sec=" << options.configured_imu_time_offset_sec << '\n'
        << "used_imu_time_offset_sec=" << options.bag.imu_time_offset_sec << '\n'
        << "imu_time_offset_overridden="
        << (options.imu_time_offset_overridden ? "true" : "false") << '\n'
        << "total_images=" << report.total_images << '\n'
        << "eligible_images=" << report.eligible_images << '\n'
        << "submitted_frames=" << report.submitted_frames << '\n'
        << "valid_poses=" << report.valid_poses << '\n'
        << "skipped_before_imu=" << report.skipped_before_imu << '\n'
        << "skipped_after_imu=" << report.skipped_after_imu << '\n'
        << "decoded_images=" << report.decoded_images << '\n'
        << "decode_failures=" << report.decode_failures << '\n'
        << "imu_samples=" << report.imu_samples << '\n'
        << "imu_insufficient_batches=" << report.imu_insufficient_batches << '\n'
        << "tracking_ok_frames=" << report.tracking_ok_frames << '\n'
        << "recently_lost_frames=" << report.recently_lost_frames << '\n'
        << "tracking_lost_frames=" << report.tracking_lost_frames << '\n'
        << "tracking_not_initialized_frames=" << report.tracking_not_initialized_frames << '\n'
        << "tracking_other_frames=" << report.tracking_other_frames << '\n'
        << "map_count=" << report.map_count << '\n'
        << "initialized_map_count=" << report.initialized_map_count << '\n'
        << "main_map_id=" << report.main_map_id << '\n'
        << "main_map_valid=" << report.main_map_valid << '\n'
        << "total_coverage=" << report.total_coverage << '\n'
        << "main_coverage=" << report.main_coverage << '\n';
    for (const ORB_SLAM3::OfflineMapInfo& map : report.maps)
    {
        out << "map=" << map.id
            << " imu_initialized=" << (map.imuInitialized ? "true" : "false")
            << " ba1=" << (map.inertialBA1 ? "true" : "false")
            << " ba2=" << (map.inertialBA2 ? "true" : "false")
            << " keyframes=" << map.keyFrameCount
            << " valid_poses=" << map.exportedFrames << '\n';
    }
    out << "orb_slam3_commit=" << options.orb_slam3_commit << '\n'
        << "opencv_version=" << options.opencv_version << '\n';
    return out.str();
}

/**
 * @brief 构造 summary JSON。
 */
std::string BuildSummaryJson(
    const OfflineRunOptions& options,
    const OfflineRunReport& report,
    const std::vector<OfflineOutputFile>& trajectory_files,
    const OfflineOutputPaths& paths)
{
    std::ostringstream out;
    out << std::setprecision(17);
    out << "{\n"
        << "  \"tool\": \"orbslam3-stereo-inertial-offline\",\n"
        << "  \"status\": " << JsonQuoted(report.status) << ",\n"
        << "  \"exit_success\": "
        << (report.status == "complete" ? "true" : "false") << ",\n"
        << "  \"failure_reason\": " << JsonQuoted(report.failure_reason) << ",\n"
        << "  \"bag\": " << JsonQuoted(report.bag_path) << ",\n"
        << "  \"output\": " << JsonQuoted(report.output_path) << ",\n"
        << "  \"vocabulary\": " << JsonQuoted(options.vocabulary_file) << ",\n"
        << "  \"settings\": " << JsonQuoted(options.settings_file) << ",\n"
        << "  \"stereo_topic\": " << JsonQuoted(options.bag.stereo_topic) << ",\n"
        << "  \"imu_topic\": " << JsonQuoted(options.bag.imu_topic) << ",\n"
        << "  \"imu_time_offset_sec\": {\n"
        << "    \"configured\": " << options.configured_imu_time_offset_sec << ",\n"
        << "    \"used\": " << options.bag.imu_time_offset_sec << ",\n"
        << "    \"overridden\": "
        << (options.imu_time_offset_overridden ? "true" : "false") << "\n"
        << "  },\n"
        << "  \"counts\": {\n"
        << "    \"total_images\": " << report.total_images << ",\n"
        << "    \"eligible_images\": " << report.eligible_images << ",\n"
        << "    \"submitted_frames\": " << report.submitted_frames << ",\n"
        << "    \"valid_poses\": " << report.valid_poses << ",\n"
        << "    \"skipped_before_imu\": " << report.skipped_before_imu << ",\n"
        << "    \"skipped_after_imu\": " << report.skipped_after_imu << ",\n"
        << "    \"decoded_images\": " << report.decoded_images << ",\n"
        << "    \"decode_failures\": " << report.decode_failures << ",\n"
        << "    \"imu_samples\": " << report.imu_samples << ",\n"
        << "    \"imu_insufficient_batches\": " << report.imu_insufficient_batches << ",\n"
        << "    \"tracking_ok_frames\": " << report.tracking_ok_frames << ",\n"
        << "    \"recently_lost_frames\": " << report.recently_lost_frames << ",\n"
        << "    \"tracking_lost_frames\": " << report.tracking_lost_frames << ",\n"
        << "    \"tracking_not_initialized_frames\": "
        << report.tracking_not_initialized_frames << ",\n"
        << "    \"tracking_other_frames\": " << report.tracking_other_frames << "\n"
        << "  },\n"
        << "  \"coverage\": {\n"
        << "    \"total\": " << report.total_coverage << ",\n"
        << "    \"main\": " << report.main_coverage << "\n"
        << "  },\n"
        << "  \"maps\": [\n";
    for (std::size_t i = 0U; i < report.maps.size(); ++i)
    {
        const ORB_SLAM3::OfflineMapInfo& map = report.maps[i];
        out << "    {\"id\": " << map.id
            << ", \"imu_initialized\": "
            << (map.imuInitialized ? "true" : "false")
            << ", \"ba1\": " << (map.inertialBA1 ? "true" : "false")
            << ", \"ba2\": " << (map.inertialBA2 ? "true" : "false")
            << ", \"keyframes\": " << map.keyFrameCount
            << ", \"valid_poses\": " << map.exportedFrames
            << ", \"is_main\": "
            << ((report.main_map_valid > 0U && map.id == report.main_map_id)
                    ? "true" : "false")
            << "}" << (i + 1U < report.maps.size() ? "," : "") << "\n";
    }
    out << "  ],\n"
        << "  \"parameters\": {\n"
        << "    \"overwrite\": " << (options.overwrite ? "true" : "false") << ",\n"
        << "    \"do_equalize\": " << (options.do_equalize ? "true" : "false") << "\n"
        << "  },\n"
        << "  \"dependencies\": {\n"
        << "    \"orb_slam3_commit\": " << JsonQuoted(options.orb_slam3_commit) << ",\n"
        << "    \"opencv_version\": " << JsonQuoted(options.opencv_version) << "\n"
        << "  },\n"
        << "  \"files\": [\n";
    std::vector<std::string> all_files;
    for (const OfflineOutputFile& file : trajectory_files)
    {
        all_files.push_back(file.path);
    }
    all_files.push_back(paths.log);
    all_files.push_back(paths.manifest);
    all_files.push_back(paths.summary);
    for (std::size_t i = 0U; i < all_files.size(); ++i)
    {
        out << "    " << JsonQuoted(all_files[i])
            << (i + 1U < all_files.size() ? "," : "") << "\n";
    }
    out << "  ]\n"
        << "}\n";
    return out.str();
}

/**
 * @brief 发布本次运行的轨迹、log、manifest 与 summary。
 *
 * --overwrite 会把旧 manifest 标识的本次命名范围产物作为一个旧代整体替换：
 * 即使本次没有轨迹（失败/零 eligible），也不会让旧轨迹继续冒充新结果。
 * 只删除旧 manifest 已验证属于本工具且非输入别名的文件。
 */
bool PublishOfflineOutputs(
    const OfflineRunOptions& options,
    const OfflineOutputPaths& paths,
    const std::vector<OfflineOutputFile>& trajectory_files,
    const std::vector<std::string>& stale_files,
    OfflineRunReport* report,
    std::string* error_message)
{
    std::string publish_error;
    const std::function<void(const std::string&)> record_failure =
        [&publish_error](const std::string& message)
    {
        if (message.empty())
        {
            return;
        }
        if (publish_error.empty())
        {
            publish_error = message;
        }
        else
        {
            publish_error += "; " + message;
        }
    };
    const std::function<void()> apply_failure = [&]()
    {
        if (publish_error.empty())
        {
            return;
        }
        report->status = "failed";
        if (report->failure_reason.empty())
        {
            report->failure_reason = publish_error;
        }
        else
        {
            report->failure_reason += "; " + publish_error;
        }
        publish_error.clear();
    };

    bool can_write = true;
    if (options.overwrite)
    {
        std::string remove_error;
        if (!RemoveStaleOfflineFiles(stale_files, &remove_error))
        {
            can_write = false;
            record_failure(remove_error);
        }
    }

    std::vector<OfflineOutputFile> written_files;
    if (can_write)
    {
        for (const OfflineOutputFile& file : trajectory_files)
        {
            std::string write_error;
            if (!WriteAtomicTextFile(
                    file.path, file.content, options.overwrite, &write_error))
            {
                can_write = false;
                record_failure(write_error);
                break;
            }
            written_files.push_back(file);
        }
        if (!can_write)
        {
            // A partial generation must not be left behind in the same naming
            // range and then be mistaken for a complete publication.
            for (const OfflineOutputFile& file : written_files)
            {
                (void)::remove(file.path.c_str());
            }
            written_files.clear();
        }
    }
    apply_failure();

    std::string log_error;
    if (!WriteAtomicTextFile(
            paths.log, BuildLogText(options, *report), options.overwrite,
            &log_error))
    {
        record_failure(log_error);
    }
    std::string manifest_error;
    if (!WriteOfflineManifest(
            paths, written_files, options.overwrite, &manifest_error))
    {
        record_failure(manifest_error);
    }
    apply_failure();

    const std::string summary = BuildSummaryJson(
        options, *report, written_files, paths);
    std::string summary_error;
    const bool summary_ok = WriteAtomicTextFile(
        paths.summary, summary, options.overwrite, &summary_error);
    if (!summary_ok)
    {
        record_failure(summary_error);
    }
    apply_failure();
    report->summary_written = summary_ok;

    if (!publish_error.empty())
    {
        if (error_message)
        {
            *error_message = report->failure_reason;
        }
        return false;
    }
    if (error_message)
    {
        error_message->clear();
    }
    return true;
}
}  // namespace

bool RunOfflinePipeline(
    OfflineFrameSource* source,
    OfflineSlamSink* sink,
    const OfflineRunOptions& options,
    OfflineRunReport* report,
    std::string* error_message)
{
    if (!source || !sink || !report || !error_message)
    {
        return false;
    }
    *report = OfflineRunReport();
    report->bag_path = options.bag.bag_path;
    report->output_path = options.output_path;
    error_message->clear();

    const OfflineOutputPaths paths = BuildOfflineOutputPaths(options.output_path);
    const std::vector<std::string> protected_inputs = {
        options.bag.bag_path, options.settings_file, options.vocabulary_file};
    std::vector<std::string> stale_files;
    if (!PreflightOfflineOutputs(
            paths, options.overwrite, protected_inputs, &stale_files, error_message))
    {
        report->failure_reason = *error_message;
        std::string finish_reason;
        (void)sink->Finish(&finish_reason);
        return false;
    }

    OfflineBagInfo info;
    if (!source->Inspect(options.bag, &info, error_message))
    {
        report->failure_reason = *error_message;
        report->status = "failed";
        std::string finish_reason;
        (void)sink->Finish(&finish_reason);
        std::string publish_error;
        PublishOfflineOutputs(
            options, paths, std::vector<OfflineOutputFile>(), stale_files,
            report, &publish_error);
        return false;
    }

    report->total_images = info.image_stamps_ns.size();
    report->imu_samples = info.imu_samples.size();
    if (!sink->Configure(info.image_stamps_ns.front(), error_message))
    {
        report->failure_reason = *error_message;
        report->status = "failed";
        std::string finish_reason;
        (void)sink->Finish(&finish_reason);
        std::string publish_error;
        PublishOfflineOutputs(
            options, paths, std::vector<OfflineOutputFile>(), stale_files,
            report, &publish_error);
        return false;
    }

    const std::int64_t offset_ns =
        TimeOffsetToNanoseconds(options.bag.imu_time_offset_sec);
    std::vector<char> eligible(info.image_stamps_ns.size(), 0);
    for (std::size_t i = 0U; i < info.image_stamps_ns.size(); ++i)
    {
        const std::int64_t stamp = info.image_stamps_ns[i];
        if (IsImageCoveredByImu(
                stamp, info.imu_samples.front().raw_timestamp_ns,
                info.imu_samples.back().raw_timestamp_ns, offset_ns))
        {
            eligible[i] = 1;
            ++report->eligible_images;
        }
        else if (stamp < info.imu_samples.front().raw_timestamp_ns + offset_ns)
        {
            ++report->skipped_before_imu;
        }
        else
        {
            ++report->skipped_after_imu;
        }
    }

    if (report->eligible_images == 0U)
    {
        report->failure_reason = "全部图像都缺少修正后 IMU 首尾覆盖";
        report->status = "failed";
        std::string finish_reason;
        (void)sink->Finish(&finish_reason);
        std::string publish_error;
        PublishOfflineOutputs(
            options, paths, std::vector<OfflineOutputFile>(), stale_files,
            report, &publish_error);
        return false;
    }

    cv::Ptr<cv::CLAHE> clahe;
    if (options.do_equalize)
    {
        clahe = cv::createCLAHE(3.0, cv::Size(8, 8));
    }

    bool loop_interrupted = false;
    bool loop_failed = false;
    OfflineImuCursor cursor;
    std::size_t image_index = 0U;

    const std::function<bool(const OfflineImageFrame&, std::string*)> callback =
        [&](const OfflineImageFrame& frame, std::string* callback_error) -> bool
    {
        if (image_index >= info.image_stamps_ns.size())
        {
            *callback_error = "decoded image count exceeds inspected stamps";
            loop_failed = true;
            return false;
        }
        const std::size_t current_index = image_index++;
        ++report->decoded_images;

        if (options.cancel && options.cancel->load())
        {
            loop_interrupted = true;
            *callback_error = "cancelled by user";
            return false;
        }
        if (!eligible[current_index])
        {
            return true;
        }

        std::string wait_reason;
        const ORB_SLAM3::OfflineWaitStatus ready = sink->WaitReady(&wait_reason);
        if (ready == ORB_SLAM3::OfflineWaitStatus::CANCELLED)
        {
            loop_interrupted = true;
            *callback_error = wait_reason;
            return false;
        }
        if (ready != ORB_SLAM3::OfflineWaitStatus::READY)
        {
            loop_failed = true;
            *callback_error = "Local Mapping not ready: " + wait_reason;
            return false;
        }

        std::vector<ORB_SLAM3::IMU::Point> measurements;
        if (!ExtractImuUntilImage(
                info.imu_samples, frame.timestamp_ns, offset_ns, &cursor,
                &measurements, callback_error))
        {
            // Every eligible image must have an inclusive corrected-time
            // endpoint in the cached stream. A missing endpoint means the
            // sparse tail is real data loss, not a rounding artefact, so the
            // run fails without shrinking the eligible denominator.
            loop_failed = true;
            return false;
        }
        if (std::getenv("ORBSLAM3_OFFLINE_TRACE") != nullptr &&
            report->submitted_frames < 8U)
        {
            std::cerr << "TRACE frame=" << frame.timestamp_ns
                      << " eligible_index=" << current_index
                      << " measurements=" << measurements.size();
            if (!measurements.empty())
            {
                std::cerr << " first_t=" << measurements.front().t
                          << " last_t=" << measurements.back().t;
            }
            std::cerr << std::endl;
        }
        if (!HasEnoughImuSamples(measurements, report->submitted_frames == 0U))
        {
            ++report->imu_insufficient_batches;
            loop_failed = true;
            std::ostringstream out;
            out << "图像 " << frame.timestamp_ns
                << "ns 的本段 IMU 样本少于预积分要求: "
                << measurements.size();
            *callback_error = out.str();
            return false;
        }

        cv::Mat left = frame.left;
        cv::Mat right = frame.right;
        if (clahe)
        {
            left = left.clone();
            right = right.clone();
            clahe->apply(left, left);
            clahe->apply(right, right);
        }

        int tracking_state = -1;
        if (!sink->Track(
                left, right, frame.timestamp_ns, measurements, &tracking_state,
                callback_error))
        {
            loop_failed = true;
            return false;
        }
        ++report->submitted_frames;
        if (tracking_state == ORB_SLAM3::Tracking::OK)
        {
            ++report->tracking_ok_frames;
        }
        else if (tracking_state == ORB_SLAM3::Tracking::RECENTLY_LOST)
        {
            ++report->recently_lost_frames;
        }
        else if (tracking_state == ORB_SLAM3::Tracking::LOST)
        {
            ++report->tracking_lost_frames;
        }
        else if (tracking_state == ORB_SLAM3::Tracking::NOT_INITIALIZED ||
                 tracking_state == ORB_SLAM3::Tracking::NO_IMAGES_YET)
        {
            ++report->tracking_not_initialized_frames;
        }
        else
        {
            ++report->tracking_other_frames;
        }
        return true;
    };

    std::string source_error;
    const bool source_ok = source->ForEachImage(
        options.bag, info, callback, &source_error);
    if (!source_ok)
    {
        if (loop_interrupted)
        {
            report->status = "interrupted";
            report->failure_reason = source_error;
        }
        else
        {
            loop_failed = true;
            report->failure_reason = source_error;
            if (source_error.find("解码") != std::string::npos ||
                source_error.find("JPEG") != std::string::npos)
            {
                ++report->decode_failures;
            }
        }
    }

    std::string finish_reason;
    const ORB_SLAM3::OfflineWaitStatus finish_status = sink->Finish(&finish_reason);
    if (finish_status == ORB_SLAM3::OfflineWaitStatus::INTERRUPTED)
    {
        loop_interrupted = true;
        if (report->failure_reason.empty())
        {
            report->failure_reason = finish_reason;
        }
    }
    else if (finish_status != ORB_SLAM3::OfflineWaitStatus::COMPLETE)
    {
        loop_failed = true;
        if (report->failure_reason.empty())
        {
            report->failure_reason = finish_reason;
        }
    }

    std::vector<ORB_SLAM3::OfflineCameraPose> poses;
    std::vector<ORB_SLAM3::OfflineMapInfo> maps;
    std::string snapshot_error;
    if (!sink->Snapshot(&poses, &maps, &snapshot_error))
    {
        loop_failed = true;
        if (report->failure_reason.empty())
        {
            report->failure_reason = snapshot_error;
        }
        poses.clear();
        maps.clear();
    }
    report->maps = maps;
    report->map_count = maps.size();
    for (const ORB_SLAM3::OfflineMapInfo& map : maps)
    {
        if (map.imuInitialized)
        {
            ++report->initialized_map_count;
        }
    }
    report->valid_poses = poses.size();

    OfflineOutputPlan plan;
    std::string plan_error;
    const bool plan_ok = SelectOfflineOutputPlan(poses, maps, &plan, &plan_error);
    if (plan_ok)
    {
        report->main_map_id = plan.maps[plan.main_index].map_id;
        report->main_map_valid = plan.main_frame_count;
        report->total_coverage = report->eligible_images == 0U ? 0.0
            : static_cast<double>(plan.total_frame_count) /
                  static_cast<double>(report->eligible_images);
        report->main_coverage = report->eligible_images == 0U ? 0.0
            : static_cast<double>(plan.main_frame_count) /
                  static_cast<double>(report->eligible_images);
    }
    else if (report->failure_reason.empty())
    {
        report->failure_reason = plan_error;
    }
    if (!plan_ok)
    {
        loop_failed = true;
    }

    // Coverage thresholds are part of the acceptance contract and are measured
    // against the unmodified eligible-image denominator.
    const double kTotalCoverageThreshold = 0.95;
    const double kMainCoverageThreshold = 0.90;
    const double kCoverageEpsilon = 1e-12;
    bool coverage_ok = plan_ok;
    if (plan_ok)
    {
        if (report->total_coverage + kCoverageEpsilon < kTotalCoverageThreshold)
        {
            coverage_ok = false;
            if (report->failure_reason.empty())
            {
                std::ostringstream out;
                out << "总覆盖率 " << report->total_coverage
                    << " 低于门槛 " << kTotalCoverageThreshold;
                report->failure_reason = out.str();
            }
        }
        if (report->main_coverage + kCoverageEpsilon < kMainCoverageThreshold)
        {
            coverage_ok = false;
            if (report->failure_reason.empty())
            {
                std::ostringstream out;
                out << "主 map 覆盖率 " << report->main_coverage
                    << " 低于门槛 " << kMainCoverageThreshold;
                report->failure_reason = out.str();
            }
            else
            {
                std::ostringstream out;
                out << "; 主 map 覆盖率 " << report->main_coverage
                    << " 低于门槛 " << kMainCoverageThreshold;
                report->failure_reason += out.str();
            }
        }
    }

    std::vector<OfflineOutputFile> trajectory_files;
    bool trajectory_plan_ok = plan_ok && !poses.empty();
    if (trajectory_plan_ok)
    {
        std::string build_error;
        if (!BuildOfflineTrajectoryFiles(plan, paths, &trajectory_files, &build_error))
        {
            loop_failed = true;
            trajectory_plan_ok = false;
            if (report->failure_reason.empty())
            {
                report->failure_reason = build_error;
            }
            else
            {
                report->failure_reason += "; " + build_error;
            }
        }
    }

    if (loop_interrupted)
    {
        report->status = "interrupted";
    }
    else if (loop_failed)
    {
        report->status = "failed";
    }
    else if (trajectory_plan_ok && coverage_ok)
    {
        report->status = "complete";
    }
    else
    {
        report->status = "failed";
        if (report->failure_reason.empty())
        {
            report->failure_reason = trajectory_plan_ok
                ? "覆盖率达到门槛但状态未标记成功"
                : "没有可导出的有效位姿";
        }
    }

    std::string publish_error;
    if (!PublishOfflineOutputs(
            options, paths, trajectory_files, stale_files, report,
            &publish_error))
    {
        if (report->failure_reason.empty())
        {
            report->failure_reason = publish_error;
        }
        report->status = "failed";
        *error_message = report->failure_reason;
        return false;
    }

    if (report->status == "complete")
    {
        error_message->clear();
        return true;
    }
    *error_message = report->failure_reason;
    return false;
}
