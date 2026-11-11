/**
 * @file offline-slam-sink.cpp
 * @brief 实现真实 ORB_SLAM3 离线 sink。
 */

#include "offline-slam-sink.hpp"

#include "Tracking.h"

OrbSlamSystemSink::OrbSlamSystemSink(
    ORB_SLAM3::System* system,
    const std::atomic<bool>* cancel)
    : system_(system), cancel_(cancel)
{
}

bool OrbSlamSystemSink::Configure(
    const std::int64_t time_origin_ns,
    std::string* error_message)
{
    if (!system_)
    {
        *error_message = "ORB_SLAM3 系统指针为空";
        return false;
    }
    system_->SetOfflineMode(true);
    system_->SetOfflineCancelFlag(cancel_);
    system_->SetOfflineTimeOriginNs(time_origin_ns);
    error_message->clear();
    return true;
}

ORB_SLAM3::OfflineWaitStatus OrbSlamSystemSink::WaitReady(std::string* reason)
{
    return system_->WaitForLocalMappingReady(reason);
}

bool OrbSlamSystemSink::Track(
    const cv::Mat& left,
    const cv::Mat& right,
    const std::int64_t timestamp_ns,
    const std::vector<ORB_SLAM3::IMU::Point>& measurements,
    int* tracking_state,
    std::string* error_message)
{
    if (!system_ || !tracking_state)
    {
        *error_message = "ORB_SLAM3 系统或 tracking state 输出为空";
        return false;
    }
    system_->TrackStereoOffline(left, right, timestamp_ns, measurements);
    *tracking_state = system_->GetTrackingState();
    error_message->clear();
    return true;
}

ORB_SLAM3::OfflineWaitStatus OrbSlamSystemSink::Finish(std::string* reason)
{
    return system_->ShutdownOffline(reason);
}

bool OrbSlamSystemSink::Snapshot(
    std::vector<ORB_SLAM3::OfflineCameraPose>* poses,
    std::vector<ORB_SLAM3::OfflineMapInfo>* maps,
    std::string* error_message)
{
    return system_->GetFinalOfflineSnapshot(poses, maps, error_message);
}
