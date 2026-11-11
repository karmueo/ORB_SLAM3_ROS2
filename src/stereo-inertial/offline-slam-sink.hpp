/**
 * @file offline-slam-sink.hpp
 * @brief 声明包装真实 ORB_SLAM3::System 的离线 sink。
 */

#ifndef __STEREO_INERTIAL_OFFLINE_SLAM_SINK_HPP__
#define __STEREO_INERTIAL_OFFLINE_SLAM_SINK_HPP__

#include "offline-runner.hpp"

#include "System.h"

#include <atomic>
#include <cstdint>
#include <string>
#include <vector>

/**
 * @brief 真实 ORB_SLAM3 离线 sink。
 */
class OrbSlamSystemSink : public OfflineSlamSink
{
public:
    /**
     * @brief 绑定已构造的 ORB_SLAM3 系统和取消标志。
     */
    explicit OrbSlamSystemSink(
        ORB_SLAM3::System* system,
        const std::atomic<bool>* cancel);

    bool Configure(std::int64_t time_origin_ns, std::string* error_message) override;
    ORB_SLAM3::OfflineWaitStatus WaitReady(std::string* reason) override;
    bool Track(
        const cv::Mat& left,
        const cv::Mat& right,
        std::int64_t timestamp_ns,
        const std::vector<ORB_SLAM3::IMU::Point>& measurements,
        int* tracking_state,
        std::string* error_message) override;
    ORB_SLAM3::OfflineWaitStatus Finish(std::string* reason) override;
    bool Snapshot(
        std::vector<ORB_SLAM3::OfflineCameraPose>* poses,
        std::vector<ORB_SLAM3::OfflineMapInfo>* maps,
        std::string* error_message) override;

private:
    ORB_SLAM3::System* system_;
    const std::atomic<bool>* cancel_;
};

#endif
