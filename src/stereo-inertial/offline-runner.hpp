/**
 * @file offline-runner.hpp
 * @brief 声明可注入图像源与 SLAM sink 的离线处理流程。
 */

#ifndef __STEREO_INERTIAL_OFFLINE_RUNNER_HPP__
#define __STEREO_INERTIAL_OFFLINE_RUNNER_HPP__

#include "offline-bag-reader.hpp"
#include "offline-output.hpp"

#include <opencv2/core/core.hpp>

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

/**
 * @brief 离线运行参数。
 */
struct OfflineRunOptions
{
    /** @brief 底层 bag 读取参数。 */
    OfflineBagOptions bag;
    /** @brief 主轨迹输出路径。 */
    std::string output_path;
    /** @brief ORB 词典路径，仅用于 summary。 */
    std::string vocabulary_file;
    /** @brief ORB 配置路径，仅用于 summary。 */
    std::string settings_file;
    /** @brief 是否允许覆盖同名输出。 */
    bool overwrite = false;
    /** @brief 是否对左右灰度图做 CLAHE。 */
    bool do_equalize = false;
    /** @brief 配置中的 IMU 时间偏移。 */
    double configured_imu_time_offset_sec = 0.0;
    /** @brief 是否由命令行覆盖 IMU 时间偏移。 */
    bool imu_time_offset_overridden = false;
    /** @brief 外部取消标志。 */
    const std::atomic<bool>* cancel = nullptr;
    /** @brief ORB_SLAM3 固定 commit，用于 summary。 */
    std::string orb_slam3_commit;
    /** @brief OpenCV 版本，用于 summary。 */
    std::string opencv_version;
};

/**
 * @brief 离线运行统计与状态。
 */
struct OfflineRunReport
{
    /** @brief complete/failed/interrupted。 */
    std::string status = "failed";
    /** @brief 失败或中断原因。 */
    std::string failure_reason;
    /** @brief 输入 bag 路径。 */
    std::string bag_path;
    /** @brief 输出路径。 */
    std::string output_path;

    std::size_t total_images = 0;
    std::size_t eligible_images = 0;
    std::size_t submitted_frames = 0;
    std::size_t valid_poses = 0;
    std::size_t skipped_before_imu = 0;
    std::size_t skipped_after_imu = 0;
    std::size_t decoded_images = 0;
    std::size_t decode_failures = 0;
    std::size_t imu_samples = 0;
    std::size_t imu_insufficient_batches = 0;

    std::size_t tracking_ok_frames = 0;
    std::size_t recently_lost_frames = 0;
    std::size_t tracking_lost_frames = 0;
    std::size_t tracking_not_initialized_frames = 0;
    std::size_t tracking_other_frames = 0;

    std::size_t map_count = 0;
    std::size_t initialized_map_count = 0;
    std::vector<ORB_SLAM3::OfflineMapInfo> maps;
    std::size_t main_map_id = 0;
    std::size_t main_map_valid = 0;
    double total_coverage = 0.0;
    double main_coverage = 0.0;

    bool summary_written = false;
};

/**
 * @brief 可注入的离线 SLAM sink。
 *
 * 生产实现封装 ORB_SLAM3::System；测试可注入小型 fake sink 验证 reader
 * 边界、统计与输出，而不必运行真实 SLAM。
 */
class OfflineSlamSink
{
public:
    virtual ~OfflineSlamSink() = default;

    /** @brief 设置离线模式与核心相对时间原点。 */
    virtual bool Configure(std::int64_t time_origin_ns, std::string* error_message) = 0;

    /** @brief 在进入 Track 前等待 Local Mapping 就绪。 */
    virtual ORB_SLAM3::OfflineWaitStatus WaitReady(std::string* reason) = 0;

    /** @brief 提交一帧左右灰度图和本帧新增 IMU 样本。 */
    virtual bool Track(
        const cv::Mat& left,
        const cv::Mat& right,
        std::int64_t timestamp_ns,
        const std::vector<ORB_SLAM3::IMU::Point>& measurements,
        int* tracking_state,
        std::string* error_message) = 0;

    /** @brief EOF 排空后台工作并 join 线程。 */
    virtual ORB_SLAM3::OfflineWaitStatus Finish(std::string* reason) = 0;

    /** @brief 读取最终只读轨迹快照。 */
    virtual bool Snapshot(
        std::vector<ORB_SLAM3::OfflineCameraPose>* poses,
        std::vector<ORB_SLAM3::OfflineMapInfo>* maps,
        std::string* error_message) = 0;
};

/**
 * @brief 执行一次离线 reader/sync/track/drain/snapshot/output 流程。
 * @param source 图像和 IMU 来源。
 * @param sink SLAM 提交与快照接口。
 * @param options 运行参数。
 * @param report 运行统计。
 * @param error_message 顶层失败诊断。
 * @return 流程和输出发布成功返回 true；status 仍可能是 interrupted。
 */
bool RunOfflinePipeline(
    OfflineFrameSource* source,
    OfflineSlamSink* sink,
    const OfflineRunOptions& options,
    OfflineRunReport* report,
    std::string* error_message);

#endif
