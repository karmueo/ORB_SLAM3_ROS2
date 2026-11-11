/**
 * @file offline-output.hpp
 * @brief 声明离线轨迹分组、命名、原子发布与覆盖保护辅助。
 */

#ifndef __STEREO_INERTIAL_OFFLINE_OUTPUT_HPP__
#define __STEREO_INERTIAL_OFFLINE_OUTPUT_HPP__

#include "System.h"

#include <cstddef>
#include <string>
#include <vector>

/**
 * @brief 单个 map 的待输出轨迹。
 */
struct OfflineOutputMap
{
    /** @brief 最终 map id。 */
    unsigned long map_id = 0;
    /** @brief 该 map 的按时间升序有效位姿。 */
    std::vector<ORB_SLAM3::OfflineCameraPose> poses;
};

/**
 * @brief 选择后的轨迹输出计划。
 */
struct OfflineOutputPlan
{
    /** @brief 按 map id 升序分组的输出。 */
    std::vector<OfflineOutputMap> maps;
    /** @brief 主轨迹在 maps 中的索引。 */
    std::size_t main_index = 0;
    /** @brief 主轨迹有效帧数。 */
    std::size_t main_frame_count = 0;
    /** @brief 全部 map 有效帧数。 */
    std::size_t total_frame_count = 0;
};

/**
 * @brief 本次运行所有命名输出路径。
 */
struct OfflineOutputPaths
{
    /** @brief 主 TUM 轨迹路径。 */
    std::string main;
    /** @brief 去掉最后一个扩展名的文件名主干。 */
    std::string stem;
    /** @brief 主轨迹扩展名，包含点号，可为空。 */
    std::string extension;
    /** @brief summary JSON 路径。 */
    std::string summary;
    /** @brief 文本日志路径。 */
    std::string log;
    /** @brief 覆盖清单路径。 */
    std::string manifest;
    /** @brief CSV 中的逐帧时间戳、最终 map 与跟踪状态。 */
    std::string frames;
};

/**
 * @brief 待写入的文本输出文件。
 */
struct OfflineOutputFile
{
    /** @brief 目标路径。 */
    std::string path;
    /** @brief 完整文本内容。 */
    std::string content;
};

/**
 * @brief 根据主输出路径构造全部命名输出。
 * @param output_path 用户指定的主输出路径。
 * @return 命名输出路径集合。
 */
OfflineOutputPaths BuildOfflineOutputPaths(const std::string& output_path);

/**
 * @brief 按最终 map 分组并选择有效帧最多的主 map。
 * @param poses 只读最终 snapshot 位姿。
 * @param maps 只读最终 snapshot map 统计。
 * @param plan 输出计划。
 * @param error_message 失败诊断。
 * @return 至少有一个非空初始化 map 时返回 true。
 */
bool SelectOfflineOutputPlan(
    const std::vector<ORB_SLAM3::OfflineCameraPose>& poses,
    const std::vector<ORB_SLAM3::OfflineMapInfo>& maps,
    OfflineOutputPlan* plan,
    std::string* error_message);

/**
 * @brief 生成各 map 的 TUM 及逐帧状态 CSV。
 * @param plan 输出计划。
 * @param paths 命名输出路径。
 * @param files 输出文件列表。
 * @param error_message 失败诊断。
 * @return 所有位姿有限且四元数归一化时返回 true。
 */
bool BuildOfflineTrajectoryFiles(
    const OfflineOutputPlan& plan,
    const OfflineOutputPaths& paths,
    std::vector<OfflineOutputFile>* files,
    std::string* error_message);

/**
 * @brief 检查输出冲突、覆盖清单和只读输入别名。
 * @param paths 命名输出路径。
 * @param overwrite 是否允许覆盖本次命名范围。
 * @param protected_inputs 只读输入路径，包含 bag、配置和词典。
 * @param stale_files overwrite 时可安全删除的旧 manifest 文件。
 * @param error_message 失败诊断。
 * @return 可以继续发布时返回 true。
 */
bool PreflightOfflineOutputs(
    const OfflineOutputPaths& paths,
    bool overwrite,
    const std::vector<std::string>& protected_inputs,
    std::vector<std::string>* stale_files,
    std::string* error_message);

/**
 * @brief 删除旧 manifest 标识的过期文件。
 * @param stale_files 已由 PreflightOfflineOutputs 验证的文件。
 * @param error_message 失败诊断。
 * @return 所有文件不存在或删除成功时返回 true。
 */
bool RemoveStaleOfflineFiles(
    const std::vector<std::string>& stale_files,
    std::string* error_message);

/**
 * @brief 原子写入一个文本文件。
 * @param path 目标路径。
 * @param content 文本内容。
 * @param overwrite 是否允许替换已有文件。
 * @param error_message 失败诊断。
 * @return 成功返回 true。
 */
bool WriteAtomicTextFile(
    const std::string& path,
    const std::string& content,
    bool overwrite,
    std::string* error_message);

/**
 * @brief 写出覆盖清单，记录本次命名输出文件。
 * @param paths 命名输出路径。
 * @param trajectory_files 已写出的主轨迹和副 map 文件。
 * @param overwrite 是否允许替换旧清单。
 * @param error_message 失败诊断。
 * @return 成功返回 true。
 */
bool WriteOfflineManifest(
    const OfflineOutputPaths& paths,
    const std::vector<OfflineOutputFile>& trajectory_files,
    bool overwrite,
    std::string* error_message);

/**
 * @brief 转义 JSON 字符串并加引号。
 * @param value 原始文本。
 * @return 可嵌入 JSON 的字符串。
 */
std::string JsonQuoted(const std::string& value);

#endif
