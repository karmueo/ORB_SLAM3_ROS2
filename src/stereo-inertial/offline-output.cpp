/**
 * @file offline-output.cpp
 * @brief 实现离线轨迹分组、命名、原子发布与覆盖保护。
 */

#include "offline-output.hpp"

#include "offline-sync.hpp"

#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>

#include <algorithm>
#include <cerrno>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <dirent.h>
#include <fstream>
#include <iomanip>
#include <limits.h>
#include <map>
#include <set>
#include <sstream>

namespace
{
const char* const kManifestOwner = "orbslam3-offline-trajectory";

/**
 * @brief 返回路径父目录。
 */
std::string DirName(const std::string& path)
{
    const std::size_t slash = path.find_last_of('/');
    if (slash == std::string::npos)
    {
        return ".";
    }
    if (slash == 0U)
    {
        return "/";
    }
    return path.substr(0U, slash);
}

/**
 * @brief 返回路径文件名。
 */
std::string BaseName(const std::string& path)
{
    const std::size_t slash = path.find_last_of('/');
    return slash == std::string::npos ? path : path.substr(slash + 1U);
}

/**
 * @brief 组合父目录与文件名。
 */
std::string JoinPath(const std::string& directory, const std::string& name)
{
    if (directory.empty() || directory == ".")
    {
        return name;
    }
    if (directory.back() == '/')
    {
        return directory + name;
    }
    return directory + "/" + name;
}

/**
 * @brief 判断路径是否存在（含断链符号链接）。
 */
bool Lexists(const std::string& path)
{
    struct stat info;
    return ::lstat(path.c_str(), &info) == 0;
}

/**
 * @brief 尽量解析为真实绝对路径，不存在时返回原路径。
 */
std::string RealPathOrSelf(const std::string& path)
{
    char resolved[PATH_MAX];
    if (::realpath(path.c_str(), resolved))
    {
        return std::string(resolved);
    }
    return path;
}

/**
 * @brief 取得规范化输出路径；目标不存在时解析父目录。
 */
std::string NormalizedOutputPath(const std::string& path)
{
    if (Lexists(path))
    {
        return RealPathOrSelf(path);
    }
    return JoinPath(RealPathOrSelf(DirName(path)), BaseName(path));
}

/**
 * @brief 判断路径是否存在且为目录。
 */
bool IsDirectory(const std::string& path)
{
    struct stat info;
    return ::stat(path.c_str(), &info) == 0 && S_ISDIR(info.st_mode);
}

/**
 * @brief 拒绝输出与只读输入同路径、符号链接或硬链接。
 */
bool EnsureNotInputAlias(
    const std::string& output_path,
    const std::vector<std::string>& protected_inputs,
    std::string* error_message)
{
    const std::string output_normalized = NormalizedOutputPath(output_path);
    struct stat output_info;
    const bool output_exists = ::stat(output_path.c_str(), &output_info) == 0;

    for (const std::string& input_path : protected_inputs)
    {
        const std::string input_normalized = RealPathOrSelf(input_path);
        if (output_normalized == input_normalized)
        {
            *error_message = "输出不能与只读输入相同: " + output_path + " -> " + input_path;
            return false;
        }
        if (output_exists)
        {
            struct stat input_info;
            if (::stat(input_path.c_str(), &input_info) == 0 &&
                output_info.st_dev == input_info.st_dev &&
                output_info.st_ino == input_info.st_ino)
            {
                *error_message = "输出不能是只读输入的硬链接: " + output_path + " -> " + input_path;
                return false;
            }
        }
        if (IsDirectory(input_path))
        {
            const std::string prefix = input_normalized + "/";
            if (output_normalized.compare(0U, prefix.size(), prefix) == 0)
            {
                *error_message = "输出不能位于只读 bag 目录内: " + output_path;
                return false;
            }
        }
    }
    return true;
}

/**
 * @brief 列出目录下所有条目的文件名。
 */
std::vector<std::string> ListDirectory(const std::string& directory)
{
    std::vector<std::string> entries;
    DIR* dir = ::opendir(directory.c_str());
    if (!dir)
    {
        return entries;
    }
    while (struct dirent* entry = ::readdir(dir))
    {
        const std::string name = entry->d_name;
        if (name == "." || name == "..")
        {
            continue;
        }
        entries.push_back(name);
    }
    ::closedir(dir);
    std::sort(entries.begin(), entries.end());
    return entries;
}

/**
 * @brief 判断文件名是否本次命名范围内的轨迹或 sidecar。
 */
bool IsOwnedFileName(const OfflineOutputPaths& paths, const std::string& name)
{
    if (name == BaseName(paths.main) || name == BaseName(paths.summary) ||
        name == BaseName(paths.log) || name == BaseName(paths.manifest) ||
        name == BaseName(paths.frames))
    {
        return true;
    }
    const std::string prefix = paths.stem + ".map-";
    if (name.compare(0U, prefix.size(), prefix) != 0)
    {
        return false;
    }
    if (!paths.extension.empty())
    {
        if (name.size() <= prefix.size() + paths.extension.size() ||
            name.compare(name.size() - paths.extension.size(), paths.extension.size(),
                         paths.extension) != 0)
        {
            return false;
        }
    }
    const std::size_t digit_end =
        paths.extension.empty() ? name.size() : name.size() - paths.extension.size();
    if (digit_end <= prefix.size())
    {
        return false;
    }
    for (std::size_t i = prefix.size(); i < digit_end; ++i)
    {
        if (name[i] < '0' || name[i] > '9')
        {
            return false;
        }
    }
    return true;
}

/**
 * @brief 判断文件名是否为副 map 轨迹。
 */
bool IsSideMapFileName(const OfflineOutputPaths& paths, const std::string& name)
{
    return name != BaseName(paths.main) && IsOwnedFileName(paths, name) &&
           name.compare(0U, (paths.stem + ".map-").size(), paths.stem + ".map-") == 0;
}

/**
 * @brief 读取旧 manifest 中的文件列表。
 */
bool ReadManifest(
    const OfflineOutputPaths& paths,
    std::vector<std::string>* listed_files,
    std::string* error_message)
{
    std::ifstream input(paths.manifest.c_str());
    if (!input.good())
    {
        *error_message = "无法读取旧 manifest: " + paths.manifest;
        return false;
    }
    std::string line;
    bool owner_seen = false;
    bool output_seen = false;
    listed_files->clear();
    while (std::getline(input, line))
    {
        if (line.empty())
        {
            continue;
        }
        const std::size_t equals = line.find('=');
        if (equals == std::string::npos)
        {
            *error_message = "旧 manifest 格式损坏: " + paths.manifest;
            return false;
        }
        const std::string key = line.substr(0U, equals);
        const std::string value = line.substr(equals + 1U);
        if (key == "owner")
        {
            if (value != kManifestOwner)
            {
                *error_message = "旧 manifest 不属于本工具: " + paths.manifest;
                return false;
            }
            owner_seen = true;
        }
        else if (key == "output")
        {
            if (value != BaseName(paths.main))
            {
                *error_message = "旧 manifest 的 output 与本次输出不一致";
                return false;
            }
            output_seen = true;
        }
        else if (key == "file")
        {
            if (value.empty() || value.find('/') != std::string::npos ||
                !IsOwnedFileName(paths, value))
            {
                *error_message = "旧 manifest 包含不明文件，拒绝覆盖: " + value;
                return false;
            }
            listed_files->push_back(value);
        }
        else
        {
            *error_message = "旧 manifest 包含未知键: " + key;
            return false;
        }
    }
    if (!owner_seen || !output_seen)
    {
        *error_message = "旧 manifest 缺少 owner 或 output 标识: " + paths.manifest;
        return false;
    }
    return true;
}

/**
 * @brief 递归创建目录。
 */
bool MakeDirectories(const std::string& directory, std::string* error_message)
{
    if (directory.empty() || directory == "." || IsDirectory(directory))
    {
        return true;
    }
    const std::string parent = DirName(directory);
    if (parent != directory && !MakeDirectories(parent, error_message))
    {
        return false;
    }
    if (::mkdir(directory.c_str(), 0775) != 0 && errno != EEXIST)
    {
        *error_message = "无法创建输出目录 " + directory + ": " + std::strerror(errno);
        return false;
    }
    return true;
}

/**
 * @brief 当四元数有限且非零时归一化。
 */
bool NormalizeQuaternion(Eigen::Quaternionf* quaternion)
{
    const float norm = quaternion->norm();
    if (!std::isfinite(norm) || norm < 1e-8f ||
        !quaternion->coeffs().allFinite())
    {
        return false;
    }
    quaternion->normalize();
    return true;
}
}  // namespace

OfflineOutputPaths BuildOfflineOutputPaths(const std::string& output_path)
{
    OfflineOutputPaths paths;
    paths.main = output_path;
    const std::string directory = DirName(output_path);
    const std::string filename = BaseName(output_path);
    const std::size_t dot = filename.find_last_of('.');
    if (dot != std::string::npos && dot > 0U)
    {
        paths.stem = filename.substr(0U, dot);
        paths.extension = filename.substr(dot);
    }
    else
    {
        paths.stem = filename;
        paths.extension.clear();
    }
    paths.summary = JoinPath(directory, paths.stem + ".summary.json");
    paths.log = JoinPath(directory, paths.stem + ".log");
    paths.manifest = JoinPath(directory, paths.stem + ".manifest.txt");
    paths.frames = JoinPath(directory, paths.stem + ".frames.csv");
    return paths;
}

bool SelectOfflineOutputPlan(
    const std::vector<ORB_SLAM3::OfflineCameraPose>& poses,
    const std::vector<ORB_SLAM3::OfflineMapInfo>& maps,
    OfflineOutputPlan* plan,
    std::string* error_message)
{
    if (!plan || !error_message)
    {
        return false;
    }
    plan->maps.clear();
    plan->main_index = 0U;
    plan->main_frame_count = 0U;
    plan->total_frame_count = 0U;
    error_message->clear();

    std::map<unsigned long, bool> initialized_maps;
    for (const ORB_SLAM3::OfflineMapInfo& map : maps)
    {
        initialized_maps[map.id] = map.imuInitialized;
    }

    std::map<unsigned long, std::vector<ORB_SLAM3::OfflineCameraPose>> grouped;
    std::set<std::int64_t> all_timestamps;
    for (const ORB_SLAM3::OfflineCameraPose& pose : poses)
    {
        std::map<unsigned long, bool>::const_iterator it =
            initialized_maps.find(pose.mapId);
        if (it == initialized_maps.end() || !it->second)
        {
            continue;
        }
        if (pose.timestampNs < 0)
        {
            *error_message = "最终 snapshot 含无效时间戳";
            return false;
        }
        if (!pose.translation.allFinite())
        {
            *error_message = "最终 snapshot 含非有限平移";
            return false;
        }
        Eigen::Quaternionf quaternion = pose.orientation;
        if (!NormalizeQuaternion(&quaternion))
        {
            *error_message = "最终 snapshot 含无效四元数";
            return false;
        }
        if (!all_timestamps.insert(pose.timestampNs).second)
        {
            *error_message = "最终 snapshot 含跨 map 重复时间戳: " +
                             FormatTimestampSeconds(pose.timestampNs);
            return false;
        }
        grouped[pose.mapId].push_back(pose);
    }

    for (std::map<unsigned long, std::vector<ORB_SLAM3::OfflineCameraPose>>::iterator it =
             grouped.begin();
         it != grouped.end(); ++it)
    {
        std::vector<ORB_SLAM3::OfflineCameraPose>& trajectory = it->second;
        std::sort(
            trajectory.begin(), trajectory.end(),
            [](const ORB_SLAM3::OfflineCameraPose& a,
               const ORB_SLAM3::OfflineCameraPose& b) {
                return a.timestampNs < b.timestampNs;
            });
        for (std::size_t i = 1U; i < trajectory.size(); ++i)
        {
            if (trajectory[i].timestampNs == trajectory[i - 1U].timestampNs)
            {
                *error_message = "同一 map 内含重复时间戳: " +
                                 FormatTimestampSeconds(trajectory[i].timestampNs);
                return false;
            }
        }
        OfflineOutputMap output_map;
        output_map.map_id = it->first;
        output_map.poses = trajectory;
        plan->total_frame_count += trajectory.size();
        plan->maps.push_back(output_map);
    }

    if (plan->maps.empty() || plan->total_frame_count == 0U)
    {
        *error_message = "没有完成惯性初始化的 map 或没有有效相机帧";
        return false;
    }

    plan->main_index = 0U;
    plan->main_frame_count = plan->maps[0].poses.size();
    for (std::size_t i = 1U; i < plan->maps.size(); ++i)
    {
        const std::size_t count = plan->maps[i].poses.size();
        if (count > plan->main_frame_count ||
            (count == plan->main_frame_count &&
             plan->maps[i].map_id < plan->maps[plan->main_index].map_id))
        {
            plan->main_index = i;
            plan->main_frame_count = count;
        }
    }
    return true;
}

bool BuildOfflineTrajectoryFiles(
    const OfflineOutputPlan& plan,
    const OfflineOutputPaths& paths,
    std::vector<OfflineOutputFile>* files,
    std::string* error_message)
{
    if (!files || !error_message)
    {
        return false;
    }
    files->clear();
    error_message->clear();
    if (plan.maps.empty() || plan.main_index >= plan.maps.size() ||
        plan.total_frame_count == 0U)
    {
        *error_message = "轨迹输出计划为空或主 map 索引无效";
        return false;
    }

    const std::string directory = DirName(paths.main);
    std::map<std::int64_t, std::pair<unsigned long, bool>> frame_states;
    for (std::size_t map_index = 0U; map_index < plan.maps.size(); ++map_index)
    {
        const OfflineOutputMap& map = plan.maps[map_index];
        if (map.poses.empty())
        {
            *error_message = "轨迹输出计划包含空 map";
            return false;
        }
        OfflineOutputFile file;
        if (map_index == plan.main_index)
        {
            file.path = paths.main;
        }
        else
        {
            file.path = JoinPath(
                directory,
                paths.stem + ".map-" + std::to_string(map.map_id) + paths.extension);
        }

        std::ostringstream out;
        out << std::setprecision(9);
        for (const ORB_SLAM3::OfflineCameraPose& pose : map.poses)
        {
            if (pose.timestampNs < 0 || pose.mapId != map.map_id ||
                !pose.translation.allFinite())
            {
                *error_message = "轨迹包含无效时间戳、map 归属或非有限平移";
                return false;
            }
            Eigen::Quaternionf quaternion = pose.orientation;
            if (!NormalizeQuaternion(&quaternion))
            {
                *error_message = "轨迹包含无效四元数";
                return false;
            }
            frame_states.emplace(pose.timestampNs, std::make_pair(map.map_id, pose.recentlyLost));
            out << FormatTimestampSeconds(pose.timestampNs)
                << ' ' << pose.translation.x()
                << ' ' << pose.translation.y()
                << ' ' << pose.translation.z()
                << ' ' << quaternion.x()
                << ' ' << quaternion.y()
                << ' ' << quaternion.z()
                << ' ' << quaternion.w()
                << '\n';
        }
        file.content = out.str();
        files->push_back(file);
    }
    std::ostringstream csv;
    csv << "timestamp_ns,map_id,tracking_state\n";
    for (const auto& frame : frame_states)
    {
        csv << frame.first << ',' << frame.second.first << ','
            << (frame.second.second ? "RECENTLY_LOST" : "OK") << '\n';
    }
    files->push_back(OfflineOutputFile{paths.frames, csv.str()});
    return true;
}

bool PreflightOfflineOutputs(
    const OfflineOutputPaths& paths,
    const bool overwrite,
    const std::vector<std::string>& protected_inputs,
    std::vector<std::string>* stale_files,
    std::string* error_message)
{
    if (!stale_files || !error_message)
    {
        return false;
    }
    stale_files->clear();
    error_message->clear();

    if (paths.stem.empty() || BaseName(paths.main).empty())
    {
        *error_message = "输出文件名不能为空";
        return false;
    }

    const std::vector<std::string> artifacts = {
        paths.main, paths.summary, paths.log, paths.manifest, paths.frames};
    std::set<std::string> artifact_paths;
    for (const std::string& artifact : artifacts)
    {
        if (!artifact_paths.insert(NormalizedOutputPath(artifact)).second)
        {
            *error_message = "输出产物路径冲突: " + artifact;
            return false;
        }
        if (!EnsureNotInputAlias(artifact, protected_inputs, error_message))
        {
            return false;
        }
    }

    const std::string directory = DirName(paths.main);
    const std::vector<std::string> entries = ListDirectory(directory);
    std::set<std::string> listed_files;
    if (Lexists(paths.manifest))
    {
        std::vector<std::string> manifest_files;
        if (!ReadManifest(paths, &manifest_files, error_message))
        {
            return false;
        }
        // manifest 自身的归属由 owner/output 校验确认，无需在 file 列表中自引用。
        listed_files.insert(BaseName(paths.manifest));
        for (const std::string& name : manifest_files)
        {
            listed_files.insert(name);
            stale_files->push_back(JoinPath(directory, name));
        }
    }

    for (const std::string& name : entries)
    {
        const std::string full_path = JoinPath(directory, name);
        if (overwrite && IsOwnedFileName(paths, name) && !listed_files.count(name))
        {
            *error_message = "发现不由旧 manifest 标识的输出，拒绝覆盖: " + full_path;
            return false;
        }
        if (IsOwnedFileName(paths, name) &&
            !EnsureNotInputAlias(full_path, protected_inputs, error_message))
        {
            return false;
        }
    }

    if (!overwrite)
    {
        for (const std::string& artifact : artifacts)
        {
            if (Lexists(artifact))
            {
                *error_message = "输出已存在，使用 --overwrite 才可覆盖: " + artifact;
                return false;
            }
        }
        for (const std::string& name : entries)
        {
            if (IsSideMapFileName(paths, name))
            {
                *error_message = "同名副 map 输出已存在，使用 --overwrite 才可覆盖: " +
                                 JoinPath(directory, name);
                return false;
            }
        }
    }
    return true;
}

bool RemoveStaleOfflineFiles(
    const std::vector<std::string>& stale_files,
    std::string* error_message)
{
    for (const std::string& path : stale_files)
    {
        if (!Lexists(path))
        {
            continue;
        }
        if (::remove(path.c_str()) != 0)
        {
            *error_message = "无法删除旧输出 " + path + ": " + std::strerror(errno);
            return false;
        }
    }
    return true;
}

bool WriteAtomicTextFile(
    const std::string& path,
    const std::string& content,
    const bool overwrite,
    std::string* error_message)
{
    *error_message = "";
    if (!overwrite && Lexists(path))
    {
        *error_message = "输出已存在，使用 --overwrite 才可覆盖: " + path;
        return false;
    }
    const std::string directory = DirName(path);
    if (!MakeDirectories(directory, error_message))
    {
        return false;
    }

    std::string temporary = JoinPath(directory, "." + BaseName(path) + ".tmp.XXXXXX");
    std::vector<char> pattern(temporary.begin(), temporary.end());
    pattern.push_back('\0');
    const int descriptor = ::mkstemp(pattern.data());
    if (descriptor < 0)
    {
        *error_message = "无法创建临时输出文件: " + std::string(std::strerror(errno));
        return false;
    }
    temporary.assign(pattern.data());
    ::fchmod(descriptor, 0644);

    bool success = true;
    std::size_t offset = 0U;
    while (offset < content.size())
    {
        const ssize_t written = ::write(
            descriptor, content.data() + offset, content.size() - offset);
        if (written < 0)
        {
            if (errno == EINTR)
            {
                continue;
            }
            success = false;
            *error_message = "写入临时文件失败: " + std::string(std::strerror(errno));
            break;
        }
        offset += static_cast<std::size_t>(written);
    }
    if (success && ::fsync(descriptor) != 0)
    {
        success = false;
        *error_message = "fsync 临时文件失败: " + std::string(std::strerror(errno));
    }
    if (::close(descriptor) != 0 && success)
    {
        success = false;
        *error_message = "关闭临时文件失败: " + std::string(std::strerror(errno));
    }

    if (success)
    {
        if (overwrite)
        {
            if (::rename(temporary.c_str(), path.c_str()) != 0)
            {
                success = false;
                *error_message = "发布输出失败 " + path + ": " + std::strerror(errno);
            }
        }
        else
        {
            if (::link(temporary.c_str(), path.c_str()) != 0)
            {
                success = false;
                *error_message = (errno == EEXIST)
                    ? "输出已存在，使用 --overwrite 才可覆盖: " + path
                    : "发布输出失败 " + path + ": " + std::strerror(errno);
            }
            else if (::unlink(temporary.c_str()) != 0)
            {
                success = false;
                *error_message = "删除临时文件失败: " + std::string(std::strerror(errno));
            }
        }
    }

    if (!success && Lexists(temporary))
    {
        ::unlink(temporary.c_str());
    }
    return success;
}

bool WriteOfflineManifest(
    const OfflineOutputPaths& paths,
    const std::vector<OfflineOutputFile>& trajectory_files,
    const bool overwrite,
    std::string* error_message)
{
    std::ostringstream out;
    out << "owner=" << kManifestOwner << '\n'
        << "output=" << BaseName(paths.main) << '\n';
    for (const OfflineOutputFile& file : trajectory_files)
    {
        out << "file=" << BaseName(file.path) << '\n';
    }
    out << "file=" << BaseName(paths.summary) << '\n'
        << "file=" << BaseName(paths.log) << '\n';
    return WriteAtomicTextFile(paths.manifest, out.str(), overwrite, error_message);
}

std::string JsonQuoted(const std::string& value)
{
    std::ostringstream out;
    out << '"';
    for (char character : value)
    {
        switch (character)
        {
            case '\\': out << "\\\\"; break;
            case '"': out << "\\\""; break;
            case '\n': out << "\\n"; break;
            case '\r': out << "\\r"; break;
            case '\t': out << "\\t"; break;
            default:
                if (static_cast<unsigned char>(character) < 0x20)
                {
                    out << "\\u" << std::hex << std::setw(4) << std::setfill('0')
                        << static_cast<int>(static_cast<unsigned char>(character))
                        << std::dec << std::setfill(' ');
                }
                else
                {
                    out << character;
                }
                break;
        }
    }
    out << '"';
    return out.str();
}
