/**
 * @file test_offline_output.cpp
 * @brief 测试离线轨迹分组、输出命名、覆盖清单和输入别名保护。
 */

#include "offline-output.hpp"

#include "gtest/gtest.h"

#include <sys/stat.h>
#include <unistd.h>

#include <cstdio>
#include <fstream>
#include <iterator>
#include <string>
#include <vector>

namespace
{
std::string MakeTempDirectory()
{
    char pattern[] = "/tmp/orbslam3_offline_output_XXXXXX";
    char* directory = ::mkdtemp(pattern);
    EXPECT_NE(directory, nullptr);
    return directory ? std::string(directory) : std::string();
}

void WriteFile(const std::string& path, const std::string& content)
{
    std::ofstream output(path.c_str());
    output << content;
}

std::string ReadFile(const std::string& path)
{
    std::ifstream input(path.c_str());
    return std::string(
        (std::istreambuf_iterator<char>(input)),
        std::istreambuf_iterator<char>());
}

bool Exists(const std::string& path)
{
    struct stat info;
    return ::lstat(path.c_str(), &info) == 0;
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
}  // namespace

TEST(OfflineOutput, SelectsLargestMapAndBreaksTieByMapId)
{
    std::vector<ORB_SLAM3::OfflineMapInfo> maps(2);
    maps[0].id = 3;
    maps[0].imuInitialized = true;
    maps[1].id = 2;
    maps[1].imuInitialized = true;
    const std::vector<ORB_SLAM3::OfflineCameraPose> poses = {
        MakePose(10, 3), MakePose(20, 2), MakePose(30, 3)};

    OfflineOutputPlan plan;
    std::string error;
    ASSERT_TRUE(SelectOfflineOutputPlan(poses, maps, &plan, &error)) << error;
    EXPECT_EQ(plan.maps.size(), 2U);
    ASSERT_EQ(plan.maps[0].map_id, 2U);
    ASSERT_EQ(plan.maps[1].map_id, 3U);
    EXPECT_EQ(plan.main_index, 1U);
    EXPECT_EQ(plan.maps[plan.main_index].map_id, 3U);
    EXPECT_EQ(plan.main_frame_count, 2U);
    EXPECT_EQ(plan.total_frame_count, 3U);
}

TEST(OfflineOutput, RejectsDuplicateTimestampsAcrossMaps)
{
    std::vector<ORB_SLAM3::OfflineMapInfo> maps(2);
    maps[0].id = 1;
    maps[0].imuInitialized = true;
    maps[1].id = 2;
    maps[1].imuInitialized = true;
    OfflineOutputPlan plan;
    std::string error;
    EXPECT_FALSE(SelectOfflineOutputPlan(
        {MakePose(10, 1), MakePose(10, 2)}, maps, &plan, &error));
    EXPECT_NE(error.find("重复时间戳"), std::string::npos);
}

TEST(OfflineOutput, BuildsExactTimestampAndDeterministicMapNames)
{
    const std::string directory = MakeTempDirectory();
    const OfflineOutputPaths paths =
        BuildOfflineOutputPaths(directory + "/trajectory.txt");
    OfflineOutputPlan plan;
    plan.maps.resize(2);
    plan.maps[0].map_id = 2;
    plan.maps[0].poses = {MakePose(1793850304571406667LL, 2)};
    plan.maps[1].map_id = 5;
    plan.maps[1].poses = {
        MakePose(1793850304591406667LL, 5),
        MakePose(1793850304611406667LL, 5)};
    plan.maps[1].poses.back().recentlyLost = true;
    plan.main_index = 1U;
    plan.main_frame_count = 2U;
    plan.total_frame_count = 3U;

    std::vector<OfflineOutputFile> files;
    std::string error;
    ASSERT_TRUE(BuildOfflineTrajectoryFiles(plan, paths, &files, &error)) << error;
    ASSERT_EQ(files.size(), 3U);
    EXPECT_EQ(files[2].path, directory + "/trajectory.frames.csv");
    EXPECT_EQ(files[2].content,
              "timestamp_ns,map_id,tracking_state\n"
              "1793850304571406667,2,OK\n"
              "1793850304591406667,5,OK\n"
              "1793850304611406667,5,RECENTLY_LOST\n");
    EXPECT_EQ(files[0].path, directory + "/trajectory.map-2.txt");
    EXPECT_EQ(files[1].path, directory + "/trajectory.txt");
    EXPECT_NE(files[0].content.find("1793850304.571406667"), std::string::npos);
    EXPECT_EQ(files[1].content.find("1793850304.571406667"), std::string::npos);

    std::vector<std::string> stale;
    ASSERT_TRUE(PreflightOfflineOutputs(
        paths, false, {}, &stale, &error)) << error;
    ASSERT_TRUE(RemoveStaleOfflineFiles(stale, &error)) << error;
    for (const OfflineOutputFile& file : files)
    {
        ASSERT_TRUE(WriteAtomicTextFile(file.path, file.content, false, &error)) << error;
    }
    EXPECT_EQ(ReadFile(paths.main), files[1].content);
    EXPECT_TRUE(Exists(directory + "/trajectory.map-2.txt"));
}

TEST(OfflineOutput, RejectsInputAliasesSymlinksAndHardlinks)
{
    const std::string directory = MakeTempDirectory();
    const std::string input = directory + "/input.bag";
    WriteFile(input, "readonly");
    const OfflineOutputPaths same_paths =
        BuildOfflineOutputPaths(input);
    std::vector<std::string> stale;
    std::string error;
    EXPECT_FALSE(PreflightOfflineOutputs(
        same_paths, true, {input}, &stale, &error));
    EXPECT_TRUE(Exists(input));

    const std::string symlink_path = directory + "/symlink_out.txt";
    ASSERT_EQ(::symlink(input.c_str(), symlink_path.c_str()), 0);
    EXPECT_FALSE(PreflightOfflineOutputs(
        BuildOfflineOutputPaths(symlink_path), true, {input}, &stale, &error));

    const std::string hardlink_path = directory + "/hardlink_out.txt";
    ASSERT_EQ(::link(input.c_str(), hardlink_path.c_str()), 0);
    EXPECT_FALSE(PreflightOfflineOutputs(
        BuildOfflineOutputPaths(hardlink_path), true, {input}, &stale, &error));
    EXPECT_EQ(ReadFile(input), "readonly");
}

TEST(OfflineOutput, NoOverwriteRejectsExistingArtifacts)
{
    const std::string directory = MakeTempDirectory();
    const OfflineOutputPaths paths =
        BuildOfflineOutputPaths(directory + "/trajectory.txt");
    WriteFile(paths.main, "old");
    std::vector<std::string> stale;
    std::string error;
    EXPECT_FALSE(PreflightOfflineOutputs(paths, false, {}, &stale, &error));
    EXPECT_NE(error.find("--overwrite"), std::string::npos);
    EXPECT_EQ(ReadFile(paths.main), "old");
}

TEST(OfflineOutput, RejectsMainTrajectoryCollidingWithLog)
{
    for (const bool overwrite : {false, true})
    {
        const std::string directory = MakeTempDirectory();
        const OfflineOutputPaths paths =
            BuildOfflineOutputPaths(directory + "/trajectory.log");
        std::vector<std::string> stale;
        std::string error;
        EXPECT_FALSE(PreflightOfflineOutputs(paths, overwrite, {}, &stale, &error));
        EXPECT_NE(error.find("冲突"), std::string::npos) << error;
        EXPECT_FALSE(Exists(paths.main));
        EXPECT_FALSE(Exists(paths.manifest));
    }
}

TEST(OfflineOutput, OverwriteRejectsUnownedArtifactsWithoutManifest)
{
    for (const char* name : {
             "trajectory.txt", "trajectory.summary.json", "trajectory.log",
             "trajectory.frames.csv", "trajectory.map-4.txt"})
    {
        SCOPED_TRACE(name);
        const std::string directory = MakeTempDirectory();
        const OfflineOutputPaths paths =
            BuildOfflineOutputPaths(directory + "/trajectory.txt");
        const std::string unowned = directory + "/" + name;
        WriteFile(unowned, "keep-me\n");
        std::vector<std::string> stale;
        std::string error;
        EXPECT_FALSE(PreflightOfflineOutputs(paths, true, {}, &stale, &error));
        EXPECT_NE(error.find("manifest"), std::string::npos) << error;
        EXPECT_EQ(ReadFile(unowned), "keep-me\n");
        EXPECT_FALSE(Exists(paths.manifest));
    }
}

TEST(OfflineOutput, OverwriteRejectsArtifactsOmittedFromValidManifest)
{
    for (const char* name : {
             "trajectory.txt", "trajectory.summary.json", "trajectory.log",
             "trajectory.frames.csv", "trajectory.map-4.txt"})
    {
        SCOPED_TRACE(name);
        const std::string directory = MakeTempDirectory();
        const OfflineOutputPaths paths =
            BuildOfflineOutputPaths(directory + "/trajectory.txt");
        const std::string manifest =
            "owner=orbslam3-offline-trajectory\noutput=trajectory.txt\n";
        WriteFile(paths.manifest, manifest);
        const std::string unowned = directory + "/" + name;
        WriteFile(unowned, "keep-me\n");
        std::vector<std::string> stale;
        std::string error;
        EXPECT_FALSE(PreflightOfflineOutputs(paths, true, {}, &stale, &error));
        EXPECT_NE(error.find("manifest"), std::string::npos) << error;
        EXPECT_EQ(ReadFile(unowned), "keep-me\n");
        EXPECT_EQ(ReadFile(paths.manifest), manifest);
    }
}

TEST(OfflineOutput, OverwriteUsesManifestToRemoveOldSideMaps)
{
    const std::string directory = MakeTempDirectory();
    const OfflineOutputPaths paths =
        BuildOfflineOutputPaths(directory + "/trajectory.txt");
    std::vector<OfflineOutputFile> old_files = {
        {paths.main, "old-main\n"},
        {directory + "/trajectory.map-9.txt", "old-side\n"}};
    std::string error;
    ASSERT_TRUE(WriteAtomicTextFile(paths.main, old_files[0].content, false, &error));
    ASSERT_TRUE(WriteAtomicTextFile(old_files[1].path, old_files[1].content, false, &error));
    ASSERT_TRUE(WriteOfflineManifest(paths, old_files, false, &error)) << error;

    std::vector<std::string> stale;
    ASSERT_TRUE(PreflightOfflineOutputs(paths, true, {}, &stale, &error)) << error;
    EXPECT_FALSE(stale.empty());
    ASSERT_TRUE(RemoveStaleOfflineFiles(stale, &error)) << error;
    EXPECT_FALSE(Exists(paths.main));
    EXPECT_FALSE(Exists(directory + "/trajectory.map-9.txt"));
}

TEST(OfflineOutput, OverwriteRejectsUnknownSideMapWithoutManifest)
{
    const std::string directory = MakeTempDirectory();
    const OfflineOutputPaths paths =
        BuildOfflineOutputPaths(directory + "/trajectory.txt");
    WriteFile(directory + "/trajectory.map-4.txt", "unknown\n");
    std::vector<std::string> stale;
    std::string error;
    EXPECT_FALSE(PreflightOfflineOutputs(paths, true, {}, &stale, &error));
    EXPECT_NE(error.find("manifest"), std::string::npos);
    EXPECT_TRUE(Exists(directory + "/trajectory.map-4.txt"));
}
