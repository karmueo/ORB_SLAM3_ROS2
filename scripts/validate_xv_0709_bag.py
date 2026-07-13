#!/usr/bin/env python3
"""运行 XV 0709 rosbag 的单目惯性完整验收，并统计 ORB_SLAM3 日志。"""

from __future__ import annotations

import argparse
import json
import os
import re
import signal
import subprocess
import sys
import time
from dataclasses import asdict, dataclass
from pathlib import Path


def resolve_package_share_directory() -> Path:
    """返回 orbslam3 资源目录，源码运行时回退到仓库根目录。"""
    try:
        from ament_index_python.packages import get_package_share_directory

        return Path(get_package_share_directory("orbslam3"))
    except (ImportError, LookupError):
        return Path(__file__).resolve().parents[1]


# 软件包资源目录，用于定位已安装或源码树内的 vocabulary 和配置。
PACKAGE_SHARE_DIR = resolve_package_share_directory()
DEFAULT_RESULTS_DIR = Path("/tmp/orbslam3_xv_0709_final")
DEFAULT_BAG_PATH = Path("/home/scl/datasets/ros2bag/0709")
DEFAULT_VOCAB_PATH = PACKAGE_SHARE_DIR / "vocabulary" / "ORBvoc.txt"
DEFAULT_SETTINGS_PATH = PACKAGE_SHARE_DIR / "config" / "monocular-inertial" / "XV_RGB_Fisheye_calibrated.yaml"
DEFAULT_CAMERA_TOPIC = "/xv_sdk/SN250801DR48FB26001253/rgb_registered/image"
DEFAULT_IMU_TOPIC = "/xv_sdk/SN250801DR48FB26001253/imu"


@dataclass
class ValidationSummary:
    """保存一次 XV 0709 bag 验收的日志统计结果。"""

    wait_imu_count: int
    filtered_imu_count: int
    active_reset_count: int
    active_reset_after_warmup_count: int
    bad_imu_count: int
    bad_imu_after_warmup_count: int
    deferred_low_motion_reset_count: int
    tracking_ok_count: int
    pose_published_count: int
    keyframe_trajectory_exists: bool
    keyframe_trajectory_size: int
    keyframe_trajectory_lines: int


def positive_int(value: str) -> int:
    """解析正整数命令行参数。"""
    parsed_value = int(value)
    if parsed_value < 0:
        raise argparse.ArgumentTypeError("value must be non-negative")
    return parsed_value


def count_lines(path: Path) -> int:
    """统计文本文件行数；文件不存在时返回 0。"""
    if not path.exists():
        return 0
    with path.open("r", encoding="utf-8", errors="replace") as handle:
        return sum(1 for _line in handle)


def summarize_log(log_text: str, trajectory_path: Path | None = None, warmup_seconds: float = 10.0) -> ValidationSummary:
    """从节点日志和轨迹文件中提取完整 bag 验收指标。"""
    # 当前扫描到的首帧图像时间戳，用于判断 reset 是否发生在预热后。
    first_image_timestamp: float | None = None
    # 当前扫描到的最近图像时间戳，用于给无时间戳的 ORB_SLAM3 cout 行归属时间。
    current_image_timestamp: float | None = None
    # 完整日志中的 active map reset 总次数。
    active_reset_count = 0
    # 预热窗口之后的 active map reset 次数。
    active_reset_after_warmup_count = 0
    # 完整日志中的 bad IMU 或 recent IMU reset 相关次数。
    bad_imu_count = 0
    # 预热窗口之后的 bad IMU 或 recent IMU reset 相关次数。
    bad_imu_after_warmup_count = 0
    # 图像时间戳匹配表达式。
    image_timestamp_pattern = re.compile(r"image_t=([0-9]+(?:\.[0-9]+)?)")

    for line in log_text.splitlines():
        # 当前行的图像时间戳匹配结果。
        image_timestamp_match = image_timestamp_pattern.search(line)
        if image_timestamp_match:
            current_image_timestamp = float(image_timestamp_match.group(1))
            if first_image_timestamp is None:
                first_image_timestamp = current_image_timestamp

        # 当前行是否位于首帧后的预热窗口之外。
        after_warmup = (
            first_image_timestamp is not None
            and current_image_timestamp is not None
            and current_image_timestamp - first_image_timestamp > warmup_seconds
        )
        if "SYSTEM-> Reseting active map" in line:
            active_reset_count += 1
            if after_warmup:
                active_reset_after_warmup_count += 1
        if "IMU is not or recently initialized" in line or "bad imu flag" in line:
            bad_imu_count += 1
            if after_warmup:
                bad_imu_after_warmup_count += 1

    # 轨迹文件路径；未传入时只统计日志指标。
    resolved_trajectory_path = trajectory_path
    # 轨迹文件是否存在。
    trajectory_exists = bool(resolved_trajectory_path and resolved_trajectory_path.exists())
    # 轨迹文件字节数。
    trajectory_size = resolved_trajectory_path.stat().st_size if trajectory_exists and resolved_trajectory_path else 0
    # 轨迹文件行数。
    trajectory_lines = count_lines(resolved_trajectory_path) if resolved_trajectory_path else 0

    return ValidationSummary(
        wait_imu_count=log_text.count("等待 IMU 覆盖"),
        filtered_imu_count=log_text.count("过滤 IMU"),
        active_reset_count=active_reset_count,
        active_reset_after_warmup_count=active_reset_after_warmup_count,
        bad_imu_count=bad_imu_count,
        bad_imu_after_warmup_count=bad_imu_after_warmup_count,
        deferred_low_motion_reset_count=log_text.count("Defer active map reset"),
        tracking_ok_count=len(re.findall(r"tracking_state=(?:2|5)(?:\D|$)", log_text)),
        pose_published_count=log_text.count("pose_published=1"),
        keyframe_trajectory_exists=trajectory_exists,
        keyframe_trajectory_size=trajectory_size,
        keyframe_trajectory_lines=trajectory_lines,
    )


def terminate_process(process: subprocess.Popen, timeout_seconds: float) -> None:
    """向进程组发送 SIGINT，并在超时后升级为 kill。"""
    if process.poll() is not None:
        return

    os.killpg(process.pid, signal.SIGINT)
    try:
        process.wait(timeout=timeout_seconds)
        return
    except subprocess.TimeoutExpired:
        pass

    os.killpg(process.pid, signal.SIGTERM)
    try:
        process.wait(timeout=5.0)
        return
    except subprocess.TimeoutExpired:
        pass

    os.killpg(process.pid, signal.SIGKILL)
    process.wait(timeout=5.0)


def build_parser() -> argparse.ArgumentParser:
    """创建命令行参数解析器。"""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--results-dir", type=Path, default=DEFAULT_RESULTS_DIR, help="验收结果输出目录")
    parser.add_argument("--bag", type=Path, default=DEFAULT_BAG_PATH, help="完整 rosbag2 目录")
    parser.add_argument("--vocabulary", type=Path, default=DEFAULT_VOCAB_PATH, help="ORB vocabulary 路径")
    parser.add_argument("--settings", type=Path, default=DEFAULT_SETTINGS_PATH, help="ORB_SLAM3 YAML 配置路径")
    parser.add_argument("--camera-topic", default=DEFAULT_CAMERA_TOPIC, help="XV RGB 图像 topic")
    parser.add_argument("--imu-topic", default=DEFAULT_IMU_TOPIC, help="XV IMU topic")
    parser.add_argument("--startup-wait-sec", type=float, default=6.0, help="节点启动后再播放 bag 的等待时间")
    parser.add_argument("--shutdown-timeout-sec", type=float, default=30.0, help="节点优雅退出等待时间")
    parser.add_argument("--warmup-sec", type=float, default=10.0, help="统计连续 reset 时忽略的预热窗口")
    parser.add_argument("--min-pose-published", type=positive_int, default=800, help="完整 bag 至少发布的位姿数量")
    parser.add_argument("--max-post-warmup-reset", type=positive_int, default=0, help="预热后允许的 active reset 数量")
    parser.add_argument("--max-post-warmup-bad-imu", type=positive_int, default=0, help="预热后允许的 bad IMU reset 数量")
    parser.add_argument("--skip-run", action="store_true", help="只统计已有日志和轨迹，不启动 ROS 进程")
    return parser


def run_validation(args: argparse.Namespace) -> int:
    """执行完整 bag 验收并返回进程退出码。"""
    # 结果目录，节点也在该目录下写 KeyFrameTrajectory.txt。
    results_dir: Path = args.results_dir
    results_dir.mkdir(parents=True, exist_ok=True)
    # 节点日志路径。
    node_log_path = results_dir / "node.log"
    # bag 播放日志路径。
    bag_log_path = results_dir / "bag.log"
    # 轨迹输出路径。
    trajectory_path = results_dir / "KeyFrameTrajectory.txt"

    if not args.skip_run:
        if trajectory_path.exists():
            trajectory_path.unlink()

        with node_log_path.open("w", encoding="utf-8", errors="replace") as node_log:
            # monocular-inertial 节点启动命令，viewer 固定为 false。
            node_command = [
                "ros2",
                "run",
                "orbslam3",
                "monocular-inertial",
                str(args.vocabulary),
                str(args.settings),
                "false",
                "--ros-args",
                "-r",
                f"camera:={args.camera_topic}",
                "-r",
                f"imu:={args.imu_topic}",
            ]
            # 节点子进程。
            node_process = subprocess.Popen(
                node_command,
                cwd=results_dir,
                stdout=node_log,
                stderr=subprocess.STDOUT,
                start_new_session=True,
                text=True,
            )

            try:
                time.sleep(args.startup_wait_sec)
                with bag_log_path.open("w", encoding="utf-8", errors="replace") as bag_log:
                    # rosbag2 完整播放命令。
                    bag_command = ["ros2", "bag", "play", str(args.bag)]
                    # bag 播放子进程。
                    bag_process = subprocess.Popen(
                        bag_command,
                        cwd=results_dir,
                        stdout=bag_log,
                        stderr=subprocess.STDOUT,
                        start_new_session=True,
                        text=True,
                    )
                    bag_return_code = bag_process.wait()
                if bag_return_code != 0:
                    terminate_process(node_process, args.shutdown_timeout_sec)
                    print(f"ros2 bag play failed with code {bag_return_code}", file=sys.stderr)
                    return bag_return_code
            finally:
                terminate_process(node_process, args.shutdown_timeout_sec)

    # 节点日志文本。
    node_log_text = node_log_path.read_text(encoding="utf-8", errors="replace") if node_log_path.exists() else ""
    # 验收摘要。
    summary = summarize_log(node_log_text, trajectory_path, args.warmup_sec)
    # 摘要 JSON 路径。
    summary_path = results_dir / "summary.json"
    summary_path.write_text(json.dumps(asdict(summary), ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(asdict(summary), ensure_ascii=False, indent=2))

    # 验收失败原因列表。
    failures: list[str] = []
    if summary.active_reset_after_warmup_count > args.max_post_warmup_reset:
        failures.append(f"预热后 active reset 次数为 {summary.active_reset_after_warmup_count}")
    if summary.bad_imu_after_warmup_count > args.max_post_warmup_bad_imu:
        failures.append(f"预热后 bad/recent IMU reset 次数为 {summary.bad_imu_after_warmup_count}")
    if summary.tracking_ok_count == 0:
        failures.append("日志未出现有效 tracking_state=2/5")
    if summary.pose_published_count < args.min_pose_published:
        failures.append(f"pose_published=1 数量 {summary.pose_published_count} 小于 {args.min_pose_published}")
    if not summary.keyframe_trajectory_exists or summary.keyframe_trajectory_size <= 0:
        failures.append("KeyFrameTrajectory.txt 不存在或为空")

    if failures:
        for failure in failures:
            print(f"FAIL: {failure}", file=sys.stderr)
        return 1
    return 0


def main() -> int:
    """解析参数并执行验收。"""
    # 命令行参数。
    args = build_parser().parse_args()
    return run_validation(args)


if __name__ == "__main__":
    raise SystemExit(main())
