"""验证 XV 0709 bag 验收脚本的日志统计逻辑。"""

from __future__ import annotations

import importlib.util
import sys
from pathlib import Path


MODULE_PATH = Path(__file__).resolve().parents[1] / "scripts" / "validate_xv_0709_bag.py"


def load_module():
    """加载未安装的验收脚本模块，便于直接测试纯日志统计函数。"""
    spec = importlib.util.spec_from_file_location("validate_xv_0709_bag", MODULE_PATH)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def test_summarize_log_counts_each_reset_once():
    """日志统计应合并重置原因日志，并识别上游兼容的跟踪状态。"""
    validator = load_module()
    log_text = (
        "等待 IMU 覆盖图像时间戳 1.0\n"
        "过滤 IMU 回跳或重复时间戳样本\n"
        "SYSTEM-> Reseting active map in monocular case\n"
        "IMU is not or recently initialized. Reseting active map...\n"
        "Not enough motion for initializing. Defer active map reset before inertial BA2.\n"
        "单目惯性帧诊断：tracking_state=2，pose_published=1\n"
        "单目惯性帧诊断：tracking_state=5，pose_published=1\n"
    )

    summary = validator.summarize_log(log_text)

    assert summary.wait_imu_count == 1
    assert summary.filtered_imu_count == 1
    assert summary.active_reset_count == 1
    assert summary.bad_imu_count == 1
    assert summary.deferred_low_motion_reset_count == 1
    assert summary.tracking_ok_count == 2
    assert summary.pose_published_count == 2

def test_run_validation_allows_transient_imu_wait(tmp_path):
    """短暂等待 IMU 只作为诊断指标，不应让成功运行失败。"""
    validator = load_module()
    results_dir = tmp_path / "results"
    results_dir.mkdir()
    (results_dir / "node.log").write_text(
        "等待 IMU 覆盖图像时间戳 1.0\n"
        "单目惯性帧诊断：tracking_state=2，pose_published=1\n",
        encoding="utf-8",
    )
    (results_dir / "KeyFrameTrajectory.txt").write_text("0 0 0 0 0 0 0 1\n", encoding="utf-8")
    args = validator.build_parser().parse_args(
        [
            "--skip-run",
            "--results-dir",
            str(results_dir),
            "--min-pose-published",
            "1",
        ]
    )

    assert validator.run_validation(args) == 0
