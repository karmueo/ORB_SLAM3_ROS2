#!/usr/bin/env python3
"""验证独立时间对齐工具的合成符号、方向和 offset 可辨识性。"""

from __future__ import annotations

import json
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
SCRIPTS = ROOT / "scripts"
TOOL = SCRIPTS / "align_ego_stereo_inertial_time.py"

sys.path.insert(0, str(SCRIPTS))

import align_ego_stereo_inertial_time as align  # noqa: E402


def test_synthetic_time_varying_offset_and_direction() -> None:
    result = align.synthetic_self_test()
    assert result["passed"], result
    assert abs(result["estimated_offset_ms"] - result["true_offset_ms"]) <= 1.0
    # 转置方向必须明显更差，说明符号/方向不是靠不可辨识的常角速度猜出来的。
    assert result["transposed_direction_error_deg"] > (
        20.0 * result["correct_direction_error_deg"]
    ), result


def test_cli_self_test_outputs_json() -> None:
    completed = subprocess.run(
        [sys.executable, str(TOOL), "--self-test"],
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        check=False,
    )
    assert completed.returncode == 0, completed.stderr
    payload = json.loads(completed.stdout)
    assert payload["passed"] is True
    assert abs(payload["estimated_offset_ms"] - payload["true_offset_ms"]) <= 1.0


def test_contract_rotation_direction() -> None:
    # Q_raw = Exp(theta) 时，契约预测必须是 R_cb Q_raw^T R_cb^T。
    rotation_cam_body = align.np.asarray(
        [[1.0, 0.0, 0.0], [0.0, 0.0, -1.0], [0.0, 1.0, 0.0]], dtype=float
    )
    theta = align.np.asarray([0.2, -0.1, 0.05], dtype=float)
    q_raw = align._matrix_exp(theta)
    predicted = align.predict_visual_rotation(q_raw, rotation_cam_body)
    expected = rotation_cam_body @ q_raw.T @ rotation_cam_body.T
    assert align.np.allclose(predicted, expected, atol=1e-12)


# Codex 独立验收：解析解不调用被测工具的旋转生成器或自测函数。
# 验证积分严格止于请求的终点，覆盖两个边界落在采样间隔内部的情况。
def test_analytic_constant_gyro_clips_requested_interval() -> None:
    import math

    stamps = align.np.asarray([0.0, 0.01, 0.02, 0.03])
    gyro = align.np.tile([0.0, 0.0, 1.0], (len(stamps), 1))
    for start, end in [(0.002, 0.007), (0.002, 0.017), (0.01, 0.02)]:
        angle = end - start
        expected = align.np.asarray([
            [math.cos(angle), -math.sin(angle), 0.0],
            [math.sin(angle), math.cos(angle), 0.0],
            [0.0, 0.0, 1.0],
        ])
        actual = align.integrate_body_rotation(stamps, gyro, start, end)
        assert align.np.allclose(actual, expected, rtol=0.0, atol=1e-12), (
            start, end, actual, expected
        )


def test_analytic_linear_gyro_interpolates_both_endpoints() -> None:
    import math

    stamps = align.np.asarray([0.0, 0.01, 0.02, 0.03])
    gyro = align.np.zeros((len(stamps), 3))
    gyro[:, 2] = 3.0 + 40.0 * stamps
    start, end = 0.002, 0.017
    angle = 3.0 * (end - start) + 20.0 * (end * end - start * start)
    expected = align.np.asarray([
        [math.cos(angle), -math.sin(angle), 0.0],
        [math.sin(angle), math.cos(angle), 0.0],
        [0.0, 0.0, 1.0],
    ])
    actual = align.integrate_body_rotation(stamps, gyro, start, end)
    assert align.np.allclose(actual, expected, rtol=0.0, atol=1e-12), (actual, expected)


# Codex 独立验收：验证真正安装的 ROS 入口，避免源码目录掩盖缺失模块。
def test_installed_alignment_cli_runs_outside_source_tree(tmp_path) -> None:
    import os

    env = os.environ.copy()
    source_entries = {ROOT.resolve(), SCRIPTS.resolve()}
    env["PYTHONPATH"] = os.pathsep.join(
        part for part in env.get("PYTHONPATH", "").split(os.pathsep)
        if part and Path(part).resolve() not in source_entries
    )
    env["PYTHONNOUSERSITE"] = "1"
    completed = subprocess.run(
        ["ros2", "run", "orbslam3", "align-ego-stereo-inertial-time", "--help"],
        cwd=tmp_path,
        env=env,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        timeout=30,
        check=False,
    )
    assert completed.returncode == 0, completed.stdout + completed.stderr
    assert "--calib" in completed.stdout and "--bag" in completed.stdout
