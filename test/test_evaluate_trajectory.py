"""验证 Vicon GT 轨迹误差测评工具的核心离线算法。"""

from __future__ import annotations

import importlib.util
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest


MODULE_PATH = Path(__file__).resolve().parents[1] / "scripts" / "evaluate_trajectory.py"


def load_module():
    """加载未安装的测评脚本模块，便于在源码树内直接测试。"""
    spec = importlib.util.spec_from_file_location("evaluate_trajectory", MODULE_PATH)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


evaluate_trajectory = load_module()


def test_read_tum_trajectory_skips_comments_and_blank_lines(tmp_path):
    """TUM 轨迹解析应跳过注释和空行，并读取时间戳、平移和姿态。"""
    trajectory_path = tmp_path / "KeyFrameTrajectory.txt"
    trajectory_path.write_text(
        "# comment\n"
        "\n"
        "1.0 1 2 3 0 0 0 1\n"
        "2.5 4 5 6 0 0 1 0\n",
        encoding="utf-8",
    )

    trajectory = evaluate_trajectory.read_tum_trajectory(trajectory_path)

    assert trajectory.timestamps.tolist() == [1.0, 2.5]
    np.testing.assert_allclose(trajectory.positions, [[1, 2, 3], [4, 5, 6]])
    np.testing.assert_allclose(trajectory.quaternions, [[0, 0, 0, 1], [0, 0, 1, 0]])


def test_read_tum_trajectory_rejects_wrong_column_count(tmp_path):
    """TUM 轨迹行列数错误时应报告明确异常。"""
    trajectory_path = tmp_path / "bad.txt"
    trajectory_path.write_text("1.0 1 2 3 0 0 1\n", encoding="utf-8")

    with pytest.raises(ValueError, match="8"):
        evaluate_trajectory.read_tum_trajectory(trajectory_path)


def test_interpolate_gt_positions_filters_times_outside_gt_range():
    """GT 插值应只保留落在 GT 时间范围内的估计点。"""
    estimate_times = np.array([0.0, 0.5, 1.5, 2.5, 3.0])
    gt_times = np.array([1.0, 2.0])
    gt_positions = np.array([[1.0, 2.0, 3.0], [3.0, 4.0, 5.0]])

    valid_times, gt_interp, valid_mask = evaluate_trajectory.interpolate_gt_positions(
        estimate_times, gt_times, gt_positions
    )

    np.testing.assert_allclose(valid_times, [1.5])
    np.testing.assert_allclose(gt_interp, [[2.0, 3.0, 4.0]])
    np.testing.assert_array_equal(valid_mask, [False, False, True, False, False])


def test_se3_alignment_recovers_known_rotation_and_translation():
    """SE3 对齐应恢复已知刚体变换，尺度固定为 1。"""
    source = np.array([[0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [0.0, 2.0, 0.0], [1.0, 2.0, 1.0]])
    angle = np.pi / 2.0
    rotation = np.array(
        [
            [np.cos(angle), -np.sin(angle), 0.0],
            [np.sin(angle), np.cos(angle), 0.0],
            [0.0, 0.0, 1.0],
        ]
    )
    translation = np.array([1.0, -2.0, 0.5])
    target = (rotation @ source.T).T + translation

    result = evaluate_trajectory.align_points_umeyama(source, target, with_scale=False)

    assert result.scale == pytest.approx(1.0)
    np.testing.assert_allclose(result.rotation, rotation, atol=1e-12)
    np.testing.assert_allclose(result.translation, translation, atol=1e-12)
    np.testing.assert_allclose(result.aligned_points, target, atol=1e-12)


def test_sim3_alignment_recovers_known_scale():
    """Sim3 对齐应恢复已知尺度、旋转和平移。"""
    source = np.array([[0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.5, 0.5, 1.0]])
    scale = 2.5
    rotation = np.eye(3)
    translation = np.array([-1.0, 3.0, 0.25])
    target = scale * (rotation @ source.T).T + translation

    result = evaluate_trajectory.align_points_umeyama(source, target, with_scale=True)

    assert result.scale == pytest.approx(scale)
    np.testing.assert_allclose(result.aligned_points, target, atol=1e-12)


def test_parse_tbc_and_convert_twc_to_twb(tmp_path):
    """body 模式应从 OpenCV YAML 解析 Tbc，并将 Twc 转换为 Twb。"""
    settings_path = tmp_path / "settings.yaml"
    settings_path.write_text(
        "Tbc: !!opencv-matrix\n"
        "   rows: 4\n"
        "   cols: 4\n"
        "   dt: f\n"
        "   data: [1, 0, 0, 1,\n"
        "          0, 1, 0, 2,\n"
        "          0, 0, 1, 3,\n"
        "          0, 0, 0, 1]\n",
        encoding="utf-8",
    )
    twc_positions = np.array([[10.0, 20.0, 30.0]])
    twc_quaternions = np.array([[0.0, 0.0, 0.0, 1.0]])

    tbc = evaluate_trajectory.read_tbc(settings_path)
    twb_positions = evaluate_trajectory.convert_camera_to_body_positions(twc_positions, twc_quaternions, tbc)

    np.testing.assert_allclose(tbc[:3, 3], [1.0, 2.0, 3.0])
    np.testing.assert_allclose(twb_positions, [[9.0, 18.0, 27.0]])


def test_read_openloris_groundtruth_file_skips_metadata_and_deduplicates(tmp_path):
    """OpenLORIS groundtruth 文件解析应跳过头部元数据并保留首个重复时间戳。"""
    groundtruth_path = tmp_path / "groundtruth.txt"
    groundtruth_path.write_text(
        "scene: cafe\n"
        "seq: cafe1-1\n"
        "frame: gt_map base_link\n"
        "# timestamp tx ty tz qx qy qz qw\n"
        "\n"
        "2.0 2 3 4 0 0 0 1\n"
        "1.0 1 2 3 0 0 0 1\n"
        "2.0 20 30 40 0 0 0 1\n",
        encoding="utf-8",
    )

    gt_times, gt_positions = evaluate_trajectory.read_groundtruth_file(groundtruth_path)

    np.testing.assert_allclose(gt_times, [1.0, 2.0])
    np.testing.assert_allclose(gt_positions, [[1.0, 2.0, 3.0], [2.0, 3.0, 4.0]])


def test_parse_imu_t_b_c1_and_convert_twc_to_twb(tmp_path):
    """body 模式应支持 OpenLORIS 配置中的 IMU.T_b_c1 外参键名。"""
    settings_path = tmp_path / "openloris.yaml"
    settings_path.write_text(
        "IMU.T_b_c1: !!opencv-matrix\n"
        "   rows: 4\n"
        "   cols: 4\n"
        "   dt: f\n"
        "   data: [1, 0, 0, 0.1,\n"
        "          0, 1, 0, 0.2,\n"
        "          0, 0, 1, 0.3,\n"
        "          0, 0, 0, 1]\n",
        encoding="utf-8",
    )

    tbc = evaluate_trajectory.read_tbc(settings_path)
    twb_positions = evaluate_trajectory.convert_camera_to_body_positions(
        np.array([[1.0, 2.0, 3.0]]),
        np.array([[0.0, 0.0, 0.0, 1.0]]),
        tbc,
    )

    np.testing.assert_allclose(tbc[:3, 3], [0.1, 0.2, 0.3])
    np.testing.assert_allclose(twb_positions, [[0.9, 1.8, 2.7]])


def test_extract_tf_message_gt_filters_child_frame():
    """TFMessage GT 读取应只提取目标 child_frame_id 的 transform。"""
    ignored_transform = SimpleNamespace(
        header=SimpleNamespace(stamp=SimpleNamespace(sec=10, nanosec=100), frame_id="gt_map"),
        child_frame_id="camera",
        transform=SimpleNamespace(translation=SimpleNamespace(x=9.0, y=9.0, z=9.0)),
    )
    selected_transform = SimpleNamespace(
        header=SimpleNamespace(stamp=SimpleNamespace(sec=10, nanosec=200), frame_id="gt_map"),
        child_frame_id="base_link",
        transform=SimpleNamespace(translation=SimpleNamespace(x=1.0, y=2.0, z=3.0)),
    )
    message = SimpleNamespace(transforms=[ignored_transform, selected_transform])

    sample = evaluate_trajectory.extract_tf_message_gt_sample(message, "base_link")

    assert sample == pytest.approx((10.0000002, [1.0, 2.0, 3.0]))


def test_deduplicate_sorted_samples_keeps_first_timestamp():
    """GT 样本排序后应删除精确重复时间戳，并保留第一条样本。"""
    times, positions = evaluate_trajectory.sort_and_deduplicate_gt_samples(
        [2.0, 1.0, 2.0],
        [[2.0, 2.0, 2.0], [1.0, 1.0, 1.0], [20.0, 20.0, 20.0]],
    )

    np.testing.assert_allclose(times, [1.0, 2.0])
    np.testing.assert_allclose(positions, [[1.0, 1.0, 1.0], [2.0, 2.0, 2.0]])


def test_compute_ate_statistics():
    """ATE 统计值应覆盖 RMSE、均值、中位数、标准差、最小值和最大值。"""
    errors = np.array([0.0, 1.0, 2.0])

    metrics = evaluate_trajectory.compute_ate_metrics(errors)

    assert metrics["rmse_m"] == pytest.approx(np.sqrt(5.0 / 3.0))
    assert metrics["mean_m"] == pytest.approx(1.0)
    assert metrics["median_m"] == pytest.approx(1.0)
    assert metrics["std_m"] == pytest.approx(np.std(errors))
    assert metrics["min_m"] == pytest.approx(0.0)
    assert metrics["max_m"] == pytest.approx(2.0)
