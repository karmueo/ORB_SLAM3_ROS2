"""验证 XV RGB 鱼眼单目惯性运行配置的关键参数。"""

from __future__ import annotations

from pathlib import Path

import cv2
import pytest


CONFIG_PATH = (
    Path(__file__).resolve().parents[1]
    / "config"
    / "monocular-inertial"
    / "XV_RGB_Fisheye_calibrated.yaml"
)


def _read_node(file_storage: cv2.FileStorage, key: str):
    """读取 OpenCV YAML 节点，节点缺失时让测试给出明确错误。"""
    node = file_storage.getNode(key)
    assert not node.empty(), f"missing YAML key: {key}"
    return node


def test_xv_rgb_fisheye_runtime_parameters_match_0709_plan():
    """0709 数据集配置应使用降采样、限帧和延后低运动 reset 策略。"""
    file_storage = cv2.FileStorage(str(CONFIG_PATH), cv2.FILE_STORAGE_READ)
    assert file_storage.isOpened(), f"cannot open {CONFIG_PATH}"

    try:
        assert _read_node(file_storage, "Camera.newWidth").real() == pytest.approx(960.0)
        assert _read_node(file_storage, "Camera.newHeight").real() == pytest.approx(960.0)
        assert _read_node(file_storage, "Camera.fps").real() == pytest.approx(15.0)
        assert _read_node(file_storage, "ROS.TargetFps").real() == pytest.approx(15.0)
        assert _read_node(file_storage, "ROS.MaxImageQueueSize").real() == pytest.approx(2.0)
        assert _read_node(file_storage, "ORBextractor.nFeatures").real() == pytest.approx(1200.0)
        assert _read_node(file_storage, "IMU.DeferLowMotionReset").real() == pytest.approx(1.0)
        assert _read_node(file_storage, "ROS.ImuTimeOffsetSec").real() == pytest.approx(
            -0.005089461604383672
        )
    finally:
        file_storage.release()


def test_xv_rgb_fisheye_tbc_is_opencv_float_matrix():
    """IMU.T_b_c1 应保持 OpenCV 4x4 CV_32F 矩阵格式。"""
    file_storage = cv2.FileStorage(str(CONFIG_PATH), cv2.FILE_STORAGE_READ)
    assert file_storage.isOpened(), f"cannot open {CONFIG_PATH}"

    try:
        tbc = _read_node(file_storage, "IMU.T_b_c1").mat()
        assert tbc.shape == (4, 4)
        assert tbc.dtype == "float32"
        assert tbc[3].tolist() == pytest.approx([0.0, 0.0, 0.0, 1.0])
    finally:
        file_storage.release()
