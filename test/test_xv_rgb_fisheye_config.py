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

# XV SDK 已校正图像对应的理想针孔配置路径。
UNDISTORTED_CONFIG_PATH = (
    Path(__file__).resolve().parents[1]
    / "config"
    / "monocular-inertial"
    / "XV_RGB_Fisheye_undistorted.yaml"
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


def test_xv_rgb_fisheye_undistorted_uses_ideal_pinhole_model():
    """SDK 已校正图像配置应使用原输出内参且不触发 ORB-SLAM3 二次去畸变。"""
    file_storage = cv2.FileStorage(str(UNDISTORTED_CONFIG_PATH), cv2.FILE_STORAGE_READ)
    assert file_storage.isOpened(), f"cannot open {UNDISTORTED_CONFIG_PATH}"

    try:
        assert _read_node(file_storage, "Camera.type").string() == "PinHole"
        assert _read_node(file_storage, "Camera1.fx").real() == pytest.approx(
            397.07575683833136
        )
        assert _read_node(file_storage, "Camera1.fy").real() == pytest.approx(
            397.07575683833136
        )
        assert _read_node(file_storage, "Camera1.cx").real() == pytest.approx(
            637.7723482481603
        )
        assert _read_node(file_storage, "Camera1.cy").real() == pytest.approx(
            640.0889202000565
        )
        for key in ("Camera1.k1", "Camera1.k2", "Camera1.p1", "Camera1.p2"):
            assert file_storage.getNode(key).empty(), f"unexpected distortion key: {key}"
        assert _read_node(file_storage, "Camera.RGB").real() == pytest.approx(1.0)
        assert _read_node(file_storage, "ROS.ImuTimeOffsetSec").real() == pytest.approx(
            -0.005089461604383672
        )
        assert file_storage.getNode("ROS.AutoAlignImuToImage").empty()
    finally:
        file_storage.release()


def test_xv_rgb_fisheye_undistorted_reuses_calibrated_camera_to_body_transform():
    """像素去畸变不改变相机坐标系，已校正配置应复用标定得到的 T_b_c。"""
    raw_storage = cv2.FileStorage(str(CONFIG_PATH), cv2.FILE_STORAGE_READ)
    undistorted_storage = cv2.FileStorage(
        str(UNDISTORTED_CONFIG_PATH), cv2.FILE_STORAGE_READ
    )
    assert raw_storage.isOpened(), f"cannot open {CONFIG_PATH}"
    assert undistorted_storage.isOpened(), f"cannot open {UNDISTORTED_CONFIG_PATH}"

    try:
        raw_tbc = _read_node(raw_storage, "IMU.T_b_c1").mat()
        undistorted_tbc = _read_node(undistorted_storage, "IMU.T_b_c1").mat()
        assert undistorted_tbc.dtype == "float32"
        assert (undistorted_tbc == raw_tbc).all()
    finally:
        raw_storage.release()
        undistorted_storage.release()
