"""验证 XV SDK 已校正 RGB 图像纯单目配置的关键参数与字段边界。"""

from pathlib import Path

import cv2
import pytest


# XV SDK 已校正图像对应的纯单目配置路径。
CONFIG_PATH = (
    Path(__file__).resolve().parents[1]
    / "config"
    / "monocular"
    / "XV_RGB_Fisheye_undistorted.yaml"
)
# 纯单目 launch 文件路径。
LAUNCH_PATH = (
    Path(__file__).resolve().parents[1]
    / "launch"
    / "xv_rgb_fisheye_undistorted_mono.launch.py"
)
# 纯单目 RViz2 配置路径。
RVIZ_CONFIG_PATH = (
    Path(__file__).resolve().parents[1]
    / "rviz"
    / "xv_rgb_fisheye_undistorted_mono.rviz"
)


def _read_node(file_storage: cv2.FileStorage, key: str):
    """读取 OpenCV YAML 节点，并在字段缺失时给出明确错误。"""
    node = file_storage.getNode(key)
    assert not node.empty(), f"missing YAML key: {key}"
    return node


def test_undistorted_monocular_uses_ideal_pinhole_camera() -> None:
    """纯单目校正流应使用针孔内参、60 Hz 配置和 960 方形内部尺寸。"""
    file_storage = cv2.FileStorage(str(CONFIG_PATH), cv2.FILE_STORAGE_READ)
    assert file_storage.isOpened(), f"cannot open {CONFIG_PATH}"

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
        assert _read_node(file_storage, "Camera.width").real() == pytest.approx(1280.0)
        assert _read_node(file_storage, "Camera.height").real() == pytest.approx(1280.0)
        assert _read_node(file_storage, "Camera.newWidth").real() == pytest.approx(960.0)
        assert _read_node(file_storage, "Camera.newHeight").real() == pytest.approx(960.0)
        assert _read_node(file_storage, "Camera.fps").real() == pytest.approx(60.0)
        assert _read_node(file_storage, "Camera.RGB").real() == pytest.approx(1.0)
    finally:
        file_storage.release()


def test_undistorted_monocular_omits_distortion_and_imu_fields() -> None:
    """纯单目校正流不应声明二次畸变参数或 IMU 配置。"""
    file_storage = cv2.FileStorage(str(CONFIG_PATH), cv2.FILE_STORAGE_READ)
    assert file_storage.isOpened(), f"cannot open {CONFIG_PATH}"

    try:
        unexpected_keys = (
            "Camera1.k1",
            "Camera1.k2",
            "Camera1.p1",
            "Camera1.p2",
            "Camera1.k3",
            "IMU.T_b_c1",
            "IMU.NoiseGyro",
            "IMU.NoiseAcc",
            "IMU.GyroWalk",
            "IMU.AccWalk",
            "IMU.Frequency",
            "ROS.TargetFps",
            "ROS.ImuTimeOffsetSec",
        )
        for key in unexpected_keys:
            assert file_storage.getNode(key).empty(), f"unexpected YAML key: {key}"
    finally:
        file_storage.release()


def test_viewer_scale_does_not_change_tracking_resolution() -> None:
    """显示和特征提取输入当前均保持 960 方形分辨率。"""
    file_storage = cv2.FileStorage(str(CONFIG_PATH), cv2.FILE_STORAGE_READ)
    assert file_storage.isOpened(), f"cannot open {CONFIG_PATH}"

    try:
        tracking_width = _read_node(file_storage, "Camera.newWidth").real()
        tracking_height = _read_node(file_storage, "Camera.newHeight").real()
        viewer_scale = _read_node(file_storage, "Viewer.imageViewScale").real()

        assert tracking_width == pytest.approx(960.0)
        assert tracking_height == pytest.approx(960.0)
        assert tracking_width * viewer_scale == pytest.approx(960.0, abs=1e-3)
        assert tracking_height * viewer_scale == pytest.approx(960.0, abs=1e-3)
    finally:
        file_storage.release()


def test_rviz_uses_compact_pose_axes_and_remapped_camera_image() -> None:
    """RViz 应缩小位姿坐标轴，并通过 launch 跟随 camera_topic 订阅视频。"""
    # 用于验证显示尺寸和内部视频话题的 RViz 配置文本。
    rviz_config = RVIZ_CONFIG_PATH.read_text(encoding="utf-8")
    # 用于验证内部视频话题重映射关系的 launch 文件文本。
    launch_source = LAUNCH_PATH.read_text(encoding="utf-8")

    assert "Marker Scale: 0.10000000149011612" in rviz_config
    assert "Axes Length: 0.10000000149011612" in rviz_config
    assert "Axes Radius: 0.009999999776482582" in rviz_config
    assert "Class: rviz_default_plugins/Image" in rviz_config
    assert "Name: Camera Image" in rviz_config
    assert "Reliability Policy: Best Effort" in rviz_config
    assert "Value: /orbslam3/input_image" in rviz_config
    assert 'remappings=[("/orbslam3/input_image", camera_topic)]' in launch_source
