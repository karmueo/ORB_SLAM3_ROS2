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
# 自有数据集和多种图像配置共用的纯单目 launch 文件路径。
DATASET_LAUNCH_PATH = (
    Path(__file__).resolve().parents[1]
    / "launch"
    / "xv_rgb_fisheye_mono.launch.py"
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
    assert "Fixed Frame: camera_start" in rviz_config
    assert "camera_start:" in rviz_config
    assert 'remappings=[("/orbslam3/input_image", camera_topic)]' in launch_source


def test_monocular_uses_default_native_feature_mask() -> None:
    """纯单目 launch 应默认加载 mask.png，并把原图和 mask 分开传入 ORB-SLAM3。"""
    # 用于验证默认掩膜资源和参数传递的 launch 文件文本。
    launch_source = LAUNCH_PATH.read_text(encoding="utf-8")
    # 纯单目节点实现路径。
    node_path = (
        Path(__file__).resolve().parents[1]
        / "src"
        / "monocular"
        / "monocular-slam-node.cpp"
    )
    # 用于验证原生掩膜调用且未修改输入帧的节点实现文本。
    node_source = node_path.read_text(encoding="utf-8")

    assert '"mask.png"' in launch_source
    assert '"XV_RGB_Fisheye_gripper_mask.png"' not in launch_source
    assert (
        "m_SLAM->TrackMonocular(\n"
        "            m_cvImPtr->image, m_feature_mask, timestamp_seconds)"
        in node_source
    )
    assert "ApplyFeatureMask" not in node_source


def test_monocular_ros_pose_output_can_be_disabled() -> None:
    """纯单目应默认发布 ROS 位姿，并支持关闭全部定位输出。"""
    # 用于验证 launch 参数声明和布尔类型传递的源码文本。
    launch_source = LAUNCH_PATH.read_text(encoding="utf-8")
    # 纯单目节点实现路径。
    node_path = (
        Path(__file__).resolve().parents[1]
        / "src"
        / "monocular"
        / "monocular-slam-node.cpp"
    )
    # 用于验证发布器条件创建和跟踪短路顺序的节点源码文本。
    node_source = node_path.read_text(encoding="utf-8")

    assert 'LaunchConfiguration("publish_ros_pose")' in launch_source
    assert '"publish_ros_pose": ParameterValue(' in launch_source
    assert "publish_ros_pose, value_type=bool" in launch_source
    assert (
        '"publish_ros_pose",\n'
        '                default_value="true"'
        in launch_source
    )
    assert 'declare_parameter<bool>("publish_ros_pose", true)' in node_source
    assert "if (m_publish_ros_pose)" in node_source
    assert "if (!m_publish_ros_pose)" in node_source
    assert "Monocular ROS pose output is disabled" in node_source

    # 三个源码位置用于验证关闭开关不会跳过 SLAM 跟踪。
    track_index = node_source.index("m_SLAM->TrackMonocular(")
    disabled_guard_index = node_source.index("if (!m_publish_ros_pose)")
    tracking_state_index = node_source.index("m_SLAM->GetTrackingState()")
    assert track_index < disabled_guard_index < tracking_state_index


def test_dataset_monocular_launch_supports_configurable_slam_and_rviz() -> None:
    """自有数据集 launch 应统一配置 mono、mask、位姿输出和 RViz2 视频。"""
    # 通用纯单目 launch 源码文本。
    launch_source = DATASET_LAUNCH_PATH.read_text(encoding="utf-8")

    # 必须同时声明并读取的通用 launch 参数名称。
    expected_arguments = (
        "vocabulary_path",
        "settings_path",
        "camera_topic",
        "feature_mask_path",
        "max_path_length",
        "publish_ros_pose",
        "use_viewer",
        "use_rviz",
        "rviz_config_path",
    )
    for argument_name in expected_arguments:
        assert f'LaunchConfiguration("{argument_name}")' in launch_source
        assert f'"{argument_name}",' in launch_source

    assert '"XV_RGB_Fisheye_calibrated.yaml"' in launch_source
    assert '"/xv_sdk/SN250801DR48FB26001253/rgb/image"' in launch_source
    assert '"fisheye_mask.png"' in launch_source
    assert '"publish_ros_pose": ParameterValue(' in launch_source
    assert "publish_ros_pose, value_type=bool" in launch_source
    assert 'remappings=[("camera", camera_topic)]' in launch_source
    assert 'remappings=[("/orbslam3/input_image", camera_topic)]' in launch_source
    assert "condition=IfCondition(use_rviz)" in launch_source
    assert '"xv_rgb_fisheye_undistorted_mono.rviz"' in launch_source
