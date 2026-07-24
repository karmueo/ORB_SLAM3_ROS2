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

# XV 校正图像单目惯性 launch 文件路径。
UNDISTORTED_LAUNCH_PATH = (
    Path(__file__).resolve().parents[1]
    / "launch"
    / "xv_rgb_fisheye_undistorted_imu.launch.py"
)

# 单目惯性节点实现文件路径。
MONOCULAR_INERTIAL_NODE_PATH = (
    Path(__file__).resolve().parents[1]
    / "src"
    / "monocular-inertial"
    / "monocular-inertial-node.cpp"
)

# XV 校正图像单目惯性 RViz2 配置路径。
UNDISTORTED_RVIZ_PATH = (
    Path(__file__).resolve().parents[1]
    / "rviz"
    / "xv_rgb_fisheye_undistorted_imu.rviz"
)


def _read_node(file_storage: cv2.FileStorage, key: str):
    """读取 OpenCV YAML 节点，节点缺失时让测试给出明确错误。"""
    node = file_storage.getNode(key)
    assert not node.empty(), f"missing YAML key: {key}"
    return node


def test_only_supported_xv_monocular_inertial_configs_remain():
    """XV 单目惯性目录只保留原始畸变流和 SDK 校正流配置。"""
    expected_names = {
        "XV_RGB_Fisheye_calibrated.yaml",
        "XV_RGB_Fisheye_undistorted.yaml",
    }
    actual_names = {
        path.name
        for path in CONFIG_PATH.parent.glob("XV_RGB_Fisheye*.yaml")
    }

    assert actual_names == expected_names


def test_calibrated_config_matches_archived_kalibr_result():
    """原始流配置应复用归档 Kalibr 文件中的相机参数和逆外参。"""
    file_storage = cv2.FileStorage(str(CONFIG_PATH), cv2.FILE_STORAGE_READ)
    assert file_storage.isOpened(), f"cannot open {CONFIG_PATH}"

    try:
        assert _read_node(file_storage, "Camera.type").string() == "KannalaBrandt8"
        expected_scalars = {
            "Camera1.fx": 397.07575683833136,
            "Camera1.fy": 397.07575683833136,
            "Camera1.cx": 637.7723482481603,
            "Camera1.cy": 640.0889202000565,
            "Camera1.k1": 0.0817184405261036,
            "Camera1.k2": -0.017473874510500673,
            "Camera1.k3": 0.004642522742384059,
            "Camera1.k4": -0.0031138505924389118,
            "ROS.ImuTimeOffsetSec": -0.005089461604383672,
        }
        for key, expected_value in expected_scalars.items():
            assert _read_node(file_storage, key).real() == pytest.approx(
                expected_value
            )

        expected_tbc = [
            0.999729781341468,
            0.00969564125634819,
            -0.0211272061467728,
            -0.0200285583895465,
            -0.0092884083006446,
            0.99977080554486,
            0.0192889048787812,
            0.0107219204777111,
            0.0213093822102039,
            -0.0190874545398372,
            0.999590705893672,
            0.0204154259017981,
            0.0,
            0.0,
            0.0,
            1.0,
        ]
        tbc = _read_node(file_storage, "IMU.T_b_c1").mat()
        assert tbc.flatten().tolist() == pytest.approx(expected_tbc)
    finally:
        file_storage.release()


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
        assert _read_node(
            file_storage, "IMU.DeferPreBA2TrackingReset"
        ).real() == pytest.approx(1.0)
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


def test_xv_rgb_fisheye_undistorted_launch_supports_bag_and_rviz():
    """单目惯性 launch 应支持关闭设备节点、限制 Path 并按需启动 RViz2。"""
    launch_source = UNDISTORTED_LAUNCH_PATH.read_text()

    assert 'LaunchConfiguration("start_xv_sdk")' in launch_source
    assert "xv_sdk_launch = OpaqueFunction(function=_create_xv_sdk_launch)" in launch_source
    assert '"start_xv_sdk",' in launch_source
    assert 'default_value="true"' in launch_source
    assert 'LaunchConfiguration("max_path_length")' in launch_source
    assert 'LaunchConfiguration("feature_mask_path")' in launch_source
    assert '"feature_mask_path": feature_mask_path' in launch_source
    assert '"mask.png"' in launch_source
    assert '"max_path_length": ParameterValue(' in launch_source
    assert 'LaunchConfiguration("save_keyframe_trajectory")' in launch_source
    assert '"save_keyframe_trajectory": ParameterValue(' in launch_source
    assert '"save_keyframe_trajectory",\n                default_value="false"' in launch_source
    assert 'LaunchConfiguration("use_rviz")' in launch_source
    assert "condition=IfCondition(use_rviz)" in launch_source
    assert 'remappings=[("/orbslam3/input_image", camera_topic)]' in launch_source
    assert "xv_rgb_fisheye_undistorted_imu.rviz" in launch_source


def test_xv_rgb_fisheye_undistorted_inertial_uses_native_feature_mask():
    """单目惯性节点应把原图和 mask 分别传给 ORB-SLAM3 原生接口。"""
    node_source = MONOCULAR_INERTIAL_NODE_PATH.read_text()

    assert 'declare_parameter<std::string>("feature_mask_path", "")' in node_source
    assert "LoadFeatureMask(featureMaskPath)" in node_source
    assert "ValidateFeatureMaskSize(" in node_source
    assert "ApplyFeatureMask(im, featureMask_)" not in node_source
    assert "TrackMonocular(im, featureMask_, tIm, vImuMeas)" in node_source
    assert "SLAM_->isInertialBA1Initialized()" in node_source
    assert 'declare_parameter<bool>("save_keyframe_trajectory", false)' in node_source
    assert "if (saveKeyframeTrajectory_)" in node_source


def test_xv_rgb_fisheye_undistorted_rviz_displays_body_outputs():
    """单目惯性 RViz2 配置应显示机体位姿、轨迹、TF 和输入图像。"""
    rviz_source = UNDISTORTED_RVIZ_PATH.read_text()

    assert "Name: Body Pose" in rviz_source
    assert "Value: /orbslam3/body_pose" in rviz_source
    assert "Name: Body Path" in rviz_source
    assert "Value: /orbslam3/path" in rviz_source
    assert "body_link:" in rviz_source
    assert "rgb_optical_frame:" in rviz_source
    assert "Value: /orbslam3/input_image" in rviz_source
    assert "Fixed Frame: map" in rviz_source
