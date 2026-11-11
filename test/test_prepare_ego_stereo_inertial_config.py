"""验证 EGO 鱼眼双目-IMU 标定到 ORB_SLAM3 配置的转换行为。"""

from __future__ import annotations

import importlib.util
import os
import shutil
import subprocess
import sys
from pathlib import Path

import cv2
import numpy as np
import pytest


MODULE_PATH = (
    Path(__file__).resolve().parents[1]
    / "scripts"
    / "prepare_ego_stereo_inertial_config.py"
)


def load_module():
    """加载未安装的转换器模块，便于直接测试纯函数和命令行入口。"""
    spec = importlib.util.spec_from_file_location("prepare_ego_config", MODULE_PATH)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


prepare_ego_config = load_module()


def _matrix_text(matrix: np.ndarray) -> str:
    """把矩阵写成 OpenCV YAML 可读的列表文本。"""
    rows = []
    for row in matrix:
        rows.append("[" + ", ".join(f"{float(value):.17g}" for value in row) + "]")
    return "[\n" + ",\n".join(rows) + "\n]"


def _write_calibration(
    path: Path,
    *,
    camera_width: int = 320,
    camera_height: int = 240,
    integral_values: bool = False,
) -> dict:
    """写出包含两个不同鱼眼相机、一致外参和固定 bias 的测试标定。"""
    if integral_values:
        cam0_intrinsics = np.array([300, 301, 160, 120])
        cam1_intrinsics = np.array([310, 311, 165, 118])
        cam0_distortion = np.array([1, 0, -1, 2])
        cam1_distortion = np.array([2, 1, -2, 1])
        accelerometer_bias = "[0, 1, -2]"
        gyroscope_bias = "[1, -2, 0]"
        noise_values = {
            "accelerometer_noise_density": "1",
            "accelerometer_random_walk": "2",
            "gyroscope_noise_density": "3",
            "gyroscope_random_walk": "4",
        }
        time_offset = "0"
    else:
        cam0_intrinsics = np.array([300.0, 301.0, 160.0, 120.0])
        cam1_intrinsics = np.array([310.0, 311.0, 165.0, 118.0])
        cam0_distortion = np.array([0.01, 0.02, -0.03, 0.04])
        cam1_distortion = np.array([0.11, 0.12, -0.13, 0.14])
        accelerometer_bias = "[0.1, -0.2, 0.3]"
        gyroscope_bias = "[1.0, -2.0, 0.5]"
        noise_values = {
            "accelerometer_noise_density": "0.001",
            "accelerometer_random_walk": "0.002",
            "gyroscope_noise_density": "0.003",
            "gyroscope_random_walk": "0.004",
        }
        time_offset = "-0.005"
    T_cam1_cam0 = np.array(
        [
            [1.0, 0.0, 0.0, -100.0],
            [0.0, 1.0, 0.0, 1.0],
            [0.0, 0.0, 1.0, 2.0],
            [0.0, 0.0, 0.0, 1.0],
        ]
    )
    T_cam0_imu = np.array(
        [
            [1.0, 0.0, 0.0, 50.0],
            [0.0, 1.0, 0.0, 20.0],
            [0.0, 0.0, 1.0, -30.0],
            [0.0, 0.0, 0.0, 1.0],
        ]
    )
    T_cam1_imu = T_cam1_cam0 @ T_cam0_imu
    path.write_text(
        (
            "cam0:\n"
            f"  T_cam_imu: {_matrix_text(T_cam0_imu)}\n"
            f"  intrinsics: {cam0_intrinsics.tolist()}\n"
            "  distortion_model: fisheye\n"
            f"  distortion_coeffs: {cam0_distortion.tolist()}\n"
            f"  resolution: [{camera_width}, {camera_height}]\n"
            "cam1:\n"
            f"  T_cam_imu: {_matrix_text(T_cam1_imu)}\n"
            f"  intrinsics: {cam1_intrinsics.tolist()}\n"
            "  distortion_model: fisheye\n"
            f"  distortion_coeffs: {cam1_distortion.tolist()}\n"
            f"  resolution: [{camera_width}, {camera_height}]\n"
            f"T_cam1_cam0: {_matrix_text(T_cam1_cam0)}\n"
            "imu:\n"
            f"  accelerometer_bias: {accelerometer_bias}\n"
            f"  gyroscope_bias: {gyroscope_bias}\n"
            f"accelerometer_noise_density: {noise_values['accelerometer_noise_density']}\n"
            f"accelerometer_random_walk: {noise_values['accelerometer_random_walk']}\n"
            f"gyroscope_noise_density: {noise_values['gyroscope_noise_density']}\n"
            f"gyroscope_random_walk: {noise_values['gyroscope_random_walk']}\n"
            "update_rate: 200\n"
            f"ROS.ImuTimeOffsetSec: {time_offset}\n"
        ),
        encoding="utf-8",
    )
    return {
        "T_cam0_imu": T_cam0_imu,
        "T_cam1_cam0": T_cam1_cam0,
        "cam0_intrinsics": cam0_intrinsics,
        "cam1_intrinsics": cam1_intrinsics,
        "cam0_distortion": cam0_distortion,
        "cam1_distortion": cam1_distortion,
    }


def _minimal_jpeg(width: int, height: int) -> bytes:
    """构造只包含 SOF0 尺寸信息的 JPEG 字节串，供转换器读取拼接尺寸。"""
    sof = b"\xff\xc0\x00\x11\x08" + int(height).to_bytes(2, "big") + int(width).to_bytes(2, "big")
    return b"\xff\xd8" + sof + b"\x03\x01\x22\x00\x02\x11\x01\x03\x11\x01\xff\xd9"


def _write_bag(path: Path, image_width: int = 640, image_height: int = 240) -> None:
    """使用 rosbag2_py 写一个最小 MCAP，包含图像和 IMU header。"""
    from rclpy.serialization import serialize_message
    from rosbag2_py import ConverterOptions, SequentialWriter, StorageOptions, TopicMetadata
    from sensor_msgs.msg import CompressedImage, Imu

    if path.exists():
        if path.is_dir():
            shutil.rmtree(path)
        else:
            path.unlink()
    writer = SequentialWriter()
    writer.open(
        StorageOptions(uri=str(path), storage_id="mcap"),
        ConverterOptions("cdr", "cdr"),
    )
    writer.create_topic(
        TopicMetadata(
            id=0,
            name="/stereo/image_raw",
            type="sensor_msgs/msg/CompressedImage",
            serialization_format="cdr",
        )
    )
    writer.create_topic(
        TopicMetadata(
            id=1,
            name="/imu",
            type="sensor_msgs/msg/Imu",
            serialization_format="cdr",
        )
    )

    for index in range(4):
        stamp_ns = 1_000_000_000 + index * 100_000_000
        message = CompressedImage()
        message.header.stamp.sec = stamp_ns // 1_000_000_000
        message.header.stamp.nanosec = stamp_ns % 1_000_000_000
        message.header.frame_id = "stereo"
        message.format = "jpeg"
        message.data = _minimal_jpeg(image_width, image_height)
        writer.write("/stereo/image_raw", serialize_message(message), stamp_ns)

    for index in range(11):
        stamp_ns = 1_000_000_000 + index * 20_000_000
        message = Imu()
        message.header.stamp.sec = stamp_ns // 1_000_000_000
        message.header.stamp.nanosec = stamp_ns % 1_000_000_000
        message.linear_acceleration.x = 0.1
        message.linear_acceleration.y = -0.2
        message.linear_acceleration.z = 9.8
        writer.write("/imu", serialize_message(message), stamp_ns)

    writer.close()


def _read_node(file_storage: cv2.FileStorage, key: str):
    """读取 OpenCV YAML 节点并给出明确缺失信息。"""
    node = file_storage.getNode(key)
    assert not node.empty(), f"missing YAML key: {key}"
    return node


def _run_conversion(
    tmp_path: Path,
    *,
    overwrite: bool = False,
    image_width: int = 640,
    integral_values: bool = False,
    output_path: Path | None = None,
) -> tuple[int, Path]:
    """准备标定和 MCAP，再调用转换器命令行入口。"""
    calib_path = tmp_path / "calib.yaml"
    bag_path = tmp_path / "sample.mcap"
    if output_path is None:
        output_path = tmp_path / "generated.yaml"
    _write_calibration(calib_path, integral_values=integral_values)
    _write_bag(bag_path, image_width=image_width)
    arguments = [
        "--calib",
        str(calib_path),
        "--bag",
        str(bag_path),
        "--output",
        str(output_path),
    ]
    if overwrite:
        arguments.append("--overwrite")
    return prepare_ego_config.main(arguments), output_path


def test_invert_transform_keeps_metric_direction_and_rejects_non_rotation():
    """外参逆变换应返回真实逆矩阵，并拒绝非正交旋转。"""
    transform = np.array(
        [
            [0.0, -1.0, 0.0, 2.0],
            [1.0, 0.0, 0.0, -3.0],
            [0.0, 0.0, 1.0, 0.5],
            [0.0, 0.0, 0.0, 1.0],
        ]
    )
    inverse = prepare_ego_config.invert_transform(transform, "T")

    np.testing.assert_allclose(inverse @ transform, np.eye(4), atol=1e-12)
    np.testing.assert_allclose(inverse[:3, 3], [3.0, 2.0, -0.5], atol=1e-12)

    invalid = transform.copy()
    invalid[0, 0] = 2.0
    with pytest.raises(prepare_ego_config.ConversionError, match="正交"):
        prepare_ego_config.invert_transform(invalid, "T")


def test_read_jpeg_size_reads_sof_dimensions_without_decoding():
    """JPEG 尺寸解析应读取 SOF 宽高。"""
    assert prepare_ego_config.read_jpeg_size(_minimal_jpeg(3840, 1080)) == (3840, 1080)
    with pytest.raises(prepare_ego_config.ConversionError, match="JPEG"):
        prepare_ego_config.read_jpeg_size(b"not a jpeg")


def test_converter_writes_native_fisheye_config_from_bag_headers(tmp_path):
    """转换器应生成独立鱼眼参数、正确外参方向、原始噪声和实测采样率。"""
    return_code, output_path = _run_conversion(tmp_path)
    assert return_code == 0
    assert output_path.exists()

    file_storage = cv2.FileStorage(str(output_path), cv2.FILE_STORAGE_READ)
    assert file_storage.isOpened()

    try:
        assert _read_node(file_storage, "Camera.type").string() == "KannalaBrandt8"
        assert _read_node(file_storage, "Camera.width").real() == pytest.approx(320.0)
        assert _read_node(file_storage, "Camera.height").real() == pytest.approx(240.0)
        assert _read_node(file_storage, "Camera.fps").real() == pytest.approx(10.0)
        assert _read_node(file_storage, "Camera.RGB").real() == pytest.approx(0.0)

        assert _read_node(file_storage, "Camera1.k1").real() == pytest.approx(0.01)
        assert _read_node(file_storage, "Camera2.k1").real() == pytest.approx(0.11)
        assert _read_node(file_storage, "Camera1.k3").real() == pytest.approx(-0.03)
        assert _read_node(file_storage, "Camera2.k3").real() == pytest.approx(-0.13)
        assert _read_node(file_storage, "Camera2.fx").real() == pytest.approx(310.0)

        assert _read_node(file_storage, "IMU.NoiseGyro").real() == pytest.approx(0.003)
        assert _read_node(file_storage, "IMU.NoiseAcc").real() == pytest.approx(0.001)
        assert _read_node(file_storage, "IMU.GyroWalk").real() == pytest.approx(0.004)
        assert _read_node(file_storage, "IMU.AccWalk").real() == pytest.approx(0.002)
        assert _read_node(file_storage, "IMU.Frequency").real() == pytest.approx(50.0)
        assert _read_node(file_storage, "ROS.ImuTimeOffsetSec").real() == pytest.approx(-0.005)
        assert file_storage.getNode("IMU.DeferLowMotionReset").empty()
        assert file_storage.getNode("IMU.DeferPreBA2TrackingReset").empty()

        T_b_c1 = _read_node(file_storage, "IMU.T_b_c1").mat()
        expected_T_b_c1 = np.linalg.inv(
            np.array(
                [
                    [1.0, 0.0, 0.0, 0.050],
                    [0.0, 1.0, 0.0, 0.020],
                    [0.0, 0.0, 1.0, -0.030],
                    [0.0, 0.0, 0.0, 1.0],
                ]
            )
        )
        np.testing.assert_allclose(T_b_c1, expected_T_b_c1, atol=1e-7)

        T_c1_c2 = _read_node(file_storage, "Stereo.T_c1_c2").mat()
        expected_T_c1_c2 = np.linalg.inv(
            np.array(
                [
                    [1.0, 0.0, 0.0, -0.100],
                    [0.0, 1.0, 0.0, 0.001],
                    [0.0, 0.0, 1.0, 0.002],
                    [0.0, 0.0, 0.0, 1.0],
                ]
            )
        )
        np.testing.assert_allclose(T_c1_c2, expected_T_c1_c2, atol=1e-7)
    finally:
        file_storage.release()


def test_converter_refuses_overwrite_and_accepts_explicit_flag(tmp_path):
    """默认不覆盖已有输出，--overwrite 才允许替换。"""
    return_code, output_path = _run_conversion(tmp_path)
    assert return_code == 0
    original_text = output_path.read_text(encoding="utf-8")

    return_code, _ = _run_conversion(tmp_path)
    assert return_code == 2
    assert output_path.read_text(encoding="utf-8") == original_text

    return_code, _ = _run_conversion(tmp_path, overwrite=True)
    assert return_code == 0


def test_converter_rejects_stitched_size_mismatch(tmp_path):
    """拼接图像宽度必须恰好是标定单目宽度的两倍。"""
    return_code, output_path = _run_conversion(tmp_path, image_width=630)
    assert return_code == 2
    assert not output_path.exists()


def _settings_probe_path() -> Path:
    """定位用真实 ORB_SLAM3 Settings 加载生成配置的测试探针。"""
    candidates = []
    configured = os.environ.get("EGO_CONFIG_SETTINGS_PROBE")
    if configured:
        candidates.append(Path(configured))
    candidates.extend(
        [
            Path(__file__).resolve().parents[1]
            / "build"
            / "orbslam3"
            / "prepare_ego_config_settings_probe",
            Path.cwd() / "prepare_ego_config_settings_probe",
        ]
    )
    for candidate in candidates:
        if candidate.is_file() and os.access(candidate, os.X_OK):
            return candidate
    pytest.fail(
        "未找到 Settings 测试探针；请先构建 prepare_ego_config_settings_probe，"
        "或设置 EGO_CONFIG_SETTINGS_PROBE"
    )


def _assert_settings_loads(output_path: Path) -> None:
    """用实际 ORB_SLAM3::Settings 加载生成的 YAML，而不是只做字符串断言。"""
    result = subprocess.run(
        [str(_settings_probe_path()), str(output_path)],
        capture_output=True,
        text=True,
        check=False,
    )
    assert result.returncode == 0, f"Settings 加载失败:\n{result.stdout}\n{result.stderr}"


def test_integral_float_fields_are_real_yaml_and_load_in_core_settings(tmp_path):
    """整数标定值也必须输出 real 字面量，并能被真实 Settings 完整读取。"""
    return_code, output_path = _run_conversion(tmp_path, integral_values=True)
    assert return_code == 0

    text = output_path.read_text(encoding="utf-8")
    float_keys = [
        "Camera1.fx",
        "Camera1.fy",
        "Camera1.cx",
        "Camera1.cy",
        "Camera1.k1",
        "Camera1.k2",
        "Camera1.k3",
        "Camera1.k4",
        "Camera2.fx",
        "Camera2.fy",
        "Camera2.cx",
        "Camera2.cy",
        "Camera2.k1",
        "Camera2.k2",
        "Camera2.k3",
        "Camera2.k4",
        "Stereo.ThDepth",
        "IMU.NoiseGyro",
        "IMU.NoiseAcc",
        "IMU.GyroWalk",
        "IMU.AccWalk",
        "IMU.Frequency",
        "ROS.ImuTimeOffsetSec",
    ]
    for key in float_keys:
        line = next(line for line in text.splitlines() if line.startswith(key + ":"))
        literal = line.split(":", 1)[1].strip()
        assert "." in literal or "e" in literal.lower(), f"{key} 不是 real 字面量: {literal}"

    _assert_settings_loads(output_path)


def _snapshot_path(path: Path) -> dict[str, bytes]:
    """保存文件或目录树的完整内容，用于确认只读输入未被改写。"""
    if path.is_dir():
        return {
            str(item.relative_to(path)): item.read_bytes()
            for item in sorted(path.rglob("*"))
            if item.is_file()
        }
    return {"": path.read_bytes()}


@pytest.mark.parametrize(
    ("input_name", "alias_kind"),
    [
        ("calib", "same"),
        ("calib", "symlink"),
        ("calib", "hardlink"),
        ("bag", "same"),
        ("bag", "symlink"),
        ("bag", "member_hardlink"),
    ],
)
def test_converter_rejects_output_aliasing_read_only_input(
    tmp_path, input_name, alias_kind
):
    """输出与标定或 bag 同路径或符号链接时，即使 --overwrite 也拒绝。"""
    calib_path = tmp_path / "calib.yaml"
    bag_path = tmp_path / "sample.mcap"
    _write_calibration(calib_path)
    _write_bag(bag_path)
    source = calib_path if input_name == "calib" else bag_path
    source_snapshot = _snapshot_path(source)

    if alias_kind == "same":
        output_path = source
    elif alias_kind == "symlink":
        output_path = tmp_path / "output.yaml"
        output_path.symlink_to(source)
    elif alias_kind == "member_hardlink":
        member = next(item for item in source.rglob("*") if item.is_file())
        output_path = tmp_path / "output.yaml"
        os.link(member, output_path)
    else:
        output_path = tmp_path / "output.yaml"
        os.link(source, output_path)

    return_code = prepare_ego_config.main(
        [
            "--calib",
            str(calib_path),
            "--bag",
            str(bag_path),
            "--output",
            str(output_path),
            "--overwrite",
        ]
    )

    assert return_code == 2
    assert _snapshot_path(source) == source_snapshot
    if alias_kind == "symlink":
        assert output_path.is_symlink()
    elif alias_kind == "hardlink":
        assert os.path.samefile(output_path, source)
    elif alias_kind == "member_hardlink":
        member = next(item for item in source.rglob("*") if item.is_file())
        assert os.path.samefile(output_path, member)


def test_write_config_preserves_preexisting_fixed_temp_name(tmp_path):
    """固定 output.yaml.tmp 旧文件属于他人，不能被转换器覆盖或删除。"""
    output_path = tmp_path / "generated.yaml"
    legacy_tmp = tmp_path / "generated.yaml.tmp"
    legacy_tmp.write_text("pre-existing unrelated content\n", encoding="utf-8")

    prepare_ego_config.write_config(output_path, "new output\n", False)

    assert output_path.read_text(encoding="utf-8") == "new output\n"
    assert (
        legacy_tmp.read_text(encoding="utf-8") == "pre-existing unrelated content\n"
    )
    assert not list(tmp_path.glob(".generated.yaml.*.tmp"))


def test_write_config_treats_dangling_symlink_as_existing(tmp_path):
    """预存在断链符号链接也属于已有目录项，默认发布必须拒绝。"""
    output_path = tmp_path / "generated.yaml"
    missing_target = tmp_path / "missing.yaml"
    output_path.symlink_to(missing_target)
    assert not output_path.exists()

    with pytest.raises(prepare_ego_config.ConversionError, match="已存在"):
        prepare_ego_config.write_config(output_path, "new output\n", False)

    assert output_path.is_symlink()
    assert not missing_target.exists()
    assert not list(tmp_path.glob(".generated.yaml.*.tmp"))


def test_write_config_no_overwrite_does_not_clobber_racing_target(tmp_path, monkeypatch):
    """检查后目标被抢占时，link 的原子 EEXIST 语义必须保住抢占文件。"""
    output_path = tmp_path / "generated.yaml"
    real_link = prepare_ego_config.os.link

    def racing_link(source, destination):
        Path(destination).write_text("racing writer\n", encoding="utf-8")
        return real_link(source, destination)

    monkeypatch.setattr(prepare_ego_config.os, "link", racing_link)

    with pytest.raises(prepare_ego_config.ConversionError, match="已存在"):
        prepare_ego_config.write_config(output_path, "new output\n", False)

    assert output_path.read_text(encoding="utf-8") == "racing writer\n"
    assert not list(tmp_path.glob(".generated.yaml.*.tmp"))


def test_converter_reports_output_directory_failure(tmp_path):
    """输出目录无法创建时应清晰失败，不能误报成功。"""
    blocker = tmp_path / "not-a-directory"
    blocker.write_text("blocked\n", encoding="utf-8")
    output_path = blocker / "generated.yaml"

    return_code, _ = _run_conversion(tmp_path, output_path=output_path)

    assert return_code == 2
    assert blocker.is_file()
    assert blocker.read_text(encoding="utf-8") == "blocked\n"
