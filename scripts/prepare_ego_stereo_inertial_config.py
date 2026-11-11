#!/usr/bin/env python3
"""从 EGO 鱼眼双目标定和 MCAP 生成可审查的 ORB_SLAM3 惯性配置。"""

from __future__ import annotations

import argparse
import json
import math
import os
import sys
import tempfile
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Iterable

import numpy as np
import yaml

DEFAULT_IMAGE_TOPIC = "/stereo/image_raw"
DEFAULT_IMU_TOPIC = "/imu"
IMAGE_MESSAGE_TYPE = "sensor_msgs/msg/CompressedImage"
IMU_MESSAGE_TYPE = "sensor_msgs/msg/Imu"


class ConversionError(ValueError):
    """标定或 bag 不满足转换前置条件时抛出。"""


@dataclass(frozen=True)
class BagStatistics:
    """保存 MCAP 中与配置生成相关的 header 统计。"""

    image_count: int
    imu_count: int
    image_rate_hz: float
    imu_rate_hz: float
    image_duration_sec: float
    imu_duration_sec: float
    image_width: int
    image_height: int
    camera_rgb: int
    first_image_stamp_ns: int
    first_imu_stamp_ns: int
    last_image_stamp_ns: int
    last_imu_stamp_ns: int


@dataclass(frozen=True)
class Calibration:
    """保存生成 ORB_SLAM3 配置所需的标定字段。"""

    camera1_intrinsics: np.ndarray
    camera1_distortion: np.ndarray
    camera2_intrinsics: np.ndarray
    camera2_distortion: np.ndarray
    width: int
    height: int
    T_b_c1: np.ndarray
    T_c1_c2: np.ndarray
    accelerometer_bias: np.ndarray
    gyroscope_bias_rad_s: np.ndarray
    noise_gyro: float
    noise_acc: float
    gyro_walk: float
    acc_walk: float
    imu_time_offset_sec: float


def _require_mapping(value: Any, name: str) -> dict[str, Any]:
    """确认 YAML 节点为映射，并返回该映射。"""
    if not isinstance(value, dict):
        raise ConversionError(f"{name} 必须是 YAML 映射")
    return value


def _numeric_vector(value: Any, length: int, name: str) -> np.ndarray:
    """读取定长有限数值向量。"""
    if not isinstance(value, (list, tuple)) or len(value) != length:
        raise ConversionError(f"{name} 需要 {length} 个数值")
    try:
        vector = np.asarray(value, dtype=float)
    except (TypeError, ValueError) as exc:
        raise ConversionError(f"{name} 包含无法解析的数值") from exc
    if vector.shape != (length,) or not np.all(np.isfinite(vector)):
        raise ConversionError(f"{name} 必须全部为有限数值")
    return vector


def _numeric_matrix(value: Any, shape: tuple[int, int], name: str) -> np.ndarray:
    """读取指定形状的有限数值矩阵。"""
    try:
        matrix = np.asarray(value, dtype=float)
    except (TypeError, ValueError) as exc:
        raise ConversionError(f"{name} 包含无法解析的数值") from exc
    if matrix.shape != shape or not np.all(np.isfinite(matrix)):
        raise ConversionError(f"{name} 需要形状 {shape} 的有限数值矩阵")
    return matrix


def invert_transform(transform: np.ndarray, name: str) -> np.ndarray:
    """校验齐次变换并返回其逆，平移单位沿用输入。"""
    if transform.shape != (4, 4) or not np.all(np.isfinite(transform)):
        raise ConversionError(f"{name} 需要 4x4 有限数值矩阵")
    if not np.allclose(transform[3, :], [0.0, 0.0, 0.0, 1.0], atol=1e-6):
        raise ConversionError(f"{name} 最后一行必须为 [0, 0, 0, 1]")
    rotation = transform[:3, :3]
    if not np.allclose(rotation.T @ rotation, np.eye(3), atol=1e-5):
        raise ConversionError(f"{name} 旋转部分不是正交矩阵")
    if np.linalg.det(rotation) < 0.0:
        raise ConversionError(f"{name} 旋转部分行列式必须为正")
    try:
        inverse = np.linalg.inv(transform)
    except np.linalg.LinAlgError as exc:
        raise ConversionError(f"{name} 不可逆") from exc
    if not np.all(np.isfinite(inverse)):
        raise ConversionError(f"{name} 求逆后包含非有限数值")
    return inverse


def read_calibration(path: Path | str) -> Calibration:
    """读取 Kalibr 风格的 EGO 双目-IMU 标定文件。"""
    calibration_path = Path(path)
    try:
        document = yaml.safe_load(calibration_path.read_text(encoding="utf-8"))
    except FileNotFoundError as exc:
        raise ConversionError(f"标定文件不存在: {calibration_path}") from exc
    except (OSError, UnicodeError) as exc:
        raise ConversionError(f"无法读取标定文件 {calibration_path}: {exc}") from exc
    except yaml.YAMLError as exc:
        raise ConversionError(f"标定文件 YAML 解析失败: {calibration_path}") from exc

    root = _require_mapping(document, "标定根节点")
    cam0 = _require_mapping(root.get("cam0"), "cam0")
    cam1 = _require_mapping(root.get("cam1"), "cam1")
    imu = _require_mapping(root.get("imu"), "imu")

    camera1_intrinsics = _numeric_vector(cam0.get("intrinsics"), 4, "cam0.intrinsics")
    camera2_intrinsics = _numeric_vector(cam1.get("intrinsics"), 4, "cam1.intrinsics")
    camera1_distortion = _numeric_vector(
        cam0.get("distortion_coeffs"), 4, "cam0.distortion_coeffs"
    )
    camera2_distortion = _numeric_vector(
        cam1.get("distortion_coeffs"), 4, "cam1.distortion_coeffs"
    )
    if cam0.get("distortion_model") != "fisheye" or cam1.get("distortion_model") != "fisheye":
        raise ConversionError("当前转换器只支持 cam0/cam1 的 fisheye 畸变模型")

    resolution0 = _numeric_vector(cam0.get("resolution"), 2, "cam0.resolution")
    resolution1 = _numeric_vector(cam1.get("resolution"), 2, "cam1.resolution")
    if not np.array_equal(resolution0, resolution1):
        raise ConversionError("cam0 与 cam1 分辨率必须一致")
    width = int(round(resolution0[0]))
    height = int(round(resolution0[1]))
    if width <= 0 or height <= 0 or not np.allclose(resolution0, [width, height]):
        raise ConversionError("相机分辨率必须为正整数")

    T_cam0_imu = _numeric_matrix(cam0.get("T_cam_imu"), (4, 4), "cam0.T_cam_imu")
    T_cam1_imu = _numeric_matrix(cam1.get("T_cam_imu"), (4, 4), "cam1.T_cam_imu")
    T_cam1_cam0 = _numeric_matrix(root.get("T_cam1_cam0"), (4, 4), "T_cam1_cam0")

    T_b_c1 = invert_transform(T_cam0_imu, "cam0.T_cam_imu")
    T_c1_c2 = invert_transform(T_cam1_cam0, "T_cam1_cam0")
    T_b_c1[:3, 3] *= 0.001
    T_c1_c2[:3, 3] *= 0.001

    # 交叉校验两个相机各自的 IMU 外参是否与 T_cam1_cam0 相容。
    # p_cam1 = T_cam1_cam0 * p_cam0 = T_cam1_imu * T_cam0_imu^-1 * p_cam0
    derived_T_cam1_cam0 = T_cam1_imu @ np.linalg.inv(T_cam0_imu)
    if not np.allclose(derived_T_cam1_cam0, T_cam1_cam0, atol=2e-4):
        raise ConversionError("T_cam1_cam0 与两个 T_cam_imu 不一致")

    accelerometer_bias = _numeric_vector(
        imu.get("accelerometer_bias"), 3, "imu.accelerometer_bias"
    )
    gyroscope_bias_deg_s = _numeric_vector(
        imu.get("gyroscope_bias"), 3, "imu.gyroscope_bias"
    )

    def _positive_float(key: str) -> float:
        value = imu.get(key, root.get(key))
        try:
            result = float(value)
        except (TypeError, ValueError) as exc:
            raise ConversionError(f"{key} 需要正数值") from exc
        if not math.isfinite(result) or result <= 0.0:
            raise ConversionError(f"{key} 需要正数值")
        return result

    try:
        imu_time_offset_sec = float(
            root.get("ROS.ImuTimeOffsetSec", imu.get("ROS.ImuTimeOffsetSec", 0.0))
        )
    except (TypeError, ValueError) as exc:
        raise ConversionError("ROS.ImuTimeOffsetSec 需要有限数值") from exc
    if not math.isfinite(imu_time_offset_sec):
        raise ConversionError("ROS.ImuTimeOffsetSec 需要有限数值")

    return Calibration(
        camera1_intrinsics=camera1_intrinsics,
        camera1_distortion=camera1_distortion,
        camera2_intrinsics=camera2_intrinsics,
        camera2_distortion=camera2_distortion,
        width=width,
        height=height,
        T_b_c1=T_b_c1,
        T_c1_c2=T_c1_c2,
        accelerometer_bias=accelerometer_bias,
        gyroscope_bias_rad_s=np.deg2rad(gyroscope_bias_deg_s),
        noise_gyro=_positive_float("gyroscope_noise_density"),
        noise_acc=_positive_float("accelerometer_noise_density"),
        gyro_walk=_positive_float("gyroscope_random_walk"),
        acc_walk=_positive_float("accelerometer_random_walk"),
        imu_time_offset_sec=imu_time_offset_sec,
    )


def read_jpeg_size(data: bytes) -> tuple[int, int]:
    """从 JPEG 字节流读取 (width, height)，不执行图像解码。"""
    if len(data) < 4 or data[0:2] != b"\xff\xd8":
        raise ConversionError("压缩图像不是 JPEG 数据")
    index = 2
    sof_markers = {
        0xC0,
        0xC1,
        0xC2,
        0xC3,
        0xC5,
        0xC6,
        0xC7,
        0xC9,
        0xCA,
        0xCB,
        0xCD,
        0xCE,
        0xCF,
    }
    while index + 3 < len(data):
        if data[index] != 0xFF:
            index += 1
            continue
        while index + 1 < len(data) and data[index] == 0xFF:
            index += 1
        if index + 1 >= len(data):
            break
        marker = data[index]
        index += 1
        if marker in (0xD8, 0xD9) or 0xD0 <= marker <= 0xD7 or marker == 0x01:
            continue
        if index + 2 > len(data):
            break
        segment_length = int.from_bytes(data[index : index + 2], "big")
        if segment_length < 2 or index + segment_length > len(data):
            break
        if marker in sof_markers:
            if segment_length < 7:
                break
            height = int.from_bytes(data[index + 3 : index + 5], "big")
            width = int.from_bytes(data[index + 5 : index + 7], "big")
            if width <= 0 or height <= 0:
                break
            return width, height
        index += segment_length
    raise ConversionError("JPEG 中找不到有效的 SOF 尺寸")


def _stamp_ns(stamp: Any) -> int:
    """把 builtin_interfaces/Time 转换为 int64 纳秒。"""
    seconds = int(stamp.sec)
    nanoseconds = int(stamp.nanosec)
    if nanoseconds < 0 or nanoseconds >= 1_000_000_000:
        raise ConversionError("header.stamp.nanosec 超出 [0, 1e9) 范围")
    return seconds * 1_000_000_000 + nanoseconds


def _rate_from_stamps(stamps: list[int], name: str) -> tuple[float, float]:
    """根据严格递增的 header 时间戳计算平均采样率和时长。"""
    if len(stamps) < 2:
        raise ConversionError(f"{name} 至少需要 2 条消息才能统计采样率")
    for previous, current in zip(stamps, stamps[1:]):
        if current <= previous:
            raise ConversionError(f"{name} header 时间戳不是严格递增")
    duration_ns = stamps[-1] - stamps[0]
    if duration_ns <= 0:
        raise ConversionError(f"{name} header 时间跨度必须为正")
    return (len(stamps) - 1) * 1_000_000_000.0 / duration_ns, duration_ns / 1e9


def _iter_bag_messages(
    bag_path: Path, image_topic: str, imu_topic: str
) -> Iterable[tuple[str, Any]]:
    """按 topic 读取并反序列化图像/IMU 消息，跳过其他自定义消息。"""
    try:
        from rclpy.serialization import deserialize_message
        from rosbag2_py import ConverterOptions, SequentialReader, StorageOptions
        from sensor_msgs.msg import CompressedImage, Imu
    except ImportError as exc:
        raise ConversionError("需要 ROS 2 rosbag2_py、rclpy 和 sensor_msgs") from exc

    try:
        reader = SequentialReader()
        reader.open(
            StorageOptions(uri=str(bag_path), storage_id="mcap"),
            ConverterOptions("cdr", "cdr"),
        )
    except Exception as exc:  # rosbag2_py 的异常类型不稳定
        raise ConversionError(f"无法打开 MCAP: {bag_path}: {exc}") from exc

    topics = {topic.name: topic.type for topic in reader.get_all_topics_and_types()}
    if topics.get(image_topic) != IMAGE_MESSAGE_TYPE:
        raise ConversionError(
            f"{bag_path} 的 {image_topic} 必须是 {IMAGE_MESSAGE_TYPE}，实际为 {topics.get(image_topic)}"
        )
    if topics.get(imu_topic) != IMU_MESSAGE_TYPE:
        raise ConversionError(
            f"{bag_path} 的 {imu_topic} 必须是 {IMU_MESSAGE_TYPE}，实际为 {topics.get(imu_topic)}"
        )

    while reader.has_next():
        topic, serialized, _storage_time_ns = reader.read_next()
        if topic == image_topic:
            yield topic, deserialize_message(serialized, CompressedImage)
        elif topic == imu_topic:
            yield topic, deserialize_message(serialized, Imu)


def inspect_bag(
    bag_path: Path | str,
    calibration: Calibration,
    image_topic: str = DEFAULT_IMAGE_TOPIC,
    imu_topic: str = DEFAULT_IMU_TOPIC,
) -> BagStatistics:
    """只读扫描 MCAP，统计 header 采样率并校验拼接图像尺寸。"""
    image_stamps: list[int] = []
    imu_stamps: list[int] = []
    first_image_size: tuple[int, int] | None = None
    camera_rgb = 0

    for topic, message in _iter_bag_messages(Path(bag_path), image_topic, imu_topic):
        stamp_ns = _stamp_ns(message.header.stamp)
        if topic == image_topic:
            image_stamps.append(stamp_ns)
            if first_image_size is None:
                first_image_size = read_jpeg_size(bytes(message.data))
                image_format = str(getattr(message, "format", "")).lower()
                camera_rgb = int("rgb8" in image_format and "bgr8" not in image_format)
        else:
            imu_stamps.append(stamp_ns)

    if first_image_size is None:
        raise ConversionError(f"{bag_path} 中没有 {image_topic} 图像")
    if not imu_stamps:
        raise ConversionError(f"{bag_path} 中没有 {imu_topic} IMU")

    image_width, image_height = first_image_size
    expected_width = calibration.width * 2
    if image_width != expected_width or image_height != calibration.height:
        raise ConversionError(
            "拼接图像尺寸与标定不一致: "
            f"实际 {image_width}x{image_height}，期望 {expected_width}x{calibration.height}"
        )

    image_rate, image_duration = _rate_from_stamps(image_stamps, "图像")
    imu_rate, imu_duration = _rate_from_stamps(imu_stamps, "IMU")
    return BagStatistics(
        image_count=len(image_stamps),
        imu_count=len(imu_stamps),
        image_rate_hz=image_rate,
        imu_rate_hz=imu_rate,
        image_duration_sec=image_duration,
        imu_duration_sec=imu_duration,
        image_width=image_width,
        image_height=image_height,
        camera_rgb=camera_rgb,
        first_image_stamp_ns=image_stamps[0],
        first_imu_stamp_ns=imu_stamps[0],
        last_image_stamp_ns=image_stamps[-1],
        last_imu_stamp_ns=imu_stamps[-1],
    )


def _format_float(value: float) -> str:
    """输出始终带小数点的最短往返浮点字面量，保证 YAML 解析为 real。"""
    text = repr(float(value))
    if "e" in text or "E" in text:
        mantissa, exponent = text.lower().split("e", 1)
        if "." not in mantissa:
            mantissa += ".0"
        return f"{mantissa}e{exponent}"
    if "." not in text:
        text += ".0"
    return text


def _matrix_block(name: str, matrix: np.ndarray) -> list[str]:
    """把矩阵格式化为 ORB_SLAM3/OpenCV YAML 的 !!opencv-matrix 节点。"""
    rows, cols = matrix.shape
    values = [_format_float(value) for value in matrix.reshape(-1)]
    lines = [f"{name}: !!opencv-matrix", f"   rows: {rows}", f"   cols: {cols}", "   dt: f"]
    for row in range(rows):
        row_values = ", ".join(values[row * cols : (row + 1) * cols])
        if row == 0:
            lines.append(f"   data: [{row_values},")
        elif row + 1 < rows:
            lines.append(f"          {row_values},")
        else:
            lines.append(f"          {row_values}]")
    return lines


def build_orb_config(
    calibration: Calibration,
    statistics: BagStatistics,
    calibration_name: str,
    bag_name: str,
) -> str:
    """根据标定和 header 统计生成 ORB_SLAM3 KannalaBrandt8 配置文本。"""
    overlap_end = calibration.width - 1
    lines: list[str] = [
        "%YAML:1.0",
        "",
        "# Generated by prepare-ego-stereo-inertial-config.",
        f"# Calibration: {calibration_name}",
        f"# Bag: {bag_name}",
        "# Input images are the original stitched side-by-side EGO frames; no rectification,",
        "# downsampling or frame dropping is configured here.",
        "",
        'File.version: "1.0"',
        "",
        "# Native fisheye Kannala-Brandt model; BOOL_RECTIFY=false.",
        'Camera.type: "KannalaBrandt8"',
        "",
        "# Left camera (cam0).",
        f"Camera1.fx: {_format_float(calibration.camera1_intrinsics[0])}",
        f"Camera1.fy: {_format_float(calibration.camera1_intrinsics[1])}",
        f"Camera1.cx: {_format_float(calibration.camera1_intrinsics[2])}",
        f"Camera1.cy: {_format_float(calibration.camera1_intrinsics[3])}",
        f"Camera1.k1: {_format_float(calibration.camera1_distortion[0])}",
        f"Camera1.k2: {_format_float(calibration.camera1_distortion[1])}",
        f"Camera1.k3: {_format_float(calibration.camera1_distortion[2])}",
        f"Camera1.k4: {_format_float(calibration.camera1_distortion[3])}",
        "",
        "# Right camera (cam1), using its own independent fisheye calibration.",
        f"Camera2.fx: {_format_float(calibration.camera2_intrinsics[0])}",
        f"Camera2.fy: {_format_float(calibration.camera2_intrinsics[1])}",
        f"Camera2.cx: {_format_float(calibration.camera2_intrinsics[2])}",
        f"Camera2.cy: {_format_float(calibration.camera2_intrinsics[3])}",
        f"Camera2.k1: {_format_float(calibration.camera2_distortion[0])}",
        f"Camera2.k2: {_format_float(calibration.camera2_distortion[1])}",
        f"Camera2.k3: {_format_float(calibration.camera2_distortion[2])}",
        f"Camera2.k4: {_format_float(calibration.camera2_distortion[3])}",
        "",
        f"Camera.width: {calibration.width}",
        f"Camera.height: {calibration.height}",
        f"# Measured from {statistics.image_count} image headers over {statistics.image_duration_sec:.9f} s.",
        f"Camera.fps: {max(1, int(round(statistics.image_rate_hz)))}",
        f"Camera.RGB: {statistics.camera_rgb}",
        "",
        "# Full-width stereo lapping areas: each half is an independent fisheye image.",
        f"Camera1.overlappingBegin: 0",
        f"Camera1.overlappingEnd: {overlap_end}",
        f"Camera2.overlappingBegin: 0",
        f"Camera2.overlappingEnd: {overlap_end}",
        "",
        "# ThDepth is in baseline units; this is the ORB_SLAM3 close/far threshold.",
        "Stereo.ThDepth: 40.0",
        "",
        "# T_c1_c2 is the inverse of calib.yaml T_cam1_cam0 (left->right), translation in metres.",
        *_matrix_block("Stereo.T_c1_c2", calibration.T_c1_c2),
        "",
        "# T_b_c1 is the inverse of cam0 T_cam_imu (IMU->camera), translation in metres.",
        "# This key is the left-camera to body/IMU transform expected by ORB_SLAM3.",
        *_matrix_block("IMU.T_b_c1", calibration.T_b_c1),
        "",
        "IMU.InsertKFsWhenLost: 0",
        "# Deferred reset switches are intentionally left at the upstream default (off).",
        "",
        "# Noise values are copied from calib.yaml without applying another scale factor.",
        f"IMU.NoiseGyro: {_format_float(calibration.noise_gyro)}",
        f"IMU.NoiseAcc: {_format_float(calibration.noise_acc)}",
        f"IMU.GyroWalk: {_format_float(calibration.gyro_walk)}",
        f"IMU.AccWalk: {_format_float(calibration.acc_walk)}",
        f"# Measured from {statistics.imu_count} IMU headers over {statistics.imu_duration_sec:.9f} s.",
        f"IMU.Frequency: {_format_float(statistics.imu_rate_hz)}",
        "",
        "# Fixed biases for the offline reader to subtract before passing IMU samples",
        "# to ORB_SLAM3. Accel is m/s^2; gyro is rad/s (converted from deg/s).",
        "IMU.AccBias: [" + ", ".join(_format_float(v) for v in calibration.accelerometer_bias) + "]",
        "IMU.GyroBias: [" + ", ".join(_format_float(v) for v in calibration.gyroscope_bias_rad_s) + "]",
        "",
        "# corrected_imu_t = raw_imu_t + ROS.ImuTimeOffsetSec",
        f"ROS.ImuTimeOffsetSec: {_format_float(calibration.imu_time_offset_sec)}",
        "",
        "ORBextractor.nFeatures: 1200",
        "ORBextractor.scaleFactor: 1.2",
        "ORBextractor.nLevels: 8",
        "ORBextractor.iniThFAST: 20",
        "ORBextractor.minThFAST: 7",
        "",
        "Viewer.KeyFrameSize: 0.05",
        "Viewer.KeyFrameLineWidth: 1.0",
        "Viewer.GraphLineWidth: 0.9",
        "Viewer.PointSize: 2.0",
        "Viewer.CameraSize: 0.08",
        "Viewer.CameraLineWidth: 3.0",
        "Viewer.ViewpointX: 0.0",
        "Viewer.ViewpointY: -0.7",
        "Viewer.ViewpointZ: -3.5",
        "Viewer.ViewpointF: 500.0",
        "",
    ]
    return "\n".join(lines)


def _normalized_path(path: Path) -> Path:
    """解析已有父目录和最终符号链接，不要求目标存在。"""
    return Path(os.path.realpath(os.fspath(path)))


def ensure_output_not_input(
    output_path: Path | str, input_paths: Iterable[Path | str]
) -> None:
    """拒绝输出与任一只读输入同路径、符号链接或硬链接，覆盖开关不绕过。"""
    output = Path(output_path)
    output_normalized = _normalized_path(output)
    for input_path in input_paths:
        input_file = Path(input_path)
        input_normalized = _normalized_path(input_file)
        try:
            same_inode = os.path.samefile(output, input_file)
        except OSError:
            same_inode = False
        inside_directory_bag = input_normalized in output_normalized.parents
        if (
            same_inode
            or output_normalized == input_normalized
            or inside_directory_bag
        ):
            raise ConversionError(
                "输出路径不能与只读输入相同或是其符号链接/硬链接: "
                f"{output} -> {input_file}"
            )

        # rosbag2 也允许目录形式输入；此时还要保护目录内的实际 MCAP 文件。
        if not input_file.is_dir() or not os.path.lexists(output):
            continue
        for child in input_file.rglob("*"):
            if not child.is_file():
                continue
            try:
                same_child_inode = os.path.samefile(output, child)
            except OSError:
                same_child_inode = False
            if same_child_inode:
                raise ConversionError(
                    "输出路径不能是指定 bag 目录内文件: " f"{output} -> {child}"
                )


def write_config(
    output_path: Path | str,
    config_text: str,
    overwrite: bool,
    protected_paths: Iterable[Path | str] = (),
) -> None:
    """用同目录唯一临时文件发布配置，默认不覆盖任何已有目录项。"""
    output = Path(output_path)
    ensure_output_not_input(output, protected_paths)
    if not overwrite and os.path.lexists(output):
        raise ConversionError(f"输出文件已存在，使用 --overwrite 才可覆盖: {output}")

    try:
        output.parent.mkdir(parents=True, exist_ok=True)
    except OSError as exc:
        raise ConversionError(f"无法创建输出目录 {output.parent}: {exc}") from exc

    temporary_path: Path | None = None
    descriptor = -1
    try:
        descriptor, temporary_name = tempfile.mkstemp(
            prefix=f".{output.name}.", suffix=".tmp", dir=str(output.parent)
        )
        temporary_path = Path(temporary_name)
        with os.fdopen(descriptor, "w", encoding="utf-8") as handle:
            descriptor = -1
            handle.write(config_text)
            handle.flush()
            os.fsync(handle.fileno())

        if overwrite:
            os.replace(temporary_path, output)
            temporary_path = None
        else:
            try:
                # link() 的 EEXIST 是原子发布语义，避免检查后目标被抢占时覆盖。
                os.link(temporary_path, output)
            except FileExistsError as exc:
                raise ConversionError(
                    f"输出文件已存在，使用 --overwrite 才可覆盖: {output}"
                ) from exc
            except OSError as exc:
                raise ConversionError(f"无法发布输出 {output}: {exc}") from exc
            temporary_path.unlink()
            temporary_path = None
    except ConversionError:
        raise
    except OSError as exc:
        raise ConversionError(f"写出输出失败 {output}: {exc}") from exc
    finally:
        if descriptor >= 0:
            os.close(descriptor)
        if temporary_path is not None:
            try:
                temporary_path.unlink()
            except FileNotFoundError:
                pass
            except OSError:
                pass


def build_parser() -> argparse.ArgumentParser:
    """构造命令行解析器。"""
    parser = argparse.ArgumentParser(
        description="从 EGO calib.yaml 与 MCAP 生成 ORB_SLAM3 stereo-inertial KannalaBrandt8 配置。"
    )
    parser.add_argument("--calib", type=Path, required=True, help="输入 calib.yaml（只读）")
    parser.add_argument("--bag", type=Path, required=True, help="输入 MCAP 文件（只读）")
    parser.add_argument("--output", "-o", type=Path, required=True, help="输出 ORB YAML 配置")
    parser.add_argument("--image-topic", default=DEFAULT_IMAGE_TOPIC, help="拼接 CompressedImage topic")
    parser.add_argument("--imu-topic", default=DEFAULT_IMU_TOPIC, help="sensor_msgs/msg/Imu topic")
    parser.add_argument("--overwrite", action="store_true", help="显式允许覆盖输出文件")
    return parser


def main(argv: list[str] | None = None) -> int:
    """命令行入口；成功时返回 0，输入或写出失败时返回 2。"""
    args = build_parser().parse_args(argv)
    try:
        protected_paths = (args.calib, args.bag)
        ensure_output_not_input(args.output, protected_paths)
        calibration = read_calibration(args.calib)
        statistics = inspect_bag(args.bag, calibration, args.image_topic, args.imu_topic)
        config_text = build_orb_config(
            calibration, statistics, args.calib.name, args.bag.name
        )
        write_config(args.output, config_text, args.overwrite, protected_paths)
    except ConversionError as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 2
    except OSError as exc:
        print(f"ERROR: 输入输出失败: {exc}", file=sys.stderr)
        return 2

    summary = {
        "output": str(args.output),
        "calib": str(args.calib),
        "bag": str(args.bag),
        "image_count": statistics.image_count,
        "imu_count": statistics.imu_count,
        "image_rate_hz": statistics.image_rate_hz,
        "imu_rate_hz": statistics.imu_rate_hz,
        "image_duration_sec": statistics.image_duration_sec,
        "imu_duration_sec": statistics.imu_duration_sec,
        "camera_width": calibration.width,
        "camera_height": calibration.height,
        "camera_fps": max(1, int(round(statistics.image_rate_hz))),
        "imu_time_offset_sec": calibration.imu_time_offset_sec,
        "baseline_m": float(np.linalg.norm(calibration.T_c1_c2[:3, 3])),
    }
    print(json.dumps(summary, ensure_ascii=False, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    sys.exit(main())
