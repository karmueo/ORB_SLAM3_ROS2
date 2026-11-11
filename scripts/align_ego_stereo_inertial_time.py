#!/usr/bin/env python3
"""用 EGO 双目视觉旋转与去 bias IMU 陀螺积分独立核验 IMU 时间偏移。

本工具不读取 ORB-SLAM 轨迹，也不使用待验证系统的时间偏移输出。视觉旋转由
原始 bag 的左右目鱼眼图像独立估计：左目 ORB 特征与右目匹配后三角化，再把
左目三维点通过 PnP 投影到另一时刻左目，得到相对相机旋转。IMU 角速度先扣除
calib.yaml 的固定陀螺 bias，再在候选偏移下积分。最后在 ±50 ms 内做粗细搜索，
并在未参与选择的留出窗口和不同时间跨度上复核。
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import site
import sys
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Iterable, Sequence

# 本任务要求系统 OpenCV 4.6。用户 site-packages 中可能有 OpenCV 4.13，
# 默认会遮蔽系统 cv2；在导入第三方模块前移除 user-site 路径，使时间工具
# 与 C++ 构建使用的 /usr/lib OpenCV 4.6 保持一致，不依赖调用者记着加 -s。
_user_sites = site.getusersitepackages()
if isinstance(_user_sites, str):
    _user_sites = [_user_sites]
_user_sites = tuple(item for item in _user_sites if item)
if _user_sites:
    sys.path[:] = [
        item for item in sys.path
        if not any(item == user or item.startswith(user + "/") for user in _user_sites)
    ]

import cv2
import numpy as np

# 复用已经过回归测试的标定解析和原子输出实现。源码目录里模块名是
# prepare_ego_stereo_inertial_config.py；ROS 安装入口可能只保留
# prepare-ego-stereo-inertial-config（无 .py/带连字符），因此需要按同目录
# 文件显式加载，不能假设安装名一定可 import。
try:
    from prepare_ego_stereo_inertial_config import (  # noqa: E402
        ConversionError,
        read_calibration,
        write_config,
    )
except ImportError:
    import importlib.util
    from importlib.machinery import SourceFileLoader

    _script_dir = Path(__file__).resolve().parent
    _converter_candidates = (
        _script_dir / "prepare_ego_stereo_inertial_config.py",
        _script_dir / "prepare-ego-stereo-inertial-config",
    )
    _converter_path = next(
        (candidate for candidate in _converter_candidates if candidate.is_file()),
        None,
    )
    if _converter_path is None:
        raise
    _loader = SourceFileLoader(
        "_ego_prepare_stereo_inertial_config", str(_converter_path)
    )
    _spec = importlib.util.spec_from_loader(_loader.name, _loader)
    if _spec is None:
        raise
    _converter_module = importlib.util.module_from_spec(_spec)
    sys.modules[_loader.name] = _converter_module
    _loader.exec_module(_converter_module)
    ConversionError = _converter_module.ConversionError
    read_calibration = _converter_module.read_calibration
    write_config = _converter_module.write_config

IMAGE_TOPIC = "/stereo/image_raw"
IMU_TOPIC = "/imu"
IMAGE_MESSAGE_TYPE = "sensor_msgs/msg/CompressedImage"
IMU_MESSAGE_TYPE = "sensor_msgs/msg/Imu"
TOOL_VERSION = "1.0"


@dataclass(frozen=True)
class BagMeta:
    """与时间对齐相关的 bag header 元数据。"""

    image_stamps_ns: np.ndarray
    imu_stamps_ns: np.ndarray
    gyro_rad_s: np.ndarray
    image_count: int
    imu_count: int
    image_rate_hz: float
    imu_rate_hz: float


@dataclass(frozen=True)
class VisualPair:
    """一组视觉相对旋转及其可审查的筛选统计。"""

    stamp_i_ns: int
    stamp_j_ns: int
    span_ms: int
    rotation_cj_ci: np.ndarray
    inliers: int
    inlier_ratio: float
    motion_rad_s: float
    stereo_points: int


@dataclass(frozen=True)
class FrameFeatures:
    """单帧左右目特征、描述子和三角化结果（按左目关键点索引存放）。"""

    keypoints: tuple[Any, ...]
    descriptors: np.ndarray
    points_cam0: np.ndarray
    valid_points: np.ndarray
    stereo_matches: int


def _stamp_ns(stamp: Any) -> int:
    """把 builtin_interfaces/Time 转为 int64 纳秒。"""
    seconds = int(stamp.sec)
    nanoseconds = int(stamp.nanosec)
    if nanoseconds < 0 or nanoseconds >= 1_000_000_000:
        raise ConversionError("header.stamp.nanosec 超出 [0, 1e9) 范围")
    return seconds * 1_000_000_000 + nanoseconds


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _open_reader(bag_path: Path):
    from rosbag2_py import ConverterOptions, SequentialReader, StorageOptions

    reader = SequentialReader()
    reader.open(
        StorageOptions(uri=str(bag_path), storage_id="mcap"),
        ConverterOptions("cdr", "cdr"),
    )
    return reader


def read_bag_meta(bag_path: Path, image_topic: str, imu_topic: str) -> BagMeta:
    """第一遍只反序列化 header/IMU，校验 topic 与时间顺序。"""
    from rclpy.serialization import deserialize_message
    from sensor_msgs.msg import CompressedImage, Imu

    image_stamps: list[int] = []
    imu_stamps: list[int] = []
    gyro: list[list[float]] = []
    reader = _open_reader(bag_path)
    topics = {topic.name: topic.type for topic in reader.get_all_topics_and_types()}
    if topics.get(image_topic) != IMAGE_MESSAGE_TYPE:
        raise ConversionError(
            f"{image_topic} 必须是 {IMAGE_MESSAGE_TYPE}，实际为 {topics.get(image_topic)}"
        )
    if topics.get(imu_topic) != IMU_MESSAGE_TYPE:
        raise ConversionError(
            f"{imu_topic} 必须是 {IMU_MESSAGE_TYPE}，实际为 {topics.get(imu_topic)}"
        )

    while reader.has_next():
        topic, serialized, _storage_ns = reader.read_next()
        if topic == image_topic:
            message = deserialize_message(serialized, CompressedImage)
            image_stamps.append(_stamp_ns(message.header.stamp))
        elif topic == imu_topic:
            message = deserialize_message(serialized, Imu)
            values = np.asarray(
                [
                    message.angular_velocity.x,
                    message.angular_velocity.y,
                    message.angular_velocity.z,
                ],
                dtype=np.float64,
            )
            if not np.all(np.isfinite(values)):
                raise ConversionError("IMU 角速度包含非有限值")
            imu_stamps.append(_stamp_ns(message.header.stamp))
            gyro.append(values.tolist())

    if len(image_stamps) < 2:
        raise ConversionError("图像少于 2 帧，无法估计视觉旋转")
    if len(imu_stamps) < 3:
        raise ConversionError("IMU 少于 3 条，无法积分")
    for name, stamps in (("图像", image_stamps), ("IMU", imu_stamps)):
        previous = stamps[0]
        for stamp in stamps[1:]:
            if stamp <= previous:
                raise ConversionError(f"{name} header 时间戳不是严格递增")
            previous = stamp

    image_array = np.asarray(image_stamps, dtype=np.int64)
    imu_array = np.asarray(imu_stamps, dtype=np.int64)
    image_rate = (len(image_stamps) - 1) * 1e9 / float(image_array[-1] - image_array[0])
    imu_rate = (len(imu_stamps) - 1) * 1e9 / float(imu_array[-1] - imu_array[0])
    return BagMeta(
        image_stamps_ns=image_array,
        imu_stamps_ns=imu_array,
        gyro_rad_s=np.asarray(gyro, dtype=np.float64),
        image_count=len(image_stamps),
        imu_count=len(imu_stamps),
        image_rate_hz=image_rate,
        imu_rate_hz=imu_rate,
    )


def _intrinsics(values: np.ndarray) -> np.ndarray:
    return np.asarray(
        [
            [values[0], 0.0, values[2]],
            [0.0, values[1], values[3]],
            [0.0, 0.0, 1.0],
        ],
        dtype=np.float64,
    )


def _undistort(points: Any, matrix: np.ndarray, distortion: np.ndarray) -> np.ndarray:
    """把鱼眼像素点转换为归一化 bearing（x/z, y/z）。"""
    pixels = np.asarray(points, dtype=np.float64).reshape(-1, 1, 2)
    return cv2.fisheye.undistortPoints(pixels, matrix, distortion).reshape(-1, 2)


def _rotation_angle(rotation_a: np.ndarray, rotation_b: np.ndarray) -> float:
    """两个旋转矩阵之间的测地角，单位 rad。"""
    value = (float(np.trace(rotation_a.T @ rotation_b)) - 1.0) / 2.0
    return math.acos(max(-1.0, min(1.0, value)))


def _matrix_exp(vector: np.ndarray) -> np.ndarray:
    rotation, _ = cv2.Rodrigues(np.asarray(vector, dtype=np.float64).reshape(3, 1))
    return rotation


def integrate_body_rotation(
    imu_rel_sec: np.ndarray,
    gyro_rad_s: np.ndarray,
    raw_start_sec: float,
    raw_end_sec: float,
) -> np.ndarray:
    """积分原始 IMU 时间 [raw_start, raw_end] 内的 body 增量旋转。

    返回 ``Q_raw = R_bi^T R_bj``（常写作 R_bj_bi），由右乘 body 角速度
    ``R(t+dt)=R(t)Exp(omega dt)`` 得到。查询边界使用相邻样本线性插值，
    内部样本使用梯形角速度。所有时间都是相对 bag 首条 IMU 的 double 秒，
    不把 epoch 纳秒直接转成 double。返回的单位四元数/矩阵约定随后由
    ``predict_visual_rotation`` 固定。
    """
    if raw_end_sec <= raw_start_sec:
        return np.eye(3)
    count = len(imu_rel_sec)
    start_index = int(np.searchsorted(imu_rel_sec, raw_start_sec, side="right") - 1)
    end_index = int(np.searchsorted(imu_rel_sec, raw_end_sec, side="left"))
    start_index = max(0, min(start_index, count - 2))
    end_index = max(start_index + 1, min(end_index, count - 1))

    # 只保留严格位于请求区间内部的样本；起点和终点始终由插值精确给出。
    # 不能把 end 之后的第一个样本也积分进去，否则短窗口会被拉长到采样间隔。
    times = [raw_start_sec]
    for sample_time in imu_rel_sec[start_index + 1 : end_index + 1].tolist():
        if raw_start_sec < sample_time < raw_end_sec:
            times.append(float(sample_time))
    times.append(raw_end_sec)
    values: list[np.ndarray] = []
    for time_value in times:
        values.append(
            np.asarray(
                [
                    np.interp(time_value, imu_rel_sec, gyro_rad_s[:, axis])
                    for axis in range(3)
                ],
                dtype=np.float64,
            )
        )

    rotation = np.eye(3)
    for index in range(len(times) - 1):
        dt = times[index + 1] - times[index]
        if dt <= 0.0:
            continue
        mean_omega = 0.5 * (values[index] + values[index + 1])
        rotation = rotation @ _matrix_exp(mean_omega * dt)
    return rotation


def predict_visual_rotation(
    rotation_q_raw: np.ndarray, rotation_cam_body: np.ndarray
) -> np.ndarray:
    """把 IMU 积分旋转映射为与 PnP 输出一致方向的相机相对旋转。

    本工具中 PnP 返回 ``p_cam_j = R p_cam_i``。视觉旋转核对后采用：

        R_cj_ci = R_cb * Q_raw^T * R_cb^T

    其中 ``Q_raw = R_bi^T R_bj``，按计划中的写法即 ``Q = R_bi_bj = Q_raw``，
    所以上式正是 ``R_cb * Q^T * R_cb^T``。这里不机械照抄符号；
    ``direction_check`` 会同时报告不转置的版本，真实数据上采用版本的误差
    显著更小。
    """
    return rotation_cam_body @ rotation_q_raw.T @ rotation_cam_body.T


def _frame_features(
    gray: np.ndarray,
    orb: Any,
    matcher: Any,
    matrix0: np.ndarray,
    matrix1: np.ndarray,
    dist0: np.ndarray,
    dist1: np.ndarray,
    rotation10: np.ndarray,
    translation10: np.ndarray,
    max_match_distance: float,
) -> FrameFeatures | None:
    """检测左右目 ORB、匹配并三角化左目三维点。"""
    width = gray.shape[1] // 2
    left = gray[:, :width]
    right = gray[:, width:]
    keypoints_left, descriptors_left = orb.detectAndCompute(left, None)
    keypoints_right, descriptors_right = orb.detectAndCompute(right, None)
    if descriptors_left is None or descriptors_right is None:
        return None
    matches = sorted(matcher.match(descriptors_left, descriptors_right), key=lambda m: m.distance)
    unique_left: set[int] = set()
    unique_right: set[int] = set()
    kept = []
    for match in matches:
        if match.distance > max_match_distance:
            continue
        if match.queryIdx in unique_left or match.trainIdx in unique_right:
            continue
        unique_left.add(match.queryIdx)
        unique_right.add(match.trainIdx)
        kept.append(match)
    if len(kept) < 20:
        return None

    left_index = np.asarray([match.queryIdx for match in kept], dtype=np.int64)
    right_index = np.asarray([match.trainIdx for match in kept], dtype=np.int64)
    bearings_left = _undistort([keypoints_left[i].pt for i in left_index], matrix0, dist0)
    bearings_right = _undistort([keypoints_right[i].pt for i in right_index], matrix1, dist1)
    projection0 = np.hstack([np.eye(3), np.zeros((3, 1))])
    projection1 = np.hstack([rotation10, translation10.reshape(3, 1)])
    homogeneous = cv2.triangulatePoints(
        projection0, projection1, bearings_left.T, bearings_right.T
    )
    finite = np.abs(homogeneous[3, :]) > 1e-12
    points = np.full((homogeneous.shape[1], 3), np.nan, dtype=np.float64)
    points[finite] = (homogeneous[:3, finite] / homogeneous[3, finite]).T
    points_in_right = points @ rotation10.T + translation10
    valid_match = (
        np.isfinite(points).all(axis=1)
        & (points[:, 2] > 0.05)
        & (points[:, 2] < 100.0)
        & (points_in_right[:, 2] > 0.05)
    )

    points_by_keypoint = np.full((len(keypoints_left), 3), np.nan, dtype=np.float64)
    valid_by_keypoint = np.zeros(len(keypoints_left), dtype=bool)
    for keypoint_index, is_valid, point in zip(left_index, valid_match, points):
        if is_valid:
            points_by_keypoint[keypoint_index] = point
            valid_by_keypoint[keypoint_index] = True
    return FrameFeatures(
        keypoints=tuple(keypoints_left),
        descriptors=descriptors_left,
        points_cam0=points_by_keypoint,
        valid_points=valid_by_keypoint,
        stereo_matches=len(kept),
    )


def _decode_window(
    bag_path: Path,
    image_topic: str,
    wanted_stamps_ns: set[int],
) -> dict[int, np.ndarray]:
    """第二遍只解码选定窗口的拼接图像为灰度图。"""
    from rclpy.serialization import deserialize_message
    from sensor_msgs.msg import CompressedImage

    frames: dict[int, np.ndarray] = {}
    reader = _open_reader(bag_path)
    while reader.has_next():
        topic, serialized, _storage_ns = reader.read_next()
        if topic != image_topic:
            continue
        message = deserialize_message(serialized, CompressedImage)
        stamp = _stamp_ns(message.header.stamp)
        if stamp not in wanted_stamps_ns:
            continue
        encoded = np.frombuffer(message.data, dtype=np.uint8)
        gray = cv2.imdecode(encoded, cv2.IMREAD_GRAYSCALE)
        if gray is None:
            raise ConversionError(f"JPEG 解码失败: {stamp}")
        frames[stamp] = gray
    missing = wanted_stamps_ns.difference(frames)
    if missing:
        raise ConversionError(f"第二遍读取缺少 {len(missing)} 帧选定图像")
    return frames


def _visual_rotation(
    base: FrameFeatures,
    target: FrameFeatures,
    matcher: Any,
    matrix0: np.ndarray,
    dist0: np.ndarray,
) -> tuple[np.ndarray, int, float] | None:
    """把 base 的三维点匹配到 target 左目，用 PnP 估计相对旋转。"""
    matches = sorted(
        matcher.match(base.descriptors, target.descriptors), key=lambda m: m.distance
    )
    object_points: list[np.ndarray] = []
    image_points: list[tuple[float, float]] = []
    seen: set[int] = set()
    for match in matches:
        if match.queryIdx in seen:
            continue
        seen.add(match.queryIdx)
        if match.queryIdx >= len(base.valid_points):
            continue
        if not base.valid_points[match.queryIdx]:
            continue
        object_points.append(base.points_cam0[match.queryIdx])
        image_points.append(target.keypoints[match.trainIdx].pt)
    if len(object_points) < 20:
        return None
    object_array = np.asarray(object_points, dtype=np.float64)
    normalized_image = _undistort(image_points, matrix0, dist0)
    ok, rotation_vector, _translation, inliers = cv2.solvePnPRansac(
        object_array,
        normalized_image,
        np.eye(3),
        None,
        iterationsCount=300,
        reprojectionError=0.008,
        confidence=0.999,
        flags=cv2.SOLVEPNP_EPNP,
    )
    if not ok or inliers is None or len(inliers) < 18:
        return None
    rotation, _ = cv2.Rodrigues(rotation_vector)
    rotation, _ = cv2.Rodrigues(rotation_vector)
    inlier_ratio = float(len(inliers)) / float(len(object_points))
    return rotation, int(len(inliers)), inlier_ratio


def select_motion_windows(
    meta: BagMeta,
    window_count: int,
    half_width_sec: float,
    min_separation_sec: float,
) -> list[dict[str, float | int]]:
    """按平滑后的陀螺模长选择互相分开的有运动时段。"""
    imu_rel = (meta.imu_stamps_ns - meta.imu_stamps_ns[0]) / 1e9
    norm = np.linalg.norm(meta.gyro_rad_s, axis=1)
    window_samples = max(1, int(round(0.2 * meta.imu_rate_hz)))
    kernel = np.ones(window_samples, dtype=np.float64) / float(window_samples)
    if len(norm) >= window_samples:
        smooth = np.convolve(norm, kernel, mode="valid")
        centers = imu_rel[: len(smooth)] + 0.5 * window_samples / meta.imu_rate_hz
    else:
        smooth = norm
        centers = imu_rel
    order = np.argsort(smooth)[::-1]
    chosen: list[dict[str, float | int]] = []
    for index in order:
        center = float(centers[index])
        if center - half_width_sec < 0.5 or center + half_width_sec > float(imu_rel[-1]) - 0.5:
            continue
        if any(abs(center - float(item["center_sec"])) < min_separation_sec for item in chosen):
            continue
        chosen.append(
            {
                "center_sec": center,
                "start_sec": center - half_width_sec,
                "end_sec": center + half_width_sec,
                "mean_gyro_norm_rad_s": float(smooth[index]),
            }
        )
        if len(chosen) >= window_count:
            break
    chosen.sort(key=lambda item: float(item["center_sec"]))
    if len(chosen) < 2:
        raise ConversionError("有运动时段少于 2 个，无法做留出复核")
    return chosen


def extract_visual_pairs(
    bag_path: Path,
    image_topic: str,
    meta: BagMeta,
    calibration: Any,
    windows: Sequence[dict[str, float | int]],
    spans_ms: Sequence[int],
    stride: int,
    max_features: int,
    max_match_distance: float,
    min_motion_rad_s: float,
) -> tuple[list[VisualPair], dict[str, Any]]:
    """逐个运动窗口提取视觉相对旋转，并记录保留/排除计数。"""
    matrix0 = _intrinsics(calibration.camera1_intrinsics)
    matrix1 = _intrinsics(calibration.camera2_intrinsics)
    dist0 = calibration.camera1_distortion
    dist1 = calibration.camera2_distortion
    transform10 = np.linalg.inv(calibration.T_c1_c2)
    rotation10 = transform10[:3, :3]
    translation10 = transform10[:3, 3]
    bias_rad_s = calibration.gyroscope_bias_rad_s
    gyro = meta.gyro_rad_s - bias_rad_s
    imu_rel = (meta.imu_stamps_ns - meta.imu_stamps_ns[0]) / 1e9

    excluded: dict[str, int] = {
        "low_motion": 0,
        "few_stereo_points": 0,
        "few_cross_frame_matches": 0,
        "pnp_failed": 0,
        "degenerate_small_rotation": 0,
        "missing_target_frame": 0,
    }
    stats: dict[str, Any] = {
        "windows": [],
        "processed_frames": 0,
        "stereo_matches": 0,
        "stereo_valid_points": 0,
        "pnp_attempts": 0,
        "retained_pairs": 0,
    }
    pair_records: list[VisualPair] = []
    orb = cv2.ORB_create(
        nfeatures=max_features,
        scaleFactor=1.2,
        nlevels=8,
        fastThreshold=15,
    )
    matcher = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)

    for window in windows:
        start_sec = float(window["start_sec"])
        end_sec = float(window["end_sec"])
        selected_indices = np.where(
            (imu_rel >= start_sec - 0.05) & (imu_rel <= end_sec + 0.05)
        )[0]
        if len(selected_indices) < 3:
            continue
        image_rel = (meta.image_stamps_ns - meta.imu_stamps_ns[0]) / 1e9
        wanted = {
            int(stamp)
            for stamp in meta.image_stamps_ns[
                np.where((image_rel >= start_sec) & (image_rel <= end_sec))[0]
            ]
        }
        # 目标帧可能略超出窗口；先收集基础帧，再按时间跨度搜索目标帧。
        base_indices = np.where((image_rel >= start_sec) & (image_rel <= end_sec))[0]
        for span in spans_ms:
            for index in base_indices[:: max(1, stride)]:
                target_index = int(
                    np.searchsorted(
                        meta.image_stamps_ns,
                        int(meta.image_stamps_ns[index]) + int(span) * 1_000_000,
                    )
                )
                if target_index < len(meta.image_stamps_ns):
                    wanted.add(int(meta.image_stamps_ns[target_index]))

        frames = _decode_window(bag_path, image_topic, wanted)
        features: dict[int, FrameFeatures] = {}
        for stamp in sorted(frames):
            feature = _frame_features(
                frames[stamp],
                orb,
                matcher,
                matrix0,
                matrix1,
                dist0,
                dist1,
                rotation10,
                translation10,
                max_match_distance,
            )
            if feature is None:
                continue
            features[stamp] = feature
            stats["processed_frames"] += 1
            stats["stereo_matches"] += feature.stereo_matches
            stats["stereo_valid_points"] += int(feature.valid_points.sum())
        window_retained = 0
        base_stamps = [
            int(meta.image_stamps_ns[index])
            for index in base_indices[:: max(1, stride)]
        ]
        for span in spans_ms:
            for stamp in base_stamps:
                if stamp not in features:
                    continue
                target_index = int(
                    np.searchsorted(
                        meta.image_stamps_ns, stamp + int(span) * 1_000_000
                    )
                )
                if target_index >= len(meta.image_stamps_ns):
                    excluded["missing_target_frame"] += 1
                    continue
                target_stamp = int(meta.image_stamps_ns[target_index])
                if target_stamp not in features:
                    excluded["missing_target_frame"] += 1
                    continue
                raw_start = (stamp - int(meta.imu_stamps_ns[0])) / 1e9
                raw_end = (target_stamp - int(meta.imu_stamps_ns[0])) / 1e9
                segment = (imu_rel >= raw_start) & (imu_rel <= raw_end)
                if segment.sum() < 3:
                    excluded["low_motion"] += 1
                    continue
                mean_motion = float(np.mean(np.linalg.norm(gyro[segment], axis=1)))
                if mean_motion < min_motion_rad_s:
                    excluded["low_motion"] += 1
                    continue
                if features[stamp].valid_points.sum() < 20:
                    excluded["few_stereo_points"] += 1
                    continue
                stats["pnp_attempts"] += 1
                visual = _visual_rotation(
                    features[stamp], features[target_stamp], matcher, matrix0, dist0
                )
                if visual is None:
                    excluded["few_cross_frame_matches"] += 1
                    continue
                rotation, inliers, inlier_ratio = visual
                if _rotation_angle(rotation, np.eye(3)) < 0.01:
                    excluded["degenerate_small_rotation"] += 1
                    continue
                pair_records.append(
                    VisualPair(
                        stamp_i_ns=stamp,
                        stamp_j_ns=target_stamp,
                        span_ms=span,
                        rotation_cj_ci=rotation,
                        inliers=inliers,
                        inlier_ratio=inlier_ratio,
                        motion_rad_s=mean_motion,
                        stereo_points=int(features[stamp].valid_points.sum()),
                    )
                )
                window_retained += 1
        stats["windows"].append(
            {
                "center_sec": float(window["center_sec"]),
                "start_sec": start_sec,
                "end_sec": end_sec,
                "decoded_frames": len(frames),
                "feature_frames": len(features),
                "retained_pairs": window_retained,
                "mean_gyro_norm_rad_s": float(window["mean_gyro_norm_rad_s"]),
            }
        )
        stats["retained_pairs"] = len(pair_records)
    stats["excluded"] = excluded
    return pair_records, stats


def evaluate_offsets(
    records: Sequence[VisualPair],
    imu_stamps_ns: np.ndarray,
    gyro_rad_s: np.ndarray,
    rotation_cam_body: np.ndarray,
    offsets_ms: Iterable[float],
) -> list[dict[str, float | int]]:
    """对每个候选偏移计算视觉/IMU 旋转测地误差中位数。"""
    imu_rel = (imu_stamps_ns - imu_stamps_ns[0]).astype(np.float64) / 1e9
    curve: list[dict[str, float | int]] = []
    for offset_ms in offsets_ms:
        offset_sec = float(offset_ms) / 1000.0
        errors = []
        for record in records:
            raw_start = (record.stamp_i_ns - int(imu_stamps_ns[0])) / 1e9 - offset_sec
            raw_end = (record.stamp_j_ns - int(imu_stamps_ns[0])) / 1e9 - offset_sec
            integrated = integrate_body_rotation(
                imu_rel, gyro_rad_s, raw_start, raw_end
            )
            predicted = predict_visual_rotation(integrated, rotation_cam_body)
            errors.append(_rotation_angle(record.rotation_cj_ci, predicted))
        if errors:
            curve.append(
                {
                    "offset_ms": float(offset_ms),
                    "median_error_rad": float(np.median(errors)),
                    "median_error_deg": float(np.rad2deg(np.median(errors))),
                    "mean_error_rad": float(np.mean(errors)),
                    "mean_error_deg": float(np.rad2deg(np.mean(errors))),
                    "count": len(errors),
                }
            )
    return curve


def _best_offset(curve: Sequence[dict[str, float | int]]) -> dict[str, float | int]:
    if not curve:
        raise ConversionError("候选偏移曲线为空")
    return min(curve, key=lambda item: float(item["median_error_rad"]))


def _segment_records(
    records: Sequence[VisualPair], windows: Sequence[dict[str, float | int]], imu_origin_ns: int
) -> tuple[list[VisualPair], list[VisualPair], list[int], list[int]]:
    """按时间交替划分训练窗口和留出窗口，避免同一时段泄漏。"""
    centers_ns = [
        int(round(float(window["center_sec"]) * 1e9)) + imu_origin_ns for window in windows
    ]
    train_windows = [index for index in range(len(windows)) if index % 2 == 0]
    holdout_windows = [index for index in range(len(windows)) if index % 2 == 1]
    train: list[VisualPair] = []
    holdout: list[VisualPair] = []
    for record in records:
        distances = [abs(record.stamp_i_ns - center) for center in centers_ns]
        nearest = int(np.argmin(distances))
        if nearest in train_windows:
            train.append(record)
        elif nearest in holdout_windows:
            holdout.append(record)
    return train, holdout, train_windows, holdout_windows


def _coarse_then_fine(
    records: Sequence[VisualPair],
    imu_stamps_ns: np.ndarray,
    gyro_rad_s: np.ndarray,
    rotation_cam_body: np.ndarray,
    half_range_ms: float,
    coarse_step_ms: float,
    fine_step_ms: float,
) -> tuple[dict[str, float | int], list[dict[str, float | int]], list[dict[str, float | int]]]:
    coarse_offsets = np.arange(-half_range_ms, half_range_ms + 1e-9, coarse_step_ms)
    coarse_curve = evaluate_offsets(
        records, imu_stamps_ns, gyro_rad_s, rotation_cam_body, coarse_offsets
    )
    coarse_best = _best_offset(coarse_curve)
    center = float(coarse_best["offset_ms"])
    fine_offsets = np.arange(
        max(-half_range_ms, center - coarse_step_ms),
        min(half_range_ms, center + coarse_step_ms) + 1e-9,
        fine_step_ms,
    )
    fine_curve = evaluate_offsets(
        records, imu_stamps_ns, gyro_rad_s, rotation_cam_body, fine_offsets
    )
    fine_best = _best_offset(fine_curve)
    return fine_best, coarse_curve, fine_curve


def _curve_at(curve: Sequence[dict[str, float | int]], offset_ms: float) -> dict[str, float | int] | None:
    return min(curve, key=lambda item: abs(float(item["offset_ms"]) - offset_ms), default=None)


def _evaluate_summary(
    records: Sequence[VisualPair],
    imu_stamps_ns: np.ndarray,
    gyro_rad_s: np.ndarray,
    rotation_cam_body: np.ndarray,
    recommended_ms: float,
    configured_ms: float,
) -> dict[str, Any]:
    imu_rel = (imu_stamps_ns - imu_stamps_ns[0]).astype(np.float64) / 1e9
    gyro = gyro_rad_s

    def median_at(offset_ms: float) -> tuple[float, float]:
        errors = []
        for record in records:
            raw_start = (record.stamp_i_ns - int(imu_stamps_ns[0])) / 1e9 - offset_ms / 1000.0
            raw_end = (record.stamp_j_ns - int(imu_stamps_ns[0])) / 1e9 - offset_ms / 1000.0
            integrated = integrate_body_rotation(imu_rel, gyro, raw_start, raw_end)
            predicted = predict_visual_rotation(integrated, rotation_cam_body)
            errors.append(_rotation_angle(record.rotation_cj_ci, predicted))
        if not errors:
            return float("nan"), float("nan")
        return float(np.rad2deg(np.median(errors))), float(np.rad2deg(np.mean(errors)))

    recommended_median, recommended_mean = median_at(recommended_ms)
    configured_median, configured_mean = median_at(configured_ms)
    zero_median, zero_mean = median_at(0.0)
    by_span: dict[str, Any] = {}
    for span in sorted({record.span_ms for record in records}):
        subset = [record for record in records if record.span_ms == span]
        span_recommended = median_at_for_records(subset, recommended_ms, imu_stamps_ns, gyro, rotation_cam_body)
        span_configured = median_at_for_records(subset, configured_ms, imu_stamps_ns, gyro, rotation_cam_body)
        by_span[str(span)] = {
            "count": len(subset),
            "recommended_median_deg": span_recommended,
            "configured_median_deg": span_configured,
            "improved": bool(
                math.isfinite(span_recommended)
                and math.isfinite(span_configured)
                and span_recommended < span_configured
            ),
        }
    return {
        "recommended_offset_ms": recommended_ms,
        "configured_offset_ms": configured_ms,
        "recommended_median_deg": recommended_median,
        "recommended_mean_deg": recommended_mean,
        "configured_median_deg": configured_median,
        "configured_mean_deg": configured_mean,
        "zero_offset_median_deg": zero_median,
        "zero_offset_mean_deg": zero_mean,
        "improvement_deg": configured_median - recommended_median,
        "improved": bool(recommended_median < configured_median),
        "by_span": by_span,
    }


def median_at_for_records(
    records: Sequence[VisualPair],
    offset_ms: float,
    imu_stamps_ns: np.ndarray,
    gyro_rad_s: np.ndarray,
    rotation_cam_body: np.ndarray,
) -> float:
    imu_rel = (imu_stamps_ns - imu_stamps_ns[0]).astype(np.float64) / 1e9
    errors = []
    for record in records:
        raw_start = (record.stamp_i_ns - int(imu_stamps_ns[0])) / 1e9 - offset_ms / 1000.0
        raw_end = (record.stamp_j_ns - int(imu_stamps_ns[0])) / 1e9 - offset_ms / 1000.0
        integrated = integrate_body_rotation(imu_rel, gyro_rad_s, raw_start, raw_end)
        predicted = predict_visual_rotation(integrated, rotation_cam_body)
        errors.append(_rotation_angle(record.rotation_cj_ci, predicted))
    return float(np.rad2deg(np.median(errors))) if errors else float("nan")


def synthetic_self_test() -> dict[str, Any]:
    """用时变角速度合成视觉旋转，验证符号、方向和 offset 可辨识性。"""
    sample_rate = 400.0
    duration = 3.0
    imu_rel = np.arange(0.0, duration, 1.0 / sample_rate)
    bias = np.deg2rad(np.asarray([0.1, -0.2, 0.15]))
    # 角速度有非零一阶/二阶变化；常角速度本身无法验证时间偏移。
    omega = np.column_stack(
        [
            0.7 * np.sin(2.0 * math.pi * 1.3 * imu_rel) + 0.2 * np.cos(2.0 * math.pi * 0.4 * imu_rel),
            0.5 * np.sin(2.0 * math.pi * 0.9 * imu_rel + 0.3) + 0.15 * imu_rel,
            0.9 * np.cos(2.0 * math.pi * 1.1 * imu_rel + 0.7) - 0.1 * np.sin(2.0 * math.pi * 2.4 * imu_rel),
        ]
    )
    gyro_raw = omega + bias
    rotation_cam_body = np.asarray(
        [[0.9999, -0.01, 0.005], [0.01, 0.9999, 0.02], [-0.005, -0.02, 0.9998]],
        dtype=np.float64,
    )
    # 保证 R_cb 是正交阵。
    u, _, vh = np.linalg.svd(rotation_cam_body)
    rotation_cam_body = u @ vh
    true_offset_ms = 27.0
    imu_origin_ns = 0
    imu_stamps_ns = (imu_rel * 1e9).astype(np.int64) + imu_origin_ns
    records: list[VisualPair] = []
    for start_sec in np.arange(0.3, duration - 0.5, 0.017):
        for span_ms in (20, 60, 160):
            start_ns = int(round(start_sec * 1e9)) + imu_origin_ns
            end_ns = start_ns + span_ms * 1_000_000
            raw_start = (start_ns - imu_origin_ns) / 1e9 - true_offset_ms / 1000.0
            raw_end = (end_ns - imu_origin_ns) / 1e9 - true_offset_ms / 1000.0
            integrated = integrate_body_rotation(imu_rel, gyro_raw - bias, raw_start, raw_end)
            visual = predict_visual_rotation(integrated, rotation_cam_body)
            records.append(
                VisualPair(
                    stamp_i_ns=start_ns,
                    stamp_j_ns=end_ns,
                    span_ms=span_ms,
                    rotation_cj_ci=visual,
                    inliers=100,
                    inlier_ratio=0.5,
                    motion_rad_s=float(np.mean(np.linalg.norm(omega, axis=1))),
                    stereo_points=500,
                )
            )
    best, coarse, fine = _coarse_then_fine(
        records, imu_stamps_ns, gyro_raw, rotation_cam_body, 50.0, 1.0, 0.1
    )
    best_ms = float(best["offset_ms"])
    predicted_wrong = predict_visual_rotation(
        integrate_body_rotation(
            imu_rel,
            gyro_raw,
            (records[0].stamp_i_ns - imu_origin_ns) / 1e9 - best_ms / 1000.0,
            (records[0].stamp_j_ns - imu_origin_ns) / 1e9 - best_ms / 1000.0,
        ),
        rotation_cam_body,
    ).T
    return {
        "true_offset_ms": true_offset_ms,
        "estimated_offset_ms": best_ms,
        "absolute_error_ms": abs(best_ms - true_offset_ms),
        "coarse_best": _best_offset(coarse),
        "fine_best": _best_offset(fine),
        "correct_direction_error_deg": float(
            np.rad2deg(_rotation_angle(records[0].rotation_cj_ci, predict_visual_rotation(
                integrate_body_rotation(
                    imu_rel, gyro_raw,
                    (records[0].stamp_i_ns - imu_origin_ns) / 1e9 - best_ms / 1000.0,
                    (records[0].stamp_j_ns - imu_origin_ns) / 1e9 - best_ms / 1000.0,
                ), rotation_cam_body)))
        ),
        "transposed_direction_error_deg": float(
            np.rad2deg(_rotation_angle(records[0].rotation_cj_ci, predicted_wrong))
        ),
        "passed": bool(abs(best_ms - true_offset_ms) <= 1.0),
    }


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="用 EGO 视觉旋转与去 bias IMU 积分核验 IMU 时间偏移。"
    )
    parser.add_argument("--calib", type=Path, help="只读 calib.yaml（非 --self-test 必填）")
    parser.add_argument("--bag", type=Path, help="只读 MCAP（非 --self-test 必填）")
    parser.add_argument("--output", type=Path, help="输出 JSON；省略时打印到 stdout")
    parser.add_argument("--image-topic", default=IMAGE_TOPIC)
    parser.add_argument("--imu-topic", default=IMU_TOPIC)
    parser.add_argument("--half-range-ms", type=float, default=50.0)
    parser.add_argument("--coarse-step-ms", type=float, default=1.0)
    parser.add_argument("--fine-step-ms", type=float, default=0.1)
    parser.add_argument("--spans-ms", default="20,60,160,320")
    parser.add_argument("--window-count", type=int, default=4)
    parser.add_argument("--window-half-width-sec", type=float, default=1.0)
    parser.add_argument("--window-separation-sec", type=float, default=8.0)
    parser.add_argument("--stride", type=int, default=3)
    parser.add_argument("--max-features", type=int, default=2000)
    parser.add_argument("--max-match-distance", type=float, default=70.0)
    parser.add_argument("--min-motion-rad-s", type=float, default=0.5)
    parser.add_argument("--overwrite", action="store_true", help="显式允许覆盖输出")
    parser.add_argument("--self-test", action="store_true", help="只运行合成符号/offset 自检")
    return parser


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    if args.self_test:
        try:
            result = synthetic_self_test()
        except Exception as exc:  # 自检失败也给出可审计的非零退出
            print(f"ERROR: 合成自检失败: {exc}", file=sys.stderr)
            return 2
        print(json.dumps(result, ensure_ascii=False, indent=2, sort_keys=True))
        return 0 if result["passed"] else 2

    if args.output is None:
        print("ERROR: 未指定 --output；本工具要求保存可审查 JSON", file=sys.stderr)
        return 2
    if args.calib is None or args.bag is None:
        print("ERROR: 非 --self-test 模式必须提供 --calib 和 --bag", file=sys.stderr)
        return 2
    try:
        if not args.calib.is_file():
            raise ConversionError(f"标定文件不存在: {args.calib}")
        if not args.bag.is_file():
            raise ConversionError(f"bag 文件不存在: {args.bag}")
        spans = [int(item) for item in args.spans_ms.split(",") if item.strip()]
        if not spans or any(span <= 0 for span in spans):
            raise ConversionError("--spans-ms 必须是正整数列表")
        start_time = time.time()
        calibration = read_calibration(args.calib)
        meta = read_bag_meta(args.bag, args.image_topic, args.imu_topic)
        windows = select_motion_windows(
            meta,
            args.window_count,
            args.window_half_width_sec,
            args.window_separation_sec,
        )
        pairs, extraction_stats = extract_visual_pairs(
            args.bag,
            args.image_topic,
            meta,
            calibration,
            windows,
            spans,
            args.stride,
            args.max_features,
            args.max_match_distance,
            args.min_motion_rad_s,
        )
        if len(pairs) < 30:
            raise ConversionError(
                f"可用视觉旋转样本只有 {len(pairs)} 组，少于 30；拒绝给出不可靠偏移"
            )
        rotation_cam_body = calibration.T_b_c1[:3, :3].T
        gyro_no_bias = meta.gyro_rad_s - calibration.gyroscope_bias_rad_s
        train, holdout, train_windows, holdout_windows = _segment_records(
            pairs, windows, int(meta.imu_stamps_ns[0])
        )
        if len(train) < 20 or len(holdout) < 10:
            raise ConversionError(
                f"训练/留出样本不足: train={len(train)}, holdout={len(holdout)}"
            )
        recommended, train_coarse, train_fine = _coarse_then_fine(
            train,
            meta.imu_stamps_ns,
            gyro_no_bias,
            rotation_cam_body,
            args.half_range_ms,
            args.coarse_step_ms,
            args.fine_step_ms,
        )
        holdout_curve = evaluate_offsets(
            holdout,
            meta.imu_stamps_ns,
            gyro_no_bias,
            rotation_cam_body,
            np.arange(-args.half_range_ms, args.half_range_ms + 1e-9, args.coarse_step_ms),
        )
        holdout_best = _best_offset(holdout_curve)
        configured_ms = calibration.imu_time_offset_sec * 1000.0
        recommended_ms = float(recommended["offset_ms"])
        train_summary = _evaluate_summary(
            train,
            meta.imu_stamps_ns,
            gyro_no_bias,
            rotation_cam_body,
            recommended_ms,
            configured_ms,
        )
        holdout_summary = _evaluate_summary(
            holdout,
            meta.imu_stamps_ns,
            gyro_no_bias,
            rotation_cam_body,
            recommended_ms,
            configured_ms,
        )
        per_window = []
        for index, window in enumerate(windows):
            center = float(window["center_sec"])
            center_ns = int(round(center * 1e9)) + int(meta.imu_stamps_ns[0])
            subset = [
                record
                for record in pairs
                if abs(record.stamp_i_ns - center_ns) <= int(round(args.window_half_width_sec * 1e9))
            ]
            if not subset:
                continue
            curve = evaluate_offsets(
                subset,
                meta.imu_stamps_ns,
                gyro_no_bias,
                rotation_cam_body,
                np.arange(-args.half_range_ms, args.half_range_ms + 1e-9, args.coarse_step_ms),
            )
            window_best = _best_offset(curve)
            per_window.append(
                {
                    "index": index,
                    "role": "train" if index in train_windows else "holdout",
                    "center_sec": center,
                    "count": len(subset),
                    "best_offset_ms": float(window_best["offset_ms"]),
                    "best_median_deg": float(window_best["median_error_deg"]),
                    "recommended_median_deg": median_at_for_records(
                        subset, recommended_ms, meta.imu_stamps_ns, gyro_no_bias, rotation_cam_body
                    ),
                    "configured_median_deg": median_at_for_records(
                        subset, configured_ms, meta.imu_stamps_ns, gyro_no_bias, rotation_cam_body
                    ),
                }
            )
        improved_windows = [
            item
            for item in per_window
            if item["recommended_median_deg"] < item["configured_median_deg"]
        ]
        # 方向核对：把预测旋转转置后误差应明显更大，否则说明符号/方向不可辨识。
        direction_errors = []
        for record in train:
            raw_start = (record.stamp_i_ns - int(meta.imu_stamps_ns[0])) / 1e9 - recommended_ms / 1000.0
            raw_end = (record.stamp_j_ns - int(meta.imu_stamps_ns[0])) / 1e9 - recommended_ms / 1000.0
            integrated = integrate_body_rotation(
                (meta.imu_stamps_ns - meta.imu_stamps_ns[0]) / 1e9,
                gyro_no_bias,
                raw_start,
                raw_end,
            )
            correct = predict_visual_rotation(integrated, rotation_cam_body)
            wrong = correct.T
            direction_errors.append(
                (
                    float(np.rad2deg(_rotation_angle(record.rotation_cj_ci, correct))),
                    float(np.rad2deg(_rotation_angle(record.rotation_cj_ci, wrong))),
                )
            )
        direction_check = {
            "selected_mapping": "R_cb * Q_raw^T * R_cb^T with Q_raw = R_bi^T R_bj",
            "correct_direction_median_deg": float(np.median([item[0] for item in direction_errors])),
            "transposed_direction_median_deg": float(np.median([item[1] for item in direction_errors])),
            "correct_direction_verified": bool(
                np.median([item[0] for item in direction_errors])
                < np.median([item[1] for item in direction_errors])
            ),
        }
        result = {
            "tool": "align-ego-stereo-inertial-time",
            "version": TOOL_VERSION,
            "status": "complete",
            "inputs": {
                "calib": str(args.calib),
                "bag": str(args.bag),
                "calib_sha256": _sha256(args.calib),
                "bag_sha256": _sha256(args.bag),
            },
            "method": {
                "visual_rotation": "fisheye ORB stereo triangulation + left-camera PnP",
                "distortion_model": "Kannala-Brandt fisheye from calib.yaml",
                "imu_rotation": "fixed gyro bias removed, trapezoidal body-rate integration over raw IMU [ti-offset,tj-offset]",
                "time_origin": "int64 raw header stamps; relative double seconds only for interpolation",
                "selection": "offset chosen on train windows only; holdout windows and span subsets are validation",
                "not_used": "ORB-SLAM3 trajectory or its estimated time offset",
            },
            "calibration": {
                "configured_imu_time_offset_sec": calibration.imu_time_offset_sec,
                "configured_imu_time_offset_ms": configured_ms,
                "gyro_bias_rad_s": calibration.gyroscope_bias_rad_s.tolist(),
                "baseline_m": float(np.linalg.norm(calibration.T_c1_c2[:3, 3])),
            },
            "bag": {
                "image_count": meta.image_count,
                "imu_count": meta.imu_count,
                "image_rate_hz": meta.image_rate_hz,
                "imu_rate_hz": meta.imu_rate_hz,
                "image_topic": args.image_topic,
                "imu_topic": args.imu_topic,
            },
            "settings": {
                "spans_ms": spans,
                "half_range_ms": args.half_range_ms,
                "coarse_step_ms": args.coarse_step_ms,
                "fine_step_ms": args.fine_step_ms,
                "window_count": len(windows),
                "window_half_width_sec": args.window_half_width_sec,
                "stride": args.stride,
                "max_features": args.max_features,
                "max_match_distance": args.max_match_distance,
                "min_motion_rad_s": args.min_motion_rad_s,
            },
            "motion_windows": windows,
            "segments": {
                "train_windows": train_windows,
                "holdout_windows": holdout_windows,
                "train_pairs": len(train),
                "holdout_pairs": len(holdout),
            },
            "sample_counts": extraction_stats,
            "candidate_curves": {
                "train_coarse": train_coarse,
                "train_fine": train_fine,
                "holdout_coarse": holdout_curve,
            },
            "recommendation": {
                "offset_sec": recommended_ms / 1000.0,
                "offset_ms": recommended_ms,
                "selected_from": "train_windows_only",
                "train": train_summary,
                "holdout": holdout_summary,
                "holdout_best_for_reference_ms": float(holdout_best["offset_ms"]),
                "holdout_best_median_deg": float(holdout_best["median_error_deg"]),
                "holdout_improved": bool(
                    holdout_summary["recommended_median_deg"]
                    < holdout_summary["configured_median_deg"]
                ),
                "windows_improved": len(improved_windows),
                "windows_total": len(per_window),
                "per_window": per_window,
                "historical_openvins_candidate_ms": [25.0, 30.0],
                "historical_candidate_note": "历史 +25~30 ms 仅作为候选，不能冒充本包真值；本工具未读取已不存在的旧验收文件。",
            },
            "direction_check": direction_check,
            "limitations": [
                "没有独立真值（GT）；该结果是视觉旋转与去 bias IMU 积分的一致性估计，不是绝对时间真值。",
                "视觉旋转来自单目左相机的双目三角化 + PnP，低纹理、弱视差、错误立体匹配和滚动快门会引入偏差。",
                "该偏移只对当前 bag/calib/采集驱动链有效，不可外推到其它包或其它相机/IMU。",
                "时间对齐可以抽样有运动窗口；正式 SLAM 仍必须使用全部原始分辨率帧、不抽帧。",
                "原 calib.yaml 中的 configured offset 保持只读，最终运行通过 --imu-time-offset-sec 显式覆盖。",
            ],
            "elapsed_sec": time.time() - start_time,
        }
        output_text = json.dumps(result, ensure_ascii=False, indent=2, sort_keys=True)
        write_config(
            args.output,
            output_text + "\n",
            args.overwrite,
            protected_paths=(args.calib, args.bag),
        )
        print(json.dumps({
            "output": str(args.output),
            "recommended_offset_sec": recommended_ms / 1000.0,
            "holdout_improved": result["recommendation"]["holdout_improved"],
            "windows_improved": f"{len(improved_windows)}/{len(per_window)}",
            "retained_pairs": len(pairs),
        }, ensure_ascii=False, sort_keys=True))
        return 0
    except ConversionError as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 2
    except OSError as exc:
        print(f"ERROR: 输入输出失败: {exc}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    sys.exit(main())
