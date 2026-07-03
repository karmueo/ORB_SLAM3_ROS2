#!/usr/bin/python3
"""离线读取 ORB_SLAM3 估计轨迹和 GT，并计算 ATE 平移误差。"""

from __future__ import annotations

import argparse
import json
import math
import re
import sys
from pathlib import Path
from typing import NamedTuple

import numpy as np


class Trajectory(NamedTuple):
    """保存 TUM 轨迹中的时间戳、平移和四元数。"""

    timestamps: np.ndarray
    positions: np.ndarray
    quaternions: np.ndarray


class AlignmentResult(NamedTuple):
    """保存 Umeyama 对齐结果和对齐后的点。"""

    rotation: np.ndarray
    translation: np.ndarray
    scale: float
    aligned_points: np.ndarray


def read_tum_trajectory(path: Path | str) -> Trajectory:
    """读取 ORB_SLAM3 导出的 TUM 格式轨迹。

    Args:
        path: 轨迹文件路径，每行格式为 timestamp x y z qx qy qz qw。

    Returns:
        包含时间戳、位置和四元数的轨迹对象。

    Raises:
        FileNotFoundError: 输入轨迹文件不存在。
        ValueError: 有效轨迹为空，或某一行不是 8 列浮点数。
    """
    trajectory_path = Path(path)
    timestamps = []
    positions = []
    quaternions = []

    with trajectory_path.open("r", encoding="utf-8") as trajectory_file:
        for line_number, raw_line in enumerate(trajectory_file, start=1):
            stripped_line = raw_line.strip()
            if not stripped_line or stripped_line.startswith("#"):
                continue

            fields = stripped_line.split()
            if len(fields) != 8:
                raise ValueError(f"{trajectory_path}:{line_number} 需要 8 列 TUM 轨迹数据，实际为 {len(fields)} 列")

            try:
                values = [float(field) for field in fields]
            except ValueError as exc:
                raise ValueError(f"{trajectory_path}:{line_number} 包含无法解析的浮点数") from exc

            timestamps.append(values[0])
            positions.append(values[1:4])
            quaternions.append(values[4:8])

    if not timestamps:
        raise ValueError(f"{trajectory_path} 不包含有效 TUM 轨迹数据")

    return Trajectory(
        timestamps=np.asarray(timestamps, dtype=float),
        positions=np.asarray(positions, dtype=float),
        quaternions=np.asarray(quaternions, dtype=float),
    )


def read_tbc(path: Path | str) -> np.ndarray:
    """从 ORB_SLAM3 OpenCV YAML 配置中读取 body 到相机外参矩阵。

    Args:
        path: ORB_SLAM3 settings YAML 文件路径。

    Returns:
        4x4 的 body/IMU 到相机外参矩阵 Tbc。

    Raises:
        FileNotFoundError: 配置文件不存在。
        ValueError: 配置中缺少 Tbc/IMU.T_b_c1，或矩阵数据不是 16 个元素。
    """
    settings_path = Path(path)
    settings_text = settings_path.read_text(encoding="utf-8")
    matrix_key_pattern = r"(?:Tbc|IMU\.T_b_c1)"
    tbc_match = re.search(rf"(?ms)^{matrix_key_pattern}:\s*!!opencv-matrix\s*(.*?)(?:\n\S|\Z)", settings_text)
    if tbc_match is None:
        raise ValueError(f"{settings_path} 缺少 Tbc 或 IMU.T_b_c1，无法在 body 模式下测评")

    tbc_block = tbc_match.group(1)
    data_match = re.search(r"(?ms)data\s*:\s*\[(.*?)\]", tbc_block)
    if data_match is None:
        raise ValueError(f"{settings_path} 的 Tbc 缺少 data 字段")

    data_values = [float(value) for value in re.split(r"[\s,]+", data_match.group(1).strip()) if value]
    if len(data_values) != 16:
        raise ValueError(f"{settings_path} 的 Tbc data 需要 16 个数值，实际为 {len(data_values)} 个")

    return np.asarray(data_values, dtype=float).reshape((4, 4))


def sort_and_deduplicate_gt_samples(
    timestamps: list[float] | np.ndarray, positions: list[list[float]] | np.ndarray
) -> tuple[np.ndarray, np.ndarray]:
    """按时间排序 GT 样本，并删除精确重复时间戳。

    Args:
        timestamps: GT 时间戳序列。
        positions: GT 平移序列，与 timestamps 一一对应。

    Returns:
        已排序、时间戳严格递增的时间数组和平移数组。

    Raises:
        ValueError: GT 样本为空，或时间戳和平移数量不一致。
    """
    timestamps_array = np.asarray(timestamps, dtype=float)
    positions_array = np.asarray(positions, dtype=float)
    if len(timestamps_array) == 0:
        raise ValueError("GT 数据为空")
    if len(timestamps_array) != len(positions_array):
        raise ValueError("GT 时间戳和平移数量不一致")

    sort_indices = np.argsort(timestamps_array, kind="mergesort")
    sorted_timestamps = timestamps_array[sort_indices]
    sorted_positions = positions_array[sort_indices]
    keep_mask = np.ones(len(sorted_timestamps), dtype=bool)
    keep_mask[1:] = np.diff(sorted_timestamps) != 0.0

    return sorted_timestamps[keep_mask], sorted_positions[keep_mask]


def read_groundtruth_file(path: Path | str) -> tuple[np.ndarray, np.ndarray]:
    """读取 OpenLORIS/TUM 格式 groundtruth 文件中的 GT 平移。

    Args:
        path: groundtruth.txt 文件路径，每条轨迹行格式为 timestamp x y z qx qy qz qw。

    Returns:
        GT 时间戳数组和 Nx3 平移数组。

    Raises:
        FileNotFoundError: 输入文件不存在。
        ValueError: 有效轨迹为空，或轨迹行不是 8 列浮点数。
    """
    groundtruth_path = Path(path)
    timestamps = []
    positions = []
    metadata_prefixes = ("scene:", "frame:", "seq:")

    with groundtruth_path.open("r", encoding="utf-8") as groundtruth_file:
        for line_number, raw_line in enumerate(groundtruth_file, start=1):
            stripped_line = raw_line.strip()
            if not stripped_line or stripped_line.startswith("#") or stripped_line.startswith(metadata_prefixes):
                continue

            fields = stripped_line.split()
            if len(fields) != 8:
                raise ValueError(f"{groundtruth_path}:{line_number} 需要 8 列 GT 轨迹数据，实际为 {len(fields)} 列")

            try:
                values = [float(field) for field in fields]
            except ValueError as exc:
                raise ValueError(f"{groundtruth_path}:{line_number} 包含无法解析的浮点数") from exc

            timestamps.append(values[0])
            positions.append(values[1:4])

    if not timestamps:
        raise ValueError(f"{groundtruth_path} 不包含有效 GT 轨迹数据")

    return sort_and_deduplicate_gt_samples(timestamps, positions)


def quaternion_to_rotation_matrix(quaternion: np.ndarray) -> np.ndarray:
    """将 qx qy qz qw 四元数转换为旋转矩阵。

    Args:
        quaternion: 长度为 4 的四元数，顺序为 qx qy qz qw。

    Returns:
        3x3 旋转矩阵。

    Raises:
        ValueError: 四元数范数为 0。
    """
    qx, qy, qz, qw = np.asarray(quaternion, dtype=float)
    norm = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
    if norm == 0.0:
        raise ValueError("轨迹包含零范数四元数")

    qx /= norm
    qy /= norm
    qz /= norm
    qw /= norm

    return np.array(
        [
            [1.0 - 2.0 * (qy * qy + qz * qz), 2.0 * (qx * qy - qz * qw), 2.0 * (qx * qz + qy * qw)],
            [2.0 * (qx * qy + qz * qw), 1.0 - 2.0 * (qx * qx + qz * qz), 2.0 * (qy * qz - qx * qw)],
            [2.0 * (qx * qz - qy * qw), 2.0 * (qy * qz + qx * qw), 1.0 - 2.0 * (qx * qx + qy * qy)],
        ],
        dtype=float,
    )


def convert_camera_to_body_positions(positions: np.ndarray, quaternions: np.ndarray, tbc: np.ndarray) -> np.ndarray:
    """将估计轨迹从 Twc 相机位姿转换为 Twb body 位姿平移。

    Args:
        positions: Nx3 的 Twc 平移。
        quaternions: Nx4 的 Twc 姿态四元数，顺序为 qx qy qz qw。
        tbc: 4x4 的相机到 body/IMU 外参矩阵。

    Returns:
        Nx3 的 Twb 平移。

    Raises:
        ValueError: Tbc 形状不是 4x4，或四元数非法。
    """
    if tbc.shape != (4, 4):
        raise ValueError(f"Tbc 需要是 4x4 矩阵，实际形状为 {tbc.shape}")

    tcb = np.linalg.inv(tbc)
    camera_to_body_translation = tcb[:3, 3]
    body_positions = []

    for position, quaternion in zip(positions, quaternions):
        rotation = quaternion_to_rotation_matrix(quaternion)
        body_positions.append(position + rotation @ camera_to_body_translation)

    return np.asarray(body_positions, dtype=float)


def interpolate_gt_positions(
    estimate_times: np.ndarray, gt_times: np.ndarray, gt_positions: np.ndarray
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """把 GT 平移按时间线性插值到估计轨迹时间戳。

    Args:
        estimate_times: 估计轨迹时间戳。
        gt_times: GT 时间戳，需与 gt_positions 一一对应。
        gt_positions: Nx3 的 GT 平移。

    Returns:
        有效估计时间戳、插值后的 GT 平移、估计点有效掩码。

    Raises:
        ValueError: GT 数据少于 2 个点，或时间戳不是严格递增。
    """
    if len(gt_times) < 2:
        raise ValueError("GT 至少需要 2 个时间点才能插值")

    if np.any(np.diff(gt_times) <= 0.0):
        raise ValueError("GT 时间戳需要严格递增")

    valid_mask = (estimate_times >= gt_times[0]) & (estimate_times <= gt_times[-1])
    valid_times = estimate_times[valid_mask]
    interpolated_axes = [
        np.interp(valid_times, gt_times, gt_positions[:, axis_index]) for axis_index in range(3)
    ]
    interpolated_positions = np.stack(interpolated_axes, axis=1) if len(valid_times) else np.empty((0, 3))

    return valid_times, interpolated_positions, valid_mask


def align_points_umeyama(source_points: np.ndarray, target_points: np.ndarray, with_scale: bool) -> AlignmentResult:
    """用 Umeyama 方法将估计点对齐到 GT 点。

    Args:
        source_points: Nx3 的估计轨迹点。
        target_points: Nx3 的 GT 轨迹点。
        with_scale: 为 True 时估计 Sim3 尺度，为 False 时执行 SE3 对齐。

    Returns:
        对齐旋转、平移、尺度和对齐后的点。

    Raises:
        ValueError: 点数少于 3，输入形状不匹配，或点集退化。
    """
    if source_points.shape != target_points.shape or source_points.ndim != 2 or source_points.shape[1] != 3:
        raise ValueError("source_points 和 target_points 需要是形状相同的 Nx3 数组")

    point_count = source_points.shape[0]
    if point_count < 3:
        raise ValueError("有效匹配点少于 3 个，无法计算轨迹对齐")

    source_mean = np.mean(source_points, axis=0)
    target_mean = np.mean(target_points, axis=0)
    source_centered = source_points - source_mean
    target_centered = target_points - target_mean
    source_variance = np.mean(np.sum(source_centered * source_centered, axis=1))
    if source_variance <= np.finfo(float).eps:
        raise ValueError("估计轨迹点集退化，无法计算轨迹对齐")

    covariance = (target_centered.T @ source_centered) / point_count
    u_matrix, singular_values, vt_matrix = np.linalg.svd(covariance)
    determinant_correction = np.ones(3)
    if np.linalg.det(u_matrix @ vt_matrix) < 0.0:
        determinant_correction[-1] = -1.0

    rotation = u_matrix @ np.diag(determinant_correction) @ vt_matrix
    scale = 1.0
    if with_scale:
        scale = float(np.sum(singular_values * determinant_correction) / source_variance)

    translation = target_mean - scale * rotation @ source_mean
    aligned_points = scale * (rotation @ source_points.T).T + translation

    return AlignmentResult(rotation=rotation, translation=translation, scale=scale, aligned_points=aligned_points)


def compute_ate_metrics(errors: np.ndarray) -> dict[str, float]:
    """计算 ATE 平移误差统计量。

    Args:
        errors: 每个匹配样本的平移误差模长。

    Returns:
        包含 rmse、均值、中位数、标准差、最小值和最大值的字典。

    Raises:
        ValueError: 误差数组为空。
    """
    if len(errors) == 0:
        raise ValueError("无法对空误差数组计算 ATE")

    return {
        "rmse_m": float(np.sqrt(np.mean(errors * errors))),
        "mean_m": float(np.mean(errors)),
        "median_m": float(np.median(errors)),
        "std_m": float(np.std(errors)),
        "min_m": float(np.min(errors)),
        "max_m": float(np.max(errors)),
    }


def extract_tf_message_gt_sample(message: object, gt_child_frame: str | None) -> tuple[float, list[float]] | None:
    """从 TFMessage 中提取一个指定 child frame 的 GT 样本。

    Args:
        message: 反序列化后的 tf2_msgs/msg/TFMessage。
        gt_child_frame: 目标 child_frame_id；None 时使用第一条 transform。

    Returns:
        匹配到的时间戳和平移；未匹配时返回 None。
    """
    for transform_stamped in message.transforms:
        if gt_child_frame and transform_stamped.child_frame_id != gt_child_frame:
            continue

        stamp = transform_stamped.header.stamp
        translation = transform_stamped.transform.translation
        timestamp = float(stamp.sec) + float(stamp.nanosec) * 1e-9
        position = [translation.x, translation.y, translation.z]
        return timestamp, position

    return None


def read_gt_from_bag(
    bag_path: Path | str, gt_topic: str, gt_child_frame: str | None = None
) -> tuple[np.ndarray, np.ndarray]:
    """从 rosbag2 中读取 TransformStamped 或 TFMessage GT topic。

    Args:
        bag_path: rosbag2 目录路径。
        gt_topic: GT topic 名称。
        gt_child_frame: TFMessage 中需要提取的 child_frame_id。

    Returns:
        GT 时间戳数组和 Nx3 平移数组。

    Raises:
        RuntimeError: ROS bag 依赖不可用、topic 不存在或消息类型不符合预期。
        ValueError: GT topic 中没有有效消息。
    """
    try:
        import rosbag2_py
        from rclpy.serialization import deserialize_message
        from rosidl_runtime_py.utilities import get_message
    except ImportError as exc:
        raise RuntimeError("读取 rosbag2 需要 ROS 2 Python 依赖：rosbag2_py、rclpy、rosidl_runtime_py") from exc

    storage_options = rosbag2_py.StorageOptions(uri=str(bag_path), storage_id="")
    converter_options = rosbag2_py.ConverterOptions(input_serialization_format="", output_serialization_format="")
    reader = rosbag2_py.SequentialReader()
    reader.open(storage_options, converter_options)

    topic_types = {topic_metadata.name: topic_metadata.type for topic_metadata in reader.get_all_topics_and_types()}
    if gt_topic not in topic_types:
        available_topics = ", ".join(sorted(topic_types))
        raise RuntimeError(f"rosbag2 中不存在 GT topic {gt_topic}；可用 topic：{available_topics}")

    gt_type = topic_types[gt_topic]
    supported_types = {"geometry_msgs/msg/TransformStamped", "tf2_msgs/msg/TFMessage"}
    if gt_type not in supported_types:
        raise RuntimeError(f"GT topic {gt_topic} 类型应为 TransformStamped 或 TFMessage，实际为 {gt_type}")

    message_type = get_message(gt_type)
    timestamps = []
    positions = []

    while reader.has_next():
        topic_name, serialized_data, _bag_time = reader.read_next()
        if topic_name != gt_topic:
            continue

        message = deserialize_message(serialized_data, message_type)
        if gt_type == "geometry_msgs/msg/TransformStamped":
            stamp = message.header.stamp
            translation = message.transform.translation
            timestamps.append(float(stamp.sec) + float(stamp.nanosec) * 1e-9)
            positions.append([translation.x, translation.y, translation.z])
        else:
            sample = extract_tf_message_gt_sample(message, gt_child_frame)
            if sample is None:
                continue
            timestamp, position = sample
            timestamps.append(timestamp)
            positions.append(position)

    if not timestamps:
        raise ValueError(f"GT topic {gt_topic} 中没有有效消息")

    return sort_and_deduplicate_gt_samples(timestamps, positions)


def read_transform_stamped_gt_from_bag(bag_path: Path | str, gt_topic: str) -> tuple[np.ndarray, np.ndarray]:
    """从 rosbag2 中读取 Vicon TransformStamped GT topic。

    Args:
        bag_path: rosbag2 目录路径。
        gt_topic: GT topic 名称。

    Returns:
        GT 时间戳数组和 Nx3 平移数组。
    """
    return read_gt_from_bag(bag_path, gt_topic, None)


def evaluate_ate(
    trajectory_path: Path | str,
    gt_times: np.ndarray,
    gt_positions: np.ndarray,
    alignment: str,
    estimate_frame: str,
    settings_path: Path | str | None,
) -> dict[str, object]:
    """计算估计轨迹相对 GT 的 ATE 指标。

    Args:
        trajectory_path: ORB_SLAM3 TUM 轨迹路径。
        gt_times: GT 时间戳。
        gt_positions: GT 平移。
        alignment: 对齐模式，可选 none、se3 或 sim3。
        estimate_frame: 估计轨迹坐标系，可选 camera 或 body。
        settings_path: body 模式读取 Tbc 所需的配置路径。

    Returns:
        包含样本数、时间范围、对齐信息和 ATE 指标的结果字典。

    Raises:
        ValueError: 参数非法或有效匹配不足。
    """
    trajectory = read_tum_trajectory(trajectory_path)
    estimate_positions = trajectory.positions

    if estimate_frame == "body":
        if settings_path is None:
            raise ValueError("--estimate-frame body 需要提供 --settings 以读取 Tbc")
        estimate_positions = convert_camera_to_body_positions(trajectory.positions, trajectory.quaternions, read_tbc(settings_path))
    elif estimate_frame != "camera":
        raise ValueError(f"未知 estimate_frame: {estimate_frame}")

    valid_times, matched_gt_positions, valid_mask = interpolate_gt_positions(trajectory.timestamps, gt_times, gt_positions)
    matched_estimate_positions = estimate_positions[valid_mask]
    if len(valid_times) < 3:
        raise ValueError(f"有效匹配点少于 3 个：{len(valid_times)}")

    if alignment == "none":
        aligned_positions = matched_estimate_positions
        scale = 1.0
    elif alignment in {"se3", "sim3"}:
        alignment_result = align_points_umeyama(
            matched_estimate_positions, matched_gt_positions, with_scale=(alignment == "sim3")
        )
        aligned_positions = alignment_result.aligned_points
        scale = alignment_result.scale
    else:
        raise ValueError(f"未知 alignment: {alignment}")

    errors = np.linalg.norm(aligned_positions - matched_gt_positions, axis=1)
    metrics = compute_ate_metrics(errors)
    result = {
        "samples": int(len(valid_times)),
        "time_start": float(valid_times[0]),
        "time_end": float(valid_times[-1]),
        "alignment": alignment,
        "estimate_frame": estimate_frame,
        "scale": float(scale),
    }
    result.update(metrics)

    return result


def build_arg_parser() -> argparse.ArgumentParser:
    """创建命令行参数解析器。

    Returns:
        配置好的 argparse 解析器。
    """
    parser = argparse.ArgumentParser(description="计算 ORB_SLAM3 TUM 轨迹相对 GT 的 ATE 平移误差")
    gt_source_group = parser.add_mutually_exclusive_group(required=True)
    gt_source_group.add_argument("--bag", help="rosbag2 目录路径")
    gt_source_group.add_argument("--gt-file", help="TUM/OpenLORIS groundtruth.txt 文件路径")
    parser.add_argument("--trajectory", default="KeyFrameTrajectory.txt", help="ORB_SLAM3 TUM 轨迹文件")
    parser.add_argument("--gt-topic", default="/vicon/firefly_sbx/firefly_sbx", help="GT TransformStamped 或 TFMessage topic")
    parser.add_argument("--gt-child-frame", help="TFMessage GT 中需要提取的 child_frame_id，例如 base_link")
    parser.add_argument("--settings", help="ORB_SLAM3 YAML 配置；body 模式用于读取 Tbc")
    parser.add_argument("--estimate-frame", choices=["camera", "body"], default="camera", help="估计轨迹所在坐标系")
    parser.add_argument("--alignment", choices=["none", "se3", "sim3"], default="se3", help="轨迹对齐模式")
    parser.add_argument("--output-json", help="可选 JSON 结果输出路径")
    return parser


def main(argv: list[str] | None = None) -> int:
    """命令行入口函数。

    Args:
        argv: 可选命令行参数列表，None 时读取 sys.argv。

    Returns:
        进程退出码，0 表示成功，非 0 表示失败。
    """
    parser = build_arg_parser()
    args = parser.parse_args(argv)

    try:
        if args.gt_file:
            gt_times, gt_positions = read_groundtruth_file(args.gt_file)
        else:
            gt_times, gt_positions = read_gt_from_bag(args.bag, args.gt_topic, args.gt_child_frame)

        result = evaluate_ate(
            trajectory_path=args.trajectory,
            gt_times=gt_times,
            gt_positions=gt_positions,
            alignment=args.alignment,
            estimate_frame=args.estimate_frame,
            settings_path=args.settings,
        )
    except Exception as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 1

    print(json.dumps(result, ensure_ascii=False, indent=2, sort_keys=True))
    if args.output_json:
        output_path = Path(args.output_json)
        output_path.write_text(json.dumps(result, ensure_ascii=False, indent=2, sort_keys=True) + "\n", encoding="utf-8")

    return 0


if __name__ == "__main__":
    sys.exit(main())
