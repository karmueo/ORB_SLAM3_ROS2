"""按需启动 XV SDK，并运行校正 RGB 鱼眼单目惯性 ORB-SLAM3 与 RViz2。"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    Shutdown,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def _create_xv_sdk_launch(context):
    """仅在实时模式中解析并启动 XV SDK，避免离线模式依赖设备工作空间。"""
    # 当前运行是否请求启动 XV SDK。
    start_xv_sdk = LaunchConfiguration("start_xv_sdk").perform(context)
    if start_xv_sdk.lower() not in {"1", "true", "yes", "on"}:
        return []

    # 用户显式设置的标定路径；空值表示使用 XV SDK 安装目录中的默认文件。
    calibration_path = LaunchConfiguration(
        "rgb_fisheye_calibration_path"
    ).perform(context)
    if not calibration_path:
        calibration_path = os.path.join(
            get_package_share_directory("xv_sdk_ros2"),
            "config",
            "kalibr_data-camchain-imucam.yaml",
        )

    # XV SDK 的主 launch 文件路径。
    xv_sdk_launch_path = os.path.join(
        get_package_share_directory("xv_sdk_ros2"),
        "launch",
        "xv_sdk_node_launch.py",
    )
    # XV SDK 校正图像发布开关。
    undistort_enable = LaunchConfiguration(
        "rgb_fisheye_undistort_enable"
    ).perform(context)
    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(xv_sdk_launch_path),
            launch_arguments={
                "rgb_fisheye_undistort_enable": undistort_enable,
                "rgb_fisheye_calibration_path": calibration_path,
            }.items(),
        )
    ]


def generate_launch_description() -> LaunchDescription:
    """构造设备实时或 rosbag 回放使用的单目惯性 SLAM launch 描述。"""
    # ORB-SLAM3 词典文件。
    vocabulary_path = LaunchConfiguration("vocabulary_path")
    # 已校正针孔图像对应的 ORB-SLAM3 配置文件。
    settings_path = LaunchConfiguration("settings_path")
    # SDK 或 rosbag 发布的已校正 RGB 图像 topic。
    camera_topic = LaunchConfiguration("camera_topic")
    # SDK 或 rosbag 发布的 IMU topic。
    imu_topic = LaunchConfiguration("imu_topic")
    # 与原始校正图像逐像素对齐的静态二值特征掩膜路径。
    feature_mask_path = LaunchConfiguration("feature_mask_path")
    # ROS Path 最多保留的有效机体位姿数量。
    max_path_length = LaunchConfiguration("max_path_length")
    # 是否在节点退出时保存关键帧轨迹。
    save_keyframe_trajectory = LaunchConfiguration("save_keyframe_trajectory")
    # Pangolin viewer 字符串开关。
    use_viewer = LaunchConfiguration("use_viewer")
    # 是否启动 RViz2。
    use_rviz = LaunchConfiguration("use_rviz")
    # RViz2 配置文件路径。
    rviz_config_path = LaunchConfiguration("rviz_config_path")

    # 运行时按 start_xv_sdk 创建设备 launch，离线模式不会解析 XV 包。
    xv_sdk_launch = OpaqueFunction(function=_create_xv_sdk_launch)

    # 单目惯性节点消费已经去畸变的 RGB 图像和同一时基的 IMU。
    orbslam_node = Node(
        package="orbslam3",
        executable="monocular-inertial",
        name="orbslam3_monocular_inertial_undistorted",
        output="screen",
        emulate_tty=True,
        arguments=[vocabulary_path, settings_path, use_viewer],
        parameters=[
            {
                "feature_mask_path": feature_mask_path,
                "max_path_length": ParameterValue(
                    max_path_length, value_type=int
                ),
                "save_keyframe_trajectory": ParameterValue(
                    save_keyframe_trajectory, value_type=bool
                ),
            }
        ],
        remappings=[("camera", camera_topic), ("imu", imu_topic)],
        on_exit=Shutdown(reason="ORB-SLAM3 单目惯性节点已退出"),
    )

    # RViz2 默认关闭；启用后显示机体位姿、轨迹、TF 和输入图像。
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="orbslam3_monocular_inertial_rviz",
        output="screen",
        arguments=["-d", rviz_config_path],
        remappings=[("/orbslam3/input_image", camera_topic)],
        condition=IfCondition(use_rviz),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "start_xv_sdk",
                default_value="true",
                description="是否启动 XV SDK；rosbag 回放时设置为 false。",
            ),
            DeclareLaunchArgument(
                "rgb_fisheye_undistort_enable",
                default_value="true",
                description="是否启用 XV SDK RGB 鱼眼去畸变输出。",
            ),
            DeclareLaunchArgument(
                "rgb_fisheye_calibration_path",
                default_value="",
                description=(
                    "XV SDK 去畸变使用的 Kalibr camchain YAML；"
                    "空值使用 xv_sdk_ros2 安装目录中的默认文件。"
                ),
            ),
            DeclareLaunchArgument(
                "vocabulary_path",
                default_value=PathJoinSubstitution(
                    [FindPackageShare("orbslam3"), "vocabulary", "ORBvoc.txt"]
                ),
                description="ORB-SLAM3 词典路径。",
            ),
            DeclareLaunchArgument(
                "settings_path",
                default_value=PathJoinSubstitution(
                    [
                        FindPackageShare("orbslam3"),
                        "config",
                        "monocular-inertial",
                        "XV_RGB_Fisheye_undistorted.yaml",
                    ]
                ),
                description="已校正 RGB 鱼眼单目惯性配置路径。",
            ),
            DeclareLaunchArgument(
                "camera_topic",
                default_value=(
                    "/xv_sdk/SN250801DR48FB26001253/"
                    "rgb_fisheye_undistorted/image"
                ),
                description="XV SDK 或 rosbag 中的已校正 RGB 图像 topic。",
            ),
            DeclareLaunchArgument(
                "imu_topic",
                default_value="/xv_sdk/SN250801DR48FB26001253/imu",
                description="XV SDK 或 rosbag 中的 IMU topic。",
            ),
            DeclareLaunchArgument(
                "feature_mask_path",
                default_value=PathJoinSubstitution(
                    [
                        FindPackageShare("orbslam3"),
                        "config",
                        "masks",
                        "mask.png",
                    ]
                ),
                description="1280x1280 二值特征掩膜路径；白色允许、黑色排除。",
            ),
            DeclareLaunchArgument(
                "max_path_length",
                default_value="10000",
                description="body path 最大位姿数；0 表示无限累计。",
            ),
            DeclareLaunchArgument(
                "save_keyframe_trajectory",
                default_value="false",
                description="退出时是否在当前工作目录保存 KeyFrameTrajectory.txt。",
            ),
            DeclareLaunchArgument(
                "use_viewer",
                default_value="true",
                description="是否启用 Pangolin viewer。",
            ),
            DeclareLaunchArgument(
                "use_rviz",
                default_value="false",
                description="是否启动预配置的 RViz2。",
            ),
            DeclareLaunchArgument(
                "rviz_config_path",
                default_value=PathJoinSubstitution(
                    [
                        FindPackageShare("orbslam3"),
                        "rviz",
                        "xv_rgb_fisheye_undistorted_imu.rviz",
                    ]
                ),
                description="单目惯性定位 RViz2 配置文件路径。",
            ),
            xv_sdk_launch,
            orbslam_node,
            rviz_node,
        ]
    )
