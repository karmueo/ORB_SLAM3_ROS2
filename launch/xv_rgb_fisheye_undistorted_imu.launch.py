"""联合启动 XV SDK 校正 RGB 鱼眼流与 ORB-SLAM3 单目惯性节点。"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    """构造设备侧去畸变和 ORB-SLAM3 联合运行的 launch 描述。"""
    # XV SDK 使用的 Kalibr camchain 文件。
    calibration_path = LaunchConfiguration("rgb_fisheye_calibration_path")
    # XV SDK 校正图像发布开关，本链路默认启用。
    undistort_enable = LaunchConfiguration("rgb_fisheye_undistort_enable")
    # ORB-SLAM3 词典文件。
    vocabulary_path = LaunchConfiguration("vocabulary_path")
    # 已校正针孔图像对应的 ORB-SLAM3 配置文件。
    settings_path = LaunchConfiguration("settings_path")
    # SDK 发布的已校正 RGB 图像 topic。
    camera_topic = LaunchConfiguration("camera_topic")
    # SDK 发布的 IMU topic。
    imu_topic = LaunchConfiguration("imu_topic")
    # Pangolin viewer 字符串开关。
    use_viewer = LaunchConfiguration("use_viewer")

    # 复用 XV SDK 自带 launch，确保动态库环境和设备参数保持一致。
    xv_sdk_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("xv_sdk_ros2"), "launch", "xv_sdk_node_launch.py"]
            )
        ),
        launch_arguments={
            "rgb_fisheye_undistort_enable": undistort_enable,
            "rgb_fisheye_calibration_path": calibration_path,
        }.items(),
    )

    # 单目惯性节点直接消费 SDK 已校正图像，配置中使用 PinHole 且不声明畸变参数。
    orbslam_node = Node(
        package="orbslam3",
        executable="monocular-inertial",
        name="orbslam3_monocular_inertial_undistorted",
        output="screen",
        emulate_tty=True,
        arguments=[vocabulary_path, settings_path, use_viewer],
        remappings=[("camera", camera_topic), ("imu", imu_topic)],
        on_exit=Shutdown(reason="ORB-SLAM3 单目惯性节点已退出"),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "rgb_fisheye_undistort_enable",
                default_value="true",
                description="是否启用 XV SDK RGB 鱼眼去畸变输出。",
            ),
            DeclareLaunchArgument(
                "rgb_fisheye_calibration_path",
                default_value=PathJoinSubstitution(
                    [
                        FindPackageShare("xv_sdk_ros2"),
                        "config",
                        "kalibr_data-camchain-imucam.yaml",
                    ]
                ),
                description="XV SDK 用于生成去畸变图像的 Kalibr camchain YAML。",
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
                description="XV SDK 已校正 RGB 图像 topic。",
            ),
            DeclareLaunchArgument(
                "imu_topic",
                default_value="/xv_sdk/SN250801DR48FB26001253/imu",
                description="XV SDK IMU topic。",
            ),
            DeclareLaunchArgument(
                "use_viewer",
                default_value="true",
                description="是否启用 Pangolin viewer。",
            ),
            xv_sdk_launch,
            orbslam_node,
        ]
    )
