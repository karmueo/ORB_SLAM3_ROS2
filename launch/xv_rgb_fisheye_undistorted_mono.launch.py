"""启动纯单目 ORB-SLAM3，并按需启动预配置的 RViz2。"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, Shutdown
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    """构造 XV 已校正图像纯单目 SLAM 与可选 RViz2 的 launch 描述。"""
    # ORB-SLAM3 词典文件路径。
    vocabulary_path = LaunchConfiguration("vocabulary_path")
    # 已校正针孔图像对应的纯单目配置文件路径。
    settings_path = LaunchConfiguration("settings_path")
    # XV SDK 发布的已校正 RGB 图像 topic。
    camera_topic = LaunchConfiguration("camera_topic")
    # 与原始校正图像对齐的静态二值特征掩膜路径。
    feature_mask_path = LaunchConfiguration("feature_mask_path")
    # ROS Path 最多保留的有效位姿数量。
    max_path_length = LaunchConfiguration("max_path_length")
    # Pangolin viewer 字符串开关。
    use_viewer = LaunchConfiguration("use_viewer")
    # 是否启动 RViz2。
    use_rviz = LaunchConfiguration("use_rviz")
    # RViz2 配置文件路径。
    rviz_config_path = LaunchConfiguration("rviz_config_path")

    # 纯单目节点直接消费 SDK 已校正图像，不在消费端重复执行去畸变。
    orbslam_node = Node(
        package="orbslam3",
        executable="mono",
        name="orbslam3_monocular_undistorted",
        output="screen",
        emulate_tty=True,
        arguments=[vocabulary_path, settings_path, use_viewer],
        parameters=[
            {
                "feature_mask_path": feature_mask_path,
                "max_path_length": ParameterValue(
                    max_path_length, value_type=int
                ),
            }
        ],
        remappings=[("camera", camera_topic)],
        on_exit=Shutdown(reason="ORB-SLAM3 纯单目节点已退出"),
    )

    # RViz2 默认关闭，启用后加载定位显示并跟随 camera_topic 订阅视频。
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="orbslam3_monocular_rviz",
        output="screen",
        arguments=["-d", rviz_config_path],
        remappings=[("/orbslam3/input_image", camera_topic)],
        condition=IfCondition(use_rviz),
    )

    return LaunchDescription(
        [
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
                        "monocular",
                        "XV_RGB_Fisheye_undistorted.yaml",
                    ]
                ),
                description="XV 已校正 RGB 图像的纯单目配置路径。",
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
                "feature_mask_path",
                default_value=PathJoinSubstitution(
                    [
                        FindPackageShare("orbslam3"),
                        "config",
                        "masks",
                        "XV_RGB_Fisheye_gripper_mask.png",
                    ]
                ),
                description="1280x1280 二值特征掩膜路径；白色允许、黑色排除。",
            ),
            DeclareLaunchArgument(
                "max_path_length",
                default_value="10000",
                description="camera_path 最大位姿数；0 表示无限累计。",
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
                        "xv_rgb_fisheye_undistorted_mono.rviz",
                    ]
                ),
                description="纯单目定位 RViz2 配置文件路径。",
            ),
            orbslam_node,
            rviz_node,
        ]
    )
