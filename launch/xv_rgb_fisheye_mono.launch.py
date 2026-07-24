"""启动可配置的 XV RGB 鱼眼纯单目 SLAM，并按需启动 RViz2。"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, Shutdown
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    """构造支持原始或校正图像配置的纯单目 SLAM 与 RViz2 启动描述。"""
    # ORB-SLAM3 词典文件路径。
    vocabulary_path = LaunchConfiguration("vocabulary_path")
    # 当前图像模型对应的 ORB-SLAM3 配置文件路径。
    settings_path = LaunchConfiguration("settings_path")
    # mono 节点订阅的图像 topic。
    camera_topic = LaunchConfiguration("camera_topic")
    # 与输入图像逐像素对齐的静态特征掩膜路径。
    feature_mask_path = LaunchConfiguration("feature_mask_path")
    # ROS Path 最多保留的有效位姿数量。
    max_path_length = LaunchConfiguration("max_path_length")
    # 是否发布 ROS 2 位姿、轨迹和 TF。
    publish_ros_pose = LaunchConfiguration("publish_ros_pose")
    # 是否启用 Pangolin Viewer。
    use_viewer = LaunchConfiguration("use_viewer")
    # 是否启动 RViz2。
    use_rviz = LaunchConfiguration("use_rviz")
    # RViz2 显示配置文件路径。
    rviz_config_path = LaunchConfiguration("rviz_config_path")

    # mono 使用所选相机配置和 mask 跟踪同一个 camera_topic。
    orbslam_node = Node(
        package="orbslam3",
        executable="mono",
        name="orbslam3_monocular",
        output="screen",
        emulate_tty=True,
        arguments=[vocabulary_path, settings_path, use_viewer],
        parameters=[
            {
                "feature_mask_path": feature_mask_path,
                "max_path_length": ParameterValue(
                    max_path_length, value_type=int
                ),
                "publish_ros_pose": ParameterValue(
                    publish_ros_pose, value_type=bool
                ),
            }
        ],
        remappings=[("camera", camera_topic)],
        on_exit=Shutdown(reason="ORB-SLAM3 纯单目节点已退出"),
    )

    # RViz2 显示输入视频、camera_pose、camera_path 和 TF。
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
                        "monocular-inertial",
                        "XV_RGB_Fisheye_calibrated.yaml",
                    ]
                ),
                description="输入图像模型对应的 ORB-SLAM3 mono 配置路径。",
            ),
            DeclareLaunchArgument(
                "camera_topic",
                default_value=(
                    "/xv_sdk/SN250801DR48FB26001253/rgb/image"
                ),
                description="mono 和 RViz2 共同使用的输入图像 topic。",
            ),
            DeclareLaunchArgument(
                "feature_mask_path",
                default_value=PathJoinSubstitution(
                    [
                        FindPackageShare("orbslam3"),
                        "config",
                        "masks",
                        "fisheye_mask.png",
                    ]
                ),
                description="与输入图像对齐的特征 mask；空字符串表示禁用。",
            ),
            DeclareLaunchArgument(
                "max_path_length",
                default_value="10000",
                description="camera_path 最大位姿数；0 表示无限累计。",
            ),
            DeclareLaunchArgument(
                "publish_ros_pose",
                default_value="true",
                description="是否发布相机 Pose、Path、动态 TF 和静态 TF。",
            ),
            DeclareLaunchArgument(
                "use_viewer",
                default_value="false",
                description="是否启用 Pangolin Viewer。",
            ),
            DeclareLaunchArgument(
                "use_rviz",
                default_value="false",
                description="是否启动显示视频、位姿和轨迹的 RViz2。",
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
                description="纯单目视频、位姿、轨迹和 TF 的 RViz2 配置路径。",
            ),
            orbslam_node,
            rviz_node,
        ]
    )
