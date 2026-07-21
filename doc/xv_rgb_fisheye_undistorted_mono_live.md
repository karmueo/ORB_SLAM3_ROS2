# XV 校正 RGB 鱼眼纯单目实时定位说明

本文说明如何使用 XV SDK 已完成畸变校正的 RGB 鱼眼视频帧运行 ORB-SLAM3 纯单目定位。目标输入为：

```text
/xv_sdk/SN250801DR48FB26001253/rgb_fisheye_undistorted/image
```

完整数据流如下：

```text
/rgb/image（原始 equidistant 鱼眼）
  -> xv_sdk_ros2 cv::fisheye::remap
  -> /rgb_fisheye_undistorted/image（PinHole、零畸变、rgb8）
  -> orbslam3 mono
  -> /orbslam3/camera_pose
  -> /orbslam3/camera_path
```

对应配置为：

```text
config/monocular/XV_RGB_Fisheye_undistorted.yaml
```

配置使用 `Camera.type: "PinHole"` 并省略畸变参数。图像校正由 XV SDK 完成，ORB-SLAM3 直接对校正后的像素执行纯单目跟踪。

## 1. 构建

先按实际环境设置工作空间和 ORB-SLAM3 源码路径：

```bash
export XV_WS=/path/to/xv_ws
export ORBSLAM3_ROS2_WS=/path/to/ORB_SLAM3_ROS2
export ORB_SLAM3_ROOT=/path/to/ORB_SLAM3
```

先构建并加载 XV SDK 工作空间：

```bash
cd "$XV_WS"
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select xv_sdk_ros2
source install/local_setup.bash
```

再构建 ORB-SLAM3 ROS 2 wrapper：

```bash
cd "$ORBSLAM3_ROS2_WS"
source /opt/ros/jazzy/setup.bash
source "$XV_WS/install/local_setup.bash"

colcon build --symlink-install --packages-select orbslam3 \
  --cmake-args \
  -DPython3_EXECUTABLE=/usr/bin/python3 \
  -DOpenCV_DIR=/usr/lib/x86_64-linux-gnu/cmake/opencv4 \
  -DORB_SLAM3_ROOT_DIR="$ORB_SLAM3_ROOT"
```

## 2. 启动 XV SDK 校正图像

终端 1 启用 RGB 鱼眼去畸变输出：

```bash
cd "$XV_WS"
source /opt/ros/jazzy/setup.bash
source install/local_setup.bash

ros2 launch xv_sdk_ros2 xv_sdk_node_launch.py \
  rgb_fisheye_undistort_enable:=true \
  rgb_fisheye_calibration_path:=src/xv_sdk_ros2/config/kalibr_data-camchain-imucam.yaml
```

确认目标图像持续发布：

```bash
ros2 topic hz /xv_sdk/SN250801DR48FB26001253/rgb_fisheye_undistorted/image
ros2 topic echo \
  /xv_sdk/SN250801DR48FB26001253/rgb_fisheye_undistorted/camera_info \
  --once --no-arr
```

相机信息应满足：

- 图像宽高为 `1280 × 1280`；
- 图像编码为 `rgb8`；
- `distortion_model` 为 `plumb_bob`；
- `d` 中的畸变参数均为 `0.0`；
- `k` 中的 `fx/fy/cx/cy` 与纯单目 YAML 配置一致。

如果相机信息与上述参数不同，需要先同步更新 `XV_RGB_Fisheye_undistorted.yaml`，避免使用错误内参定位。

## 3. 启动纯单目 SLAM

终端 2 加载环境并启动默认链路：

```bash
source /opt/ros/jazzy/setup.bash
source "$ORBSLAM3_ROS2_WS/install/local_setup.bash"

ros2 launch orbslam3 xv_rgb_fisheye_undistorted_mono.launch.py
```

远程终端或无显示环境关闭 Pangolin viewer：

```bash
ros2 launch orbslam3 xv_rgb_fisheye_undistorted_mono.launch.py use_viewer:=false
```

设备序列号变化时覆盖图像 topic：

```bash
ros2 launch orbslam3 xv_rgb_fisheye_undistorted_mono.launch.py \
  camera_topic:=/xv_sdk/实际序列号/rgb_fisheye_undistorted/image
```

### 3.1 配置 Current Frame 显示分辨率

默认配置将 ORB-SLAM3 的 `Current Frame` 视频显示区域保持为约 `960 × 960`：

```yaml
Camera.newWidth: 960
Camera.newHeight: 960
Viewer.imageViewScale: 1.0
```

`Camera.newWidth/newHeight` 决定送入特征提取和跟踪的图像分辨率，当前保持为
`960 × 960`。`Viewer.imageViewScale` 只在 ORB-SLAM3 完成跟踪和特征点绘制后缩放
显示帧，计算关系为：

```text
显示视频宽高 = 算法处理宽高 × Viewer.imageViewScale
960 × 1.0 = 960
```

如需其他等比例显示尺寸，可以复制
`config/monocular/XV_RGB_Fisheye_undistorted.yaml`，只修改
`Viewer.imageViewScale`，然后通过 `settings_path` 选择该配置。例如显示区域约为
`480 × 480` 时设置 `Viewer.imageViewScale: 0.5`：

```bash
ros2 launch orbslam3 xv_rgb_fisheye_undistorted_mono.launch.py \
  settings_path:=/绝对路径/XV_RGB_Fisheye_undistorted_display_480.yaml
```

不要为调整显示窗口而修改 `Camera.width/height`、`Camera.newWidth/newHeight` 或
`Camera.imageScale`，这些字段会影响相机模型或算法实际处理的图像。窗口底部还会附加
ORB-SLAM3 跟踪状态栏，因此桌面窗口的总高度会略大于上述视频区域高度。

### 3.2 排除夹爪区域的特征点

launch 默认加载以下静态二值掩膜：

```text
config/masks/XV_RGB_Fisheye_gripper_mask.png
```

该模板尺寸为 `1280 × 1280`，与 XV SDK 发布的校正图像逐像素对齐。像素含义如下：

- 白色 `255`：允许提取 ORB 特征；
- 黑色 `0`：排除 ORB 特征。

仓库中的默认模板为全白图，因此初始行为与未启用掩膜一致。制作实际夹爪 mask 时，
应采集夹爪最小开度、最大开度和常用姿态的校正图像，把所有姿态下夹爪轮廓的并集涂黑，
其余区域保持白色。建议把黑区适当扩展到夹爪轮廓外侧，为 ORB 特征方向和描述子采样
预留安全边界。

使用自定义 mask：

```bash
ros2 launch orbslam3 xv_rgb_fisheye_undistorted_mono.launch.py \
  feature_mask_path:=/绝对路径/gripper_mask.png
```

临时禁用 mask：

```bash
ros2 launch orbslam3 xv_rgb_fisheye_undistorted_mono.launch.py \
  feature_mask_path:=""
```

节点在送入跟踪前创建图像副本，并把 mask 黑区对应的像素清零；ROS 输入消息和 RViz
显示的原始图像不会被修改。处理后的图像通过 ORB-SLAM3 官方
`TrackMonocular(image, timestamp)` 接口跟踪，因此无需额外的 ORB-SLAM3 API 补丁。
ORB-SLAM3 随后按配置把图像从 `1280 × 1280` 缩放到 `960 × 960`，
`Camera.newWidth/newHeight`、内参和算法处理分辨率保持不变。

启动日志会打印 mask 路径、尺寸和排除比例。以下情况会停止处理并给出错误：

- 文件无法读取；
- mask 与首帧输入图像尺寸不一致；
- mask 为全黑图；
- mask 无法归一化为单通道 8 位图像。

相机输出分辨率、裁剪方式或去畸变映射发生变化后，需要重新生成逐像素对齐的 mask。

也可以直接运行 `mono`：

```bash
PKG="$(ros2 pkg prefix orbslam3)/share/orbslam3"
ros2 run orbslam3 mono \
  "$PKG/vocabulary/ORBvoc.txt" \
  "$PKG/config/monocular/XV_RGB_Fisheye_undistorted.yaml" \
  false \
  --ros-args \
  -p feature_mask_path:="$PKG/config/masks/XV_RGB_Fisheye_gripper_mask.png" \
  -p max_path_length:=10000 \
  -r camera:=/xv_sdk/SN250801DR48FB26001253/rgb_fisheye_undistorted/image
```

当前数据流和配置中的 `Camera.fps` 均为 60 Hz。纯单目链路保留连续输入帧，不在 ROS wrapper 中主动限帧，以维持快速运动时的帧间重叠。

## 4. 检查定位输出

检查实时相机位姿和轨迹：

```bash
ros2 topic hz /orbslam3/camera_pose
ros2 topic echo /orbslam3/camera_pose --once
ros2 topic echo /orbslam3/camera_path --once --no-arr
ros2 run tf2_ros tf2_echo map camera_link
ros2 run tf2_ros tf2_echo camera_link camera_optical_frame
```

输出含义：

- `/orbslam3/camera_pose`：当前 `camera_link` 在 `map` 中的位姿，采用 x 前、y 左、z 上约定；
- `/orbslam3/camera_path`：最多保留 `max_path_length` 个有效 `camera_link` 位姿；
- `map -> camera_link`：与当前位姿同时间戳、同变换的动态 TF；
- `camera_link -> camera_optical_frame`：零平移的静态 TF，将机体系转换为 x 右、y 下、z 前的光学系；
- `KeyFrameTrajectory.txt`：节点正常退出时保存的 TUM 格式关键帧轨迹。

位姿、轨迹和 TF 只在 ORB-SLAM3 状态为 `OK` 或 `OK_KLT` 时发布。进入 `LOST`
后会暂停发布；恢复有效跟踪时清空旧 Path，再从当前位姿重新累计。默认
`max_path_length:=10000`，设置为 `0` 时允许进程内无限累计，负数会导致节点启动失败。

### 4.1 在 RViz2 中查看

关闭 Pangolin 并启动包内预配置 RViz2：

```bash
ros2 launch orbslam3 xv_rgb_fisheye_undistorted_mono.launch.py \
  use_viewer:=false \
  use_rviz:=true \
  max_path_length:=10000
```

RViz2 的 Fixed Frame 默认为 `map`，并自动显示 Grid、TF、Camera Pose 和 Camera Path。
Camera Image 默认订阅与 SLAM 相同的 `camera_topic`，launch 会自动完成话题重映射；覆盖
`camera_topic:=/其他图像话题` 时，SLAM 和 RViz 视频会一起切换。
`map` 和 `camera_link` 采用 ROS 机体坐标约定：x 向前、y 向左、z 向上。RViz
坐标轴颜色固定为红色 x、绿色 y、蓝色 z，Grid 位于 `map` 的 XY 平面。
`camera_optical_frame` 作为 `camera_link` 的静态子坐标系保留，采用 x 向右、y 向下、
z 向前的图像坐标约定。也可以通过
`rviz_config_path:=/绝对路径/custom.rviz` 加载自定义显示配置。

`map` 的方向以 ORB 地图首个参考相机为基准，因此初始相机前方对应 `map +X`，
初始相机上方对应 `map +Z`。纯单目没有重力观测，`map +Z` 不保证与真实重力反方向严格对齐。

纯单目系统缺少深度、双目基线和 IMU 尺度约束。上述位置和路径只有任意尺度，可用于相对运动、回环和跟踪稳定性观察，不能直接当作米制定位结果。

## 5. 成功判据

一次可接受的实时 smoke test 应满足：

- XV SDK 持续发布校正图像；
- `ros2 node info /orbslam3_monocular_undistorted` 显示订阅目标图像 topic；
- ORB-SLAM3 日志出现地图创建，且没有持续打印 `Reset map`；
- `/orbslam3/camera_pose` 持续发布；
- `/orbslam3/camera_path` 中的 pose 数量随运动增长且不超过配置上限；
- `tf2_echo map camera_link` 持续输出同时间戳动态变换；
- `tf2_echo camera_link camera_optical_frame` 输出稳定的零平移固定旋转；
- 启用 `use_rviz:=true` 后能看到相机坐标系和轨迹；
- 退出节点后生成非空 `KeyFrameTrajectory.txt`。

轨迹文件写在启动节点时的当前目录。退出后执行：

```bash
test -s KeyFrameTrajectory.txt
wc -l KeyFrameTrajectory.txt
head KeyFrameTrajectory.txt
```

## 6. 关键约束

- 校正图像只配合 `PinHole` 且无畸变参数的纯单目配置使用；
- 原始鱼眼图像继续配合 `KannalaBrandt8 + k1/k2/k3/k4` 配置使用；
- 本链路不订阅 IMU，也不使用任何 `IMU.*` 参数；
- launch 默认只启动 ORB-SLAM3；`use_rviz:=true` 时额外启动 RViz2，XV SDK 去畸变
  发布端始终需要提前启动；
- `camera_path` 默认保留最近 10000 个有效位姿；设置 `max_path_length:=0` 后消息会随
  运行时间持续增大。
