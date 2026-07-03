# OpenLORIS Cafe1-1 单目惯性跑通教程

本文记录在本 ROS 2 wrapper 中使用 ORB_SLAM3 `IMU_MONOCULAR` 跑通 OpenLORIS Cafe1-1 的最小链路。当前阶段只使用 T265 左鱼眼图像和 T265 IMU：

- `/t265/fisheye1/image_raw`
- `/t265/imu`

本阶段不订阅、不处理 D400 深度或彩色图，包括 `/d400/aligned_depth_to_color/image_raw` 和 `/d400/color/image_raw`。

## 1. 环境检查

确认 ROS 2、OpenCV 和 ORB_SLAM3 路径与本机一致：

```bash
ORB_SLAM3_ROOT=/path/to/ORB_SLAM3
source /opt/ros/jazzy/setup.bash
test -d "$ORB_SLAM3_ROOT"
test -f /usr/lib/x86_64-linux-gnu/cmake/opencv4/OpenCVConfig.cmake
```

确认 OpenLORIS bag 至少包含图像和合并后的 IMU：

```bash
BAG=/path/to/OpenLORIS-Scene/cafe1-1_2_ros2/cafe1-1_vins
ros2 bag info "$BAG"
```

期望看到：

- `/t265/fisheye1/image_raw`
- `/t265/imu`

如果只有 `/t265/accel/sample` 和 `/t265/gyro/sample`，需要先生成合并后的 `/t265/imu` bag。

## 2. 构建

在 ROS 2 工作空间根目录执行：

```bash
source /opt/ros/jazzy/setup.bash
ORB_SLAM3_ROOT=/path/to/ORB_SLAM3
colcon build --symlink-install --packages-select orbslam3 \
  --cmake-args \
  -DPython3_EXECUTABLE=/usr/bin/python3 \
  -DOpenCV_DIR=/usr/lib/x86_64-linux-gnu/cmake/opencv4 \
  -DORB_SLAM3_ROOT_DIR="$ORB_SLAM3_ROOT"
```

构建完成后加载工作空间：

```bash
source install/local_setup.bash
ros2 pkg executables orbslam3
```

输出中应包含：

```text
orbslam3 monocular-inertial
```

## 3. 词典准备

运行时需要解压后的 ORB 词典：

```bash
PKG=/path/to/ORB_SLAM3_ROS2
ls "$PKG/vocabulary/ORBvoc.txt"
```

如果只有 `ORBvoc.txt.tar.gz`，先在 `vocabulary/` 目录解压。

## 4. 启动单目惯性节点

终端 1：

```bash
source /opt/ros/jazzy/setup.bash
source install/local_setup.bash

PKG=/path/to/ORB_SLAM3_ROS2
ros2 run orbslam3 monocular-inertial \
  "$PKG/vocabulary/ORBvoc.txt" \
  "$PKG/config/monocular-inertial/OpenLORIS_Cafe_T265.yaml" \
  false \
  --ros-args \
  -r camera:=/t265/fisheye1/image_raw \
  -r imu:=/t265/imu
```

第三个参数 `false` 会关闭 Pangolin viewer，适合无显示或远程终端环境。需要可视化时改成 `true`。

## 5. 播放 bag

终端 2：

```bash
source /opt/ros/jazzy/setup.bash
BAG=/path/to/OpenLORIS-Scene/cafe1-1_2_ros2/cafe1-1_vins
ros2 bag play "$BAG" \
  --topics /t265/fisheye1/image_raw /t265/imu
```

节点启动后应打印 `Monocular-Inertial`，播放 bag 后持续处理图像和 IMU，退出时生成：

```text
KeyFrameTrajectory.txt
```

## 6. 轨迹检查

停止节点后确认轨迹文件非空：

```bash
test -s KeyFrameTrajectory.txt
wc -l KeyFrameTrajectory.txt
head KeyFrameTrajectory.txt
```

单目惯性初始化需要足够运动激励。若轨迹为空或很短，优先检查 bag 是否包含连续 IMU、图像时间戳是否单调、以及启动后是否播放了足够长的运动片段。

## 7. 轨迹误差测评

节点生成 `KeyFrameTrajectory.txt` 后，可以用离线测评命令读取同一个 rosbag2 中的 OpenLORIS `/gt`，并计算 ATE 平移误差：

```bash
source /opt/ros/jazzy/setup.bash
source install/local_setup.bash

PKG=/path/to/ORB_SLAM3_ROS2
BAG=/path/to/OpenLORIS-Scene/cafe1-1_2_ros2/cafe1-1_vins
ros2 run orbslam3 evaluate_trajectory \
  --bag "$BAG" \
  --trajectory KeyFrameTrajectory.txt \
  --gt-topic /gt \
  --gt-child-frame base_link \
  --settings "$PKG/config/monocular-inertial/OpenLORIS_Cafe_T265.yaml" \
  --estimate-frame body \
  --alignment se3 \
  --output-json cafe1_1_mono_inertial_ate.json
```

如果本机只保留了 OpenLORIS groundtruth 文本文件，也可以直接读取 TUM 格式真值：

```bash
source /opt/ros/jazzy/setup.bash
source install/local_setup.bash

PKG=/path/to/ORB_SLAM3_ROS2
GT=/path/to/OpenLORIS-Scene/groundtruth/per-sequence/cafe1-1/groundtruth.txt
ros2 run orbslam3 evaluate_trajectory \
  --gt-file "$GT" \
  --trajectory KeyFrameTrajectory.txt \
  --settings "$PKG/config/monocular-inertial/OpenLORIS_Cafe_T265.yaml" \
  --estimate-frame body \
  --alignment se3 \
  --output-json cafe1_1_mono_inertial_ate.json
```

参数说明：

- `--bag`：rosbag2 目录；OpenLORIS Cafe1-1 中 `/gt` 通常是 `tf2_msgs/msg/TFMessage`。
- `--gt-file`：备用 GT 输入源，读取 `groundtruth/per-sequence/cafe1-1/groundtruth.txt` 这类 TUM/OpenLORIS 文本文件；`--bag` 和 `--gt-file` 二选一。
- `--gt-topic`：bag 中的 GT topic；OpenLORIS 使用 `/gt`，Vicon bag 仍可使用 TransformStamped topic。
- `--gt-child-frame`：从 TFMessage 中选择目标坐标系；Cafe1-1 使用 `base_link`，对应 `gt_map -> base_link`。
- `--settings`：ORB_SLAM3 YAML 配置；`--estimate-frame body` 时用于读取 `IMU.T_b_c1`。
- `--estimate-frame body`：把 ORB_SLAM3 导出的 `Twc` 相机轨迹转换为 `Twb = Twc * inverse(T_body_camera)`，再与 OpenLORIS `base_link` GT 比较。
- `--alignment se3`：单目惯性轨迹具有真实尺度时使用刚体对齐。`sim3` 会额外估计尺度，适合作为尺度异常诊断；如果 `sim3` 的 `scale` 明显偏离 `1.0`，优先检查初始化、IMU 配置和外参。
- `--output-json`：可选，写出机器可读 JSON 结果。

输出字段包含 `samples`、`time_start`、`time_end`、`alignment`、`estimate_frame`、`scale`，以及 ATE 的 `rmse_m`、`mean_m`、`median_m`、`std_m`、`min_m`、`max_m`。其中 `samples` 是落在 GT 时间范围内并参与对齐和误差计算的估计轨迹点数量；`rmse_m` 是 ATE 平移均方根误差，单位为米；`scale` 在 `se3` 下固定为 `1.0`，在 `sim3` 下表示额外估计出的轨迹尺度。

通常优先关注 `rmse_m` 和 `samples`：`rmse_m` 越小表示轨迹整体越接近 OpenLORIS GT；`samples` 过少时，结果只代表很短片段。`body` 坐标系测评会把估计轨迹转换到 T265 IMU/body 坐标系，与 `/gt` 的 `base_link` 含义对齐。

## 8. 常见问题

### 找不到可执行

重新 source 工作空间：

```bash
source install/local_setup.bash
ros2 pkg executables orbslam3
```

若仍无 `monocular-inertial`，重新运行构建命令并检查 CMake 输出。

### 图像编码

当前节点接受单通道 T265 fisheye 图像，包括 `8UC1` 和 `mono8`。若 bag 中编码异常，先用以下命令检查图像消息：

```bash
ros2 topic echo /t265/fisheye1/image_raw --once
```

OpenLORIS Cafe 的 T265 fisheye 图像通常应为单通道灰度图；彩色图像会在节点内转为灰度图再送入 ORB_SLAM3。

### 初始化失败

单目惯性初始化依赖足够平移、转动和连续 IMU。可尝试从更早时间播放、降低播放倍率，或选择运动更充分的片段：

```bash
BAG=/path/to/OpenLORIS-Scene/cafe1-1_2_ros2/cafe1-1_vins
ros2 bag play "$BAG" \
  --rate 0.5 \
  --topics /t265/fisheye1/image_raw /t265/imu
```

### 模式错误

确认运行的是 `monocular-inertial`，配置为 `OpenLORIS_Cafe_T265.yaml`，并且节点输出包含 `Monocular-Inertial`。该链路调用 ORB_SLAM3 原生 `IMU_MONOCULAR` 和 `TrackMonocular(..., vImuMeas)`。

## 9. 后续 TODO

- 使用 T265/OpenLORIS 实测结果替换当前沿用的 RealSense T265 示例 IMU 噪声。
- 基于 OpenLORIS ground truth 补充 RPE 评估流程或更多误差诊断指标。
- 设计 D400 深度重投影或 RGB-D-inertial 链路。
- 将 `camera`、`imu`、viewer、轨迹输出路径参数化。
- 增加 OpenLORIS 专用 launch 文件。
