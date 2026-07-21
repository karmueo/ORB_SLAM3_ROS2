# XV 校正 RGB 鱼眼与 IMU 实时运行说明

本文说明如何联合启动 `xv_sdk_ros2` 和 ORB-SLAM3，使用设备侧已经去畸变的 RGB 鱼眼图像与 IMU 运行单目惯性 SLAM。

这条链路的数据流为：

```text
/rgb/image（原始 equidistant 鱼眼）
  -> xv_sdk_ros2 cv::fisheye::remap
  -> /rgb_fisheye_undistorted/image（PinHole、零畸变、rgb8）
  -> orbslam3 monocular-inertial

/imu
  -> 时间偏移与小窗口重排
  -> orbslam3 monocular-inertial
```

对应的 ORB-SLAM3 配置为：

```text
config/monocular-inertial/XV_RGB_Fisheye_undistorted.yaml
```

该配置使用 `Camera.type: "PinHole"`，并有意省略 `Camera1.k1/k2/p1/p2`。鱼眼校正已经由 XV SDK 完成，ORB-SLAM3 不再执行第二次去畸变。

设备发布的校正图像与 IMU 必须使用同一 `header.stamp` 时间基准。wrapper 启动时比较图像与最近 IMU
的原始时间戳，绝对差超过 1 秒时输出 FATAL 日志并以非零状态退出，不会在消费端自动平移时间基准。
`ROS.ImuTimeOffsetSec` 只保留 Kalibr 标定得到的毫秒级物理采样偏移。

## 1. 构建两个工作空间

先按实际环境设置工作空间和 ORB-SLAM3 源码路径：

```bash
export XV_WS=/path/to/xv_ws
export ORBSLAM3_ROS2_WS=/path/to/ORB_SLAM3_ROS2
export ORB_SLAM3_ROOT=/path/to/ORB_SLAM3
```

先构建设备工作空间：

```bash
cd "$XV_WS"
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select xv_sdk_ros2
```

再构建 ORB-SLAM3 wrapper：

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

构建 `orbslam3` 时 source 设备工作空间，可以让运行依赖 `xv_sdk_ros2` 在当前环境中可解析。

## 2. 推荐：联合启动

依次 source ROS 2、设备工作空间和本仓库：

```bash
source /opt/ros/jazzy/setup.bash
source "$XV_WS/install/local_setup.bash"
source "$ORBSLAM3_ROS2_WS/install/local_setup.bash"
```

一次启动设备校正流和 ORB-SLAM3：

```bash
ros2 launch orbslam3 xv_rgb_fisheye_undistorted_imu.launch.py
```

联合 launch 等价于给设备侧传入：

```bash
ros2 launch xv_sdk_ros2 xv_sdk_node_launch.py \
  rgb_fisheye_undistort_enable:=true \
  rgb_fisheye_calibration_path:="$XV_WS/src/xv_sdk_ros2/config/kalibr_data-camchain-imucam.yaml"
```

launch 默认使用已安装到 `xv_sdk_ros2` share 目录中的同名标定文件，因此不依赖源码目录的相对路径。

远程终端或无显示环境可以关闭 viewer：

```bash
ros2 launch orbslam3 xv_rgb_fisheye_undistorted_imu.launch.py use_viewer:=false
```

设备序列号变化时，覆盖两个 topic：

```bash
ros2 launch orbslam3 xv_rgb_fisheye_undistorted_imu.launch.py \
  camera_topic:=/xv_sdk/实际序列号/rgb_fisheye_undistorted/image \
  imu_topic:=/xv_sdk/实际序列号/imu
```

## 3. 分终端启动

如果需要分别观察设备与 SLAM 日志，可以按两个终端启动。

终端 1：

```bash
cd "$XV_WS"
source /opt/ros/jazzy/setup.bash
source install/local_setup.bash

ros2 launch xv_sdk_ros2 xv_sdk_node_launch.py \
  rgb_fisheye_undistort_enable:=true \
  rgb_fisheye_calibration_path:=src/xv_sdk_ros2/config/kalibr_data-camchain-imucam.yaml
```

终端 2：

```bash
source /opt/ros/jazzy/setup.bash
source "$ORBSLAM3_ROS2_WS/install/local_setup.bash"

PKG="$(ros2 pkg prefix orbslam3)/share/orbslam3"
ros2 run orbslam3 monocular-inertial \
  "$PKG/vocabulary/ORBvoc.txt" \
  "$PKG/config/monocular-inertial/XV_RGB_Fisheye_undistorted.yaml" \
  true \
  --ros-args \
  -r camera:=/xv_sdk/SN250801DR48FB26001253/rgb_fisheye_undistorted/image \
  -r imu:=/xv_sdk/SN250801DR48FB26001253/imu
```

## 4. 输入校验

先确认校正图像、相机信息和 IMU 均在发布：

```bash
ros2 topic hz /xv_sdk/SN250801DR48FB26001253/rgb_fisheye_undistorted/image
ros2 topic hz /xv_sdk/SN250801DR48FB26001253/imu
ros2 topic echo \
  /xv_sdk/SN250801DR48FB26001253/rgb_fisheye_undistorted/camera_info \
  --once --no-arr
```

相机信息应满足：

- `width: 1280`、`height: 1280`；
- `distortion_model: plumb_bob`；
- `d` 的 5 个值均为 `0.0`；
- `k` 中的 `fx/fy/cx/cy` 与 `XV_RGB_Fisheye_undistorted.yaml` 一致。

再检查 SLAM 输出：

```bash
ros2 topic hz /orbslam3/body_pose
ros2 topic hz /orbslam3/path
ros2 topic echo /orbslam3/body_pose --once
```

一次可接受的 smoke test 应满足：

- XV SDK 持续发布校正图像和 IMU；
- ORB-SLAM3 日志没有相机配置缺失错误；
- `/orbslam3/body_pose` 连续输出；
- `/orbslam3/path` 中的 pose 数量持续增长；
- 退出后生成非空 `KeyFrameTrajectory.txt`。

## 5. 关键约束

- 校正图像只能配合 `XV_RGB_Fisheye_undistorted.yaml` 使用。
- 原始 `/rgb/image` 应继续配合 `KannalaBrandt8 + k1/k2/k3/k4` 配置使用。
- `rgb_registered/image` 属于 RGB-D 对齐链路，不作为本实现的单目惯性输入。
- SDK 的 remap 使用原 Kalibr 内参作为输出针孔内参，因此配置中的 `fx/fy/cx/cy` 无需再次换算。
- 像素校正没有旋转相机坐标系，`IMU.T_b_c1` 继续使用同一份 IMU-camera 外参。
- 输入约为 58 Hz、`ROS.TargetFps` 为 30 Hz 时，累计限帧丢弃约占一半属于预期现象；该信息以 INFO 级别输出。
