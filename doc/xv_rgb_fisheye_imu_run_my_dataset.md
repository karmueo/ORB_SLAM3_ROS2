# XV RGB 鱼眼单目惯性自有数据集运行教程

本文说明如何使用已经由 `doc/xv_rgb_fisheye_imu_calibration.md` 标定得到的配置，在自有 rosbag2 数据集上运行 ORB-SLAM3 单目惯性节点。

默认运行数据集为：

```bash
/mnt/data/slam/my_umi_rosbag/5
```

默认使用的标定配置为：

```bash
config/monocular-inertial/XV_RGB_Fisheye_calibrated.yaml
```

该配置的来源如下：

- 相机内参和鱼眼畸变参数来自 Kalibr 相机标定。
- `IMU.T_b_c1` 来自 Kalibr IMU-camera 外参标定，表示机体系到相机系的变换。
- `IMU.NoiseGyro`、`IMU.NoiseAcc`、`IMU.GyroWalk`、`IMU.AccWalk` 来自 Allan 方差估计。

## 1. 数据集信息

先确认 bag 可读：

```bash
source /opt/ros/jazzy/setup.bash
BAG=/mnt/data/slam/my_umi_rosbag/5
ros2 bag info "$BAG"
```

当前 `/mnt/data/slam/my_umi_rosbag/5` 的关键信息为：

- 存储格式：MCAP
- ROS 发行版：Jazzy
- 时长：约 `66.47 s`
- 图像 topic：`/xv_sdk/SN250801DR48FB26001253/rgb_registered/image`
- 图像消息数：`2907`，平均频率约 `43.73 Hz`
- IMU topic：`/xv_sdk/SN250801DR48FB26001253/imu`
- IMU 消息数：`23753`，平均频率约 `357.33 Hz`

本文只播放 RGB 图像和 IMU，减少无关 topic 对检查输出的干扰。

## 2. 环境检查

确认 ROS 2 和本仓库路径：

```bash
source /opt/ros/jazzy/setup.bash

PKG=/home/scl/work/slam/ORB_SLAM3_ROS2
BAG=/mnt/data/slam/my_umi_rosbag/5

test -d "$PKG"
test -d "$BAG"
test -s "$PKG/config/monocular-inertial/XV_RGB_Fisheye_calibrated.yaml"
test -s "$PKG/vocabulary/ORBvoc.txt"
```

如果 `vocabulary/ORBvoc.txt` 不存在，但 `vocabulary/ORBvoc.txt.tar.gz` 存在，先解压词典：

```bash
cd "$PKG/vocabulary"
tar -xzf ORBvoc.txt.tar.gz
```

确认运行数据包含目标 topic：

```bash
ros2 bag info "$BAG" | grep -E "/rgb/image|/imu"
```

期望至少看到：

```text
/xv_sdk/SN250801DR48FB26001253/rgb_registered/image
/xv_sdk/SN250801DR48FB26001253/imu
```

## 3. 构建

在 ROS 2 工作空间根目录或当前仓库目录执行构建。以下命令使用本机常用路径：

```bash
cd /home/scl/work/slam/ORB_SLAM3_ROS2
source /opt/ros/jazzy/setup.bash

colcon build --symlink-install --packages-select orbslam3 \
  --cmake-args \
  -DPython3_EXECUTABLE=/usr/bin/python3 \
  -DOpenCV_DIR=/usr/lib/x86_64-linux-gnu/cmake/opencv4 \
  -DORB_SLAM3_ROOT_DIR=/home/scl/work/slam/ORB_SLAM3
```

构建完成后加载工作空间，并确认可执行存在：

```bash
source install/local_setup.bash
ros2 pkg executables orbslam3
```

输出中应包含：

```text
orbslam3 monocular-inertial
```

如果在上层 colcon workspace 构建，把 `cd` 切换到 workspace 根目录，并对应 source 该 workspace 的 `install/local_setup.bash`。

## 4. 启动单目惯性节点

终端 1 启动 ORB-SLAM3：

```bash
source /opt/ros/jazzy/setup.bash
source /home/scl/work/slam/ORB_SLAM3_ROS2/install/local_setup.bash

PKG=/home/scl/work/slam/ORB_SLAM3_ROS2
ros2 run orbslam3 monocular-inertial \
  "$PKG/vocabulary/ORBvoc.txt" \
  "$PKG/config/monocular-inertial/XV_RGB_Fisheye_calibrated.yaml" \
  true \
  --ros-args \
  -r camera:=/xv_sdk/SN250801DR48FB26001253/rgb_registered/image \
  -r imu:=/xv_sdk/SN250801DR48FB26001253/imu
```

第三个参数 `false` 表示关闭 Pangolin viewer，适合远程终端、SSH 或无显示环境。需要本地可视化时可以改成 `true`。

节点启动后应完成 ORB vocabulary 加载，并打印单目惯性相关初始化信息。

## 5. 播放 bag

终端 2 只播放图像和 IMU：

```bash
source /opt/ros/jazzy/setup.bash

BAG=/mnt/data/slam/my_umi_rosbag/5
ros2 bag play "$BAG" \
  --topics \
  /xv_sdk/SN250801DR48FB26001253/rgb_registered/image \
  /xv_sdk/SN250801DR48FB26001253/imu
```

如果初始化较慢、轨迹为空，或怀疑机器处理不过来，可以降低播放速率：

```bash
ros2 bag play "$BAG" \
  --rate 0.5 \
  --topics \
  /xv_sdk/SN250801DR48FB26001253/rgb_registered/image \
  /xv_sdk/SN250801DR48FB26001253/imu
```

## 6. 运行中检查

播放 bag 后，在另一个终端检查节点输出：

```bash
source /opt/ros/jazzy/setup.bash
source /home/scl/work/slam/ORB_SLAM3_ROS2/install/local_setup.bash

ros2 topic hz /orbslam3/body_pose
ros2 topic echo /orbslam3/body_pose --once
ros2 topic hz /orbslam3/path
```

也可以直接检查输入 topic 是否正在发布：

```bash
ros2 topic hz /xv_sdk/SN250801DR48FB26001253/rgb_registered/image
ros2 topic hz /xv_sdk/SN250801DR48FB26001253/imu
```

一次可接受的 smoke test 应满足：

- `monocular-inertial` 节点完成词典加载。
- 节点收到 `/rgb_registered/image` 和 `/imu` 数据。
- `/orbslam3/body_pose` 有连续输出。
- `/orbslam3/path` 的 pose 数量持续增长。
- 节点退出后生成非空 `KeyFrameTrajectory.txt`。

## 7. 轨迹文件检查

`KeyFrameTrajectory.txt` 会写在启动 `monocular-inertial` 节点时所在目录。建议在固定目录启动节点，方便归档结果。

停止 bag 播放并退出节点后检查轨迹：

```bash
test -s KeyFrameTrajectory.txt
wc -l KeyFrameTrajectory.txt
head KeyFrameTrajectory.txt
```

如果 `test -s` 退出码为 0，表示文件存在且非空。`wc -l` 行数越多，通常表示成功跟踪的关键帧越多；具体质量仍需要结合轨迹形状、尺度和后续真值评估判断。

## 8. IMU 频率提示

当前运行配置 `XV_RGB_Fisheye_calibrated.yaml` 中：

```yaml
IMU.Frequency: 493.2
```

该值来自标定阶段使用的 IMU 数据实测频率。`/mnt/data/slam/my_umi_rosbag/5` 中 IMU 消息数为 `23753`，按约 `66.47 s` 时长估算，平均频率约为 `357.33 Hz`。

按当前需求，先使用已经标定完成的 `XV_RGB_Fisheye_calibrated.yaml` 直接运行。如果出现以下现象，再单独复制一份运行配置做对比实验：

- 单目惯性长时间无法初始化。
- 轨迹尺度明显异常。
- IMU 预积分或跟踪表现不稳定。
- `ros2 bag info` 或 `ros2 topic hz` 显示运行 bag 的 IMU 频率持续低于配置值。

对比实验可以把配置复制为运行专用文件，例如：

```bash
cp "$PKG/config/monocular-inertial/XV_RGB_Fisheye_calibrated.yaml" \
   "$PKG/config/monocular-inertial/XV_RGB_Fisheye_my_dataset.yaml"
```

然后在副本中调整 `IMU.Frequency`，并检查 bag 是否存在 IMU 丢帧或 topic 录制不完整问题。

## 9. 常见问题

### 找不到 monocular-inertial 可执行

重新 source 工作空间并检查可执行：

```bash
source /opt/ros/jazzy/setup.bash
source /home/scl/work/slam/ORB_SLAM3_ROS2/install/local_setup.bash
ros2 pkg executables orbslam3
```

如果输出中没有 `orbslam3 monocular-inertial`，重新执行构建命令，并检查 CMake 输出中的 ORB_SLAM3、OpenCV 和 ROS 2 依赖路径。

### 没有 /orbslam3/body_pose 输出

`/orbslam3/body_pose` 只在 ORB-SLAM3 跟踪状态有效时发布。优先检查：

- 启动命令是否运行 `orbslam3 monocular-inertial`。
- `camera` remap 是否指向 `/xv_sdk/SN250801DR48FB26001253/rgb_registered/image`。
- `imu` remap 是否指向 `/xv_sdk/SN250801DR48FB26001253/imu`。
- `ros2 topic hz` 是否能看到图像和 IMU 输入。
- 节点是否仍处于单目惯性初始化阶段。

### 轨迹为空或很短

先降低播放速率：

```bash
ros2 bag play /mnt/data/slam/my_umi_rosbag/5 \
  --rate 0.5 \
  --topics \
  /xv_sdk/SN250801DR48FB26001253/rgb_registered/image \
  /xv_sdk/SN250801DR48FB26001253/imu
```

如果仍然为空，继续检查图像曝光、运动激励、IMU 时间戳连续性、`IMU.T_b_c1` 方向和 IMU 噪声量级。

### Camera.fps parameter must be an integer number

ORB-SLAM3 新版 `Settings` 解析要求 `Camera.fps` 写成整数。如果启动时报错：

```text
Camera.fps parameter must be an integer number, aborting...
```

检查配置文件：

```bash
grep -n "Camera.fps" "$PKG/config/monocular-inertial/XV_RGB_Fisheye_calibrated.yaml"
```

该字段应写成：

```yaml
Camera.fps: 47
```

不要写成 `47.0`。

### 远程终端或无显示环境卡住

启动命令第三个参数保持 `false`：

```bash
ros2 run orbslam3 monocular-inertial \
  "$PKG/vocabulary/ORBvoc.txt" \
  "$PKG/config/monocular-inertial/XV_RGB_Fisheye_calibrated.yaml" \
  false \
  --ros-args \
  -r camera:=/xv_sdk/SN250801DR48FB26001253/rgb_registered/image \
  -r imu:=/xv_sdk/SN250801DR48FB26001253/imu
```

### 找不到 KeyFrameTrajectory.txt

`KeyFrameTrajectory.txt` 写入节点启动时所在目录。运行前可以先切换到结果目录：

```bash
mkdir -p /tmp/orbslam3_xv_run_5
cd /tmp/orbslam3_xv_run_5
```

再启动 `ros2 run orbslam3 monocular-inertial ...`。节点退出后在该目录检查轨迹文件。
