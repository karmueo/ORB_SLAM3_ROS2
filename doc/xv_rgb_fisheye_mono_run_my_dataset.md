# XV RGB 鱼眼纯单目自有数据集运行教程

本文记录在当前仓库中使用 ORB-SLAM3 `MONOCULAR` 模式跑通 `/home/scl/datasets/ros2bag/0707` 的最小链路。该流程只使用 XV RGB 鱼眼图像 topic：

```bash
/xv_sdk/SN250801DR48FB26001253/rgb/image
```

纯单目运行的核心区别如下：

- 不订阅 IMU，也不使用配置文件中的 `IMU.*` 参数。
- 不发布 `/orbslam3/body_pose` 或 `/orbslam3/path`。
- 输出轨迹只有单目视觉尺度，轨迹尺度不能直接按真实米制理解。
- 退出节点后只保存 `KeyFrameTrajectory.txt`。

## 1. 数据集信息

默认运行数据集为：

```bash
/home/scl/datasets/ros2bag/0707
```

先确认 bag 可读：

```bash
source /opt/ros/jazzy/setup.bash
BAG=/home/scl/datasets/ros2bag/0707
ros2 bag info "$BAG"
```

当前 bag 的关键信息为：

- 存储格式：MCAP
- ROS 发行版：Jazzy
- 时长：约 `44.38 s`
- 图像 topic：`/xv_sdk/SN250801DR48FB26001253/rgb/image`
- 图像消息数：`2664`
- IMU topic：`/xv_sdk/SN250801DR48FB26001253/imu`
- IMU 消息数：`21888`

本文的纯单目流程只播放 RGB 图像 topic。bag 中虽然包含 IMU topic，`mono` 节点不会订阅它。

## 2. 环境检查

确认 ROS 2、本仓库、配置文件和词典存在：

```bash
source /opt/ros/jazzy/setup.bash

PKG=/home/scl/work/slam/ORB_SLAM3_ROS2
BAG=/home/scl/datasets/ros2bag/0707

test -d "$PKG"
test -d "$BAG"
test -s "$PKG/config/monocular-inertial/XV_RGB_Fisheye_calibrated_resize_bag4.yaml"
test -s "$PKG/vocabulary/ORBvoc.txt"
```

如果 `vocabulary/ORBvoc.txt` 不存在，但 `vocabulary/ORBvoc.txt.tar.gz` 存在，先解压词典：

```bash
cd "$PKG/vocabulary"
tar -xzf ORBvoc.txt.tar.gz
```

确认运行数据包含目标图像 topic：

```bash
ros2 bag info "$BAG" | grep -E "/rgb/image|/imu"
```

期望至少看到：

```text
/xv_sdk/SN250801DR48FB26001253/rgb/image
/xv_sdk/SN250801DR48FB26001253/imu
```

确认 `mono` 可执行已安装：

```bash
source "$PKG/install/local_setup.bash"
ros2 pkg executables orbslam3
```

输出中应包含：

```text
orbslam3 mono
```

## 3. 配置文件说明

本流程使用以下配置：

```bash
config/monocular-inertial/XV_RGB_Fisheye_calibrated_resize_bag4.yaml
```

选择该配置的原因是它已经包含 XV RGB 鱼眼相机模型、内参、畸变参数、`960x960` 内部 resize 和本次实测可用的 ORB 参数。虽然文件位于 `config/monocular-inertial/` 目录并包含 `IMU.*` 字段，但 `mono` 可执行以 `ORB_SLAM3::System::MONOCULAR` 初始化，运行时只使用相机、ORB 和 viewer 相关配置。

## 4. 构建

在当前仓库目录执行构建：

```bash
cd /home/scl/work/slam/ORB_SLAM3_ROS2
source /opt/ros/jazzy/setup.bash

colcon build --symlink-install --packages-select orbslam3
```

如果本机需要显式指定 ORB-SLAM3、OpenCV 或 Python 路径，可以追加项目常用 CMake 参数：

```bash
colcon build --symlink-install --packages-select orbslam3 \
  --cmake-args \
  -DPython3_EXECUTABLE=/usr/bin/python3 \
  -DOpenCV_DIR=/usr/lib/x86_64-linux-gnu/cmake/opencv4 \
  -DORB_SLAM3_ROOT_DIR=/home/scl/work/slam/ORB_SLAM3
```

构建完成后加载工作空间，并确认可执行存在：

```bash
source install/local_setup.bash
ros2 pkg executables orbslam3 | grep "orbslam3 mono"
```

## 5. 启动纯单目节点

终端 1 启动 ORB-SLAM3 纯单目节点：

```bash
source /opt/ros/jazzy/setup.bash
source /home/scl/work/slam/ORB_SLAM3_ROS2/install/local_setup.bash

PKG=/home/scl/work/slam/ORB_SLAM3_ROS2
xvfb-run -a ros2 run orbslam3 mono \
  "$PKG/vocabulary/ORBvoc.txt" \
  "$PKG/config/monocular-inertial/XV_RGB_Fisheye_calibrated_resize_bag4.yaml" \
  --ros-args \
  -r camera:=/xv_sdk/SN250801DR48FB26001253/rgb/image
```

当前 `mono` 可执行中的 Pangolin viewer 写死为开启。SSH、远程终端或无显示环境建议使用 `xvfb-run -a`。如果在本地桌面环境运行，也可以去掉 `xvfb-run -a`。

节点启动后应完成 ORB vocabulary 加载，并开始等待 `camera` remap 后的图像输入。

## 6. 播放 bag

终端 2 只播放 RGB 图像 topic。推荐先用 `0.5` 倍速，降低实时处理压力：

```bash
source /opt/ros/jazzy/setup.bash

ros2 bag play /home/scl/datasets/ros2bag/0707 \
  --rate 0.5 \
  --topics /xv_sdk/SN250801DR48FB26001253/rgb/image
```

如果 `0.5` 倍速稳定，再尝试默认速度：

```bash
ros2 bag play /home/scl/datasets/ros2bag/0707 \
  --topics /xv_sdk/SN250801DR48FB26001253/rgb/image
```

## 7. 成功判据

一次可接受的 smoke test 应满足：

- `mono` 节点完成词典加载。
- 日志中出现一次 `New Map created`。
- `one frame has been sent` 持续出现，表示图像已送入 ORB-SLAM3。
- 不出现反复跟踪丢失或重置，例如连续打印 `Fail to track local map`、`Reset map`、`Relocalization`。
- 停止 bag 并退出节点后生成非空 `KeyFrameTrajectory.txt`。

可以把节点日志写入文件，方便检查：

```bash
xvfb-run -a ros2 run orbslam3 mono \
  "$PKG/vocabulary/ORBvoc.txt" \
  "$PKG/config/monocular-inertial/XV_RGB_Fisheye_calibrated_resize_bag4.yaml" \
  --ros-args \
  -r camera:=/xv_sdk/SN250801DR48FB26001253/rgb/image \
  2>&1 | tee mono_0707.log
```

运行后检查关键日志：

```bash
grep -c "one frame has been sent" mono_0707.log
grep -c "New Map created" mono_0707.log
grep -E "Fail to track local map|Reset map|Relocalization" mono_0707.log
```

## 8. 本次实测结果

使用上述配置和 `/home/scl/datasets/ros2bag/0707` 进行纯单目实测，结果摘要如下：

- 送入图像帧数：`2664`
- `New Map created` 次数：`1`
- `Reset map` 次数：`0`
- `Relocalization` 次数：`0`

该结果说明当前图像 topic、相机配置和 ORB 参数能让纯单目链路稳定跑完此 bag。轨迹尺度仍然是纯单目尺度，不能作为真实米制轨迹直接使用。

## 9. 轨迹文件检查

`KeyFrameTrajectory.txt` 会写在启动 `mono` 节点时所在目录。建议在固定目录启动节点，方便归档结果。

停止 bag 播放并退出节点后检查轨迹：

```bash
test -s KeyFrameTrajectory.txt
wc -l KeyFrameTrajectory.txt
head KeyFrameTrajectory.txt
```

`test -s` 退出码为 0 表示文件存在且非空。`wc -l` 行数越多，通常表示成功跟踪的关键帧越多；具体质量需要结合轨迹形状、尺度和后续真值评估判断。

## 10. 常见问题

### 找不到 mono 可执行

重新 source 工作空间并检查可执行：

```bash
source /opt/ros/jazzy/setup.bash
source /home/scl/work/slam/ORB_SLAM3_ROS2/install/local_setup.bash
ros2 pkg executables orbslam3 | grep "orbslam3 mono"
```

如果没有输出，重新执行构建命令，并检查 CMake 输出中的 ORB_SLAM3、OpenCV 和 ROS 2 依赖路径。

### 没有 /orbslam3/body_pose 输出

这是当前纯单目节点的预期行为。`/orbslam3/body_pose` 是 `monocular-inertial` 节点发布的 topic，`mono` 节点只订阅图像并在退出时保存 `KeyFrameTrajectory.txt`。

### 无显示环境启动失败

`mono` 可执行当前固定启用 Pangolin viewer。远程或无显示环境使用：

```bash
xvfb-run -a ros2 run orbslam3 mono ...
```

如果需要从命令行关闭 viewer，需要修改 `src/monocular/mono.cpp` 增加 viewer 开关参数；本文流程不修改代码。

### 轨迹尺度看起来不对

纯单目没有 IMU、双目基线或深度约束，轨迹只有相似变换意义下的尺度。该输出可用于观察跟踪稳定性和相对运动趋势，不能直接当作米制位姿使用。
