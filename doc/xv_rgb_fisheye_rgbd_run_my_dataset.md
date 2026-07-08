# XV registered RGB-D 自有数据集运行教程

本文记录在当前仓库中使用 ORB-SLAM3 `RGBD` 模式跑通 `/home/scl/datasets/ros2bag/0708` 的最小链路。该流程使用 XV 已对齐的 RGB 与深度图 topic：

```bash
/xv_sdk/SN250801DR48FB26001253/rgb_registered/image
/xv_sdk/SN250801DR48FB26001253/rgb_registered/depth
```

RGB-D 运行的核心特点如下：

- 只订阅 RGB 图像和深度图，不订阅 IMU。
- 深度图编码为 `32FC1`，按米单位直接送入 ORB-SLAM3。
- 当前 `rgbd` 节点不会发布 `/orbslam3/body_pose` 或 `/orbslam3/path`。
- 退出节点后只保存 `KeyFrameTrajectory.txt`。

## 1. 数据集信息

默认运行数据集为：

```bash
/home/scl/datasets/ros2bag/0708
```

先确认 bag 可读：

```bash
source /opt/ros/jazzy/setup.bash
BAG=/home/scl/datasets/ros2bag/0708
ros2 bag info "$BAG"
```

当前 0708 bag 的关键信息为：

- 存储格式：MCAP
- ROS 发行版：Jazzy
- 时长：约 `71.22 s`
- RGB topic：`/xv_sdk/SN250801DR48FB26001253/rgb_registered/image`
- Depth topic：`/xv_sdk/SN250801DR48FB26001253/rgb_registered/depth`
- RGB 图像尺寸：`1280x1280`
- RGB 编码：`rgb8`
- Depth 编码：`32FC1`
- RGB 帧数：`1122`，平均频率约 `15.75 Hz`

本文只播放 RGB 和 depth 两个 topic。bag 中如果包含 IMU 或其他 topic，`rgbd` 节点不会订阅它们。

## 2. 环境检查

确认 ROS 2、本仓库、配置文件和词典存在：

```bash
source /opt/ros/jazzy/setup.bash

PKG=/home/scl/work/slam/ORB_SLAM3_ROS2
BAG=/home/scl/datasets/ros2bag/0708

test -d "$PKG"
test -d "$BAG"
test -s "$PKG/config/rgb-d/XV_RGB_Registered_RGBD_0708.yaml"
test -s "$PKG/vocabulary/ORBvoc.txt"
```

如果 `vocabulary/ORBvoc.txt` 不存在，但 `vocabulary/ORBvoc.txt.tar.gz` 存在，先解压词典：

```bash
cd "$PKG/vocabulary"
tar -xzf ORBvoc.txt.tar.gz
```

确认运行数据包含目标 RGB-D topic：

```bash
ros2 bag info "$BAG" | grep -E "/rgb_registered/image|/rgb_registered/depth"
```

期望至少看到：

```text
/xv_sdk/SN250801DR48FB26001253/rgb_registered/image
/xv_sdk/SN250801DR48FB26001253/rgb_registered/depth
```

可以在播放 bag 时检查编码和尺寸：

```bash
ros2 bag play "$BAG" \
  --start-paused \
  --topics \
  /xv_sdk/SN250801DR48FB26001253/rgb_registered/image \
  /xv_sdk/SN250801DR48FB26001253/rgb_registered/depth
```

另开终端执行：

```bash
source /opt/ros/jazzy/setup.bash

ros2 topic echo /xv_sdk/SN250801DR48FB26001253/rgb_registered/image --once --no-arr \
  | sed -n '/height:/p;/width:/p;/encoding:/p'
ros2 topic echo /xv_sdk/SN250801DR48FB26001253/rgb_registered/depth --once --no-arr \
  | sed -n '/height:/p;/width:/p;/encoding:/p'
```

## 3. 配置文件说明

本流程使用以下配置：

```bash
config/rgb-d/XV_RGB_Registered_RGBD_0708.yaml
```

该配置按 registered/pinhole RGB-D 链路编写：

- `Camera.type: "PinHole"`
- `Camera1.fx/fy/cx/cy` 先沿用现有 XV RGB 标定内参作为初始值。
- `Camera1.k1/k2/p1/p2/k3` 全部置 `0.0`，按已校正图像处理。
- `Camera.width/height: 1280`，对应 bag 中原始图像尺寸。
- `Camera.newWidth/newHeight: 960`，降低 1280x1280 输入的处理负载。
- `Camera.fps: 16`，对应 1122 帧 / 约 71.22 秒。
- `Camera.RGB: 1`，对应 RGB 图像编码 `rgb8`。
- `RGBD.DepthMapFactor: 1.0`，对应米单位 `32FC1` 深度图。
- `Stereo.ThDepth: 40.0`、`Stereo.b: 0.0745`，作为 RGB-D 近远点阈值和虚拟基线初始值。

由于 bag 中没有 `CameraInfo`，registered 流真实 pinhole 内参需要后续从设备 SDK 或标定结果中补充。拿到真实内参后，优先替换 `Camera1.fx/fy/cx/cy`。

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
ros2 pkg executables orbslam3 | grep "orbslam3 rgbd"
```

## 5. 启动 RGB-D 节点

建议先进入固定结果目录，方便定位 `KeyFrameTrajectory.txt`：

```bash
mkdir -p /tmp/orbslam3_rgbd_0708
cd /tmp/orbslam3_rgbd_0708
```

终端 1 启动 ORB-SLAM3 RGB-D 节点：

```bash
source /opt/ros/jazzy/setup.bash
source /home/scl/work/slam/ORB_SLAM3_ROS2/install/local_setup.bash

PKG=/home/scl/work/slam/ORB_SLAM3_ROS2
ros2 run orbslam3 rgbd \
  "$PKG/vocabulary/ORBvoc.txt" \
  "$PKG/config/rgb-d/XV_RGB_Registered_RGBD_0708.yaml" \
  false \
  --ros-args \
  -r camera/rgb:=/xv_sdk/SN250801DR48FB26001253/rgb_registered/image \
  -r camera/depth:=/xv_sdk/SN250801DR48FB26001253/rgb_registered/depth
```

第三个参数 `false` 表示关闭 Pangolin viewer，适合 SSH、远程终端或无显示环境。需要本地可视化时可以改成 `true`。

节点启动后应完成 ORB vocabulary 加载，并等待同步后的 RGB-D 输入。

## 6. 播放 bag

终端 2 只播放 RGB 与 depth 两个 topic。推荐先用 `0.5` 倍速，降低实时处理压力：

```bash
source /opt/ros/jazzy/setup.bash

BAG=/home/scl/datasets/ros2bag/0708
ros2 bag play "$BAG" \
  --rate 0.5 \
  --topics \
  /xv_sdk/SN250801DR48FB26001253/rgb_registered/image \
  /xv_sdk/SN250801DR48FB26001253/rgb_registered/depth
```

如果 `0.5` 倍速稳定，再尝试默认速度：

```bash
ros2 bag play "$BAG" \
  --topics \
  /xv_sdk/SN250801DR48FB26001253/rgb_registered/image \
  /xv_sdk/SN250801DR48FB26001253/rgb_registered/depth
```

## 7. 运行中检查

播放 bag 后，可以检查输入 topic 频率：

```bash
source /opt/ros/jazzy/setup.bash

ros2 topic hz /xv_sdk/SN250801DR48FB26001253/rgb_registered/image
ros2 topic hz /xv_sdk/SN250801DR48FB26001253/rgb_registered/depth
```

检查 `rgbd` 节点订阅关系：

```bash
source /opt/ros/jazzy/setup.bash
source /home/scl/work/slam/ORB_SLAM3_ROS2/install/local_setup.bash

ros2 node info /ORB_SLAM3_ROS2
```

输出中应能看到节点订阅了 remap 后的 RGB 与 depth topic。一次可接受的 smoke test 应满足：

- `rgbd` 节点完成词典加载。
- 节点订阅了 `/rgb_registered/image` 和 `/rgb_registered/depth`。
- RGB 与 depth 输入频率接近，时间戳能被 approximate sync 匹配。
- ORB-SLAM3 日志中出现地图创建或持续跟踪相关输出。
- 停止 bag 并退出节点后生成非空 `KeyFrameTrajectory.txt`。

当前 `rgbd` 节点只保存 `KeyFrameTrajectory.txt`，不会发布 `/orbslam3/body_pose` 或 `/orbslam3/path`。检查这两个 topic 没有输出属于当前实现的预期行为。

## 8. 轨迹文件检查

`KeyFrameTrajectory.txt` 会写在启动 `rgbd` 节点时所在目录。停止 bag 播放并退出节点后检查轨迹：

```bash
test -s KeyFrameTrajectory.txt
wc -l KeyFrameTrajectory.txt
head KeyFrameTrajectory.txt
```

`test -s` 退出码为 0 表示文件存在且非空。`wc -l` 行数越多，通常表示成功跟踪的关键帧越多；具体质量需要结合轨迹形状和后续真值评估判断。

## 9. 常见问题

### 找不到 rgbd 可执行

重新 source 工作空间并检查可执行：

```bash
source /opt/ros/jazzy/setup.bash
source /home/scl/work/slam/ORB_SLAM3_ROS2/install/local_setup.bash
ros2 pkg executables orbslam3 | grep "orbslam3 rgbd"
```

如果没有输出，重新执行构建命令，并检查 CMake 输出中的 ORB_SLAM3、OpenCV 和 ROS 2 依赖路径。

### 没有 /orbslam3/body_pose 输出

这是当前 `rgbd` 节点的预期行为。`/orbslam3/body_pose` 和 `/orbslam3/path` 由其他节点实现发布；当前 `src/rgbd/rgbd-slam-node.cpp` 只订阅 RGB-D 图像，并在析构时保存 `KeyFrameTrajectory.txt`。

### 节点收不到同步 RGB-D 帧

优先检查 remap 和时间戳：

```bash
ros2 node info /ORB_SLAM3_ROS2
ros2 topic hz /xv_sdk/SN250801DR48FB26001253/rgb_registered/image
ros2 topic hz /xv_sdk/SN250801DR48FB26001253/rgb_registered/depth
```

`rgbd` 节点使用 approximate sync 同步 `camera/rgb` 和 `camera/depth`。如果两个 topic 时间戳差异过大、其中一路缺帧严重，节点可能长时间不进入 `TrackRGBD`。

### 深度尺度明显不对

当前 depth 编码为 `32FC1` 且单位按米处理，因此配置中使用：

```yaml
RGBD.DepthMapFactor: 1.0
```

如果后续使用 `16UC1` 毫米深度图，应改为：

```yaml
RGBD.DepthMapFactor: 1000.0
```

### 远程终端或无显示环境启动失败

远程或无显示环境下，启动命令第三个参数保持 `false`：

```bash
ros2 run orbslam3 rgbd \
  "$PKG/vocabulary/ORBvoc.txt" \
  "$PKG/config/rgb-d/XV_RGB_Registered_RGBD_0708.yaml" \
  false \
  --ros-args \
  -r camera/rgb:=/xv_sdk/SN250801DR48FB26001253/rgb_registered/image \
  -r camera/depth:=/xv_sdk/SN250801DR48FB26001253/rgb_registered/depth
```

如果仍然看到 `libEGL warning: DRI3 error`，检查启动命令中 `false` 是否位于配置文件参数之后、`--ros-args` 之前。

### 轨迹为空或很短

先降低播放速率：

```bash
ros2 bag play /home/scl/datasets/ros2bag/0708 \
  --rate 0.5 \
  --topics \
  /xv_sdk/SN250801DR48FB26001253/rgb_registered/image \
  /xv_sdk/SN250801DR48FB26001253/rgb_registered/depth
```

如果仍然为空，继续检查 RGB 图像曝光、深度有效值比例、RGB-D 时间戳同步、registered 流真实 pinhole 内参和 `RGBD.DepthMapFactor`。
