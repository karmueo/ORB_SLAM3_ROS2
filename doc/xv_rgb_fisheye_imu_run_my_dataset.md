# XV RGB 鱼眼单目惯性自有数据集运行教程

本文给出使用 ORB-SLAM3 `monocular-inertial` 节点运行
`$HOME/datasets/ros2bag/0722_slam` 的完整命令。按下面顺序打开终端并复制执行即可。
该流程与纯单目教程使用同一原始 RGB 鱼眼图像，只额外订阅同一设备发布的 IMU。

输入 topic：

```text
/xv_sdk/SN250801DR48FB26001253/rgb/image
/xv_sdk/SN250801DR48FB26001253/imu
```

运行结果包括：

- `/orbslam3/body_pose`：当前机体位姿；
- `/orbslam3/path`：累计机体轨迹；
- `map -> body_link`：动态 TF；
- `body_link -> rgb_optical_frame`：根据相机—IMU 外参生成的静态 TF；
- `KeyFrameTrajectory.txt`：启用保存参数并正常退出节点时生成的关键帧轨迹。

`map` 是 ORB-SLAM3 当前惯性地图的世界坐标系，`body_link` 是配置中
`IMU.T_b_c1` 对应的 IMU 机体系。静态 TF 的子坐标系取自输入图像的
`header.frame_id`；本数据集为 `rgb_optical_frame`。

单目惯性初始化会估计重力方向、速度、IMU bias 和尺度。节点只在惯性 BA1 完成并连续
稳定后发布 `/orbslam3/body_pose`、`/orbslam3/path` 和动态 TF，因此启动后短时间没有
ROS 位姿输出属于正常现象。

## 步骤 1：设置固定路径

在终端中执行：

```bash
export SLAM_WS=$HOME/work/slam
export ORB_SLAM3_ROOT=$HOME/work/slam/ORB_SLAM3
export BAG=$HOME/datasets/ros2bag/0722_slam
export RESULT_DIR=$HOME/results/orbslam3_imu_0722_slam
```

检查目录和资源：

```bash
test -d "$SLAM_WS/ORB_SLAM3_ROS2"
test -d "$ORB_SLAM3_ROOT"
test -d "$BAG"
test -s "$SLAM_WS/ORB_SLAM3_ROS2/vocabulary/ORBvoc.txt.tar.gz"
```

以上命令没有输出时表示路径有效。

## 步骤 2：构建 ORB-SLAM3 和 ROS 2 包

先构建 ORB-SLAM3 核心库：

```bash
cd $HOME/work/slam/ORB_SLAM3
bash build.sh
```

然后构建 ROS 2 包：

```bash
cd $HOME/work/slam
source /opt/ros/jazzy/setup.bash

colcon build --symlink-install --packages-select orbslam3 \
  --cmake-args \
  -DPython3_EXECUTABLE=/usr/bin/python3 \
  -DOpenCV_DIR=/usr/lib/x86_64-linux-gnu/cmake/opencv4 \
  -DORB_SLAM3_ROOT_DIR=$HOME/work/slam/ORB_SLAM3
```

加载构建结果并确认 `monocular-inertial` 已安装：

```bash
source /opt/ros/jazzy/setup.bash
source $HOME/work/slam/install/local_setup.bash

ros2 pkg executables orbslam3 \
  | grep '^orbslam3 monocular-inertial$'
```

期望输出：

```text
orbslam3 monocular-inertial
```

如果没有输出，停在本步骤检查 `colcon build` 的错误信息。

## 步骤 3：检查 bag 和输入 topic

执行：

```bash
source /opt/ros/jazzy/setup.bash
export BAG=$HOME/datasets/ros2bag/0722_slam

ros2 bag info "$BAG"
```

再确认本教程使用的图像和 IMU topic：

```bash
ros2 bag info "$BAG" \
  | grep -E '/xv_sdk/SN250801DR48FB26001253/(rgb/image|imu)'
```

必须同时看到：

```text
/xv_sdk/SN250801DR48FB26001253/rgb/image
/xv_sdk/SN250801DR48FB26001253/imu
```

当前 bag 时长约 `42.87 s`，包含 `2574` 帧原始 RGB 鱼眼图像和 `21133`
条 IMU 消息。后续只播放这两个 topic，减少其他图像流和相机信息对回放的影响。

## 步骤 4：检查配置和 mask

本数据集使用：

```text
config/monocular-inertial/XV_RGB_Fisheye_calibrated.yaml
config/masks/fisheye_mask.png
```

配置按 `1280 × 1280` 原始 RGB 鱼眼图像标定，使用 ORB-SLAM3
`KannalaBrandt8` 相机模型，并在内部缩放到 `960 × 960`。其中：

- `IMU.T_b_c1`：相机光学系到 IMU 机体系的外参；
- `ROS.ImuTimeOffsetSec`：用于对齐相机与 IMU 时间戳的固定偏移；
- `IMU.NoiseGyro`、`IMU.NoiseAcc`：陀螺仪和加速度计白噪声密度；
- `IMU.GyroWalk`、`IMU.AccWalk`：陀螺仪和加速度计随机游走；
- `IMU.Frequency: 493.0`：标定配置采用的 IMU 发布频率。

加载安装目录并检查资源：

```bash
source /opt/ros/jazzy/setup.bash
source $HOME/work/slam/install/local_setup.bash

export ORB_SHARE="$(ros2 pkg prefix orbslam3)/share/orbslam3"

test -s "$ORB_SHARE/vocabulary/ORBvoc.txt"
test -s \
  "$ORB_SHARE/config/monocular-inertial/XV_RGB_Fisheye_calibrated.yaml"
test -s "$ORB_SHARE/config/masks/fisheye_mask.png"
file "$ORB_SHARE/config/masks/fisheye_mask.png"
```

`file` 应显示 mask 为 `1280 x 1280` 单通道灰度 PNG。

Mask 像素规则：

- `0`：排除该位置的 ORB 候选特征点；
- 任意非零值：允许该位置的候选特征点。

节点会保持原图不变，在 FAST 检出候选点后按 mask 过滤，再执行八叉树分配和描述子计算。
mask 与首帧图像尺寸不一致、文件无法读取或 mask 全黑时，节点会打印错误并停止。

## 步骤 5：终端 1 启动单目惯性节点

新开终端 1，完整复制以下命令：

```bash
source /opt/ros/jazzy/setup.bash
source $HOME/work/slam/install/local_setup.bash

export ORB_SHARE="$(ros2 pkg prefix orbslam3)/share/orbslam3"
export RESULT_DIR=$HOME/results/orbslam3_imu_0722_slam

mkdir -p "$RESULT_DIR"
cd "$RESULT_DIR"

ros2 run orbslam3 monocular-inertial \
  "$ORB_SHARE/vocabulary/ORBvoc.txt" \
  "$ORB_SHARE/config/monocular-inertial/XV_RGB_Fisheye_calibrated.yaml" \
  false \
  --ros-args \
  -p feature_mask_path:="$ORB_SHARE/config/masks/fisheye_mask.png" \
  -p max_path_length:=10000 \
  -p save_keyframe_trajectory:=true \
  -r camera:=/xv_sdk/SN250801DR48FB26001253/rgb/image \
  -r imu:=/xv_sdk/SN250801DR48FB26001253/imu \
  2>&1 | tee imu_0722_slam.log
```

第三个位置参数 `false` 关闭 Pangolin Viewer，适合 SSH 或无显示环境。
`save_keyframe_trajectory:=true` 让节点正常退出时在当前结果目录保存
`KeyFrameTrajectory.txt`。

启动日志应包含：

```text
Vocabulary loaded!
Loaded monocular-inertial feature mask
Monocular-Inertial
```

保持终端 1 运行。

### 步骤 5.1：启动参数说明

| 参数或 remap | 当前值 | 作用 |
| --- | --- | --- |
| 第 1 个位置参数 | 安装目录中的 `vocabulary/ORBvoc.txt` | 选择 ORB 词典。 |
| 第 2 个位置参数 | `XV_RGB_Fisheye_calibrated.yaml` | 选择原始鱼眼单目惯性配置。 |
| 第 3 个位置参数 | `false` | 控制 Pangolin Viewer。 |
| `feature_mask_path` | `config/masks/fisheye_mask.png` | 选择与原始图像逐像素对齐的 mask；空字符串禁用。 |
| `max_path_length` | `10000` | 限制 `/orbslam3/path` 位姿数；`0` 表示无限累计。 |
| `save_keyframe_trajectory` | `true` | 控制退出时是否保存 `KeyFrameTrajectory.txt`。 |
| `camera` remap | `/xv_sdk/SN250801DR48FB26001253/rgb/image` | 指定与纯单目教程相同的原始 RGB 鱼眼图像。 |
| `imu` remap | `/xv_sdk/SN250801DR48FB26001253/imu` | 指定同一设备、同一时间基准的 IMU。 |

`settings`、`camera` 和 `feature_mask_path` 必须描述同一种图像模型和相同的像素尺寸。
相机与 IMU 还必须使用同一时间基准，并与配置中的外参和时间偏移对应。

## 步骤 6：终端 2 播放 bag

新开终端 2，先用 `0.5` 倍速同时播放图像和 IMU：

```bash
source /opt/ros/jazzy/setup.bash

ros2 bag play $HOME/datasets/ros2bag/0722_slam \
  --rate 0.5 \
  --topics \
  /xv_sdk/SN250801DR48FB26001253/rgb/image \
  /xv_sdk/SN250801DR48FB26001253/imu
```

等待 bag 播放完成。确认 `0.5` 倍速可以稳定初始化和跟踪后，下次可使用原速：

```bash
source /opt/ros/jazzy/setup.bash

ros2 bag play $HOME/datasets/ros2bag/0722_slam \
  --topics \
  /xv_sdk/SN250801DR48FB26001253/rgb/image \
  /xv_sdk/SN250801DR48FB26001253/imu
```

回放速度只影响消息到达的墙上时间，不会改写消息中的采集时间戳。

## 步骤 7：终端 3 检查实时输出

bag 正在播放时，新开终端 3 执行：

```bash
source /opt/ros/jazzy/setup.bash
source $HOME/work/slam/install/local_setup.bash

ros2 topic list \
  | grep -E '^/orbslam3/(body_pose|path)$'
```

期望看到：

```text
/orbslam3/body_pose
/orbslam3/path
```

先检查输入频率：

```bash
ros2 topic hz /xv_sdk/SN250801DR48FB26001253/rgb/image
ros2 topic hz /xv_sdk/SN250801DR48FB26001253/imu
```

每条命令看到持续更新的频率后按 `Ctrl+C`，再执行下一条。

检查图像和 IMU 的时间戳及坐标系：

```bash
ros2 topic echo \
  /xv_sdk/SN250801DR48FB26001253/rgb/image \
  --field header --once

ros2 topic echo \
  /xv_sdk/SN250801DR48FB26001253/imu \
  --field header --once
```

两类消息的 `header.stamp` 应位于同一时间范围。本数据集图像的
`frame_id` 为 `rgb_optical_frame`，IMU 的 `frame_id` 为
`xv_sdk/imu_optical_frame`。

等待终端 1 出现惯性 BA1 完成并稳定的相关日志后检查位姿：

```bash
ros2 topic hz /orbslam3/body_pose
```

看到持续更新的频率后按 `Ctrl+C`。继续检查动态和静态 TF：

```bash
ros2 run tf2_ros tf2_echo map body_link
ros2 run tf2_ros tf2_echo body_link rgb_optical_frame
```

每条命令看到连续或静态变换后按 `Ctrl+C`。

## 步骤 8：结束节点并保存轨迹

先等待终端 2 的 bag 播放完成，并观察终端 1 的处理统计停止增长；再回到终端 1 按
`Ctrl+C`，执行 ORB-SLAM3 关闭和轨迹保存流程。

节点正常退出时会在结果目录写入：

```text
$HOME/results/orbslam3_imu_0722_slam/KeyFrameTrajectory.txt
```

检查结果：

```bash
export RESULT_DIR=$HOME/results/orbslam3_imu_0722_slam
cd "$RESULT_DIR"

test -s KeyFrameTrajectory.txt
wc -l KeyFrameTrajectory.txt
head -n 5 KeyFrameTrajectory.txt
```

再检查关键日志：

```bash
cd $HOME/results/orbslam3_imu_0722_slam

grep -E \
  "图像与 IMU 时间基准检查通过|等待惯性 BA1|单目惯性统计" \
  imu_0722_slam.log

grep -E \
  "时间基准未对齐|过滤 IMU|Reset map|Fail to track local map" \
  imu_0722_slam.log
```

`KeyFrameTrajectory.txt` 非空且运行期间能持续收到 `/orbslam3/body_pose`，说明原始
鱼眼图像、IMU、惯性初始化和 ROS 位姿输出链路已经跑通。第二组日志存在持续告警时，
应先处理时间戳、IMU 连续性或跟踪稳定性问题。

## 步骤 9：需要禁用 mask 时这样启动

只在对比实验或 mask 与当前图像不匹配时使用本步骤。新开终端并执行：

```bash
source /opt/ros/jazzy/setup.bash
source $HOME/work/slam/install/local_setup.bash

export ORB_SHARE="$(ros2 pkg prefix orbslam3)/share/orbslam3"
export RESULT_DIR=$HOME/results/orbslam3_imu_0722_slam_no_mask

mkdir -p "$RESULT_DIR"
cd "$RESULT_DIR"

ros2 run orbslam3 monocular-inertial \
  "$ORB_SHARE/vocabulary/ORBvoc.txt" \
  "$ORB_SHARE/config/monocular-inertial/XV_RGB_Fisheye_calibrated.yaml" \
  false \
  --ros-args \
  -p feature_mask_path:="" \
  -p max_path_length:=10000 \
  -p save_keyframe_trajectory:=true \
  -r camera:=/xv_sdk/SN250801DR48FB26001253/rgb/image \
  -r imu:=/xv_sdk/SN250801DR48FB26001253/imu \
  2>&1 | tee imu_0722_slam_no_mask.log
```

日志应包含：

```text
Monocular-inertial feature mask is disabled
```

随后按步骤 6 播放 bag，并按步骤 8 结束节点和检查轨迹。

## 步骤 10：需要自定义 mask 时这样启动

自定义 mask 必须是与输入图像逐像素对齐的 `1280 × 1280` 灰度图。
假设文件路径为 `$HOME/config/my_gripper_mask.png`，执行：

```bash
source /opt/ros/jazzy/setup.bash
source $HOME/work/slam/install/local_setup.bash

export ORB_SHARE="$(ros2 pkg prefix orbslam3)/share/orbslam3"
export CUSTOM_MASK=$HOME/config/my_gripper_mask.png
export RESULT_DIR=$HOME/results/orbslam3_imu_0722_slam_custom_mask

test -s "$CUSTOM_MASK"
file "$CUSTOM_MASK"

mkdir -p "$RESULT_DIR"
cd "$RESULT_DIR"

ros2 run orbslam3 monocular-inertial \
  "$ORB_SHARE/vocabulary/ORBvoc.txt" \
  "$ORB_SHARE/config/monocular-inertial/XV_RGB_Fisheye_calibrated.yaml" \
  false \
  --ros-args \
  -p feature_mask_path:="$CUSTOM_MASK" \
  -p max_path_length:=10000 \
  -p save_keyframe_trajectory:=true \
  -r camera:=/xv_sdk/SN250801DR48FB26001253/rgb/image \
  -r imu:=/xv_sdk/SN250801DR48FB26001253/imu \
  2>&1 | tee imu_0722_slam_custom_mask.log
```

日志中的 `Loaded monocular-inertial feature mask` 应显示自定义路径、`1280x1280`
尺寸和排除比例。随后按步骤 6 播放 bag，并按步骤 8 保存结果。

## 步骤 11：需要 Pangolin 窗口时这样启动

本地桌面环境可以把步骤 5 命令的第三个位置参数从 `false` 改为 `true`：

```bash
source /opt/ros/jazzy/setup.bash
source $HOME/work/slam/install/local_setup.bash

export ORB_SHARE="$(ros2 pkg prefix orbslam3)/share/orbslam3"
export RESULT_DIR=$HOME/results/orbslam3_imu_0722_slam_viewer

mkdir -p "$RESULT_DIR"
cd "$RESULT_DIR"

ros2 run orbslam3 monocular-inertial \
  "$ORB_SHARE/vocabulary/ORBvoc.txt" \
  "$ORB_SHARE/config/monocular-inertial/XV_RGB_Fisheye_calibrated.yaml" \
  true \
  --ros-args \
  -p feature_mask_path:="$ORB_SHARE/config/masks/fisheye_mask.png" \
  -p max_path_length:=10000 \
  -p save_keyframe_trajectory:=true \
  -r camera:=/xv_sdk/SN250801DR48FB26001253/rgb/image \
  -r imu:=/xv_sdk/SN250801DR48FB26001253/imu \
  2>&1 | tee imu_0722_slam_viewer.log
```

SSH 或无显示环境继续使用步骤 5 的 `false`。

## 常见问题

### 找不到 monocular-inertial 可执行

重新 source 工作空间并检查可执行：

```bash
source /opt/ros/jazzy/setup.bash
source $HOME/work/slam/install/local_setup.bash

ros2 pkg executables orbslam3 \
  | grep '^orbslam3 monocular-inertial$'
```

如果没有输出，重新执行步骤 2 的构建命令，并检查 ORB_SLAM3、OpenCV 和 ROS 2
依赖路径。

### 没有 /orbslam3/body_pose 输出

该 topic 需要有效视觉跟踪、惯性 BA1 完成并连续稳定后才会发布。依次检查：

- 终端 1 是否出现 `图像与 IMU 时间基准检查通过`；
- 图像和 IMU topic 是否持续发布；
- 日志中的 `跟踪图像` 和 `入队IMU` 计数是否持续增加；
- 设备运动是否包含足够的平移、转动和速度变化；
- 图像是否有足够纹理，曝光和运动模糊是否可控；
- 配置中的外参、时间偏移和 IMU 噪声是否来自当前设备标定。

### 图像与 IMU 时间基准未对齐

节点发现图像与最近 IMU 的原始时间戳相差超过允许上限时会退出，并打印：

```text
图像与 IMU 时间基准未对齐，程序即将退出
```

分别查看两类消息的 header：

```bash
ros2 topic echo \
  /xv_sdk/SN250801DR48FB26001253/rgb/image \
  --field header --once

ros2 topic echo \
  /xv_sdk/SN250801DR48FB26001253/imu \
  --field header --once
```

两类消息必须由发布端使用同一时钟生成 `header.stamp`。配置中的
`ROS.ImuTimeOffsetSec` 只处理标定得到的毫秒级相机—IMU偏移，不能修复系统时间与
设备时间等不同时间基准。

### IMU 时间戳重复、回跳或不连续

日志中的 `过滤 IMU 回跳或重复时间戳样本` 持续增长时，检查：

```bash
ros2 topic hz /xv_sdk/SN250801DR48FB26001253/imu
ros2 topic echo \
  /xv_sdk/SN250801DR48FB26001253/imu \
  --field header.stamp
```

少量并发到达乱序可由节点的 IMU 重排窗口处理。持续重复、明显回跳或长时间断流需要
在录制端或设备发布端修复。

### 惯性初始化时间长或反复重置

先使用步骤 6 的 `0.5` 倍速回放，并确认：

- 开始阶段有连续、清晰且具备视差的图像；
- 设备进行了多方向转动和带加减速的平移；
- IMU 没有断流，时间戳严格前进；
- mask 没有排除过多有效纹理区域；
- `IMU.T_b_c1` 的方向和数值与当前相机—IMU组合一致。

匀速直线、静止或仅绕单一轴缓慢旋转通常无法提供充分的惯性初始化激励。

### 轨迹为空或很短

确认启动命令包含：

```text
-p save_keyframe_trajectory:=true
```

然后先等待 bag 播放完成，再按 `Ctrl+C` 正常退出节点。节点被强制终止、跟踪始终未
初始化或结果目录不可写时，可能无法得到有效轨迹。

### Camera.fps parameter must be an integer number

ORB-SLAM3 新版 `Settings` 解析要求 `Camera.fps` 使用整数。检查安装目录中的配置：

```bash
grep -n "Camera.fps" \
  "$ORB_SHARE/config/monocular-inertial/XV_RGB_Fisheye_calibrated.yaml"
```

本配置应显示：

```yaml
Camera.fps: 15
```

### 远程终端或无显示环境卡住

保持启动命令的第三个位置参数为 `false`：

```text
ros2 run orbslam3 monocular-inertial <vocabulary> <settings> false
```

该设置关闭 Pangolin Viewer，不影响 SLAM、ROS 位姿、TF 或轨迹文件输出。

### 找不到 KeyFrameTrajectory.txt

轨迹文件写入节点启动时的当前目录，并且只有
`save_keyframe_trajectory:=true` 时才会保存。运行前先进入固定结果目录：

```bash
mkdir -p $HOME/results/orbslam3_imu_0722_slam
cd $HOME/results/orbslam3_imu_0722_slam
```

再执行步骤 5 的启动命令，节点正常退出后在该目录检查轨迹。
