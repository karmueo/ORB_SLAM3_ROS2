# XV RGB 鱼眼纯单目自有数据集运行教程

本文给出使用 ORB-SLAM3 `mono` 节点运行
`$HOME/datasets/ros2bag/0722_slam` 的完整命令。按下面顺序打开终端并复制执行即可。
默认使用 `xv_rgb_fisheye_mono.launch.py` 统一配置 mono、特征 mask、ROS 2 位姿输出和
RViz2。

该流程使用图像 topic：

```text
/xv_sdk/SN250801DR48FB26001253/rgb/image
```

运行结果包括：

- `/orbslam3/camera_pose`：当前相机位姿；
- `/orbslam3/camera_path`：累计相机轨迹；
- `camera_start -> camera_link`：动态 TF；
- `camera_link -> camera_optical_frame`：静态 TF；
- `KeyFrameTrajectory.txt`：节点退出时保存的关键帧轨迹。

`camera_start` 与 ORB-SLAM3 成功建图时选定的初始化首帧相机坐标系重合。
该初始化帧可能晚于 bag 的第一张图像，例如特征不足或两视图初始化重试时会顺延。
ROS Pose、Path 和动态 TF 采用 x 前、y 左、z 上的机体系约定；
`KeyFrameTrajectory.txt` 保持 ORB-SLAM3 原生 TUM 光学坐标约定，原点仍对应同一初始化相机。

纯单目轨迹没有真实尺度约束，适合检查跟踪稳定性和相对运动趋势。

## 步骤 1：设置固定路径

在终端中执行：

```bash
export SLAM_WS=$HOME/work/slam
export ORB_SLAM3_ROOT=$HOME/work/slam/ORB_SLAM3
export BAG=$HOME/datasets/ros2bag/0722_slam
export RESULT_DIR=$HOME/results/orbslam3_mono_0722_slam
```

检查目录：

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

加载构建结果并确认 `mono` 已安装：

```bash
source /opt/ros/jazzy/setup.bash
source $HOME/work/slam/install/local_setup.bash

ros2 pkg executables orbslam3 | grep '^orbslam3 mono$'
```

期望输出：

```text
orbslam3 mono
```

如果没有输出，停在本步骤检查 `colcon build` 的错误信息，不要继续启动节点。

## 步骤 3：检查 bag 和输入 topic

执行：

```bash
source /opt/ros/jazzy/setup.bash
export BAG=$HOME/datasets/ros2bag/0722_slam

ros2 bag info "$BAG"
```

再单独确认目标图像 topic：

```bash
ros2 bag info "$BAG" \
  | grep '/xv_sdk/SN250801DR48FB26001253/rgb/image'
```

必须看到：

```text
/xv_sdk/SN250801DR48FB26001253/rgb/image
```

`mono` 节点不会订阅 bag 中的 IMU topic，因此播放时只选择图像即可。

## 步骤 4：检查配置和 mask

本数据集使用：

```text
config/monocular-inertial/XV_RGB_Fisheye_calibrated.yaml
```

该配置按 `1280 × 1280` 输入图像标定，并在 ORB-SLAM3 内部缩放到
`960 × 960`。文件中的 `IMU.*` 字段在纯单目模式下不会参与计算。

加载安装目录并检查资源：

```bash
source /opt/ros/jazzy/setup.bash
source $HOME/work/slam/install/local_setup.bash

export ORB_SHARE="$(ros2 pkg prefix orbslam3)/share/orbslam3"

test -s "$ORB_SHARE/vocabulary/ORBvoc.txt"
test -s "$ORB_SHARE/config/monocular-inertial/XV_RGB_Fisheye_calibrated.yaml"
test -s "$ORB_SHARE/config/masks/fisheye_mask.png"
file "$ORB_SHARE/config/masks/fisheye_mask.png"
```

`file` 应显示 mask 为 `1280 x 1280` 单通道灰度 PNG。

Mask 像素规则：

- `0`：排除该位置的 ORB 候选特征点；
- 任意非零值：允许该位置的候选特征点。

节点会保持原图不变，在 FAST 检出候选点后按 mask 过滤，再执行八叉树分配和描述子计算。
mask 与首帧图像尺寸不一致、文件无法读取或 mask 全黑时，节点会打印错误并停止。

## 步骤 5：终端 1 启动 mono 节点

新开终端 1，完整复制以下命令：

```bash
source /opt/ros/jazzy/setup.bash
source $HOME/work/slam/install/local_setup.bash

export RESULT_DIR=$HOME/results/orbslam3_mono_0722_slam

mkdir -p "$RESULT_DIR"
cd "$RESULT_DIR"

ros2 launch orbslam3 xv_rgb_fisheye_mono.launch.py \
  use_viewer:=false \
  use_rviz:=true \
  2>&1 | tee mono_0722_slam.log
```

上述命令关闭 Pangolin Viewer，并打开 RViz2 显示输入视频、相机位姿、轨迹和 TF。
SSH 或无显示环境将 `use_rviz` 改为 `false`。`publish_ros_pose` 默认为 `true`；改为
`false` 后仍运行 SLAM 并在退出时保存 `KeyFrameTrajectory.txt`，但不会创建 ROS 2
位姿、轨迹或 TF 接口。
启动日志应包含：

```text
Vocabulary loaded!
Loaded monocular feature mask
Monocular
```

保持终端 1 运行。

### 步骤 5.1：按输入类型覆盖 mono 配置

`xv_rgb_fisheye_mono.launch.py` 的主要参数如下：

| 参数 | 默认值 | 作用 |
| --- | --- | --- |
| `vocabulary_path` | 安装目录中的 `vocabulary/ORBvoc.txt` | 选择 ORB 词典。 |
| `settings_path` | `config/monocular-inertial/XV_RGB_Fisheye_calibrated.yaml` | 选择与输入图像模型匹配的 mono 配置。 |
| `camera_topic` | `/xv_sdk/SN250801DR48FB26001253/rgb/image` | 同时指定 mono 和 RViz2 的视频输入。 |
| `feature_mask_path` | `config/masks/fisheye_mask.png` | 选择逐像素对齐的 mask；空字符串禁用。 |
| `publish_ros_pose` | `true` | 控制 Pose、Path、动态 TF 和静态 TF。 |
| `max_path_length` | `10000` | 限制 Path 位姿数；`0` 表示无限累计。 |
| `use_viewer` | `false` | 控制 Pangolin Viewer。 |
| `use_rviz` | `false` | 控制 RViz2 视频、位姿和轨迹显示。 |
| `rviz_config_path` | 包内纯单目 RViz2 配置 | 选择自定义 RViz2 配置。 |

输入 XV SDK 已校正图像时，复制执行：

```bash
source /opt/ros/jazzy/setup.bash
source $HOME/work/slam/install/local_setup.bash

export ORB_SHARE="$(ros2 pkg prefix orbslam3)/share/orbslam3"

ros2 launch orbslam3 xv_rgb_fisheye_mono.launch.py \
  settings_path:="$ORB_SHARE/config/monocular/XV_RGB_Fisheye_undistorted.yaml" \
  camera_topic:=/xv_sdk/SN250801DR48FB26001253/rgb_fisheye_undistorted/image \
  feature_mask_path:="$ORB_SHARE/config/masks/mask.png" \
  use_viewer:=false \
  use_rviz:=true
```

`settings_path`、`camera_topic` 和 `feature_mask_path` 必须描述同一种输入图像及相同的
像素尺寸。原始鱼眼图像使用 `KannalaBrandt8` 配置，SDK 校正图像使用 `PinHole` 配置。

## 步骤 6：终端 2 播放 bag

新开终端 2，先用 `0.5` 倍速播放：

```bash
source /opt/ros/jazzy/setup.bash

ros2 bag play $HOME/datasets/ros2bag/0722_slam \
  --rate 0.5 \
  --topics /xv_sdk/SN250801DR48FB26001253/rgb/image
```

等待 bag 播放完成。确认 `0.5` 倍速可以稳定运行后，下次可使用原速：

```bash
source /opt/ros/jazzy/setup.bash

ros2 bag play $HOME/datasets/ros2bag/0722_slam \
  --topics /xv_sdk/SN250801DR48FB26001253/rgb/image
```

## 步骤 7：终端 3 检查实时输出

bag 正在播放时，新开终端 3 执行：

```bash
source /opt/ros/jazzy/setup.bash
source $HOME/work/slam/install/local_setup.bash

ros2 topic list \
  | grep -E '^/orbslam3/(camera_pose|camera_path)$'
```

期望看到：

```text
/orbslam3/camera_path
/orbslam3/camera_pose
```

检查位姿发布频率：

```bash
ros2 topic hz /orbslam3/camera_pose
```

看到持续更新的频率后按 `Ctrl+C` 退出频率检查。

如需检查 TF，再执行：

```bash
ros2 run tf2_ros tf2_echo camera_start camera_link
```

看到连续变换后按 `Ctrl+C` 退出。

## 步骤 8：结束节点并保存轨迹

先等待终端 2 的 bag 播放完成，再回到终端 1 按 `Ctrl+C`。

节点正常退出时会在结果目录写入：

```text
$HOME/results/orbslam3_mono_0722_slam/KeyFrameTrajectory.txt
```

检查结果：

```bash
export RESULT_DIR=$HOME/results/orbslam3_mono_0722_slam
cd "$RESULT_DIR"

test -s KeyFrameTrajectory.txt
wc -l KeyFrameTrajectory.txt
head -n 5 KeyFrameTrajectory.txt
```

再检查关键日志：

```bash
cd $HOME/results/orbslam3_mono_0722_slam

grep -c "New Map created" mono_0722_slam.log
grep -E "Fail to track local map|Reset map|Relocalization" mono_0722_slam.log
```

`KeyFrameTrajectory.txt` 非空且运行期间能持续收到
`/orbslam3/camera_pose`，说明纯单目链路已经跑通。日志中偶发一次跟踪失败可以结合
轨迹连续性判断；持续失败或反复重建地图时，应先降低 bag 播放速度并重新运行步骤 5 至步骤 8。

## 步骤 9：需要禁用 mask 时这样启动

只在对比实验或 mask 与当前图像不匹配时使用本步骤。新开终端并执行：

```bash
source /opt/ros/jazzy/setup.bash
source $HOME/work/slam/install/local_setup.bash

export RESULT_DIR=$HOME/results/orbslam3_mono_0722_slam_no_mask

mkdir -p "$RESULT_DIR"
cd "$RESULT_DIR"

ros2 launch orbslam3 xv_rgb_fisheye_mono.launch.py \
  feature_mask_path:="" \
  use_viewer:=false \
  use_rviz:=true \
  2>&1 | tee mono_0722_slam_no_mask.log
```

日志应包含：

```text
Monocular feature mask is disabled
```

随后按步骤 6 播放 bag，并按步骤 8 结束节点和检查轨迹。

## 步骤 10：需要自定义 mask 时这样启动

自定义 mask 必须是与输入图像逐像素对齐的 `1280 × 1280` 灰度图。
假设文件路径为 `$HOME/config/my_gripper_mask.png`，执行：

```bash
source /opt/ros/jazzy/setup.bash
source $HOME/work/slam/install/local_setup.bash

export CUSTOM_MASK=$HOME/config/my_gripper_mask.png
export RESULT_DIR=$HOME/results/orbslam3_mono_0722_slam_custom_mask

test -s "$CUSTOM_MASK"
file "$CUSTOM_MASK"

mkdir -p "$RESULT_DIR"
cd "$RESULT_DIR"

ros2 launch orbslam3 xv_rgb_fisheye_mono.launch.py \
  feature_mask_path:="$CUSTOM_MASK" \
  use_viewer:=false \
  use_rviz:=true \
  2>&1 | tee mono_0722_slam_custom_mask.log
```

日志中的 `Loaded monocular feature mask` 应显示自定义路径、`1280x1280`
尺寸和排除比例。随后按步骤 6 播放 bag，并按步骤 8 保存结果。

## 步骤 11：需要 Pangolin 窗口时这样启动

本地桌面环境可以把步骤 5 命令中的 viewer 参数从 `false` 改为
`true`。完整命令如下：

```bash
source /opt/ros/jazzy/setup.bash
source $HOME/work/slam/install/local_setup.bash

export RESULT_DIR=$HOME/results/orbslam3_mono_0722_slam_viewer

mkdir -p "$RESULT_DIR"
cd "$RESULT_DIR"

ros2 launch orbslam3 xv_rgb_fisheye_mono.launch.py \
  use_viewer:=true \
  use_rviz:=false \
  2>&1 | tee mono_0722_slam_viewer.log
```

SSH 或无显示环境继续使用步骤 5 的 `false`，无需 `xvfb-run`。

## 本数据集已有实测记录

2026-07-22 使用默认配置以 1.0 倍速完整回放
`$HOME/datasets/ros2bag/0722_slam`，实测结果如下：

- 回放原始鱼眼图像：`2574` 帧；
- 成功初始化地图，初始地图点：`75`；
- 有效 ROS 位姿：`1869` 个，frame_id 全部为 `camera_start`；
- `KeyFrameTrajectory.txt`：`81` 条，首条关键帧为单位位姿；
- `New Map created`：`1` 次；
- `Reset map`、`Relocalization` 和 `Fail to track local map`：均为 `0` 次；
- RViz2 在虚拟 X Server 中成功启动 OpenGL，未出现 Fixed Frame 或 TF 显示错误。

本次结果文件保存在 `$HOME/results/orbslam3_mono_0722_slam`。
