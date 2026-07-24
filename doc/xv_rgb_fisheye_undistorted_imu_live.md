# XV 校正 RGB 鱼眼与 IMU 回放定位说明

本文说明如何使用 `$HOME/datasets/ros2bag/0721` 中已经去畸变的 RGB 鱼眼图像和
IMU，运行 ORB-SLAM3 单目惯性定位，并发布机体位姿、轨迹和 TF。

数据流如下：

```text
0721 rosbag2
  ├─ /rgb_fisheye_undistorted/image（PinHole、零畸变、rgb8）
  └─ /imu
        ↓ 时间偏移、单调性检查和小窗口重排
orbslam3 monocular-inertial
  ├─ /orbslam3/body_pose
  ├─ /orbslam3/path
  ├─ map -> body_link
  ├─ body_link -> rgb_optical_frame
  └─ KeyFrameTrajectory.txt（按参数选择保存）
```

ORB-SLAM3 使用以下配置：

```text
config/monocular-inertial/XV_RGB_Fisheye_undistorted.yaml
```

该配置采用 `Camera.type: "PinHole"` 并省略畸变参数。图像在录制前已经由 XV SDK
完成校正，`IMU.T_b_c1`、IMU 噪声、时间偏移和频率使用相机/IMU 联合标定结果。

## 步骤 1：确认 0721 bag

加载 ROS 2 并查看 bag 信息：

```bash
source /opt/ros/jazzy/setup.bash

BAG=$HOME/datasets/ros2bag/0721
ros2 bag info "$BAG"
```

当前数据实测信息：

- 存储格式：MCAP；
- 大小：约 `10.6 GiB`；
- 时长：`38.644251438 s`；
- 校正图像：2320 帧，平均约 `60.04 Hz`；
- IMU：19065 条，平均约 `493.32 Hz`；
- 图像格式：`1280 × 1280`、`rgb8`；
- 图像坐标系：`rgb_optical_frame`；
- 图像和 IMU `header.stamp` 均严格递增，图像时间范围被 IMU 覆盖。

需要使用的 topic：

```text
/xv_sdk/SN250801DR48FB26001253/rgb_fisheye_undistorted/image
/xv_sdk/SN250801DR48FB26001253/rgb_fisheye_undistorted/camera_info
/xv_sdk/SN250801DR48FB26001253/imu
```

`camera_info` 的畸变模型为 `plumb_bob`，5 个畸变参数均为 `0.0`，内参为：

```text
fx = 397.07575683833136
fy = 397.07575683833136
cx = 637.7723482481603
cy = 640.0889202000565
```

这些参数与 `XV_RGB_Fisheye_undistorted.yaml` 一致。配置中的
`IMU.Frequency: 493.3` 也与 bag 实测频率一致，无需修改配置。

## 步骤 2：构建并加载工作空间

```bash
cd $HOME/work/slam/ORB_SLAM3_ROS2
source /opt/ros/jazzy/setup.bash

colcon build --symlink-install --packages-select orbslam3 \
  --cmake-args \
  -DPython3_EXECUTABLE=/usr/bin/python3 \
  -DOpenCV_DIR=/usr/lib/x86_64-linux-gnu/cmake/opencv4 \
  -DORB_SLAM3_ROOT_DIR=$HOME/work/slam/ORB_SLAM3

source install/local_setup.bash
```

确认单目惯性可执行文件和 launch 参数：

```bash
ros2 pkg executables orbslam3 | grep monocular-inertial
ros2 launch orbslam3 xv_rgb_fisheye_undistorted_imu.launch.py --show-args
```

离线回放只需要加载 ROS 2 和当前工作空间，不需要加载 `xv_sdk_ros2` 工作空间。

## 步骤 3：确认 launch 参数默认值

`xv_rgb_fisheye_undistorted_imu.launch.py` 的参数如下：

| 参数 | 默认值 | 作用 |
| --- | --- | --- |
| `start_xv_sdk` | `true` | 控制 launch 是否同时启动 XV SDK 相机驱动。`true` 用于连接 XV 实时设备；bag 回放时设为 `false`，避免启动设备节点和产生同名输入 topic。 |
| `rgb_fisheye_undistort_enable` | `true` | 传递给 XV SDK 驱动，控制是否发布经过标定校正的 RGB 鱼眼图像。ORB-SLAM3 本流程要求使用校正图像，因此实时设备模式应保持 `true`；`start_xv_sdk:=false` 时该参数不会作用于 bag 数据。 |
| `rgb_fisheye_calibration_path` | 空字符串 | 指定 XV SDK 进行 RGB 鱼眼校正时使用的 camchain 标定文件。空值会解析为 `xv_sdk_ros2/share/xv_sdk_ros2/config/kalibr_data-camchain-imucam.yaml`；使用其他设备标定时传入对应文件的绝对路径。 |
| `vocabulary_path` | `<orbslam3_share>/vocabulary/ORBvoc.txt` | 指定 ORB-SLAM3 特征词典文件。节点启动时会加载该文件，用于关键帧检索、回环检测和重定位；文件较大，加载完成后再播放 bag。 |
| `settings_path` | `<orbslam3_share>/config/monocular-inertial/XV_RGB_Fisheye_undistorted.yaml` | 指定 ORB-SLAM3 单目惯性配置文件，包含校正后相机内参、图像尺寸与频率、IMU 噪声参数、IMU 频率、时间偏移以及 `IMU.T_b_c1` 相机—IMU 外参。 |
| `camera_topic` | `/xv_sdk/SN250801DR48FB26001253/rgb_fisheye_undistorted/image` | 指定单目惯性节点订阅的校正 RGB 图像 topic。输入消息类型为 `sensor_msgs/msg/Image`；消息的 `header.frame_id` 同时决定静态 TF 的相机子坐标系名称，空值时回退为 `camera_optical_frame`。 |
| `imu_topic` | `/xv_sdk/SN250801DR48FB26001253/imu` | 指定单目惯性节点订阅的 IMU topic，输入消息类型为 `sensor_msgs/msg/Imu`。节点使用角速度和线加速度，并按消息时间戳与图像进行惯性预积分和同步。 |
| `feature_mask_path` | `<orbslam3_share>/config/masks/mask.png` | 指定与校正 RGB 图像逐像素对齐的二值特征掩膜。原始图像保持不变，ORB-SLAM3 在候选特征进入八叉树分配和描述子计算前按 mask 过滤：白色像素允许，黑色像素排除；空字符串表示禁用 mask。默认文件尺寸为 `1280 × 1280`，黑色排除区域约占 `31.44%`。 |
| `max_path_length` | `10000` | 限制 `/orbslam3/path` 中保留的实时 body 位姿数量，避免长时间运行时消息持续增大。正数表示最多保留最近 N 个位姿，`0` 表示无限累计，负数会导致节点启动失败。 |
| `save_keyframe_trajectory` | `false` | 控制节点正常退出时是否在启动命令的当前工作目录保存 `KeyFrameTrajectory.txt`。默认不保存；需要导出 ORB-SLAM3 相机关键帧轨迹时设为 `true`。 |
| `use_viewer` | `true` | 控制是否启用 ORB-SLAM3 自带的 Pangolin Viewer，用于查看地图点、关键帧和当前相机状态。无图形界面的离线运行应设为 `false`。 |
| `use_rviz` | `false` | 控制 launch 是否启动 RViz2。设为 `true` 时加载 `rviz_config_path`，显示校正 RGB 图像、`map -> body_link` 动态 TF、相机静态 TF、body 位姿和 Path。 |
| `rviz_config_path` | `<orbslam3_share>/rviz/xv_rgb_fisheye_undistorted_imu.rviz` | 指定 `use_rviz:=true` 时 RViz2 加载的配置文件，可替换为自定义 RViz 配置；`use_rviz:=false` 时不会使用该文件。 |

本次无界面离线回放需要覆盖以下参数：

```text
start_xv_sdk:=false
use_viewer:=false
use_rviz:=false
```

`max_path_length` 使用默认值 `10000`。传入负数时节点会启动失败。

使用实际夹爪 mask 时传入与 `1280 × 1280` 校正图像逐像素对齐的文件：

```bash
ros2 launch orbslam3 xv_rgb_fisheye_undistorted_imu.launch.py \
  feature_mask_path:=/绝对路径/gripper_mask.png
```

临时禁用 mask：

```bash
ros2 launch orbslam3 xv_rgb_fisheye_undistorted_imu.launch.py \
  feature_mask_path:=""
```

节点启动时会打印 mask 路径、尺寸和排除比例。首次处理图像时会校验 mask 尺寸；尺寸不一致、文件无法读取或 mask 为全黑图时会停止启动或处理。ORB-SLAM3 使用最近邻插值生成各层 mask，在 FAST 检测后仅排除关键点中心落在零值像素上的候选点，不会额外腐蚀允许区域。

## 步骤 4：启动离线单目惯性节点

打开终端 1。先创建结果目录，节点退出时会把关键帧轨迹写入该目录：

```bash
RESULT_DIR=/tmp/orbslam3_0721
mkdir -p "$RESULT_DIR"
cd "$RESULT_DIR"

source /opt/ros/jazzy/setup.bash
source $HOME/work/slam/ORB_SLAM3_ROS2/install/local_setup.bash

ros2 launch orbslam3 xv_rgb_fisheye_undistorted_imu.launch.py \
  start_xv_sdk:=false \
  use_viewer:=false \
  use_rviz:=false \
  max_path_length:=10000 \
  save_keyframe_trajectory:=true
```

有 GUI 时可以选择，不保留轨迹，可视化启动

```bash
source install/setup.bash

ros2 launch orbslam3 xv_rgb_fisheye_undistorted_imu.launch.py \
  start_xv_sdk:=false \
  use_viewer:=true \
  use_rviz:=true \
  max_path_length:=10000 \
  save_keyframe_trajectory:=false
```

等待终端 1 出现：

```text
Vocabulary loaded!
Monocular-Inertial
```

看到上述日志后再执行下一步。词典加载期间播放 bag 会丢失开头数据。

## 步骤 5：以 1.0 倍速回放图像和 IMU

打开终端 2：

```bash
source /opt/ros/jazzy/setup.bash

BAG=$HOME/datasets/ros2bag/0721
IMAGE_TOPIC=/xv_sdk/SN250801DR48FB26001253/rgb_fisheye_undistorted/image
IMU_TOPIC=/xv_sdk/SN250801DR48FB26001253/imu

ros2 bag play "$BAG" \
  --rate 1.0 \
  --topics \
  "$IMAGE_TOPIC" \
  "$IMU_TOPIC"
```

图像输入约为 60 Hz，配置中的 `ROS.TargetFps: 30` 会平均保留一半图像。运行日志中
出现“按目标帧率丢弃图像”属于正常限帧。稳定运行时日志应满足：

- `跟踪/限帧后` 接近 `1.000`；
- `队列溢出丢弃=0`；
- `过滤IMU=0`；
- 最近跟踪耗时低于约 `33 ms`。

需要降低处理压力时，停止当前回放并改为 0.5 倍速：

```bash
ros2 bag play "$BAG" \
  --rate 0.5 \
  --topics \
  "$IMAGE_TOPIC" \
  "$IMU_TOPIC"
```

0.5 倍速下 ROS 观察到的位姿频率约为 15 Hz，对应 bag 时间约为 30 Hz。

## 步骤 6：检查位姿、Path 和 TF

在 bag 回放期间打开终端 3：

```bash
source /opt/ros/jazzy/setup.bash
source $HOME/work/slam/ORB_SLAM3_ROS2/install/local_setup.bash
```

检查实时机体位姿：

```bash
ros2 topic hz /orbslam3/body_pose
ros2 topic echo /orbslam3/body_pose --once
```

检查累计 Path：

```bash
ros2 topic echo /orbslam3/path --once --no-arr
```

检查动态 TF：

```bash
ros2 run tf2_ros tf2_echo map body_link
```

检查相机静态 TF：

```bash
ros2 run tf2_ros tf2_echo body_link rgb_optical_frame
```

输出含义：

- `/orbslam3/body_pose`：当前 Kalibr IMU body 在 ORB `map` 中的位姿；
- `/orbslam3/path`：有效 body 位姿序列，长度不超过 `max_path_length`；
- `map -> body_link`：与 `body_pose` 同时间戳、同数值的动态 TF；
- `body_link -> rgb_optical_frame`：由 `IMU.T_b_c1` 生成的静态 TF。

输入图像 `header.frame_id` 为空时，静态 TF 子坐标系使用
`camera_optical_frame`。`tf2_echo` 在首个有效位姿发布前可能显示
`frame does not exist`，位姿开始发布后会自动输出变换。

`map` 和 `body_link` 保留 ORB-SLAM3/Kalibr 原生数值语义。位姿、Path 和动态 TF
从当前地图完成惯性 BA1 并连续稳定 1 秒后开始发布，并且要求跟踪状态为 `OK` 或
`OK_KLT`。这个门控会过滤初始定尺度、重力对齐、BA1 优化收敛尾部以及新地图重新
初始化期间的坐标调整。进入 `LOST` 后暂停发布，恢复有效跟踪时清空旧 Path 并重新累计。

## 步骤 7：使用 RViz2 查看位姿和轨迹

先停止步骤 4 中的节点，再在终端 1 重新启动并启用 RViz2：

```bash
cd /tmp/orbslam3_0721
source /opt/ros/jazzy/setup.bash
source $HOME/work/slam/ORB_SLAM3_ROS2/install/local_setup.bash

ros2 launch orbslam3 xv_rgb_fisheye_undistorted_imu.launch.py \
  start_xv_sdk:=false \
  use_viewer:=false \
  use_rviz:=true \
  max_path_length:=10000 \
  save_keyframe_trajectory:=true
```

等待词典加载完成，然后在终端 2 重新执行步骤 5 的 bag 回放命令。

RViz2 默认显示：

- Fixed Frame：`map`；
- Grid；
- `map -> body_link -> rgb_optical_frame` TF；
- `/orbslam3/body_pose`；
- `/orbslam3/path`；
- 校正 RGB 图像。

需要使用其他 RViz2 配置时：

```bash
ros2 launch orbslam3 xv_rgb_fisheye_undistorted_imu.launch.py \
  start_xv_sdk:=false \
  use_viewer:=false \
  use_rviz:=true \
  rviz_config_path:=/绝对路径/custom.rviz
```

## 步骤 8：正常退出并检查轨迹文件

步骤 4 或步骤 7 已传入 `save_keyframe_trajectory:=true`。bag 播放结束后，在终端 1
按 `Ctrl-C`，等待以下日志完成：

```text
Shutdown
Saving keyframe trajectory to KeyFrameTrajectory.txt ...
```

检查轨迹文件：

```bash
cd /tmp/orbslam3_0721

test -s KeyFrameTrajectory.txt
wc -l KeyFrameTrajectory.txt
head KeyFrameTrajectory.txt
```

两类轨迹的含义：

- `/orbslam3/path` 是有效跟踪帧对应的实时 body 位姿序列；
- `KeyFrameTrajectory.txt` 是 ORB-SLAM3 相机关键帧轨迹，采用 TUM 文本格式。

### 0721 bag 的实测结论

2026-07-22 使用 1.0 倍速、关闭 Pangolin Viewer 和 RViz2 完整回放，输入链路满足：

- 限帧后的图像跟踪处理率约为 `99.9%`；
- 图像队列溢出为 `0`；
- IMU 回跳/重复过滤为 `0`；
- 单帧处理通常为 `15 ms` 至 `20 ms`，低于 30 Hz 的 `33.3 ms` 帧周期。

这些指标排除了 ROS 2 同步、处理积压和回放速度导致的丢帧。特征 mask 临时禁用后，
初始化阶段还会增加一次丢跟踪，因此应保留默认 mask。把 ORB 特征数从 1200 增加到
2000 也没有改善快速转向处的跟踪。

末段连续出现 `Fail to track local map!` 时，图像仍然清晰，离线检查每帧可检测到
2000 个以上的 ORB 候选点；运行时日志中的 `detected` 也保持在配置上限附近。失败开始
时 `map_inliers` 约为 8 至 13，随后降到 0 至 2。该现象表示特征提取仍在工作，局部地图
关联已经不足以通过跟踪判定。尝试放宽投影搜索半径和内点阈值后仍然创建了新地图，且会
降低误匹配保护，因此没有保留这组实验参数。

此前 Viewer 没有处理 ORB-SLAM3 的 `RECENTLY_LOST` 绘制分支：状态进入短时恢复窗口后，
它会显示原始图像，却不绘制任何特征覆盖层，视觉效果就是特征点突然全部消失。当前代码
已经补齐该状态；绿色表示局部地图匹配点，蓝色表示仅用于视觉里程计的匹配点，橙色表示
已经检测到、但在 `RECENTLY_LOST` 或 `LOST` 状态下未能关联到地图的特征点。状态栏同时
显示 `Detected` 和 `Map matches`，可以直接区分特征提取失败与地图关联失败。

诊断日志采用限频输出，首次失败及之后每连续 30 次失败记录一次：

- `detected`：当前帧提取到的特征数；
- `map_inliers`：通过当前跟踪判定的地图内点数；
- `attempted_local_map`：本帧是否实际执行了局部地图跟踪；
- `consecutive_failures`：连续局部地图跟踪失败次数。

另外对 1280×1280、60 Hz 配置做了对比回放。单帧处理约需 23 至 30 ms，超过 60 Hz
对应的 16.7 ms 周期，最终发生 689 次图像队列溢出，只处理约 68.2% 的输入图像。
960×960、30 Hz 配置处理率为 100%，队列溢出为 0，因此当前配置保持
`Camera.newWidth/Height: 960`、`Camera.fps: 30`、`ROS.TargetFps: 30`、
`ROS.MaxImageQueueSize: 2` 和 `ROS.ImuReorderWindowSec: 0.01`。

这段数据的前约 3 秒接近静止，后续运动以原地旋转为主。ORB-SLAM3 只有在相邻关键帧
产生足够平移时才累计惯性初始化运动时间；本次多轮运行在快速转向前只累计约 6 秒至
9 秒，而惯性 BA2 需要累计超过 15 秒。画面在约 30 秒处快速转向窗口和强光区域，
纯视觉模式在同一位置也会出现 `Fail to track local map!`，说明直接触发条件是视差和
可重复视觉特征不足。惯性 BA2 尚未完成会进一步降低丢失后的恢复能力。

当前配置和代码包含以下保护：

- `IMU.DeferLowMotionReset: 1`：低运动阶段延后 LocalMapping 主动清图；
- `IMU.DeferPreBA2TrackingReset: 1`：已经完成初始 IMU 对齐、BA2 尚未完成时，先使用
  `RECENTLY_LOST` 窗口尝试恢复，避免单次短暂失败立即重置地图；
- `/orbslam3/body_pose`、Path 和动态 TF 从惯性 BA1 连续稳定 1 秒后才发布；跟踪丢失和
  新地图重新完成该门控之前均暂停发布，防止把不稳定尺度或新地图坐标直接表现为位姿跳变。

保护可以消除初始化调整和活动地图切换造成的已发布位姿跳变。若快速转向期间连续多帧
没有足够特征，ORB-SLAM3 仍会进入 `LOST` 并最终创建新地图，保护无法生成缺失的视觉
约束。因此 0721 bag 适合验证消息链路和失败保护，不适合作为稳定 VIO 轨迹的验收数据；
非空 `KeyFrameTrajectory.txt` 也不能单独证明惯性初始化和连续跟踪成功。

加入 BA1 完成门控后，首轮录制仍在 BA1 刚完成约 0.13 秒处观察到一次 `0.322 m` 的
优化收敛调整。增加 1 秒稳定等待后再次完整回放，最终发布 335 条位姿、覆盖 11.18 秒：

- 相邻平移 P95 为 `0.0257 m`，最大值为 `0.0507 m`；
- 相邻旋转 P95 为 `1.57°`，最大值为 `4.51°`；
- 相邻位姿时间间隔中位数为 `0.0333 s`，最大值为 `0.0666 s`；
- 以平移大于 `0.1 m`、旋转大于 `10°` 或间隔大于 `0.1 s` 为阈值，跳变事件为 `0`。

快速转向导致连续丢跟踪后，位姿发布会停住；新地图未重新通过 BA1 稳定门控前不会输出
另一套地图坐标。

重新采集验收数据时，建议按以下顺序操作：

1. 对准纹理丰富、静态、远近层次明显的场景，避免夹爪或动态物体占据主要视野。
2. 开始后先平稳保持约 1 秒，再进行至少 15 至 20 秒的平滑多方向平移，配合适度的
   多轴转动；平移需要产生清晰视差。
3. 看到 `start VIBA 1`、`end VIBA 1` 后继续保持充分平移，等待 BA2 完成，再进行快速
   转向和正式作业。
4. 避免运动模糊、直接朝向强光窗口以及长时间原地转动。降低 bag 回放速率不会补足
   这些视觉和运动约束。

若应用场景始终以原地旋转为主，可以使用纯单目模式获得更宽松的视觉重定位行为，代价是
轨迹没有可靠的米制尺度和重力对齐。

## 步骤 9：切换到 XV 实时设备

实时模式使用默认参数 `start_xv_sdk:=true`。依次加载 ROS 2、XV SDK 工作空间和当前
工作空间：

```bash
source /opt/ros/jazzy/setup.bash
source /path/to/xv_ws/install/local_setup.bash
source $HOME/work/slam/ORB_SLAM3_ROS2/install/local_setup.bash

ros2 launch orbslam3 xv_rgb_fisheye_undistorted_imu.launch.py
```

默认会启动 XV SDK、启用 RGB 鱼眼校正、打开 Pangolin Viewer，并使用默认序列号对应
的图像与 IMU topic。

指定其他 camchain 文件：

```bash
ros2 launch orbslam3 xv_rgb_fisheye_undistorted_imu.launch.py \
  rgb_fisheye_calibration_path:=/绝对路径/camchain-imucam.yaml
```

设备序列号变化时同时覆盖图像和 IMU topic：

```bash
ros2 launch orbslam3 xv_rgb_fisheye_undistorted_imu.launch.py \
  camera_topic:=/xv_sdk/实际序列号/rgb_fisheye_undistorted/image \
  imu_topic:=/xv_sdk/实际序列号/imu
```

需要 RViz2 并关闭 Pangolin 时：

```bash
ros2 launch orbslam3 xv_rgb_fisheye_undistorted_imu.launch.py \
  use_viewer:=false \
  use_rviz:=true
```
