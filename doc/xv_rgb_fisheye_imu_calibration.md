# XV RGB 鱼眼相机与 IMU 标定流程

本文说明如何为当前 `orbslam3` ROS 2 wrapper 标定 XV RGB 鱼眼相机和 IMU，并把 Kalibr/Allan 结果写入 `config/monocular-inertial/XV_RGB_Fisheye.yaml`，用于 `monocular-inertial` 节点。

目标链路使用：

- 图像 topic：`/xv_sdk/SN250801DR48FB26001253/rgb/image`
- IMU topic：`/xv_sdk/SN250801DR48FB26001253/imu`
- ORB-SLAM3 模式：`IMU_MONOCULAR`
- 相机模型：`KannalaBrandt8`
- 输出配置：`config/monocular-inertial/XV_RGB_Fisheye.yaml`

## 1. 数据和文件

建议使用以下数据分工：

- `/mnt/data/slam/my_umi_rosbag/3`：AprilGrid 相机和 IMU-camera 标定数据，目录内已有 `cam0/` 和 `imu/imu_clean.csv`。
- `/mnt/data/slam/my_umi_rosbag/1`：长时间静止或缓慢运动 IMU 数据，用于估计 Allan 噪声。
- `/mnt/data/slam/my_umi_rosbag/2_filtered_vins_aux`：SLAM 目标数据，用于最终运行和 smoke test。

你还需要准备与 3 号 AprilGrid 标定板匹配的 Kalibr `target.yaml`。该文件必须描述标定板类型、行列数、格子间距和标签间距，例如：

```yaml
target_type: aprilgrid
tagCols: 6
tagRows: 6
tagSize: 0.088
tagSpacing: 0.3
```

上面的数值只是格式示例，需要替换成实际标定板参数。

## 2. 标定工具安装

本文默认在 Ubuntu 24.04 + ROS 2 Jazzy 主环境中整理数据、运行当前 ROS 2 节点，并使用 Docker 运行 Kalibr。Kalibr 官方文档建议 ROS 2 用户通过 Docker 使用 Kalibr，标定输入仍使用 ROS 1 bag；因此 ROS 2 bag 需要先转换为 ROS 1 bag，或把图片目录和 IMU CSV 打包为 Kalibr 可读 bag。

参考资料：

- Kalibr 安装说明：https://github.com/ethz-asl/kalibr/wiki/installation
- Kalibr ROS 2 标定指南：https://github.com/ethz-asl/kalibr/wiki/ROS2-Calibration-Using-Kalibr
- ROS 2 Allan 工具：https://github.com/Autoliv-Research/allan_variance_ros2
- ROS 1 Allan 工具：https://github.com/ori-drs/allan_variance_ros

### 2.1 安装前提

先安装 Docker Engine，并确认普通用户可以运行 Docker：

```bash
docker run hello-world
```

再安装基础命令行工具：

```bash
sudo apt update
sudo apt install -y git python3-pip x11-xserver-utils
```

如果需要显示 Kalibr 图形窗口或报告预览，可以在运行容器前允许本地 X11 访问：

```bash
xhost +local:root
```

### 2.2 构建 Kalibr Docker 镜像

拉取 Kalibr 源码并构建 ROS 1 Noetic 对应的 Docker 镜像：

```bash
mkdir -p ~/tools
cd ~/tools
git clone https://github.com/ethz-asl/kalibr.git
cd kalibr
docker build -t kalibr -f Dockerfile_ros1_20_04 .
```

本文后续命令默认镜像名为 `kalibr`。如果本机已经有可用 Kalibr 镜像，可以把命令中的 `kalibr` 替换成实际镜像名。

验证容器内 Kalibr 命令可执行：

```bash
docker run --rm --entrypoint /bin/bash kalibr -lc "\
  source /catkin_ws/devel/setup.bash && \
  rospack find kalibr && \
  test -x /catkin_ws/devel/.private/kalibr/lib/kalibr/kalibr_calibrate_cameras && \
  test -x /catkin_ws/devel/.private/kalibr/lib/kalibr/kalibr_calibrate_imu_camera && \
  test -x /catkin_ws/devel/.private/kalibr/lib/kalibr/kalibr_bagcreater && \
  printf '%s\n' \
    kalibr_calibrate_cameras \
    kalibr_calibrate_imu_camera \
    kalibr_bagcreater"
```

如果输出 Kalibr 包路径和三个工具名，且命令退出码为 0，说明 Kalibr 包和三个命令入口已经在容器环境中可用。Kalibr Dockerfile 默认 `ENTRYPOINT` 会直接进入 `/catkin_ws` 下的交互 shell；非交互执行命令时需要用 `--entrypoint /bin/bash` 覆盖默认入口。

### 2.3 安装 ROS 2 bag 转换工具

Kalibr ROS 2 指南使用 `rosbags` 把 rosbag2 转换为 ROS 1 bag。当前机器如果默认 `python3` 指向 conda 环境，建议明确使用系统 Python 安装，避免工具被安装到 conda 环境中：

```bash
which python3
/usr/bin/python3 --version
```

使用系统 Python 安装到当前用户环境：

```bash
/usr/bin/python3 -m pip install --user -U "rosbags>=0.9.12"
```

确认命令可用：

```bash
export PATH="$HOME/.local/bin:$PATH"
which rosbags-convert
rosbags-convert --help
```

把 rosbag2 转为 ROS 1 bag 的示例命令如下：

```bash
ROS2_BAG=/mnt/data/slam/my_umi_rosbag/3
ROS1_BAG=/mnt/data/slam/my_umi_rosbag/3/xv_rgb_imu_calib.bag

rosbags-convert --src "$ROS2_BAG" --dst "$ROS1_BAG"
```

转换后可以把生成的 `.bag` 挂载进 Kalibr 容器，并在 `kalibr_calibrate_cameras` 或 `kalibr_calibrate_imu_camera` 中作为 `--bag` 输入。

### 2.4 安装 Allan 方差工具

ROS 2 场景优先使用 `Autoliv-Research/allan_variance_ros2`。该工具可以从 ROS 2 IMU bag 估计噪声参数，并生成 Kalibr 使用的 `imu.yaml`：

```bash
mkdir -p ~/allan_ws/src
cd ~/allan_ws/src
git clone https://github.com/Autoliv-Research/allan_variance_ros2.git
cd ~/allan_ws
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
```

如果选择 `ori-drs/allan_variance_ros`，建议在 ROS 1 Noetic 或对应 Docker 环境中使用。该工具适合 ROS 1 bag 输入；使用前应先用 `rosbags-convert` 把 ROS 2 IMU bag 转成 ROS 1 bag。

Allan 工具输出的 `imu.yaml` 字段会在后文映射到 ORB-SLAM3 的 IMU 噪声配置。不同工具对随机游走的命名和单位可能不同，写入配置前需要按工具文档复核单位。

### 2.5 工具链安装验证

安装完成后建议逐项验证：

```bash
docker run hello-world

docker run --rm --entrypoint /bin/bash kalibr -lc "\
  source /catkin_ws/devel/setup.bash && \
  rospack find kalibr && \
  test -x /catkin_ws/devel/.private/kalibr/lib/kalibr/kalibr_calibrate_cameras && \
  test -x /catkin_ws/devel/.private/kalibr/lib/kalibr/kalibr_calibrate_imu_camera && \
  test -x /catkin_ws/devel/.private/kalibr/lib/kalibr/kalibr_bagcreater && \
  printf '%s\n' \
    kalibr_calibrate_cameras \
    kalibr_calibrate_imu_camera \
    kalibr_bagcreater"

rosbags-convert --help

cd ~/allan_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
```

这些验证只确认工具链安装和命令入口可用，不代表相机、IMU 或外参标定结果有效。标定结果仍需要结合 Kalibr 报告、Allan 输出单位、重投影误差、残差和 smoke test 判断。

## 3. 标定输出和 ORB-SLAM3 字段对应关系

Kalibr 相机模型使用 `pinhole-equi`，对应 ORB-SLAM3 配置中的：

| Kalibr 字段 | ORB-SLAM3 字段 |
| --- | --- |
| `intrinsics: [fx, fy, cx, cy]` | `Camera1.fx/fy/cx/cy` |
| `distortion_coeffs: [k1, k2, k3, k4]` | `Camera1.k1/k2/k3/k4` |
| `resolution: [1280, 1280]` | `Camera.width/height` |
| `T_cam_imu` 或外参矩阵 | 按报告语义转换后写入 `IMU.T_b_c1` |

当前节点发布 body 位姿时也读取同一个 `IMU.T_b_c1`，并按以下公式把 ORB-SLAM3 返回的相机位姿转换为机体系位姿：

```text
Twb = Tcw.inverse() * IMU.T_b_c1.inverse()
```

因此 `IMU.T_b_c1` 必须表示机体系到相机系的变换。Kalibr 标准 `T_cam_imu` 通常表示把 IMU 坐标变换到相机坐标，语义与 `IMU.T_b_c1` 一致，可以直接写入。若你的标定脚本或后处理文件给出的是相机到 IMU 的矩阵，需要先取逆。写入前应结合报告中的坐标系说明复核一次。

## 4. 生成 Kalibr 输入数据

如果已经有 Kalibr 支持的 `cam0/` 图片目录和 `imu/imu_clean.csv`，可以直接打包成 Kalibr bag。建议先整理目录：

```bash
CALIB_ROOT=/mnt/data/slam/my_umi_rosbag/3
find "$CALIB_ROOT/cam0" -maxdepth 1 -type f | head
head "$CALIB_ROOT/imu/imu_clean.csv"
```

Kalibr 常用 IMU CSV 格式为：

```text
timestamp,omega_x,omega_y,omega_z,alpha_x,alpha_y,alpha_z
```

其中 `timestamp` 通常为纳秒时间戳，角速度单位为 `rad/s`，线加速度单位为 `m/s^2`。如果你的 CSV 字段顺序不同，应先转换成 Kalibr 期望格式。

使用 `kalibr_bagcreater` 生成标定 bag：

```bash
CALIB_ROOT=/mnt/data/slam/my_umi_rosbag/3
KALIBR_WS=/work

docker run --rm -it \
  -v "$CALIB_ROOT:$KALIBR_WS/calib_data" \
  --entrypoint /bin/bash \
  kalibr \
  -lc "\
    source /catkin_ws/devel/setup.bash && \
    rosrun kalibr kalibr_bagcreater \
      --folder $KALIBR_WS/calib_data \
      --output-bag $KALIBR_WS/calib_data/xv_rgb_imu_calib.bag"
```

如果本机 Kalibr Docker 镜像名不同，把 `kalibr` 替换成实际镜像名。

## 5. Allan 噪声估计

用长时间 IMU 数据估计噪声参数，输出 Kalibr `imu.yaml`。如果 1 号数据是 rosbag2，先转换或提取 IMU CSV：

```bash
IMU_BAG=/mnt/data/slam/my_umi_rosbag/1
ros2 bag info "$IMU_BAG"
```

推荐方案：

- 使用 `allan_variance_ros2` 从 ROS 2 IMU bag 估计 Allan 方差，并生成 Kalibr `imu.yaml`。

备选方案：

- 使用 `imu_utils` 或 `allan_variance_ros` 从 ROS 1 bag 估计 Allan 方差；如果原始数据是 rosbag2，先转换为 ROS 1 bag。
- 使用已有 Python/Matlab Allan 工具读取 IMU CSV，计算噪声密度和随机游走。

Kalibr `imu.yaml` 需要包含以下字段：

```yaml
rostopic: /imu0
update_rate: 381.0
accelerometer_noise_density: 0.0
accelerometer_random_walk: 0.0
gyroscope_noise_density: 0.0
gyroscope_random_walk: 0.0
```

把 Allan 结果写入 ORB-SLAM3 时对应关系为：

| Allan/Kalibr 字段 | ORB-SLAM3 字段 |
| --- | --- |
| `gyroscope_noise_density` | `IMU.NoiseGyro` |
| `accelerometer_noise_density` | `IMU.NoiseAcc` |
| `gyroscope_random_walk` | `IMU.GyroWalk` |
| `accelerometer_random_walk` | `IMU.AccWalk` |
| `update_rate` | `IMU.Frequency` |

注意单位：

- 陀螺噪声密度：`rad/s/sqrt(Hz)`
- 加速度噪声密度：`m/s^2/sqrt(Hz)`
- 陀螺随机游走：`rad/s^2/sqrt(Hz)` 或 Kalibr 工具输出定义对应值
- 加速度随机游走：`m/s^3/sqrt(Hz)` 或 Kalibr 工具输出定义对应值

不同 Allan 工具对随机游走的命名略有差异，写入前应确认工具文档中的单位。

## 6. 相机内参与畸变标定

运行单相机标定，使用 `pinhole-equi` 模型：

```bash
CALIB_ROOT=/mnt/data/slam/my_umi_rosbag/3
TARGET=/mnt/data/slam/my_umi_rosbag/target.yaml

test -d "$CALIB_ROOT"
test -f "$CALIB_ROOT/xv_rgb_imu_calib.bag"
test -f "$TARGET"
file "$TARGET"

docker run --rm -it \
  -v "$CALIB_ROOT:/work/calib_data" \
  -v "$TARGET:/work/target.yaml:ro" \
  --entrypoint /bin/bash \
  kalibr \
  -lc "\
    source /catkin_ws/devel/setup.bash && \
    cd /work/calib_data && \
    rosrun kalibr kalibr_calibrate_cameras \
      --bag xv_rgb_imu_calib.bag \
      --topics /cam0/image_raw \
      --models pinhole-equi \
      --target /work/target.yaml"
```

`TARGET` 必须指向宿主机上已经存在的普通 YAML 文件。Docker 绑定挂载文件时，如果左侧路径不存在，可能会创建同名目录，Kalibr 随后会报 `IsADirectoryError: /work/target.yaml`。运行前应确认 `file "$TARGET"` 输出为 YAML 文本文件或普通文本文件。

如果还没有实际标定板文件，先创建 `target.yaml`，并把参数替换成真实 AprilGrid 尺寸：

```bash
TARGET=/mnt/data/slam/my_umi_rosbag/target.yaml

cat > "$TARGET" <<'EOF'
target_type: aprilgrid
tagCols: 6
tagRows: 6
tagSize: 0.088
tagSpacing: 0.3
EOF
```

检查报告重点：

- 重投影误差应稳定，均值和尾部误差不能异常偏大。
- 图像四周应有足够 AprilGrid 观测，鱼眼边缘区域覆盖不足会影响畸变参数。
- `resolution` 应为 `[1280, 1280]`。

输出文件通常包含 `camchain-*.yaml` 和 PDF 报告。

## 7. IMU-camera 外参标定

使用相机标定结果和 Allan 生成的 `imu.yaml` 运行 IMU-camera 标定：

```bash
CALIB_ROOT=/mnt/data/slam/my_umi_rosbag/3
TARGET=/mnt/data/slam/my_umi_rosbag/target.yaml
IMU_YAML=/path/to/imu.yaml
CAMCHAIN=/mnt/data/slam/my_umi_rosbag/3/camchain-xv_rgb_imu_calib.yaml

test -f "$TARGET"
test -f "$IMU_YAML"
test -f "$CAMCHAIN"
file "$TARGET" "$IMU_YAML" "$CAMCHAIN"

docker run --rm -it \
  -v "$CALIB_ROOT:/work/calib_data" \
  -v "$TARGET:/work/target.yaml:ro" \
  -v "$IMU_YAML:/work/imu.yaml:ro" \
  -v "$CAMCHAIN:/work/camchain.yaml:ro" \
  --entrypoint /bin/bash \
  kalibr \
  -lc "\
    source /catkin_ws/devel/setup.bash && \
    cd /work/calib_data && \
    rosrun kalibr kalibr_calibrate_imu_camera \
      --bag xv_rgb_imu_calib.bag \
      --cam /work/camchain.yaml \
      --imu /work/imu.yaml \
      --target /work/target.yaml"
```

检查报告重点：

- `T_cam_imu` 平移量级应符合设备结构尺寸。
- 时间偏移 `timeshift cam0 to imu0` 不应异常跳变。
- IMU 残差和相机重投影误差应稳定。
- 多次标定结果的外参方向和数值应接近。

如果结果不稳定，优先检查 AprilGrid 覆盖、运动激励、图像曝光、IMU 时间戳单位和 CSV 字段顺序。

## 8. 写入 ORB-SLAM3 配置

复制模板并保留模板文件：

```bash
PKG=/home/scl/work/slam/ORB_SLAM3_ROS2
cp "$PKG/config/monocular-inertial/XV_RGB_Fisheye.yaml" \
   "$PKG/config/monocular-inertial/XV_RGB_Fisheye_calibrated.yaml"
```

在 `XV_RGB_Fisheye_calibrated.yaml` 中替换：

- `Camera1.fx`
- `Camera1.fy`
- `Camera1.cx`
- `Camera1.cy`
- `Camera1.k1`
- `Camera1.k2`
- `Camera1.k3`
- `Camera1.k4`
- `IMU.T_b_c1`
- `IMU.NoiseGyro`
- `IMU.NoiseAcc`
- `IMU.GyroWalk`
- `IMU.AccWalk`

示例外参格式：

```yaml
IMU.T_b_c1: !!opencv-matrix
   rows: 4
   cols: 4
   dt: f
   data: [r00, r01, r02, tx,
          r10, r11, r12, ty,
          r20, r21, r22, tz,
          0.0, 0.0, 0.0, 1.0]
```

如果 Kalibr 输出矩阵为 `T_cam_imu` 且报告语义为 IMU 到相机，直接写入上面的 `data`。如果报告语义为相机到 IMU，先求逆：

```python
"""将 Kalibr 4x4 外参矩阵求逆，便于写入 ORB-SLAM3 IMU.T_b_c1。"""

import numpy as np

T = np.array([
    [1.0, 0.0, 0.0, 0.0],
    [0.0, 1.0, 0.0, 0.0],
    [0.0, 0.0, 1.0, 0.0],
    [0.0, 0.0, 0.0, 1.0],
], dtype=float)

T_inv = np.linalg.inv(T)
print(T_inv)
```

## 9. 构建当前项目

```bash
cd /home/scl/work/slam/ORB_SLAM3_ROS2
source /opt/ros/jazzy/setup.bash

colcon build --symlink-install --packages-select orbslam3 \
  --cmake-args \
  -DPython3_EXECUTABLE=/usr/bin/python3 \
  -DOpenCV_DIR=/usr/lib/x86_64-linux-gnu/cmake/opencv4 \
  -DORB_SLAM3_ROOT_DIR=/home/scl/work/slam/ORB_SLAM3

source install/local_setup.bash
```

如果使用 ROS workspace 根目录构建，把 `cd` 切换到对应 workspace 根目录。

## 10. 运行 SLAM

启动单目惯性节点：

```bash
source /opt/ros/jazzy/setup.bash
source /home/scl/work/slam/ORB_SLAM3_ROS2/install/local_setup.bash

PKG=/home/scl/work/slam/ORB_SLAM3_ROS2
ros2 run orbslam3 monocular-inertial \
  "$PKG/vocabulary/ORBvoc.txt" \
  "$PKG/config/monocular-inertial/XV_RGB_Fisheye_calibrated.yaml" \
  false \
  --ros-args \
  -r camera:=/xv_sdk/SN250801DR48FB26001253/rgb/image \
  -r imu:=/xv_sdk/SN250801DR48FB26001253/imu
```

播放目标 bag：

```bash
ros2 bag play /mnt/data/slam/my_umi_rosbag/2_filtered_vins_aux \
  --topics \
  /xv_sdk/SN250801DR48FB26001253/rgb/image \
  /xv_sdk/SN250801DR48FB26001253/imu
```

运行时检查实时输出：

```bash
ros2 topic hz /orbslam3/body_pose
ros2 topic echo /orbslam3/body_pose --once
ros2 topic hz /orbslam3/path
```

退出节点后检查轨迹文件：

```bash
test -s KeyFrameTrajectory.txt
wc -l KeyFrameTrajectory.txt
head KeyFrameTrajectory.txt
```

## 11. Smoke test 标准

一次可接受的 smoke test 应满足：

- 节点完成 ORB vocabulary 加载。
- 图像和 IMU 时间戳单调，节点没有持续丢弃图像。
- `/orbslam3/body_pose` 有连续输出。
- `/orbslam3/path` 的 pose 数量持续增长。
- 退出后 `KeyFrameTrajectory.txt` 非空。

如果只做标定配置验证，可以先关闭 viewer：

```bash
ros2 run orbslam3 monocular-inertial \
  "$PKG/vocabulary/ORBvoc.txt" \
  "$PKG/config/monocular-inertial/XV_RGB_Fisheye_calibrated.yaml" \
  false \
  --ros-args \
  -r camera:=/xv_sdk/SN250801DR48FB26001253/rgb/image \
  -r imu:=/xv_sdk/SN250801DR48FB26001253/imu
```

## 12. 常见问题排查

### 初始化失败

优先检查：

- `IMU.T_b_c1` 方向是否正确。
- `IMU.NoiseGyro`、`IMU.NoiseAcc`、`IMU.GyroWalk`、`IMU.AccWalk` 是否为真实 Allan 结果。
- bag 播放倍率是否过高。
- 图像和 IMU 是否有足够重叠时间段。
- 标定段运动是否包含充分平移和转动。

### 轨迹尺度异常

单目惯性轨迹尺度主要受 IMU 噪声、外参、时间偏移和初始化运动影响。优先对比：

- Kalibr 报告中的时间偏移。
- Allan 噪声量级。
- `IMU.Frequency` 是否接近实测 `381.0`。
- `Camera.fps` 是否接近实测 `47.0`。

### 发布 topic 无输出

`/orbslam3/body_pose` 只在 ORB-SLAM3 跟踪状态为 `OK` 或 `OK_KLT` 时发布。可先检查节点是否收到数据：

```bash
ros2 topic hz /xv_sdk/SN250801DR48FB26001253/rgb/image
ros2 topic hz /xv_sdk/SN250801DR48FB26001253/imu
```

再检查 remap 是否和启动命令一致。

### OpenCV 链接警告

如果构建时出现 OpenCV 4.6 和 4.14 混用警告，优先使用系统 OpenCV：

```bash
-DOpenCV_DIR=/usr/lib/x86_64-linux-gnu/cmake/opencv4
```

并确认 ORB_SLAM3 本体和 ROS 2 wrapper 使用同一套 OpenCV 构建。

## 13. 建议归档内容

完成一次有效标定后，建议保存：

- `target.yaml`
- Allan 输出报告和 `imu.yaml`
- Kalibr `camchain-*.yaml`
- Kalibr `camchain-imucam-*.yaml`
- Kalibr PDF 报告
- 最终 `XV_RGB_Fisheye_calibrated.yaml`
- smoke test 命令和 `KeyFrameTrajectory.txt`

这些文件可以帮助复现实验，并在后续定位初始化失败或轨迹漂移问题时快速回溯参数来源。
