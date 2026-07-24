# XV RGB 鱼眼相机与 IMU 标定流程

本文说明如何为当前 `orbslam3` ROS 2 wrapper 标定 XV RGB 鱼眼相机和 IMU，并把 Kalibr/Allan 结果写入 `config/monocular-inertial/XV_RGB_Fisheye_calibrated.yaml`，用于 `monocular-inertial` 节点。当前设备归档的原始 Kalibr 结果位于 `$HOME/ros2_ws/src/xv_sdk_ros2/config/kalibr_data-camchain-imucam.yaml`。

标定输入和原始鱼眼运行链路使用：

- 原始鱼眼图像 topic：`/xv_sdk/SN250801DR48FB26001253/rgb/image`
- IMU topic：`/xv_sdk/SN250801DR48FB26001253/imu`
- ORB-SLAM3 模式：`IMU_MONOCULAR`
- 相机模型：`KannalaBrandt8`
- 原始 Kalibr 结果：`$HOME/ros2_ws/src/xv_sdk_ros2/config/kalibr_data-camchain-imucam.yaml`
- 输出配置：`config/monocular-inertial/XV_RGB_Fisheye_calibrated.yaml`

`cam0/` 中的标定图片来自未去畸变的原始 RGB 鱼眼帧。设备侧校正图像会发布到
`/xv_sdk/<序列号>/rgb_fisheye_undistorted/image`，该输出应使用 `PinHole` 配置，具体参见
`doc/xv_rgb_fisheye_undistorted_imu_live.md`。两条链路不能混用相机模型和畸变参数。

## 1. 数据和文件

建议使用以下数据分工：

以下命令默认数据根目录为 `${ORB_SLAM3_DATA_ROOT:-$HOME/datasets/rosbag}`；如果你的数据存放位置不同，需要按实际环境修改该路径。

- `${ORB_SLAM3_DATA_ROOT:-$HOME/datasets/rosbag}/3`：AprilGrid 相机和 IMU-camera 标定数据，目录内已有 `cam0/` 和 `imu/imu_clean.csv`。
- `${ORB_SLAM3_DATA_ROOT:-$HOME/datasets/rosbag}/1`：长时间静止或缓慢运动 IMU 数据，用于估计 Allan 噪声。
- `${ORB_SLAM3_DATA_ROOT:-$HOME/datasets/rosbag}/2_filtered_vins_aux`：SLAM 目标数据，用于最终运行和 smoke test。

你还需要准备与 3 号 AprilGrid 标定板匹配的 Kalibr target 文件。当前数据目录下已有 `${ORB_SLAM3_DATA_ROOT:-$HOME/datasets/rosbag}/april_6x6.yaml`，该文件描述 AprilGrid 的行列数、tag 尺寸和间距，例如：

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
ROS2_BAG=${ORB_SLAM3_DATA_ROOT:-$HOME/datasets/rosbag}/3
ROS1_BAG=${ORB_SLAM3_DATA_ROOT:-$HOME/datasets/rosbag}/3/xv_rgb_imu_calib.bag

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
rosdep install --from-paths src --ignore-src -r -y

env -i HOME="$HOME" USER="$USER" SHELL=/bin/bash TERM="${TERM:-xterm}" \
  PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin \
  bash --noprofile --norc -c "\
    source /opt/ros/jazzy/setup.bash && \
    colcon build --symlink-install --cmake-args -DBUILD_PLOT_IMU=OFF"
```

本机实际跑通时，`allan_variance_ros2` 做了三个小修正：

- `plot_imu` 改为可选目标，并默认关闭，避免构建 Allan 计算工具时强制下载 `rerun_cpp_sdk.zip`。
- `allan_variance` 计算结束后直接退出，避免同步计算完成后继续停在 `rclcpp::spin()`。
- 读取 IMU bag 时按 `measure_rate` 时间间隔降采样，使 `measure_rate: 100` 真正对应约 100 Hz 计算数据。

如果重新拉取上游仓库后又遇到 `rerun_cpp_sdk.zip` 下载失败、计算完成后进程不退出，或 `measure_rate` 没有降低缓存数据量，应先检查上述本地修正是否仍在。

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
colcon build --symlink-install --cmake-args -DBUILD_PLOT_IMU=OFF
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

其中 `IMU.T_b_c1` 是 `T_b_c`，表示把相机坐标变换到 IMU/body 坐标。Kalibr 标准
`T_cam_imu` 是 `T_c_b`，表示把 IMU/body 坐标变换到相机坐标，因此写入
`IMU.T_b_c1` 前需要取逆。若标定脚本已经输出相机到 IMU/body 的矩阵，可以直接写入。
写入前应结合报告中的坐标系说明复核一次。

## 4. 生成 Kalibr 输入数据

如果已经有 Kalibr 支持的 `cam0/` 图片目录和 `imu/imu_clean.csv`，可以直接打包成 Kalibr bag。建议先整理目录：

```bash
CALIB_ROOT=${ORB_SLAM3_DATA_ROOT:-$HOME/datasets/rosbag}/3
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
CALIB_ROOT=${ORB_SLAM3_DATA_ROOT:-$HOME/datasets/rosbag}/3
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

用长时间 IMU 数据估计噪声参数，输出 Kalibr `imu.yaml`。当前 1 号数据是 ROS 2 MCAP bag，包含一个 IMU topic，适合直接用 `allan_variance_ros2` 处理：

```bash
IMU_BAG=${ORB_SLAM3_DATA_ROOT:-$HOME/datasets/rosbag}/1
ros2 bag info "$IMU_BAG"
```

当前数据检查结果应接近：

```text
Files:             1_0.mcap
Storage id:        mcap
Duration:          68216.136665469s
Messages:          33644598
Topic:             /xv_sdk/SN250801DR48FB26001253/imu
Type:              sensor_msgs/msg/Imu
```

平均 IMU 频率约为：

```text
33644598 / 68216.136665469 = 493.2 Hz
```

### 5.1 准备 Allan 配置

`allan_variance_ros2` 的配置文件需要写明 IMU topic、原始频率、降采样频率、使用时长和起始偏移。建议先使用 12 小时数据，并跳过开始 5 分钟，避免刚放置设备时的扰动：

```bash
ALLAN_ROOT=${ORB_SLAM3_DATA_ROOT:-$HOME/datasets/rosbag}/1_allan
mkdir -p "$ALLAN_ROOT"

cat > "$ALLAN_ROOT/xv_imu_allan.yaml" <<'EOF'
imu_topic: "/xv_sdk/SN250801DR48FB26001253/imu"
imu_rate: 493.2
measure_rate: 100
sequence_duration: 43200
sequence_offset: 300
EOF
```

字段含义：

| 字段 | 建议值 | 说明 |
| --- | --- | --- |
| `imu_topic` | `/xv_sdk/SN250801DR48FB26001253/imu` | rosbag 中的 `sensor_msgs/msg/Imu` topic |
| `imu_rate` | `493.2` | 原始 IMU 平均频率，来自 `消息数 / 时长` |
| `measure_rate` | `100` | Allan 计算使用的降采样频率，可降低计算量 |
| `sequence_duration` | `43200` | 使用 12 小时数据，单位为秒 |
| `sequence_offset` | `300` | 跳过开始 5 分钟，单位为秒 |

如果设备在 12 小时窗口内被移动过，应调整 `sequence_offset` 或缩短 `sequence_duration`，选择尽量静止且温度变化平稳的数据段。

### 5.2 运行 Allan 方差计算

`allan_variance_ros2` 可以直接读取 MCAP bag，不需要播放 bag，也不需要转换为 ROS 1 bag。计算程序不会自动创建输出目录，因此先确认 `ALLAN_ROOT` 已存在：

```bash
IMU_BAG=${ORB_SLAM3_DATA_ROOT:-$HOME/datasets/rosbag}/1
ALLAN_ROOT=${ORB_SLAM3_DATA_ROOT:-$HOME/datasets/rosbag}/1_allan

cd ~/allan_ws

env -i HOME="$HOME" USER="$USER" SHELL=/bin/bash TERM="${TERM:-xterm}" \
  PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin \
  bash --noprofile --norc -c "\
    source /opt/ros/jazzy/setup.bash && \
    source $HOME/allan_ws/install/setup.bash && \
    ros2 run allan_variance_ros2 allan_variance \
      $IMU_BAG \
      $ALLAN_ROOT/xv_imu_allan.yaml \
      $ALLAN_ROOT \
      > $ALLAN_ROOT/run.log 2>&1"
```

计算完成后应生成：

```bash
test -s ${ORB_SLAM3_DATA_ROOT:-$HOME/datasets/rosbag}/1_allan/allan_variance.csv
wc -l ${ORB_SLAM3_DATA_ROOT:-$HOME/datasets/rosbag}/1_allan/allan_variance.csv
grep -E "Finished buffering data|Data written" ${ORB_SLAM3_DATA_ROOT:-$HOME/datasets/rosbag}/1_allan/run.log
```

当前 12 小时配置的实测结果为：降采样后缓存 `4260475` 条 IMU 测量，`allan_variance.csv` 输出 `9999` 行。若首次验证工具链，可以先把 `sequence_duration` 改为 `1200` 或 `10800`；正式结果建议使用更长的静止数据段。

### 5.3 生成 Kalibr imu.yaml

分析脚本会读取 `allan_variance.csv`，拟合 Allan 曲线，并把 Kalibr 需要的噪声字段写入当前工作目录下的 `imu.yaml`。因此应进入输出目录后再运行脚本：

```bash
cd ${ORB_SLAM3_DATA_ROOT:-$HOME/datasets/rosbag}/1_allan

env -i HOME="$HOME" USER="$USER" MPLBACKEND=Agg \
  PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin \
  /usr/bin/python3 \
    $HOME/allan_ws/src/allan_variance_ros2/src/allan_variance_ros2/scripts/analysis.py \
    --data allan_variance.csv \
    --config xv_imu_allan.yaml \
    > analysis.log 2>&1
```

输出文件包括：

```bash
test -s ${ORB_SLAM3_DATA_ROOT:-$HOME/datasets/rosbag}/1_allan/imu.yaml
test -s ${ORB_SLAM3_DATA_ROOT:-$HOME/datasets/rosbag}/1_allan/acceleration.png
test -s ${ORB_SLAM3_DATA_ROOT:-$HOME/datasets/rosbag}/1_allan/gyro.png
```

`imu.yaml` 中的 `rostopic` 应保持为真实 IMU topic，`update_rate` 应保持为 `493.2` 左右：

```yaml
rostopic: '/xv_sdk/SN250801DR48FB26001253/imu'
update_rate: 493.2
accelerometer_noise_density: 0.011362633638908732
accelerometer_random_walk: 0.0003604781951727939
gyroscope_noise_density: 0.00013340717339568767
gyroscope_random_walk: 4.905584115261025e-06
```

上面 4 个噪声字段是当前 `${ORB_SLAM3_DATA_ROOT:-$HOME/datasets/rosbag}/1` 数据的实测结果。换数据或调整 Allan 时间窗口后，应重新生成并替换。

### 5.4 检查 Allan 结果

检查 `acceleration.png` 和 `gyro.png`：

- 前段应有接近 `-1/2` 斜率的白噪声区。
- 后段应有接近 `+1/2` 斜率的随机游走区。
- 三轴曲线量级不应相差异常大。
- 如果曲线有明显突变，通常说明数据段内 IMU 被移动、受振动影响或温度变化太剧烈。

如果结果不稳定，优先尝试：

- 增大 `sequence_offset`，避开开头扰动。
- 缩短 `sequence_duration`，只使用确定静止的数据段。
- 检查采集期间设备是否放在稳固、减振、温度较稳定的位置。

推荐方案的核心流程为：

- 使用 `allan_variance_ros2` 从 ROS 2 IMU bag 估计 Allan 方差，并生成 Kalibr `imu.yaml`。

备选方案：

- 使用 `imu_utils` 或 `allan_variance_ros` 从 ROS 1 bag 估计 Allan 方差；如果原始数据是 rosbag2，先转换为 ROS 1 bag。
- 使用已有 Python/Matlab Allan 工具读取 IMU CSV，计算噪声密度和随机游走。

Kalibr `imu.yaml` 需要包含以下字段：

```yaml
rostopic: /xv_sdk/SN250801DR48FB26001253/imu
update_rate: 493.2
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
CALIB_ROOT=${ORB_SLAM3_DATA_ROOT:-$HOME/datasets/rosbag}/3
TARGET=${ORB_SLAM3_DATA_ROOT:-$HOME/datasets/rosbag}/april_6x6.yaml

test -d "$CALIB_ROOT"
test -f "$CALIB_ROOT/xv_rgb_imu_calib.bag"
test -f "$TARGET"
file "$TARGET"

docker run --rm -it \
  -v "$CALIB_ROOT:/work/calib_data" \
  -v "$TARGET:/work/april_6x6.yaml:ro" \
  --entrypoint /bin/bash \
  kalibr \
  -lc "\
    source /catkin_ws/devel/setup.bash && \
    cd /work/calib_data && \
    rosrun kalibr kalibr_calibrate_cameras \
      --bag xv_rgb_imu_calib.bag \
      --topics /cam0/image_raw \
      --models pinhole-equi \
      --target /work/april_6x6.yaml"
```

`TARGET` 必须指向宿主机上已经存在的普通 YAML 文件。Docker 绑定挂载文件时，如果左侧路径不存在，可能会创建同名目录，Kalibr 随后会报 `IsADirectoryError`。运行前应确认 `file "$TARGET"` 输出为 YAML 文本文件或普通文本文件。

如果还没有实际标定板文件，先创建 `april_6x6.yaml`，并把参数替换成真实 AprilGrid 尺寸：

```bash
TARGET=${ORB_SLAM3_DATA_ROOT:-$HOME/datasets/rosbag}/april_6x6.yaml

cat > "$TARGET" <<'EOF'
target_type: aprilgrid
tagCols: 6
tagRows: 6
tagSize: 0.088
tagSpacing: 0.3
EOF
```

如果改用 `${ORB_SLAM3_DATA_ROOT:-$HOME/datasets/rosbag}/checkerboard.yaml`，代表标定数据中拍摄的是棋盘格，Kalibr target 文件字段也会变为 `target_type: checkerboard`、`targetCols`、`targetRows`、`rowSpacingMeters` 和 `colSpacingMeters`。AprilGrid 适合自动识别带 ID 的 AprilTag 网格，部分遮挡和姿态变化时更稳；checkerboard 依赖棋盘格角点检测，图案更简单，但需要画面中角点清晰且覆盖充分。两者不能混用：拍摄 AprilGrid 就使用 `april_6x6.yaml`，拍摄棋盘格才使用 `checkerboard.yaml`。

检查报告重点：

- 重投影误差应稳定，均值和尾部误差不能异常偏大。
- 图像四周应有足够 AprilGrid 观测，鱼眼边缘区域覆盖不足会影响畸变参数。
- `resolution` 应为 `[1280, 1280]`。

输出文件通常包含 `camchain-*.yaml` 和 PDF 报告。

## 7. IMU-camera 外参标定

使用相机标定结果和 Allan 生成的噪声参数运行 IMU-camera 标定。注意这里使用的是第 4 节 `kalibr_bagcreater` 生成的 ROS 1 bag，该 bag 内的 IMU topic 是 `/imu_clean`，相机 topic 是 `/cam0/image_raw`。因此不能直接把 Allan 原始输出中的 `rostopic: /xv_sdk/.../imu` 用于这个 Kalibr bag。

先检查 bag 内 topic：

```bash
docker run --rm \
  -v ${ORB_SLAM3_DATA_ROOT:-$HOME/datasets/rosbag}/3:/work/calib_data \
  --entrypoint /bin/bash \
  kalibr \
  -lc "\
    source /catkin_ws/devel/setup.bash && \
    python3 - <<'PY'
import rosbag
bag = '/work/calib_data/xv_rgb_imu_calib.bag'
with rosbag.Bag(bag) as b:
    for topic, info in sorted(b.get_type_and_topic_info()[1].items()):
        print(topic, info.msg_type, info.message_count, info.frequency)
PY"
```

当前 bag 应输出：

```text
/cam0/image_raw sensor_msgs/Image 172 3.0019843029693565
/imu_clean sensor_msgs/Imu 28351 493.3398947081743
```

为 IMU-camera 标定创建一个专用 IMU YAML，只修改 `rostopic`，噪声参数仍使用第 5 节 Allan 结果：

```bash
cat > ${ORB_SLAM3_DATA_ROOT:-$HOME/datasets/rosbag}/1_allan/imu_xv_rgb_imu_calib.yaml <<'EOF'
accelerometer_noise_density: 0.011362633638908732
accelerometer_random_walk: 0.0003604781951727939
gyroscope_noise_density: 0.00013340717339568767
gyroscope_random_walk: 4.905584115261025e-06
rostopic: /imu_clean
update_rate: 493.2
EOF
```

再运行 IMU-camera 标定：

```bash
# 相机和 IMU-camera 标定数据目录，目录内应包含 xv_rgb_imu_calib.bag。
CALIB_ROOT=${ORB_SLAM3_DATA_ROOT:-$HOME/datasets/rosbag}/3
# AprilGrid 标定板参数文件，需要与实际标定板尺寸一致。
TARGET=${ORB_SLAM3_DATA_ROOT:-$HOME/datasets/rosbag}/april_6x6.yaml
# 与 xv_rgb_imu_calib.bag topic 匹配的 IMU 噪声参数文件。
IMU_YAML=${ORB_SLAM3_DATA_ROOT:-$HOME/datasets/rosbag}/1_allan/imu_xv_rgb_imu_calib.yaml
# 第 6 节相机内参与畸变标定输出的相机链文件。
CAMCHAIN=${ORB_SLAM3_DATA_ROOT:-$HOME/datasets/rosbag}/3/xv_rgb_imu_calib-camchain.yaml
# 标定日志文件。
LOG=${ORB_SLAM3_DATA_ROOT:-$HOME/datasets/rosbag}/3/imu_camera_calibration_full.log

test -f "$TARGET"
test -f "$IMU_YAML"
test -f "$CAMCHAIN"
file "$TARGET" "$IMU_YAML" "$CAMCHAIN"

docker run --rm \
  -v "$CALIB_ROOT:/work/calib_data" \
  -v "$TARGET:/work/april_6x6.yaml:ro" \
  -v "$IMU_YAML:/work/imu.yaml:ro" \
  -v "$CAMCHAIN:/work/camchain.yaml:ro" \
  --entrypoint /bin/bash \
  kalibr \
  -lc "\
    source /catkin_ws/devel/setup.bash && \
    cd /work/calib_data && \
    rosrun kalibr kalibr_calibrate_imu_camera \
      --bag xv_rgb_imu_calib.bag \
      --cams /work/camchain.yaml \
      --imu /work/imu.yaml \
      --target /work/april_6x6.yaml \
      --dont-show-report" \
  > "$LOG" 2>&1

tail -n 80 "$LOG"
```

`--dont-show-report` 只禁止容器里弹出报告窗口，PDF 报告仍会生成。无显示环境下如果看到 `Unable to init server` 或 `Gdk-CRITICAL`，但命令最终退出码为 0 且结果文件生成，通常可以忽略。

当前数据完整运行后输出结果为：

```text
After Optimization (Results)
Reprojection error (cam0):     mean 0.2309385278160352, median 0.1884228377304846, std: 0.17987780937512207
Gyroscope error (imu0) [rad/s]:     mean 0.0027696514279387094, median 0.00226372808828341, std: 0.0025868181873392334
Accelerometer error (imu0) [m/s^2]: mean 0.04831288141062636, median 0.03894430407608586, std: 0.05758893522509604

Transformation T_cam0_imu0 (imu0 to cam0, T_ci):
[[ 0.99986001 -0.00761431  0.01489915  0.02186039]
 [ 0.00774522  0.99993173 -0.00874884 -0.01169692]
 [-0.01483152  0.00886301  0.99985073 -0.02367157]
 [ 0.          0.          0.          1.        ]]

cam0 to imu0 time: [s] (t_imu = t_cam + shift)
0.005642667607369631
```

检查报告重点：

- `T_cam_imu` 平移量级应符合设备结构尺寸。
- 时间偏移 `timeshift cam0 to imu0` 不应异常跳变。
- IMU 残差和相机重投影误差应稳定。
- 多次标定结果的外参方向和数值应接近。

结果文件包括：

- `${ORB_SLAM3_DATA_ROOT:-$HOME/datasets/rosbag}/3/xv_rgb_imu_calib-camchain-imucam.yaml`
- `${ORB_SLAM3_DATA_ROOT:-$HOME/datasets/rosbag}/3/xv_rgb_imu_calib-imu.yaml`
- `${ORB_SLAM3_DATA_ROOT:-$HOME/datasets/rosbag}/3/xv_rgb_imu_calib-results-imucam.txt`
- `${ORB_SLAM3_DATA_ROOT:-$HOME/datasets/rosbag}/3/xv_rgb_imu_calib-report-imucam.pdf`

这些文件由 Docker 容器写出时，所有者可能是 `root`。如果后续需要直接编辑，可在宿主机上执行：

```bash
sudo chown "$USER:$USER" ${ORB_SLAM3_DATA_ROOT:-$HOME/datasets/rosbag}/3/xv_rgb_imu_calib-*imucam* \
  ${ORB_SLAM3_DATA_ROOT:-$HOME/datasets/rosbag}/3/xv_rgb_imu_calib-imu.yaml
```

如果结果不稳定，优先检查 AprilGrid 覆盖、运动激励、图像曝光、IMU 时间戳单位和 CSV 字段顺序。

## 8. 写入 ORB-SLAM3 配置

仓库只保留两份 XV 单目惯性运行配置：

- `XV_RGB_Fisheye_calibrated.yaml`：输入包含 equidistant 畸变的原始或注册图像；
- `XV_RGB_Fisheye_undistorted.yaml`：输入为 XV SDK 已校正的理想针孔图像。

先确认当前设备的原始 Kalibr 文件和目标配置存在：

```bash
PKG=$HOME/work/slam/ORB_SLAM3_ROS2
KALIBR=$HOME/ros2_ws/src/xv_sdk_ros2/config/kalibr_data-camchain-imucam.yaml

test -s "$KALIBR"
test -s "$PKG/config/monocular-inertial/XV_RGB_Fisheye_calibrated.yaml"
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

如果 Kalibr 输出矩阵为标准 `T_cam_imu`，且报告语义为 IMU 到相机，需要先求逆再写入
上面的 `data`。如果报告给出的矩阵语义为相机到 IMU，可以直接写入：

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
cd $HOME/work/slam/ORB_SLAM3_ROS2
source /opt/ros/jazzy/setup.bash

colcon build --symlink-install --packages-select orbslam3 \
  --cmake-args \
  -DPython3_EXECUTABLE=/usr/bin/python3 \
  -DOpenCV_DIR=/usr/lib/x86_64-linux-gnu/cmake/opencv4 \
  -DORB_SLAM3_ROOT_DIR=$HOME/work/slam/ORB_SLAM3

source install/local_setup.bash
```

如果使用 ROS workspace 根目录构建，把 `cd` 切换到对应 workspace 根目录。

## 10. 运行 SLAM

启动单目惯性节点：

```bash
source /opt/ros/jazzy/setup.bash
source $HOME/work/slam/ORB_SLAM3_ROS2/install/local_setup.bash

PKG=$HOME/work/slam/ORB_SLAM3_ROS2
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
ros2 bag play ${ORB_SLAM3_DATA_ROOT:-$HOME/datasets/rosbag}/2_filtered_vins_aux \
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

### Allan 工具加载 Anaconda libstdc++ 失败

如果运行 `allan_variance_ros2` 时出现类似错误：

```text
$HOME/allan_ws/install/allan_variance_ros2/lib/allan_variance_ros2/allan_variance: $HOME/anaconda3/lib/libstdc++.so.6: version `GLIBCXX_3.4.32' not found
```

说明当前进程加载了 Anaconda 的 `libstdc++.so.6`。ROS 2 Jazzy 和 Allan 工具需要系统 GCC 对应的 `libstdc++`。如果 `allan_variance` 是在带 Anaconda 路径的环境中构建的，CMake 可能已经把 `$HOME/anaconda3/lib` 写入二进制 `RUNPATH`，此时只清理 `LD_LIBRARY_PATH` 仍然无效。

先检查二进制链接路径：

```bash
readelf -d $HOME/allan_ws/install/allan_variance_ros2/lib/allan_variance_ros2/allan_variance | grep -E "RPATH|RUNPATH"
ldd $HOME/allan_ws/install/allan_variance_ros2/lib/allan_variance_ros2/allan_variance | grep libstdc++
```

如果输出中出现 `$HOME/anaconda3/lib`，清理构建产物后用干净环境重建：

```bash
rm -rf $HOME/allan_ws/build $HOME/allan_ws/install $HOME/allan_ws/log

cd $HOME/allan_ws
env -i HOME=$HOME USER=scl SHELL=/bin/bash TERM="${TERM:-xterm}" \
  PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin \
  bash --noprofile --norc -c "\
    source /opt/ros/jazzy/setup.bash && \
    colcon build --symlink-install --cmake-args -DBUILD_PLOT_IMU=OFF"
```

重建后确认应看到系统库路径，且 `RUNPATH` 不应包含 Anaconda：

```bash
readelf -d $HOME/allan_ws/install/allan_variance_ros2/lib/allan_variance_ros2/allan_variance | grep -E "RPATH|RUNPATH"
ldd $HOME/allan_ws/install/allan_variance_ros2/lib/allan_variance_ros2/allan_variance | grep libstdc++
```

期望输出中的 `libstdc++.so.6` 来自 `/usr/lib/x86_64-linux-gnu/` 或 `/lib/x86_64-linux-gnu/`，不应来自 `$HOME/anaconda3/lib/`。

### analysis.py 加载 Anaconda matplotlib/numpy 失败

如果生成 `imu.yaml` 时出现类似错误：

```text
A module that was compiled using NumPy 1.x cannot be run in NumPy 2.x
ImportError: numpy.core.multiarray failed to import
```

说明 `python3` 或 `PYTHONPATH` 使用了 Anaconda 的 Python 包。使用系统 Python 和干净环境运行分析脚本：

```bash
cd ${ORB_SLAM3_DATA_ROOT:-$HOME/datasets/rosbag}/1_allan

env -i HOME="$HOME" USER="$USER" MPLBACKEND=Agg \
  PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin \
  /usr/bin/python3 \
    $HOME/allan_ws/src/allan_variance_ros2/src/allan_variance_ros2/scripts/analysis.py \
    --data allan_variance.csv \
    --config xv_imu_allan.yaml \
    > analysis.log 2>&1
```

运行前可确认系统 Python 依赖齐全：

```bash
env -i PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin \
  /usr/bin/python3 - <<'PY'
import numpy, scipy, matplotlib, yaml
print(numpy.__version__, scipy.__version__, matplotlib.__version__)
PY
```

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
- `IMU.Frequency` 是否接近实测 IMU 频率；当前 1 号 Allan 数据约为 `493.2`。
- `Camera.fps` 是否接近实测 `47`；ORB-SLAM3 新版 `Settings` 解析要求该字段写成整数。

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

- `april_6x6.yaml`，即当前 AprilGrid 标定板配置文件
- `checkerboard.yaml`，仅在使用棋盘格标定流程时需要
- Allan 输出报告和 `imu.yaml`
- Kalibr `camchain-*.yaml`
- Kalibr `camchain-imucam-*.yaml`
- Kalibr PDF 报告
- 最终 `XV_RGB_Fisheye_calibrated.yaml`
- smoke test 命令和 `KeyFrameTrajectory.txt`

这些文件可以帮助复现实验，并在后续定位初始化失败或轨迹漂移问题时快速回溯参数来源。
