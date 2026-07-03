# Ubuntu 24.04 + ROS 2 Jazzy 运行 ORB_SLAM3_ROS2 stereo-inertial

本文记录在 Ubuntu 24.04.4 和 ROS 2 Jazzy 环境下构建 `orbslam3`，并使用 `/mnt/data/slam/vicon_room1_ros2/V1_02_medium` 做双目惯性 smoke test 的步骤。

## 1. 环境确认

确认 ROS 发行版：

```bash
echo "$ROS_DISTRO"
```

期望输出为：

```text
jazzy
```

本机存在两个 OpenCV：

- ROS Jazzy 的 `cv_bridge` 使用系统 OpenCV 4.6.0，CMake 路径为 `/usr/lib/x86_64-linux-gnu/cmake/opencv4`。
- `/usr/local` 下还有 OpenCV 4.14.0，`pkg-config --modversion opencv4` 可能优先显示该版本。

构建 ORB_SLAM3 和本 ROS 2 wrapper 时应显式指定系统 OpenCV 4.6.0，避免与 `cv_bridge` 链接到不同 ABI。

当前 shell 的 `python3` 可能来自 Anaconda。构建 ROS 2 包时建议显式指定：

```bash
-DPython3_EXECUTABLE=/usr/bin/python3
```

## 2. 安装依赖

```bash
sudo apt-get install -y \
  ros-jazzy-pangolin \
  ros-jazzy-sophus \
  ros-jazzy-vision-opencv \
  ros-jazzy-message-filters
```

如果 `apt-get update` 因 `http://packages.ros.org/ros/ubuntu noble` 报 `404 Not Found`，说明系统里配置了不适用于 Ubuntu 24.04 Noble 的 ROS 1 源。ROS 2 Jazzy 源正常时，可以先直接安装上述包；长期使用建议移除或禁用该 ROS 1 源。

## 3. 构建 ORB_SLAM3

README 推荐的 fork 为 `zang09/ORB-SLAM3-STEREO-FIXED`。本教程将其放在 `/home/scl/work/slam/ORB_SLAM3`：

```bash
cd /home/scl/work/slam
git clone https://github.com/zang09/ORB-SLAM3-STEREO-FIXED.git ORB_SLAM3
cd ORB_SLAM3
```

构建时固定使用系统 OpenCV：

```bash
export OpenCV_DIR=/usr/lib/x86_64-linux-gnu/cmake/opencv4
./build.sh
```

如果手动调用 CMake，也传入同一个 OpenCV 路径：

```bash
cmake .. -DOpenCV_DIR=/usr/lib/x86_64-linux-gnu/cmake/opencv4
```

构建完成后应能看到：

```bash
ls /home/scl/work/slam/ORB_SLAM3/lib/libORB_SLAM3.so
ls /home/scl/work/slam/ORB_SLAM3/Thirdparty/DBoW2/lib/libDBoW2.so
ls /home/scl/work/slam/ORB_SLAM3/Thirdparty/g2o/lib/libg2o.so
```

## 4. 构建 ROS 2 wrapper

在 ROS 2 workspace 中构建本包：

```bash
cd /home/scl/work/slam/ORB_SLAM3_ROS2
source /opt/ros/jazzy/setup.bash

colcon build --symlink-install --packages-select orbslam3 \
  --cmake-args \
  -DPython3_EXECUTABLE=/usr/bin/python3 \
  -DOpenCV_DIR=/usr/lib/x86_64-linux-gnu/cmake/opencv4 \
  -DORB_SLAM3_ROOT_DIR=/home/scl/work/slam/ORB_SLAM3
```

`ORB_SLAM3_ROOT_DIR` 也可以通过环境变量提供：

```bash
export ORB_SLAM3_ROOT_DIR=/home/scl/work/slam/ORB_SLAM3
```

## 5. 解压词典

首次运行前解压 ORB 词典：

```bash
cd /home/scl/work/slam/ORB_SLAM3_ROS2
tar -xzf vocabulary/ORBvoc.txt.tar.gz -C vocabulary
```

生成文件为：

```text
vocabulary/ORBvoc.txt
```

## 6. 运行 stereo-inertial 节点

启动节点：

```bash
source /opt/ros/jazzy/setup.bash
source install/local_setup.bash

PKG=/home/scl/work/slam/ORB_SLAM3_ROS2
ros2 run orbslam3 stereo-inertial \
  "$PKG/vocabulary/ORBvoc.txt" \
  "$PKG/config/stereo-inertial/EuRoC.yaml" \
  true \
  false
```

`stereo-inertial` 参数格式：

```bash
ros2 run orbslam3 stereo-inertial \
  path_to_vocabulary \
  path_to_settings \
  do_rectify \
  [do_equalize] \
  [use_viewer]
```

参数含义：

- `path_to_vocabulary`：ORB 词典路径，例如 `vocabulary/ORBvoc.txt`。
- `path_to_settings`：ORB_SLAM3 配置文件路径，例如 `config/stereo-inertial/EuRoC.yaml`。该文件提供相机内参、双目矫正参数、IMU 噪声参数和 `Tbc` 外参。
- `do_rectify`：必填布尔值，`true` 表示按配置文件中的 `LEFT.*` 和 `RIGHT.*` 参数对左右目图像做 rectification，`false` 表示直接把输入图像送入 ORB_SLAM3。当前 Vicon/EuRoC 链路使用 `true`。
- `do_equalize`：可选布尔值，控制是否对左右目灰度图做 CLAHE 均衡化；未传入时默认 `false`。光照较差或对比度不足时可以尝试 `true`。
- `use_viewer`：可选布尔值，控制是否启用 ORB_SLAM3 Pangolin viewer；未传入时默认 `true`。远程或无显示环境建议设为 `false`。

上述示例中的 `true false` 分别表示：启用双目图像 rectification，不启用 CLAHE 图像均衡化。

无显示环境可以追加第五个运行参数关闭 ORB_SLAM3 viewer：

```bash
ros2 run orbslam3 stereo-inertial \
  "$PKG/vocabulary/ORBvoc.txt" \
  "$PKG/config/stereo-inertial/EuRoC.yaml" \
  true \
  false \
  false
```

## 7. 播放 V1_02_medium bag

另开终端播放需要的三路 topic，并 remap 到 wrapper 默认订阅名：

```bash
source /opt/ros/jazzy/setup.bash

ros2 bag play /mnt/data/slam/vicon_room1_ros2/V1_02_medium \
  --topics /cam0/image_raw /cam1/image_raw /imu0 \
  --remap \
  /cam0/image_raw:=/camera/left \
  /cam1/image_raw:=/camera/right \
  /imu0:=/imu
```

只播放上述 topic 可以避开 bag 中其他自定义消息类型缺失导致的播放失败，例如 `asctec_hl_comm`。

## 8. 验证

检查可执行文件链接的 OpenCV 版本：

```bash
ldd install/orbslam3/lib/orbslam3/stereo-inertial | grep opencv
```

期望主要 OpenCV 动态库来自系统路径，文件名包含 `406`，例如：

```text
/usr/lib/x86_64-linux-gnu/libopencv_core.so.406
```

smoke test 通过标准：

- 节点完成 vocabulary 加载。
- 播放 bag 后节点能收到左右目图像和 IMU，并进入跟踪流程。
- `Ctrl-C` 后进程可以退出。
- 当前目录生成 `KeyFrameTrajectory.txt`。

## 9. 轨迹误差测评

节点生成 `KeyFrameTrajectory.txt` 后，可以用离线测评命令读取同一个 rosbag2 中的 Vicon GT，并计算 ATE 平移误差：

```bash
source /opt/ros/jazzy/setup.bash
source install/local_setup.bash

PKG=/home/scl/work/slam/ORB_SLAM3_ROS2
ros2 run orbslam3 evaluate_trajectory \
  --bag /mnt/data/slam/vicon_room1_ros2/V1_02_medium \
  --trajectory KeyFrameTrajectory.txt \
  --gt-topic /vicon/firefly_sbx/firefly_sbx \
  --settings "$PKG/config/stereo-inertial/EuRoC.yaml" \
  --estimate-frame body \
  --alignment se3 \
  --output-json v1_02_medium_ate.json
```

参数说明：

- `--bag`：rosbag2 目录。
- `--trajectory`：ORB_SLAM3 导出的 TUM 格式轨迹，默认 `KeyFrameTrajectory.txt`。
- `--gt-topic`：Vicon GT topic，默认 `/vicon/firefly_sbx/firefly_sbx`。
- `--settings`：ORB_SLAM3 YAML 配置；`--estimate-frame body` 时用于读取 `Tbc`。
- `--estimate-frame`：`camera` 直接使用 `Twc`，`body` 会转换为 `Twb = Twc * inverse(Tbc)`。
- `--alignment`：`none` 不对齐，`se3` 使用有尺度 stereo-inertial 推荐的 SE3 对齐，`sim3` 适合单目等尺度不确定场景。
- `--output-json`：可选，写出机器可读 JSON 结果。

输出字段包含 `samples`、`time_start`、`time_end`、`alignment`、`estimate_frame`、`scale`，以及 ATE 的 `rmse_m`、`mean_m`、`median_m`、`std_m`、`min_m`、`max_m`。脚本只反序列化 GT topic，可避开 bag 中 `asctec_hl_comm` 等自定义消息类型依赖。

当前已有样例轨迹的 sanity check：`samples` 约为 `110`，`rmse_m` 约为 `0.099`。重新运行 SLAM 后结果可能有小幅变化。

一次运行可能打印或写出如下 JSON：

```json
{
  "alignment": "se3",
  "estimate_frame": "body",
  "max_m": 0.14627500246834263,
  "mean_m": 0.0932087553678668,
  "median_m": 0.0816584588446917,
  "min_m": 0.04703683009032711,
  "rmse_m": 0.09743834743159631,
  "samples": 106,
  "scale": 1.0,
  "std_m": 0.02839646937514658,
  "time_end": 1403715609.112143,
  "time_start": 1403715524.012143
}
```

该结果表示测评脚本在 `1403715524.012143` 到 `1403715609.112143` 秒之间匹配到 `106` 个轨迹样本，先按 `se3` 做刚体位姿对齐，然后在 `body` 机体系下计算估计轨迹和 Vicon GT 的平移误差。`scale` 为 `1.0` 表示 SE3 对齐没有估计额外尺度；这符合 stereo-inertial 轨迹本身有真实尺度的预期。

误差字段单位都是米：

- `rmse_m`：ATE 平移均方根误差，综合反映整体轨迹误差；示例中约为 `9.74 cm`。
- `mean_m`：平均平移误差；示例中约为 `9.32 cm`。
- `median_m`：中位数平移误差；示例中约为 `8.17 cm`，比平均值更不容易受少量大误差影响。
- `std_m`：误差标准差；示例中约为 `2.84 cm`，用于观察误差波动。
- `min_m` / `max_m`：最小和最大平移误差；示例中误差范围约为 `4.70 cm` 到 `14.63 cm`。

通常优先关注 `rmse_m` 和 `samples`：`rmse_m` 越小表示轨迹整体越接近 Vicon GT；`samples` 过少时，结果只能说明很短时间段内的误差，代表性较弱。若 `alignment`、`estimate_frame` 或 `settings` 变化，误差数值不能直接和该示例横向比较。

## 10. 常见问题

### 找不到 ORB_SLAM3

确认 `ORB_SLAM3_ROOT_DIR` 指向已构建的 ORB_SLAM3 根目录：

```bash
ls "$ORB_SLAM3_ROOT_DIR/include/System.h"
ls "$ORB_SLAM3_ROOT_DIR/lib/libORB_SLAM3.so"
```

### OpenCV 版本混用

如果链接检查中出现 `/usr/local/lib/libopencv_*.so.414`，清理 build cache 后重新构建：

```bash
rm -rf build/orbslam3 install/orbslam3 log
colcon build --symlink-install --packages-select orbslam3 \
  --cmake-args \
  -DPython3_EXECUTABLE=/usr/bin/python3 \
  -DOpenCV_DIR=/usr/lib/x86_64-linux-gnu/cmake/opencv4 \
  -DORB_SLAM3_ROOT_DIR=/home/scl/work/slam/ORB_SLAM3
```

### 无图形界面

默认运行会打开 Pangolin viewer。远程或无显示环境可以使用第五个运行参数 `false` 关闭 viewer，或准备桌面会话、X forwarding。
