# ORB_SLAM3_ROS2
This repository is ROS2 wrapping to use ORB_SLAM3

---

## Demo Video
[![orbslam3_ros2](https://user-images.githubusercontent.com/31432135/220839530-786b8a28-d5af-4aa5-b4ed-6234c2f4ca33.PNG)](https://www.youtube.com/watch?v=zXeXL8q72lM)

## Prerequisites
- I have tested on below version.
  - Ubuntu 22.04
  - ROS2 Humble
  - OpenCV 4.5.4

- Build ORB_SLAM3
  - Go to this [repo](https://github.com/zang09/ORB-SLAM3-STEREO-FIXED) and follow build instruction.

- Install related ROS2 package
```
$ sudo apt install ros-$ROS_DISTRO-vision-opencv && sudo apt install ros-$ROS_DISTRO-message-filters
```

## How to build
1. Clone repository to your ROS workspace
```
$ mkdir -p colcon_ws/src
$ cd ~/colcon_ws/src
$ git clone https://github.com/zang09/ORB_SLAM3_ROS2.git orbslam3_ros2
```

2. Change this [line](https://github.com/zang09/ORB_SLAM3_ROS2/blob/ee82428ed627922058b93fea1d647725c813584e/CMakeLists.txt#L5) to your own `python site-packages` path

3. Provide your `ORB_SLAM3` root path when building. You can pass it as a CMake argument:

Now, you are ready to build!
```
$ cd ~/colcon_ws
$ colcon build --symlink-install --packages-select orbslam3 --cmake-args -DORB_SLAM3_ROOT_DIR=/path/to/ORB_SLAM3
```

Alternatively, export the path before building:
```
$ export ORB_SLAM3_ROOT_DIR=/path/to/ORB_SLAM3
$ colcon build --symlink-install --packages-select orbslam3
```

## Troubleshootings
1. If you cannot find `sophus/se3.hpp`:  
Go to your `ORB_SLAM3_ROOT_DIR` and install sophus library.
```
$ cd ~/{ORB_SLAM3_ROOT_DIR}/Thirdparty/Sophus/build
$ sudo make install
```
2. Please compile with `OpenCV 4.5.4` version.

## How to use
1. Source the workspace  
```
$ source ~/colcon_ws/install/local_setup.bash
```

2. Run orbslam mode, which you want.  
This repository only support `MONO, STEREO, RGBD, STEREO-INERTIAL` mode now.  
You can find vocabulary file and config file in here. (e.g. `orbslam3_ros2/vocabulary/ORBvoc.txt`, `orbslam3_ros2/config/monocular/TUM1.yaml` for monocular SLAM).
  - `MONO` mode  
```
$ ros2 run orbslam3 mono PATH_TO_VOCABULARY PATH_TO_YAML_CONFIG_FILE
```
  - `STEREO` mode  
```
$ ros2 run orbslam3 stereo PATH_TO_VOCABULARY PATH_TO_YAML_CONFIG_FILE BOOL_RECTIFY
```
  - `RGBD` mode  
```
$ ros2 run orbslam3 rgbd PATH_TO_VOCABULARY PATH_TO_YAML_CONFIG_FILE
```
  - `STEREO-INERTIAL` mode  
```
$ ros2 run orbslam3 stereo-inertial PATH_TO_VOCABULARY PATH_TO_YAML_CONFIG_FILE BOOL_RECTIFY [BOOL_EQUALIZE]
```

## Run with rosbag
To play ros1 bag file, you should install `ros1 noetic` & `ros1 bridge`.  
Here is a [link](https://www.theconstructsim.com/ros2-qa-217-how-to-mix-ros1-and-ros2-packages/) to demonstrate example of `ros1-ros2 bridge` procedure.  
If you have `ros1 noetic` and `ros1 bridge` already, open your terminal and follow this:  
(Shell A, B, C, D is all different terminal, e.g. `stereo-inertial` mode)
1. Download EuRoC Dataset (`V1_02_medium.bag`)
```
$ wget -P ~/Downloads http://robotics.ethz.ch/~asl-datasets/ijrr_euroc_mav_dataset/vicon_room1/V1_02_medium/V1_02_medium.bag
```  

2. Launch Terminal  
(e.g. `ROS1_INSTALL_PATH`=`/opt/ros/noetic`, `ROS2_INSTALL_PATH`=`/opt/ros/foxy`)
```
#Shell A:
source ${ROS1_INSTALL_PATH}/setup.bash
roscore

#Shell B:
source ${ROS1_INSTALL_PATH}/setup.bash
source ${ROS2_INSTALL_PATH}/setup.bash
export ROS_MASTER_URI=http://localhost:11311
ros2 run ros1_bridge dynamic_bridge

#Shell C:
source ${ROS1_INSTALL_PATH}/setup.bash
rosbag play ~/Downloads/V1_02_medium.bag --pause /cam0/image_raw:=/camera/left /cam1/image_raw:=/camera/right /imu0:=/imu

#Shell D:
source ${ROS2_INSTALL_PATH}/setup.bash
ros2 run orbslam3 stereo-inertial PATH_TO_VOCABULARY PATH_TO_YAML_CONFIG_FILE BOOL_RECTIFY [BOOL_EQUALIZE]
```

3. Press `spacebar` in `Shell C` to resume bag file.  

## EGO 离线双目惯性最短入口

以下命令使用变量/占位符，不写死本机绝对路径。`EGO_BAG`、`EGO_CALIB` 是只读输入；`EGO_ROOT` 是当前仓库；`EGO_RESULTS` 建议放在仓库内被忽略的 `results/`。

```bash
source /opt/ros/jazzy/setup.bash
source "$EGO_ROOT/install/local_setup.bash"
export EGO_VOCAB="$(ros2 pkg prefix orbslam3)/share/orbslam3/vocabulary/ORBvoc.txt"
export EGO_CONFIG="$EGO_ROOT/config/stereo-inertial/EGO/EGO_stereo_inertial_job_004.yaml"
mkdir -p "$EGO_RESULTS/time_alignment"

# 1) 可选：从原 calib.yaml + MCAP 重新生成默认配置（默认拒绝覆盖）。
ros2 run orbslam3 prepare-ego-stereo-inertial-config \
  --calib "$EGO_CALIB" --bag "$EGO_BAG" \
  --output "$EGO_RESULTS/generated_ego.yaml"

# 2) 独立视觉旋转 vs 去 bias IMU 积分，输出可审查 JSON。
ros2 run orbslam3 align-ego-stereo-inertial-time \
  --calib "$EGO_CALIB" --bag "$EGO_BAG" \
  --output "$EGO_RESULTS/time_alignment/imu_time_alignment.json"

# 3) 完整离线运行；原配置保留标定 offset，推荐值用 CLI 显式覆盖。
mkdir -p "$EGO_RESULTS/validated_offset"
ros2 run orbslam3 stereo-inertial \
  "$EGO_VOCAB" "$EGO_CONFIG" false false false \
  --bag "$EGO_BAG" \
  --output "$EGO_RESULTS/validated_offset/trajectory.txt" \
  --imu-time-offset-sec 0.0281
```

结果查看：`trajectory.txt` 是左 cam `Twc` 的 TUM（秒/米/归一化 `qx qy qz qw`，时间保留原图像 ns）；`trajectory.summary.json` 有 complete/failed、eligible/提交/有效、total/main 覆盖率、map 初始化/BA 和实际使用 offset；`trajectory.log` 是可读统计。默认不覆盖已有输出，只有 `--overwrite` 才替换本 stem 的旧 manifest 产物；输入 bag/calib/config/vocabulary 只读，不能用覆盖参数绕过。失败、未初始化、后台未排空、低于 95%/90% 门槛或读/解码异常都会非零退出。完整依赖准备、补丁栈、配置生成、offset 核验和验收记录见 `doc/ego_offline_stereo_inertial_dependencies.md`。

## Acknowledgments
This repository is modified from [this](https://github.com/curryc/ros2_orbslam3) repository.  
To add `stereo-inertial` mode and improve build difficulites.
