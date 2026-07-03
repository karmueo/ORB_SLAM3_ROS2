# RGB-D Raw Data 转换节点说明

## 节点职责

`rgbd-raw-data-converter` 用于将 FastUMI SDK 发布的 packed RGB-D raw 图像拆分为 ORB_SLAM3 RGB-D 模式可直接订阅的两路标准图像消息。

该节点只做数据格式转换和发布，不处理相机标定，不发布 `CameraInfo`。ORB_SLAM3 仍通过 RGB-D YAML 配置读取相机内参、畸变参数和深度缩放参数。深度输出为米单位 `32FC1` 时，配置文件中的 `DepthMapFactor` 应设置为 `1.0`。

## 话题接口

输入话题：

```bash
rgbd/raw/image
```

消息类型：

```bash
sensor_msgs/msg/Image
```

输出话题：

```bash
camera/rgb
camera/depth
```

输出消息类型均为：

```bash
sensor_msgs/msg/Image
```

## 数据转换规则

彩色图像输出：

- `header` 沿用输入消息头
- `height` 和 `width` 沿用输入尺寸
- `encoding` 默认为 `rgb8`
- `step = width * 3`
- `data` 由每个输入像素的前 3 字节生成，默认保持 RGB 字节顺序

深度图像输出：

- `header` 沿用输入消息头
- `height` 和 `width` 沿用输入尺寸
- `encoding = 32FC1`
- `step = width * sizeof(float)`
- `data` 由每个输入像素 RGB 后面的 4 字节 `float32` 深度复制生成
- 深度单位保持为米

## 参数

`rgb_encoding` 控制彩色图像输出编码：

- `rgb8`：默认值，保持输入 RGB 顺序
- `bgr8`：逐像素交换 R/B 通道后发布

示例：

```bash
ros2 run orbslam3 rgbd-raw-data-converter --ros-args -p rgb_encoding:=bgr8
```

## 数据校验

节点会校验以下条件：

- `height > 0`
- `width > 0`
- `step == width * 7`
- `data.size() >= height * step`
- `rgb_encoding` 为 `rgb8` 或 `bgr8`

校验失败时，当前帧会被丢弃，并输出 warning 日志。

## 构建方式

`rgbd-raw-data-converter` 只依赖标准 `sensor_msgs/msg/Image`，构建 `orbslam3` 即可生成可执行文件：

```bash
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select orbslam3
```

如需显式指定 ORB_SLAM3 路径，在同一条构建命令中添加 `ORB_SLAM3_ROOT_DIR`：

```bash
colcon build --symlink-install --packages-select orbslam3 \
  --cmake-args -DORB_SLAM3_ROOT_DIR=/path/to/ORB_SLAM3
```

构建完成后 source 工作空间：

```bash
source install/local_setup.bash
```

检查可执行文件是否已注册：

```bash
ros2 pkg executables orbslam3 | grep rgbd-raw-data-converter
```

预期输出包含：

```bash
orbslam3 rgbd-raw-data-converter
```

播放完整 FastUMI bag 时，如果 bag 中还包含 `xv_ros2_msgs` 自定义消息话题，建议按顺序 source ROS 2、FastUMI 消息包所在 underlay 和当前工作空间，避免 rosbag 播放其他话题时类型支持缺失：

```bash
source /opt/ros/jazzy/setup.bash
source /home/scl/ros2_ws/install/setup.bash
source /home/scl/work/slam/ORB_SLAM3_ROS2/install/local_setup.bash
```

## 运行方式

将相对输入话题 remap 到设备实际发布的话题：

```bash
ros2 run orbslam3 rgbd-raw-data-converter \
  --ros-args \
  -r rgbd/raw/image:=/xv_sdk/SN250801DR48FB26001253/rgbd/raw/image
```

然后启动 ORB_SLAM3 RGB-D 节点。默认情况下，RGB-D 节点订阅 `camera/rgb` 和 `camera/depth`：

```bash
ros2 run orbslam3 rgbd PATH_TO_VOCABULARY PATH_TO_YAML_CONFIG_FILE
```

## 验证命令

播放包含原始 RGB-D 数据的 rosbag：

```bash
ros2 bag play /mnt/data/slam/my_umi_rosbag/2_filtered_vins_aux \
  --topics /xv_sdk/SN250801DR48FB26001253/rgbd/raw/image
```

检查转换后的图像话题：

```bash
ros2 topic echo /camera/rgb --once
ros2 topic echo /camera/depth --once
```
