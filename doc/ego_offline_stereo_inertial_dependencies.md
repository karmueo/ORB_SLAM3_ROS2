# EGO 离线双目惯性：依赖准备与运行

从仓库根目录准备本地核心、构建 wrapper，再用原始 EGO `calib.yaml` 和 MCAP 生成配置、核验 IMU 偏移并导出 TUM 轨迹。以下命令使用同一 Bash 终端；输入只读，依赖放在忽略目录 `.deps/`，不使用 `sudo`、不安装系统软件或全局 Python 包。下载需要网络及可用的固定版本 apt 源。

适用基线：Ubuntu 24.04 amd64、ROS 2 Jazzy、GCC/G++ 13.3.0、CMake 3.28.3、C++14、Eigen 3.4.0（系统 `libeigen3-dev`）。系统需已有 OpenGL/EGL、`libepoxy0`、与 `cv_bridge` 一致的 OpenCV 4.6.0，以及 ROS 构建和 MCAP 读取依赖。Python 工具需要系统 Python3、`python3-numpy`、`python3-opencv`、`python3-yaml`、`rosbag2_py` 和 `sensor_msgs`，不需要 `ego_msgs`。

准备脚本固定使用以下版本（[脚本依据](../scripts/prepare_offline_core.sh)）：

| 组件 | 版本／位置 |
| --- | --- |
| ORB_SLAM3 | `zang09/ORB-SLAM3-STEREO-FIXED`，提交 `f8ac79153b103dd44f7ea3cf2a1a4aea10b18483`，位于 `.deps/ORB_SLAM3` |
| Pangolin | `0.9.6-1noble.20260820.143839`，解压到 `.deps/local` |
| Sophus | `1.22.9102-2noble.20260225.142549`，解压到 `.deps/local`；核心使用内置 `Thirdparty/Sophus` 头文件 |
| GLFW | `3.3.10-1build1`，本地解压运行库和开发包 |
| libepoxy | 开发包 `1.5.10-1build1`，本地解压并使用系统运行库 |
| DBoW2／g2o | 核心内置版本，在 `Thirdparty/` 内构建 |

## 1. 设置路径

先将下面两个 `/path/to/` 示例替换为实际输入路径，在仓库根目录执行：

```bash
# 保存仓库根目录
export \
  EGO_ROOT="$PWD"
# 指定原始标定文件（替换示例路径）
export \
  EGO_CALIB="/path/to/calib.yaml"
# 指定原始 MCAP（替换示例路径）
export \
  EGO_BAG="/path/to/job_004.mcap"
# 设置本次结果目录
export \
  EGO_RESULTS="$EGO_ROOT/results/ego_job_004"
```

`EGO_CALIB` 始终指向原始标定，不能传入生成的 ORB_SLAM3 配置。`EGO_RESULTS` 使用新目录或未使用的输出文件名，避免覆盖历史报告。

| 支持的参数／配置项 | 参数作用 | 输入/输出 | 默认值/支持的值 |
| --- | --- | --- | --- |
| `EGO_ROOT`（环境变量） | 当前仓库根目录 | 输入目录 | 未设置；此处取 `$PWD` |
| `EGO_CALIB`（环境变量） | 原始 EGO 标定 | 输入文件 | 未设置 |
| `EGO_BAG`（环境变量） | 原始 MCAP | 输入文件 | 未设置 |
| `EGO_RESULTS`（环境变量） | 配置、报告及轨迹的存放目录 | 输出目录 | 未设置 |

## 2. 准备固定核心与本地依赖

```bash
# 创建准备日志目录
mkdir \
  -p \
  log
```

| 支持的参数／配置项 | 参数作用 | 输入/输出 | 默认值/支持的值 |
| --- | --- | --- | --- |
| `-p` | 补齐父目录，允许目录已存在 | 行为控制项 | 不自动创建缺失的父目录；已有目录会报错 |
| 目录位置参数 | 要创建的目录 | 输出目录 | 必填，无默认值 |

```bash
# 准备固定核心与依赖，并保存日志
./scripts/prepare_offline_core.sh \
  > log/00_prepare_offline_core.log 2>&1
```

在仓库根目录执行，依次应用 [0001](../patches/0001-offline-core-compat.patch)、[0002](../patches/0002-offline-lifecycle-snapshot.patch)，将依赖解压到 `.deps/local` 并构建 DBoW2、g2o 和核心。[补丁脚本](../scripts/apply_core_patch_stack.sh)仅接受 fresh、仅 0001、0001+0002 三种 tracked 工作树状态；可重复执行，未知更改会被拒绝并保留。

固定包下载、系统依赖、补丁校验或工作树检查失败时，先解决报错，不要跳过补丁或更换版本继续。退出成功并输出 `ORB_SLAM3 ready:` 后再构建 wrapper；需保留旧日志时换文件名。

| 支持的参数／配置项 | 参数作用 | 输入/输出 | 默认值/支持的值 |
| --- | --- | --- | --- |
| 无 | 使用脚本内固定版本和目录 | 输入补丁；输出 `.deps/` | 不适用（不解析命令行选项） |
| `CMAKE_BUILD_PARALLEL_LEVEL`（环境变量） | DBoW2、g2o 和核心构建并行数 | 行为控制项 | 未设置或为空时为 `8` |
| `LD_LIBRARY_PATH`（环境变量） | 核心构建时在原值前追加本地库目录 | 输入目录搜索路径 | 原值未设置时仅使用脚本追加的路径 |
| `>`（Bash 重定向） | 将标准输出写入文件，截断已有文件 | 输出文件 | 未指定时继承终端标准输出 |
| `2>&1`（Bash 重定向） | 标准错误合并到当前标准输出 | 输出流 | 未指定时继承终端标准错误 |

## 3. 构建 wrapper

```bash
# 加载 ROS 2 Jazzy 环境
source \
  /opt/ros/jazzy/setup.bash
```

新终端先加载 ROS，再恢复第 1 节路径和本节库搜索环境。

| 支持的参数／配置项 | 参数作用 | 输入/输出 | 默认值/支持的值 |
| --- | --- | --- | --- |
| 脚本路径位置参数 | ROS 或工作空间的环境脚本 | 输入文件；更新当前终端环境 | 必填，无默认值 |

```bash
# 指定本地依赖前缀
export \
  LOCAL="$EGO_ROOT/.deps/local"
# 前置本地动态库搜索路径
export \
  LD_LIBRARY_PATH="$LOCAL/opt/ros/jazzy/lib/x86_64-linux-gnu:$LOCAL/usr/lib/x86_64-linux-gnu:${LD_LIBRARY_PATH:-}"
# 前置本地 CMake 搜索路径
export \
  CMAKE_PREFIX_PATH="$LOCAL/opt/ros/jazzy:$LOCAL/usr:${CMAKE_PREFIX_PATH:-}"
```

`export` 用法见第 1 节，新增变量如下。

| 支持的参数／配置项 | 参数作用 | 输入/输出 | 默认值/支持的值 |
| --- | --- | --- | --- |
| `LOCAL`（环境变量） | 本地解压前缀 | 输入目录 | 未设置 |
| `LD_LIBRARY_PATH`（环境变量） | 运行时动态库搜索路径 | 输入目录搜索路径 | 依赖当前终端环境；未设置时无额外搜索路径 |
| `CMAKE_PREFIX_PATH`（环境变量） | CMake 依赖搜索前缀 | 输入目录搜索路径 | 依赖已加载的 ROS 环境 |

```bash
# 使用固定核心、系统 OpenCV 和 Python 构建 wrapper
CMAKE_BUILD_PARALLEL_LEVEL=8 colcon build \
  --paths . \
  --symlink-install \
  --packages-select orbslam3 \
  --cmake-args \
  -DORB_SLAM3_ROOT_DIR="$EGO_ROOT/.deps/ORB_SLAM3" \
  -DOpenCV_DIR=/usr/lib/x86_64-linux-gnu/cmake/opencv4 \
  -DPython3_EXECUTABLE=/usr/bin/python3 \
  -DCMAKE_PREFIX_PATH="$CMAKE_PREFIX_PATH" \
  -DCMAKE_CXX_FLAGS="-I$LOCAL/usr/include"
```

wrapper、核心与 g2o 必须使用一致的 C++14 和 ISA 编译选项；不要单侧修改 `-march=native`。此处固定系统 OpenCV 4.6，避免误用用户目录的 4.13。依据：[构建配置](../CMakeLists.txt)、[核心查找配置](../CMakeModules/FindORB_SLAM3.cmake)。

| 支持的参数／配置项 | 参数作用 | 输入/输出 | 默认值/支持的值 |
| --- | --- | --- | --- |
| `--paths` | 指定待发现包的路径 | 输入目录 | 依赖 colcon 包发现扩展；此处显式指定 `.` |
| `--symlink-install` | 尽可能以符号链接安装 | 行为控制项 | 关闭 |
| `--packages-select` | 选择 `orbslam3` | 输入值 | 不按此选项限制包 |
| `--cmake-args` | 向 CMake 传递后续参数 | 行为控制项 | 不追加此组参数 |
| `CMAKE_BUILD_PARALLEL_LEVEL`（环境变量） | 控制 CMake 构建并行数 | 行为控制项 | 依赖环境与构建工具 |
| `-DORB_SLAM3_ROOT_DIR` | 固定核心根目录 | 输入目录 | 读取同名环境变量；两者都未设置则报错 |
| `-DOpenCV_DIR` | 系统 OpenCV CMake 配置目录 | 输入目录 | CMake 自动查找，可能选错版本 |
| `-DPython3_EXECUTABLE` | 系统 Python3 解释器 | 输入文件 | 依赖环境与 CMake 查找结果 |
| `-DCMAKE_PREFIX_PATH` | 显式传入本地和 ROS 前缀 | 输入目录搜索路径 | 依赖缓存和环境搜索路径 |
| `-DCMAKE_CXX_FLAGS` | 加入本地开发头文件搜索路径 | 输入值（编译选项） | 依赖工具链、环境与已有缓存 |

```bash
# 检查 DBoW2 链接的 OpenCV
ldd \
  .deps/ORB_SLAM3/Thirdparty/DBoW2/lib/libDBoW2.so |
# 筛选 OpenCV 核心库依赖
grep \
  opencv_core
# 检查核心链接的 OpenCV
ldd \
  .deps/ORB_SLAM3/lib/libORB_SLAM3.so |
# 筛选 OpenCV 核心库依赖
grep \
  opencv_core
# 检查 wrapper 链接的 OpenCV
ldd \
  install/orbslam3/lib/orbslam3/stereo-inertial |
# 筛选 OpenCV 核心库依赖
grep \
  opencv_core
```

各项应解析为 `libopencv_core.so.406`。若出现 `.so.413`，重新配置对应构建目录；若库缺失或显示 `not found`，先修复搜索路径或构建结果，再继续。

`ldd` 参数：

| 支持的参数／配置项 | 参数作用 | 输入/输出 | 默认值/支持的值 |
| --- | --- | --- | --- |
| 文件位置参数（`ldd`） | 待检查的库或可执行文件 | 输入文件；输出标准输出 | 必填，无默认值；支持动态库或可执行文件路径 |

`grep` 参数：

| 支持的参数／配置项 | 参数作用 | 输入/输出 | 默认值/支持的值 |
| --- | --- | --- | --- |
| 模式位置参数（`grep`） | 匹配包含该文本的行 | 输入值；从管道读入、向标准输出写出 | 必填，无默认值；支持正则表达式，此处为 `opencv_core` |
| `\|`（Bash 管道） | 将左侧标准输出交给右侧标准输入 | 输入／输出流 | 不适用 |

```bash
# 加载本次构建的工作空间
source \
  "$EGO_ROOT/install/local_setup.bash"
```

仅在构建成功后继续；`source` 参数见本节首表。新终端依次加载 ROS、设置路径和本地依赖环境、加载本工作空间。

## 4. 生成常规标定配置

```bash
# 从原始标定和 MCAP 生成常规 ORB_SLAM3 配置
ros2 run orbslam3 prepare-ego-stereo-inertial-config \
  --calib "$EGO_CALIB" \
  --bag "$EGO_BAG" \
  --output "$EGO_RESULTS/generated_ego.yaml"
```

输出目录自动创建；YAML 使用原生 Kannala-Brandt 双目、原分辨率、全帧处理。常规配置不启用两个 Defer 开关，与第 6 节的 job_004 专用配置分开使用。默认拒绝覆盖；即使指定 `--overwrite`，也不能覆盖输入或其符号链接、硬链接。依据：[转换器](../scripts/prepare_ego_stereo_inertial_config.py)。

| 支持的参数／配置项 | 参数作用 | 输入/输出 | 默认值/支持的值 |
| --- | --- | --- | --- |
| `--calib` | 原始 EGO `calib.yaml` | 输入文件 | 必填，无默认值 |
| `--bag` | 原始 MCAP | 输入文件 | 必填，无默认值 |
| `--output`／`-o` | ORB_SLAM3 YAML 路径 | 输出文件 | 必填，无默认值 |
| `--image-topic` | 拼接 `CompressedImage` topic | 输入值 | `/stereo/image_raw` |
| `--imu-topic` | `sensor_msgs/msg/Imu` topic | 输入值 | `/imu` |
| `--overwrite` | 显式允许替换输出 | 行为控制项 | 关闭 |
| `--help`／`-h` | 显示帮助后退出 | 行为控制项 | 关闭 |

## 5. 独立核验 IMU 时间偏移

```bash
# 独立核验 IMU 时间偏移并保存报告
ros2 run orbslam3 align-ego-stereo-inertial-time \
  --calib "$EGO_CALIB" \
  --bag "$EGO_BAG" \
  --output "$EGO_RESULTS/time_alignment_final/imu_time_alignment.json" \
  --window-count 4 \
  --stride 5
```

只读取原始 bag/calib；`--stride 5` 复现实验参数，仅影响核验采样，正式 SLAM 仍处理全帧。按 `corrected_t = raw_imu_t + offset` 解释偏移；使用推荐值前检查 JSON 的 holdout 和各窗口是否改善。`exit 0` 仅表示报告生成成功。参数及默认值见[时间核验工具](../scripts/align_ego_stereo_inertial_time.py)。

第 6 节的 `+28.1 ms` 来自 2026-11-09 的 job_004 记录，仅适用于同一 bag/calib/采集链，无 GT；输入身份、核验结果及历史方向描述勘误见[运行记录](ego_job_004_run_record.md#1-固定输入与版本)和[时间核验记录](ego_job_004_run_record.md#3-独立时间偏移核验a02)。

| 支持的参数／配置项 | 参数作用 | 输入/输出 | 默认值/支持的值 |
| --- | --- | --- | --- |
| `--calib` | 原始标定，不能使用生成的 ORB YAML | 输入文件 | 非自测模式必填，无默认值 |
| `--bag` | 原始 MCAP | 输入文件 | 非自测模式必填，无默认值 |
| `--output` | 保存可审查 JSON | 输出文件 | 非自测模式必填；省略时报错，虽帮助文字称可打印到 stdout |
| `--image-topic` | 拼接压缩图像 topic | 输入值 | `/stereo/image_raw` |
| `--imu-topic` | IMU topic | 输入值 | `/imu` |
| `--half-range-ms` | 对称偏移搜索半范围（ms） | 输入值 | 默认：`50.0`；类型：浮点数 |
| `--coarse-step-ms` | 粗搜步长（ms） | 输入值 | 默认：`1.0`；类型：浮点数 |
| `--fine-step-ms` | 粗搜最优值附近细搜步长（ms） | 输入值 | 默认：`0.1`；类型：浮点数 |
| `--spans-ms` | 逗号分隔的正整数帧对时间跨度（ms） | 输入值 | 默认：`20,60,160,320`；支持：逗号分隔的正整数列表 |
| `--window-count` | 运动窗口数量 | 输入值 | 默认：`4`；类型：整数 |
| `--window-half-width-sec` | 窗口半宽（秒） | 输入值 | 默认：`1.0`；类型：浮点数 |
| `--window-separation-sec` | 窗口分隔（秒） | 输入值 | 默认：`8.0`；类型：浮点数 |
| `--stride` | 基础帧采样步长，实际至少为 1 | 输入值 | 默认：`3`；类型：整数；小于 1 时按 1 采样 |
| `--max-features` | ORB 特征数量上限 | 输入值 | 默认：`2000`；类型：整数 |
| `--max-match-distance` | 特征匹配距离上限 | 输入值 | 默认：`70.0`；类型：浮点数 |
| `--min-motion-rad-s` | 角速度运动筛选门限（rad/s） | 输入值 | 默认：`0.5`；类型：浮点数 |
| `--overwrite` | 显式允许替换输出，仍保护输入别名 | 行为控制项 | 关闭 |
| `--self-test` | 仅运行合成符号、方向和偏移自检，结果打印到 stdout | 行为控制项 | 关闭；启用时不要求输入，也不写 `--output` |
| `--help`／`-h` | 显示帮助后退出 | 行为控制项 | 关闭 |

## 6. 运行完整 job_004

本节复现记录中的 job_004：使用仓库[专用配置](../config/stereo-inertial/EGO/EGO_stereo_inertial_job_004.yaml)（3000 特征、fastInit 和两个 Defer 开关），通过 CLI 覆盖原 offset。其他输入应改用第 4 节生成的配置和自己的独立核验偏移，不直接套用这组配置或数值；配置适用范围见[运行记录](ego_job_004_run_record.md)。

```bash
# 从当前安装前缀取得词典路径
export \
  EGO_VOCAB="$(ros2 pkg prefix \
  orbslam3)/share/orbslam3/vocabulary/ORBvoc.txt"
```

词典由构建过程解压并安装，依据：[CMakeLists.txt](../CMakeLists.txt)。`export` 用法见第 1 节。

`ros2 pkg prefix` 参数：

| 支持的参数／配置项 | 参数作用 | 输入/输出 | 默认值/支持的值 |
| --- | --- | --- | --- |
| 包名位置参数 `orbslam3` | 待查询的 ROS 包 | 输入值；标准输出为前缀目录 | 必填，无默认值 |

`export` 新增变量：

| 支持的参数／配置项 | 参数作用 | 输入/输出 | 默认值/支持的值 |
| --- | --- | --- | --- |
| `EGO_VOCAB`（环境变量） | 后续离线输入词典 | 输入文件 | 未设置 |

```bash
# 创建本次轨迹输出目录
mkdir \
  -p \
  "$EGO_RESULTS/validated_offset"
```

`mkdir` 参数见第 2 节；保留旧实验时使用新的目录名。

```bash
# 使用 job_004 专用配置和已核验偏移处理整包
ros2 run orbslam3 stereo-inertial \
  "$EGO_VOCAB" \
  "$EGO_ROOT/config/stereo-inertial/EGO/EGO_stereo_inertial_job_004.yaml" \
  false \
  false \
  false \
  --bag "$EGO_BAG" \
  --output "$EGO_RESULTS/validated_offset/trajectory.txt" \
  --imu-time-offset-sec 0.0281
```

三个布尔位置参数依次为矫正、均衡化、viewer，均只接受 `true`／`false`。原生 Kannala-Brandt 配置必须关闭 rectification，离线模式必须关闭 viewer。CLI 偏移覆盖配置值，不改写配置；需要原标定对照时换输出目录，并将偏移换成 `-0.005089461604383672`。参数依据：[解析实现](../src/stereo-inertial/offline-cli.cpp)、[默认值](../src/stereo-inertial/offline-cli.hpp)。

| 支持的参数／配置项 | 参数作用 | 输入/输出 | 默认值/支持的值 |
| --- | --- | --- | --- |
| `VOCAB`（位置参数 1） | ORB 词典 | 输入文件 | 必填，无默认值 |
| `ORB_CONFIG`（位置参数 2） | ORB_SLAM3 YAML | 输入文件 | 必填，无默认值 |
| `DO_RECTIFY`（位置参数 3） | 是否矫正；本流程必须为 `false` | 行为控制项 | 必填，无默认值；仅 `true`／`false` |
| `DO_EQUALIZE`（可选位置参数 4） | 是否做 CLAHE 均衡化 | 行为控制项 | 默认：`false`；支持：`true`／`false` |
| `USE_VIEWER`（可选位置参数 5） | 是否显示 viewer | 行为控制项 | 默认：离线 `false`、在线 `true`；支持：`true`／`false`，离线仅 `false` |
| `--bag` | 离线 MCAP 或 rosbag2 路径 | 输入文件／目录 | 离线必填，无默认值 |
| `--output` | 主 TUM 路径及同 stem 产物命名基准 | 输出文件 | 离线必填，无默认值 |
| `--stereo-topic` | 拼接压缩图像 topic | 输入值 | `/stereo/image_raw` |
| `--imu-topic` | IMU topic | 输入值 | `/imu` |
| `--imu-time-offset-sec` | 加到原始 IMU 时间上的有限秒数 | 输入值 | 使用 `ROS.ImuTimeOffsetSec`；该配置缺失时为 `0.0` |
| `--overwrite` | 按旧 manifest 校验并整体替换本 stem 产物 | 行为控制项 | 关闭，已有任一产物即拒绝 |
| `--help`／`-h` | 显示帮助后退出，可单独传入 | 行为控制项 | 关闭 |

运行结束后核对以下产物与继续条件（依据：[离线运行实现](../src/stereo-inertial/offline-runner.cpp)、[输出实现](../src/stereo-inertial/offline-output.cpp)）：

- `trajectory.txt`：左相机 `Twc`，惯性世界坐标，不首帧归零；格式 `timestamp tx ty tz qx qy qz qw`，位置米、归一化四元数，时间保留原图像 header 的秒和 9 位纳秒小数。
- 主轨迹选已完成惯性初始化且有效位姿最多的 map，平局取较小 id；其他初始化 map 写 `trajectory.map-<id>.txt`，不得跨 map 拼接。lost、未 set 和非有限位姿不导出，无有效位姿时不生成空 TUM。
- 同目录生成 `trajectory.summary.json`、`trajectory.log`、`trajectory.manifest.txt`。只有 CLI 退出 0、summary 的 `status=complete` 且 `exit_success=true` 才算成功；`maps` 记录各图初始化、BA 和主图信息。
- total 有效位姿/eligible 必须 `>=95%`，main 必须 `>=90%`。eligible 只扣首尾修正后 IMU 覆盖不足；低覆盖可保留部分轨迹，但退出非零。无覆盖、预积分样本不足、读/解码/时间/数值异常、无初始化 map 或后台未完成也均非零退出，先处理失败原因再继续。
- 输入只读，输出输入别名被拒绝。`--overwrite` 仅替换旧 manifest 证明属于该 stem 的产物，未知冲突会拒绝；本次失败或无 eligible 时也不会保留旧轨迹。保留历史实验应换目录。

覆盖达标和退出成功不能代表绝对精度。历史运行、原标定偏移对照及失败证据见[运行记录第 4 节](ego_job_004_run_record.md#4-完整-job_004-两次对照a07)，不代表本次已重新执行。
