# EGO job_004 运行与验收记录（2026-11-09）

本文是计划分块 3 的可审查运行记录，使用实施目录相对路径引用日志/结果。命令中的
`EGO_ROOT`、`EGO_BAG`、`EGO_CALIB`、`EGO_RESULTS` 是变量/占位符；本机实际路径没有
写入配置或文档。原始 bag/calib 只读，未修改。

## 1. 固定输入与版本

| 项 | 值 |
| --- | --- |
| bag | `job_004.mcap`，sha256 `47c0b0b1e94f363ca4dc3c3f938aa104fb38079efb115b3408b62d4f7e7dc672` |
| calib | `calib.yaml`，sha256 `adcac62a6e0d2bb6643be33ec3e808ce18dedff6c94133c9f3aa25e28663aa72` |
| 图像 | `/stereo/image_raw`，3234 条 JPEG，3840x1080，左右各 1920x1080 |
| IMU | `/imu`，26308 条，约 405.6 Hz |
| 核心 | `zang09/ORB-SLAM3-STEREO-FIXED` `f8ac79153b103dd44f7ea3cf2a1a4aea10b18483` |
| 核心补丁 | 0001 sha256 `dc2f389b1d77dce98193d91fd68feba32de75bc6d97e905f28ffc8334e7818d8`；0002 sha256 `29882f2bf5bfe25e66a61aad854734219d42ee1e1a1e518912214bcfb1087337` |
| ROS / 编译器 | ROS 2 Jazzy；GCC/G++ 13.3.0；CMake 3.28.3 |
| OpenCV | C++/核心链接系统 4.6.0；时间对齐 Python 工具也强制系统 `cv2 4.6.0` |
| 历史运行时配置 | `config/stereo-inertial/EGO/EGO_stereo_inertial_job_004.yaml`，当时文件 sha256 `28c75e5c3cd0643c1b6342784c8c301b8df92854d361e2e70b0a6d585db14ca3` |

运行后配置文件仅修订注释，配置键值未变；因此当前文件的 SHA256 与表中历史运行时文件的 SHA256 不同，表中哈希只用于追溯当时的运行。

最终配置保留原标定 `ROS.ImuTimeOffsetSec: -0.005089461604383672`。最终推荐运行通过
`--imu-time-offset-sec 0.0281` 显式覆盖；配置文件里的原值没有被改写。

## 2. 依赖/补丁栈可重复性（F10）

命令：

```bash
cd "$EGO_ROOT"
MAKEFLAGS='-j8 -l8' CMAKE_BUILD_PARALLEL_LEVEL=8 bash scripts/prepare_offline_core.sh
```

实际执行两次均 `exit 0`，两次都识别为 `Patch stack already complete (2/2)`。日志：

- `log/round5/prepare_offline_core_final_run1_v2.log`、`prepare_offline_core_final_run1_v2.exit`（0）
- `log/round5/prepare_offline_core_final_run2_v2.log`、`prepare_offline_core_final_run2_v2.exit`（0）

`scripts/apply_core_patch_stack.sh` 在临时 index 中构造 fresh、0001-only、0001+0002
三种期望 tree，再比较实际 tracked 工作树。行为测试 `test/test_prepare_offline_core_patch_stack.py`
覆盖 fresh、只应用 0001、完整重跑、未知更改拒绝且原样保留；实际结果
`3 passed`，日志 `log/round5/patch_stack_behavior.log`、exit `0`。另外在真实补丁的
临时 shared clone 上直接验证：fresh 依次应用 0001+0002 成功；只应用 0001 时只补 0002
成功；未知 tracked 更改 exit 3 且内容逐字节保留；fresh 重跑识别完整。日志
`log/round5/patch_stack_actual_behavior.log`。未知 tracked 更改不会被覆盖/误标已就绪。

完整运行前还发现并修复了长时 `PoseOptimization` 的 `double free or corruption`：
`Thirdparty/g2o` 现在与核心一样显式用 C++14 构建。失败/修复前后日志保留在
`results/ego_job_004/round5_original_offset/`、`gdb/`、`g2o_rebuild_original/`。

全量构建/测试最终结果：`colcon build` exit 0；`colcon test` exit 0；`colcon test-result`
显示 172 tests、0 errors、0 failures、0 skipped。日志：
`log/round5/colcon_build_final_v3.log`、`colcon_build_final_v3.exit`、
`log/round5/colcon_test_final_v3.log`、`colcon_test_final_v3.exit`、
`log/round5/colcon_test_result_final_v3.log`。

## 3. 独立时间偏移核验（A02）

实测参数的复现命令（使用新的输出路径以免覆盖旧报告）：

```bash
ros2 run orbslam3 align-ego-stereo-inertial-time \
  --calib "$EGO_CALIB" --bag "$EGO_BAG" \
  --output "$EGO_RESULTS/time_alignment_final/imu_time_alignment.json" \
  --window-count 4 --stride 5
```

工具默认 `--stride 3`；本记录的实测使用 `--stride 5`，只对时间核验采样，正式 SLAM 仍处理全部帧。

实际结果（`results/ego_job_004/time_alignment_final/`）：

- 4 个有运动窗口，416 帧进入特征/立体处理；318 次 PnP 尝试，307 组保留；排除：
  低运动 2、跨帧匹配少 5、几何退化小旋转 6、少立体点 0、缺目标帧 0。
- train 窗口仅用于选择：粗搜 + 细搜推荐 `+28.1 ms`；holdout 最优参考 `+27.0 ms`。
- train/holdout 中位旋转误差 `0.170°` / `0.221°`；4/4 窗口相对配置 offset 改善。
- 方向核对：采用 `R_cb * Q_raw^T * R_cb^T`（`Q_raw=R_bi^T R_bj`）中位误差 `0.170°`，
  不转置方向 `13.5°`；合成时变角速度自检真值 `27.0 ms` → 估计 `27.1 ms`，
  转置方向误差 `2.77°` 对比正确方向 `0.006°`，`passed=true`。
- 历史 OpenVINS `+25~30 ms` 仅作为候选记录，没有读取/冒充已不存在的旧验收文件。

日志：`results/ego_job_004/time_alignment_final/{stdout.log,stderr.log,exit,self_test.log,self_test.exit,imu_time_alignment.json}`。
更早运行保留在 `results/ego_job_004/time_alignment/`、`time_alignment_cv46/`。三个历史报告 JSON 中 `direction_check.selected_mapping` 的字符串遗漏 `Q_raw` 的转置；实际数值按带转置的公式计算，未重新计算或更改。原始报告保留不变，机器可读勘误见 `results/ego_job_004/time_alignment_final/imu_time_alignment.errata.json`；新生成报告会使用正确描述。

## 4. 完整 job_004 两次对照（A07）

两次运行命令只差 `--imu-time-offset-sec`，均使用同一最终配置、原分辨率、全部帧、
`viewer=false`、`equalize=false`：

```bash
ros2 run orbslam3 stereo-inertial "$EGO_VOCAB" "$EGO_CONFIG" false false false \
  --bag "$EGO_BAG" --output "$EGO_RESULTS/<run>/trajectory.txt" \
  --imu-time-offset-sec <offset>
```

| 运行 | offset | eligible | 提交 | 有效位姿 | total | main | map/初始化 | 退出 |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | --- | ---: |
| 原标定 | -0.005089461604383672 s | 3233 | 3233 | 3231 | 0.999381 | 0.999381 | map 0，IMU/BA1/BA2 均完成 | 0 |
| 独立核验 | +0.0281 s | 3234 | 3234 | 3232 | 0.999382 | 0.999382 | map 0，IMU/BA1/BA2 均完成 | 0 |

两次都自动 EOF、后台 drain/join 完成，TUM 非空，timestamp 为原图像 ns，主文件为
左 cam `Twc`、惯性世界坐标、不首帧归零。summary/log 见：

- 原标定：`results/ego_job_004/round5_original_offset_final2/trajectory.summary.json`、
  `.log`、`console.log`、`console.err`、`exit`。
- 独立核验：`results/ego_job_004/round5_validated_offset_final2/trajectory.summary.json`、
  `.log`、`console.log`、`console.err`、`exit`。

Codex `check_calibration.py` 对最终配置自测通过：`results/ego_job_004/reports/check_calibration_final.log`，exit 0。

Codex `check_tum.py` 自测（不作为最终独立验收）：

- `results/ego_job_004/reports/check_tum_round5_original_offset_final2.log`，exit 0；
- `results/ego_job_004/reports/check_tum_round5_validated_offset_final2.log`，exit 0。

轨迹形状（无 GT，不提供 ATE/绝对精度结论）：

| 运行 | 路径长 | 最大相邻步进 | 平均步进 | x/y/z 范围 | 关键帧 |
| --- | ---: | ---: | ---: | --- | ---: |
| 原标定 | 1274.36 m | 1.726 m | 0.395 m | 1225.8 × 194.3 × 83.7 m | 1829 |
| 独立核验 | 42.89 m | 0.0299 m | 0.0133 m | 0.668 × 0.567 × 0.689 m | 47 |

原标定运行虽然覆盖/退出状态合法，但形状明显发散，不能当作精度通过；本记录如实
保留。独立核验运行形状更可信，但仍没有 GT，不能宣称绝对精度或通用 offset。

`+28.1 ms` 的一次早期运行在 VIBA2 后出现 SIGSEGV，失败证据保留在
`results/ego_job_004/round5_validated_offset_final/`；后续 `gdb_281/` 复跑成功，没有
捕获崩溃栈，因此该次崩溃的根因未确定。

另行审查发现旧 GBA worker 在 `mpThreadGBA` 已清空但尚未 join 时，新 worker 可能
启动并共享 `mbStopGBA`/map。修复为在 `mMutexGBAThread` 持有期间完成 stop、join、
delete 和新 worker 创建；独立并发探针验证新 worker 等待旧 worker join 完成。修复后
pi 和 Codex 分别以同一 `+28.1 ms`、同一配置完成整包并 exit 0，证据位于
`round5_validated_offset_final2/` 和 `codex_independent_final/`。这些结果不能证明该竞态
就是上述 SIGSEGV 的原因。

统计/形状原始输出：`results/ego_job_004/reports/shape_round5_original_offset_final2.json`、
`shape_round5_validated_offset_final2.json`；汇总 `results/ego_job_004/reports/round5_summary.json`。

## 5. 验收门槛与局限

- 成功门槛未修改：total 有效位姿/eligible `>=95%`、main map `>=90%`；eligible 只扣
  首尾修正后 IMU 覆盖不足。
- 最终配置的 `nFeatures=3000`、`IMU.fastInit=1`、两个 Defer reset 开关均有完整包日志
  证据；1200 特征、这些开关关闭及 `+0.0267 s` 的独立基线也达到 3103/3234（95.949%），
  因此不能把 3000 视为达标必要条件或把覆盖差异单独归因于特征数，见
  `results/ego_job_004/codex_independent_baseline/`。没有降分辨率/帧率、没有放大噪声、
  没有改外参、没有关闭惯性、没有伪造位姿。
- 原 offset 与验证 offset 的两次运行使用同一算法参数；原 offset 的形状发散不等于
  验证 offset 有绝对真值。
- `+28.1 ms` 只对当前 bag/calib/采集链有效，不可外推到其它包；原 calib offset 保持
  只读，运行命令显式覆盖。
- 最终验收仍由 Codex 独立执行；本记录状态为 `review_required`，不是 accepted。
