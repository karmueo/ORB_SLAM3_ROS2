# Repository Guidelines

## Project Structure & Module Organization

This repository is a ROS 2 `ament_cmake` wrapper around ORB_SLAM3. Core C++ sources live under `src/`, grouped by runtime mode: `src/monocular/`, `src/rgbd/`, `src/stereo/`, and `src/stereo-inertial/`. Shared headers are in `include/`. Camera and dataset settings are stored in `config/`, with subdirectories matching SLAM modes and datasets. CMake find modules are in `CMakeModules/`. The ORB vocabulary archive is kept in `vocabulary/`.

## Build, Test, and Development Commands

Use this package inside a ROS 2 workspace, typically `~/colcon_ws/src/orbslam3_ros2`.

```bash
colcon build --symlink-install --packages-select orbslam3
```

Builds the package and installs the `mono`, `rgbd`, `stereo`, and `stereo-inertial` executables.

```bash
source ~/colcon_ws/install/local_setup.bash
ros2 run orbslam3 mono vocabulary/ORBvoc.txt config/monocular/TUM1.yaml
```

Sources the workspace and runs a monocular example. Adjust vocabulary and YAML paths for the selected mode. Before building, verify `CMakeLists.txt` `PYTHONPATH` and `CMakeModules/FindORB_SLAM3.cmake` `ORB_SLAM3_ROOT_DIR` match the local machine.

## Coding Style & Naming Conventions

The package uses C++14 with `-Wall -Wextra -Wpedantic`. Follow the existing ROS 2 C++ style: 2-space CMake indentation, lower-case executable names, and mode-specific node names such as `monocular-slam-node.cpp`. Keep mode entry points small (`mono.cpp`, `rgbd.cpp`) and place ROS subscription logic in the corresponding node class files.

## Testing Guidelines

No automated test suite is currently included. For changes, run `colcon build --symlink-install --packages-select orbslam3` and perform a smoke test with the affected mode. When adding tests, prefer ROS 2 `ament_cmake_gtest` or launch tests, place them under `test/`, and name files by behavior or mode, for example `test_stereo_node.cpp`.

## Commit & Pull Request Guidelines

Recent history uses short, imperative summaries such as `fix CMakeLists.txt install error` and `added opencv target library (#10)`. Keep commits focused and describe user-visible behavior or build impact. Pull requests should include a concise description, tested ROS 2 distribution, ORB_SLAM3/OpenCV versions, commands run, and any dataset or configuration required to reproduce the result.

## Configuration Notes

Do not commit local absolute paths, generated build directories, or expanded vocabulary files. Keep dataset-specific calibration changes in the matching `config/<mode>/` directory and document any non-default topic remapping in the PR.
