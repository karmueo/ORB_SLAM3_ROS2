#!/usr/bin/env bash
# 在仓库内 .deps/ 准备固定版本 ORB_SLAM3 及其本地 Pangolin/Sophus 依赖。
# 该脚本不调用 sudo，不安装系统软件，只下载公开 .deb 并解压到本地 prefix。
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
DEPS="${ROOT}/.deps"
ORB_ROOT="${DEPS}/ORB_SLAM3"
LOCAL="${DEPS}/local"
APT_CACHE="${DEPS}/apt-cache"
ORB_COMMIT="f8ac79153b103dd44f7ea3cf2a1a4aea10b18483"
OPENCV_CMAKE_DIR="/usr/lib/x86_64-linux-gnu/cmake/opencv4"

PANGOLIN_VERSION="0.9.6-1noble.20260820.143839"
SOPHUS_VERSION="1.22.9102-2noble.20260225.142549"
GLFW_VERSION="3.3.10-1build1"
EPOXY_VERSION="1.5.10-1build1"

mkdir -p "${DEPS}" "${APT_CACHE}" "${LOCAL}"
cd "${ROOT}"

if [[ ! -d "${ORB_ROOT}/.git" ]]; then
  git clone --no-checkout https://github.com/zang09/ORB-SLAM3-STEREO-FIXED.git "${ORB_ROOT}"
fi
git -C "${ORB_ROOT}" fetch --depth 1 origin "${ORB_COMMIT}"
git -C "${ORB_ROOT}" checkout --detach "${ORB_COMMIT}"
if [[ "$(git -C "${ORB_ROOT}" rev-parse HEAD)" != "${ORB_COMMIT}" ]]; then
  echo "ERROR: ORB_SLAM3 checkout did not reach ${ORB_COMMIT}" >&2
  exit 1
fi

# 校验固定 patch 文件，避免误用被改动过的补丁。
PATCH_0001="${ROOT}/patches/0001-offline-core-compat.patch"
PATCH_0002="${ROOT}/patches/0002-offline-lifecycle-snapshot.patch"
PATCH_0001_SHA256="dc2f389b1d77dce98193d91fd68feba32de75bc6d97e905f28ffc8334e7818d8"
PATCH_0002_SHA256="29882f2bf5bfe25e66a61aad854734219d42ee1e1a1e518912214bcfb1087337"
for spec in "${PATCH_0001}:${PATCH_0001_SHA256}" "${PATCH_0002}:${PATCH_0002_SHA256}"; do
  patch_path="${spec%%:*}"
  expected_sha="${spec##*:}"
  if [[ ! -f "${patch_path}" ]]; then
    echo "ERROR: 缺少 patch: ${patch_path}" >&2
    exit 1
  fi
  actual_sha="$(sha256sum "${patch_path}" | awk '{print $1}')"
  if [[ "${actual_sha}" != "${expected_sha}" ]]; then
    echo "ERROR: patch sha256 不匹配: ${patch_path}" >&2
    echo "  expected=${expected_sha}" >&2
    echo "  actual  =${actual_sha}" >&2
    exit 1
  fi
done

# 按顺序应用离线兼容/生命周期补丁。辅助脚本会比较完整工作树与
# fresh / 0001 / 0001+0002 的期望 tree，安全处理重复执行和部分应用；
# 未知 tracked 更改会被拒绝且原样保留。0002 依赖 0001，顺序不能调换。
bash "${ROOT}/scripts/apply_core_patch_stack.sh" \
  "${ORB_ROOT}" "${PATCH_0001}" "${PATCH_0002}"

download_deb() {
  local package="$1"
  local version="$2"
  local architecture="amd64"
  local expected="${APT_CACHE}/${package}_${version}_${architecture}.deb"
  if [[ ! -f "${expected}" ]]; then
    (cd "${APT_CACHE}" && apt-get download "${package}=${version}")
  fi
}

download_deb ros-jazzy-pangolin "${PANGOLIN_VERSION}"
download_deb ros-jazzy-sophus "${SOPHUS_VERSION}"
download_deb libglfw3 "${GLFW_VERSION}"
download_deb libglfw3-dev "${GLFW_VERSION}"
download_deb libepoxy-dev "${EPOXY_VERSION}"

for deb in "${APT_CACHE}"/*.deb; do
  dpkg-deb -x "${deb}" "${LOCAL}"
done

# libepoxy-dev 只提供开发符号链接，运行时 .so 来自系统 libepoxy0；
# 在本地 prefix 中补链，避免修改系统目录。
if [[ -f /usr/lib/x86_64-linux-gnu/libepoxy.so.0 ]]; then
  cp -L /usr/lib/x86_64-linux-gnu/libepoxy.so.0 "${LOCAL}/usr/lib/x86_64-linux-gnu/libepoxy.so.0.0.0"
  ln -sfn libepoxy.so.0.0.0 "${LOCAL}/usr/lib/x86_64-linux-gnu/libepoxy.so.0"
  ln -sfn libepoxy.so.0 "${LOCAL}/usr/lib/x86_64-linux-gnu/libepoxy.so"
fi

# Debian PangolinConfig 把 libepoxy.so 写成绝对系统路径；改为本地 prefix
# 后无需 sudo 创建系统符号链接。
for config in "${LOCAL}"/opt/ros/jazzy/lib/x86_64-linux-gnu/cmake/Pangolin/PangolinTargets*.cmake; do
  if [[ -f "${config}" ]]; then
    sed -i "s#/usr/lib/x86_64-linux-gnu/libepoxy.so#${LOCAL}/usr/lib/x86_64-linux-gnu/libepoxy.so.0#g" "${config}"
  fi
done

cmake -S "${ORB_ROOT}/Thirdparty/DBoW2" -B "${ORB_ROOT}/Thirdparty/DBoW2/build" \
  -DCMAKE_BUILD_TYPE=Release \
  -DOpenCV_DIR="${OPENCV_CMAKE_DIR}"
cmake --build "${ORB_ROOT}/Thirdparty/DBoW2/build" -j"${CMAKE_BUILD_PARALLEL_LEVEL:-8}"

# g2o 与核心共享 Eigen/g2o 类型；C++ 标准与核心 C++14 明确对齐。
cmake -S "${ORB_ROOT}/Thirdparty/g2o" -B "${ORB_ROOT}/Thirdparty/g2o/build" \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_CXX_STANDARD=14 \
  -DCMAKE_CXX_STANDARD_REQUIRED=ON
cmake --build "${ORB_ROOT}/Thirdparty/g2o/build" -j"${CMAKE_BUILD_PARALLEL_LEVEL:-8}"

export LD_LIBRARY_PATH="${LOCAL}/opt/ros/jazzy/lib/x86_64-linux-gnu:${LOCAL}/usr/lib/x86_64-linux-gnu:${LD_LIBRARY_PATH:-}"
cmake -S "${ORB_ROOT}" -B "${ORB_ROOT}/build" \
  -DCMAKE_BUILD_TYPE=Release \
  -DOpenCV_DIR="${OPENCV_CMAKE_DIR}" \
  -DCMAKE_PREFIX_PATH="${LOCAL}/opt/ros/jazzy;${LOCAL}/usr" \
  -DCMAKE_CXX_FLAGS="-I${LOCAL}/usr/include" \
  -DCMAKE_BUILD_RPATH="${LOCAL}/opt/ros/jazzy/lib/x86_64-linux-gnu;${LOCAL}/usr/lib/x86_64-linux-gnu" \
  -DCMAKE_INSTALL_RPATH="${LOCAL}/opt/ros/jazzy/lib/x86_64-linux-gnu;${LOCAL}/usr/lib/x86_64-linux-gnu"
cmake --build "${ORB_ROOT}/build" --target ORB_SLAM3 -j"${CMAKE_BUILD_PARALLEL_LEVEL:-8}"

echo "ORB_SLAM3 ready: ${ORB_ROOT}/lib/libORB_SLAM3.so"
