#!/usr/bin/env bash
# 按顺序把一组 git patch 应用到固定核心工作树。
#
# 该脚本先在一个临时 git index 中构造每个累计 patch 状态的期望 tree，再与
# 实际工作树比较。只有实际 tracked 内容与 fresh、部分应用或完整应用中的
# 某个状态完全一致时才推进；任何未知 tracked 更改都会被明确拒绝并原样保留。
#
# 用法: apply_core_patch_stack.sh REPO PATCH [PATCH ...]
# 退出码: 0 成功/已完整应用; 2 参数或 patch 错误; 3 未知工作树状态。
set -euo pipefail

if [[ $# -lt 2 ]]; then
  echo "Usage: $0 REPO PATCH [PATCH ...]" >&2
  exit 2
fi

REPO="$1"
shift
PATCHES=()
for patch in "$@"; do
  if [[ ! -f "${patch}" ]]; then
    echo "ERROR: patch 不存在: ${patch}" >&2
    exit 2
  fi
  PATCHES+=("$(realpath "${patch}")")
done

if [[ ! -d "${REPO}/.git" ]]; then
  echo "ERROR: 不是 git 工作树: ${REPO}" >&2
  exit 2
fi

TMP_DIR="$(mktemp -d "${TMPDIR:-/tmp}/patch-stack.XXXXXXXX")"
cleanup() {
  rm -rf "${TMP_DIR}"
}
trap cleanup EXIT

# 用临时 index 把 HEAD + 前 k 个 patch 物化为 tree 对象，不触碰工作树。
build_expected_tree() {
  local patch_count="$1"
  local index_file="${TMP_DIR}/index.${patch_count}"
  local i
  GIT_INDEX_FILE="${index_file}" git -C "${REPO}" read-tree HEAD
  for ((i = 0; i < patch_count; ++i)); do
    GIT_INDEX_FILE="${index_file}" \
      git -C "${REPO}" apply --cached --whitespace=nowarn "${PATCHES[$i]}"
  done
  GIT_INDEX_FILE="${index_file}" git -C "${REPO}" write-tree
}

TREES=()
for ((i = 0; i <= ${#PATCHES[@]}; ++i)); do
  TREES+=("$(build_expected_tree "${i}")")
done

# 从“全部已应用”向“fresh”查找第一个完全一致的状态。
match=-1
for ((i = ${#PATCHES[@]}; i >= 0; --i)); do
  if git -C "${REPO}" diff --quiet "${TREES[$i]}" --; then
    match="$i"
    break
  fi
done

if [[ "${match}" -lt 0 ]]; then
  echo "ERROR: 核心工作树含未知 tracked 更改，拒绝修改以保留用户内容。" >&2
  echo "当前相对固定 HEAD 的 tracked 变更:" >&2
  git -C "${REPO}" diff --name-status HEAD -- >&2 || true
  echo "期望状态为以下之一: fresh / 已应用前 k 个 patch 且无其他更改。" >&2
  exit 3
fi

if [[ "${match}" -eq "${#PATCHES[@]}" ]]; then
  echo "Patch stack already complete (${#PATCHES[@]}/${#PATCHES[@]})"
  exit 0
fi

for ((i = match; i < ${#PATCHES[@]}; ++i)); do
  git -C "${REPO}" apply --whitespace=nowarn "${PATCHES[$i]}"
  echo "Applied patch: $(basename "${PATCHES[$i]}")"
done

# 应用后必须精确等于期望的最终 tree，防止 patch 以非预期方式落盘。
if ! git -C "${REPO}" diff --quiet "${TREES[${#PATCHES[@]}]}" --; then
  echo "ERROR: 应用 patch 后工作树与期望 patch stack 不一致。" >&2
  git -C "${REPO}" diff --name-status "${TREES[${#PATCHES[@]}]}" -- >&2 || true
  exit 3
fi
echo "Patch stack complete (${#PATCHES[@]}/${#PATCHES[@]})"
