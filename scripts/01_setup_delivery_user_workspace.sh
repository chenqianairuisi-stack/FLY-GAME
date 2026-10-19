#!/usr/bin/env bash
set -Eeuo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
ROS_SETUP="${ROS_SETUP:-/opt/ros/humble/setup.bash}"
ROSDEP_SKIP_KEYS="${ROSDEP_SKIP_KEYS:-ament_python}"
BINARY_ARCHIVE="${WORKSPACE_DIR}/packages/ican_delivery_env_binary.tar.gz"
BINARY_SETUP="${WORKSPACE_DIR}/binary_install/setup.bash"

log() {
  printf '\033[1;34m[setup-workspace]\033[0m %s\n' "$*"
}

warn() {
  printf '\033[1;33m[setup-workspace]\033[0m %s\n' "$*" >&2
}

die() {
  printf '\033[1;31m[setup-workspace]\033[0m %s\n' "$*" >&2
  exit 1
}

source_setup_file() {
  local setup_file="$1"
  set +u
  # shellcheck disable=SC1090
  source "${setup_file}"
  set -u
}

RUN_ROSDEP=1
RUN_BUILD=1

while [[ $# -gt 0 ]]; do
  case "$1" in
    --skip-rosdep)
      RUN_ROSDEP=0
      shift
      ;;
    --skip-build)
      RUN_BUILD=0
      shift
      ;;
    *)
      die "未知选项：$1"
      ;;
  esac
done

[[ -f "${ROS_SETUP}" ]] || die "未找到 ${ROS_SETUP}，请先运行 scripts/00_install_ros2_gazebo.sh。"
[[ -f "${BINARY_ARCHIVE}" ]] || die "未找到二进制基础环境包：${BINARY_ARCHIVE}"

cd "${WORKSPACE_DIR}"
source_setup_file "${ROS_SETUP}"

if [[ ! -f "${BINARY_SETUP}" ]]; then
  log "正在解包物资投送基础仿真、传感器和飞控二进制包。"
  tar -xzf "${BINARY_ARCHIVE}" -C "${WORKSPACE_DIR}"
fi

[[ -f "${BINARY_SETUP}" ]] || die "二进制环境解包失败：${BINARY_SETUP}"
source_setup_file "${BINARY_SETUP}"

if [[ "${RUN_ROSDEP}" == "1" ]]; then
  if command -v rosdep >/dev/null 2>&1; then
    log "正在安装用户示例包 ROS 依赖。"
    rosdep_args=(--from-paths src --ignore-src -r -y)
    if [[ -n "${ROSDEP_SKIP_KEYS}" ]]; then
      rosdep_args+=(--skip-keys "${ROSDEP_SKIP_KEYS}")
    fi
    rosdep install "${rosdep_args[@]}"
  else
    warn "未找到 rosdep，跳过依赖安装。"
  fi
fi

if [[ "${RUN_BUILD}" == "1" ]]; then
  command -v colcon >/dev/null 2>&1 || die "未找到 colcon，请先安装 python3-colcon-common-extensions。"
  log "正在编译用户示例包。"
  # shellcheck disable=SC2086
  colcon build --symlink-install ${COLCON_ARGS:-}
fi

log "用户工作区配置完成。下一步：source scripts/setup_env.sh"
