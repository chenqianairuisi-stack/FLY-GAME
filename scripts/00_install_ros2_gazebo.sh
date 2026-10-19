#!/usr/bin/env bash
set -Eeuo pipefail

SCRIPT_NAME="$(basename "$0")"
ROS_DISTRO_NAME="${ROS_DISTRO_NAME:-humble}"

log() {
  printf '\033[1;34m[%s]\033[0m %s\n' "${SCRIPT_NAME}" "$*"
}

warn() {
  printf '\033[1;33m[%s]\033[0m %s\n' "${SCRIPT_NAME}" "$*" >&2
}

die() {
  printf '\033[1;31m[%s]\033[0m %s\n' "${SCRIPT_NAME}" "$*" >&2
  exit 1
}

if [[ -r /etc/os-release ]]; then
  # shellcheck disable=SC1091
  . /etc/os-release
else
  die "无法读取 /etc/os-release"
fi

if [[ "${ID:-}" != "ubuntu" || "${VERSION_ID:-}" != "22.04" ]]; then
  die "当前脚本只面向 Ubuntu 22.04 LTS。当前系统：${PRETTY_NAME:-unknown}"
fi

if [[ $EUID -eq 0 ]]; then
  SUDO=()
else
  command -v sudo >/dev/null 2>&1 || die "缺少 sudo 命令"
  SUDO=(sudo)
fi

log "准备安装 ROS 2 Humble、Gazebo Classic、RViz、传感器消息和构建工具。"
"${SUDO[@]}" apt-get update

packages=(
  "ros-${ROS_DISTRO_NAME}-desktop"
  "ros-${ROS_DISTRO_NAME}-gazebo-ros-pkgs"
  "ros-${ROS_DISTRO_NAME}-gazebo-plugins"
  "ros-${ROS_DISTRO_NAME}-tf2-ros"
  "ros-${ROS_DISTRO_NAME}-robot-state-publisher"
  "ros-${ROS_DISTRO_NAME}-rviz2"
  "ros-${ROS_DISTRO_NAME}-sensor-msgs-py"
  gazebo
  libgazebo-dev
  gnome-terminal
  python3-colcon-common-extensions
  python3-rosdep
  python3-pip
  build-essential
  cmake
)

"${SUDO[@]}" DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends "${packages[@]}"

if command -v rosdep >/dev/null 2>&1; then
  if [[ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]]; then
    log "正在初始化 rosdep。"
    "${SUDO[@]}" rosdep init || warn "rosdep init 未完成；如果提示已初始化，可忽略。"
  fi
  log "正在更新 rosdep 数据。"
  rosdep update || warn "rosdep update 未完成；网络恢复后可手动重试。"
fi

[[ -f "/opt/ros/${ROS_DISTRO_NAME}/setup.bash" ]] || die "未找到 /opt/ros/${ROS_DISTRO_NAME}/setup.bash"
log "基础环境安装完成。建议继续运行：bash scripts/01_setup_delivery_user_workspace.sh"
