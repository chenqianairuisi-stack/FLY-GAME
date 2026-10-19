#!/usr/bin/env bash
set -Eeuo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
ROS_SETUP="${ROS_SETUP:-/opt/ros/humble/setup.bash}"
BINARY_SETUP="${WORKSPACE_DIR}/binary_install/setup.bash"
WORKSPACE_SETUP="${WORKSPACE_DIR}/install/setup.bash"
FAILED=0

source_setup_file() {
  local setup_file="$1"
  set +u
  # shellcheck disable=SC1090
  source "${setup_file}"
  set -u
}

check() {
  local label="$1"
  shift
  if "$@" >/tmp/ican_delivery_user_check.out 2>/tmp/ican_delivery_user_check.err; then
    printf '\033[1;32m[OK]\033[0m %s\n' "${label}"
  else
    printf '\033[1;31m[FAIL]\033[0m %s\n' "${label}" >&2
    sed -n '1,20p' /tmp/ican_delivery_user_check.err >&2
    FAILED=1
  fi
}

clean_terminal_env() {
  env \
    -u SNAP \
    -u SNAP_ARCH \
    -u SNAP_INSTANCE_NAME \
    -u SNAP_NAME \
    -u SNAP_REVISION \
    -u SNAP_USER_COMMON \
    -u SNAP_USER_DATA \
    -u GTK_PATH \
    -u GI_TYPELIB_PATH \
    -u GDK_PIXBUF_MODULE_FILE \
    -u GDK_PIXBUF_MODULEDIR \
    "$@"
}

check "ROS 2 Humble setup" test -f "${ROS_SETUP}"
check "基础二进制环境 setup" test -f "${BINARY_SETUP}"
check "用户工作区 setup" test -f "${WORKSPACE_SETUP}"

if [[ -f "${ROS_SETUP}" ]]; then
  source_setup_file "${ROS_SETUP}"
fi

if [[ -f "${BINARY_SETUP}" ]]; then
  source_setup_file "${BINARY_SETUP}"
fi

if [[ -f "${WORKSPACE_SETUP}" ]]; then
  source_setup_file "${WORKSPACE_SETUP}"
fi

check "ros2 命令" command -v ros2
check "Gazebo 命令" command -v gazebo
check "colcon 命令" command -v colcon
check "gnome-terminal 命令" command -v gnome-terminal
check "gnome-terminal 可运行" clean_terminal_env gnome-terminal --version
check "gazebo_ros 包" ros2 pkg prefix gazebo_ros
check "tf2_ros 包" ros2 pkg prefix tf2_ros
check "sensor_msgs_py 包" python3 -c "import sensor_msgs_py"
check "ican_delivery_sim 包" ros2 pkg prefix ican_delivery_sim
check "ican_delivery_flight_control 包" ros2 pkg prefix ican_delivery_flight_control
check "ican_delivery_examples 包" ros2 pkg prefix ican_delivery_examples
check "物资投送仿真 launch 参数" ros2 launch ican_delivery_sim delivery.launch.py --show-args
check "飞控 launch 参数" ros2 launch ican_delivery_flight_control flight_control.launch.py --show-args

rm -f /tmp/ican_delivery_user_check.out /tmp/ican_delivery_user_check.err
exit "${FAILED}"
