#!/usr/bin/env bash
# Full practice mission. Does not start the original minimal drop demo.
set -eo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
cd "$WORKSPACE_DIR"
if pgrep -x gzserver >/dev/null; then
    echo "检测到已有 Gazebo 仿真。请先关闭旧仿真及其飞控/示例，再启动完整 Demo，避免重复控制。" >&2
    exit 1
fi
# Ubuntu's ROS/OpenCV binaries require the distribution's NumPy ABI.
export PYTHONNOUSERSITE=1
export ROS2CLI_NO_DAEMON=1
source scripts/setup_env.sh
/usr/bin/python3 -c 'import cv2, numpy, cv_bridge; assert hasattr(cv2, "aruco"), "需要系统 python3-opencv 的 ArUco 模块"'
if [[ ! -f install/ican_delivery_examples/lib/ican_delivery_examples/full_mission || ! -f install/ican_delivery_throw/lib/libdelivery_throw.so ]]; then
    colcon build --symlink-install --packages-up-to ican_delivery_examples
    source install/setup.bash
fi
exec ros2 launch ican_delivery_examples full_demo.launch.py "$@"
