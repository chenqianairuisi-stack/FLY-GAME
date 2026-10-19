#!/usr/bin/env bash
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
SETUP_CMD="cd \"${WORKSPACE_DIR}\"; source \"${WORKSPACE_DIR}/scripts/setup_env.sh\""

gnome-terminal --window --title="delivery sim" -e "bash -c \"${SETUP_CMD}; ros2 launch ican_delivery_sim delivery.launch.py rviz:=true; exec bash\"" \
--tab --title="flight control" -e "bash -c \"sleep 5; ${SETUP_CMD}; ros2 launch ican_delivery_flight_control flight_control.launch.py; exec bash\"" \
--tab --title="minimal example" -e "bash -c \"sleep 9; ${SETUP_CMD}; ros2 run ican_delivery_examples minimal_drop_demo; exec bash\""
