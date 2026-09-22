#!/usr/bin/env bash
set -eo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
source /opt/ros/humble/setup.bash
cd "$ROOT/ros2_ws"
colcon build --packages-select console_servo_stamp robot_console_servo --symlink-install
