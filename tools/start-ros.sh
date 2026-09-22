#!/usr/bin/env bash
set -eo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
MODE="${1:-sim}"
if [[ -n "${VIRTUAL_ENV:-}" || -n "${CONDA_PREFIX:-}" ]]; then
  echo '请先退出 Python 虚拟环境（deactivate / conda deactivate），ROS 使用系统 Python；相机桥接仍可使用原虚拟环境。' >&2
  exit 2
fi
case "$MODE" in sim|readonly|real) ;; *) echo 'Real motion unavailable: firmware/model/CPV stop-hold not verified.' >&2; exit 2;; esac
shift || true
source /opt/ros/humble/setup.bash
source "$ROOT/ros2_ws/install/setup.bash"
export ROS_LOCALHOST_ONLY=1
export PYTHONPATH="$ROOT/vendor/pyAgxArm${PYTHONPATH:+:$PYTHONPATH}"
if [[ "$MODE" == readonly || "$MODE" == real ]]; then export PYTHONPATH="$ROOT/.deps-readonly${PYTHONPATH:+:$PYTHONPATH}"; fi
cd "$ROOT"
CAN_NAME=can0
if [[ -f config/imported-connection.json ]]; then
  CAN_NAME="$(/usr/bin/python3 -c 'import json;print(json.load(open("config/imported-connection.json"))["can_name"])')"
fi
if [[ "$MODE" == real ]]; then
  /usr/bin/python3 tools/check-can-owner.py
  /usr/bin/python3 tools/check-v81-environment.py
fi
exec /usr/bin/python3 /opt/ros/humble/bin/ros2 launch robot_console_servo console.launch.py mode:="$MODE" can_name:="$CAN_NAME" "$@"
