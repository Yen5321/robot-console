#!/usr/bin/env bash
set -eo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
if [[ $# -eq 0 ]]; then
  source /opt/ros/humble/setup.bash
  export PYTHONPATH="$ROOT/.deps-readonly:$ROOT/vendor/pyAgxArm${PYTHONPATH:+:$PYTHONPATH}"
  exec python3 "$ROOT/tools/check-v81-environment.py"
fi
LOCK="${1:-$ROOT/config/dependencies.lock.json}"
ACTION="${2:-install}"
if [[ $# -gt 2 || ( "$ACTION" != install && "$ACTION" != --dry-run ) ]]; then
  echo 'Usage: bash tools/install-deps.sh [lock.json] [--dry-run]' >&2; exit 2
fi
source /etc/os-release
if [[ "$VERSION_ID" != '22.04' ]]; then echo 'Requires Ubuntu 22.04; no OS changes will be made.' >&2; exit 2; fi
if [[ ! -f /opt/ros/humble/setup.bash ]]; then
  echo 'Install ROS 2 Humble from https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html first.' >&2
  exit 2
fi
# 默认仍使用原测试清单；现场版本必须显式选择，绝不自动换成候选版本。
# 先检查读取成功，再转成数组；进程替换会隐藏 Python 失败，不能用于这里。
PIN_TEXT="$(python3 - "$LOCK" <<'PY'
import json,re,sys
with open(sys.argv[1],encoding='utf-8') as f:d=json.load(f)
if d.get('ros_distribution')!='humble':raise ValueError('Requires a Humble lock')
pins=d.get('requested_debian_packages',d.get('tested_debian_packages'))
if not isinstance(pins,dict) or not pins:raise ValueError('Missing package pins')
for name,version in pins.items():
    if not re.fullmatch(r'ros-humble-[a-z0-9-]+',name) or not isinstance(version,str) or not re.fullmatch(r'[0-9A-Za-z.+:~_-]+',version):
        raise ValueError('Invalid package pin')
print('\n'.join(k+'='+v for k,v in pins.items()))
PY
)"
mapfile -t PINNED <<< "$PIN_TEXT"
echo "Dependency lock: $LOCK"
# launch_ros 导入 launch 的 PathSubstitution；APT 的宽松依赖下仅更新前者
# 可能保留旧 launch，因此显式请求同一镜像的完整启动组件。
EXTRA=(python3-colcon-common-extensions python3-pip python3-numpy python3-scipy python3-yaml ros-humble-ament-cmake ros-humble-launch ros-humble-launch-ros ros-humble-launch-xml ros-humble-launch-yaml ros-humble-ros2launch ros-humble-rosbag2)
if [[ "$ACTION" == --dry-run ]]; then
  sudo apt-get --simulate --no-remove install "${PINNED[@]}" "${EXTRA[@]}"
  exit 0
fi
# 允许 APT 解算同一 Humble 发行版的依赖；不允许删除已有软件包。
sudo apt-get --no-remove install "${PINNED[@]}" "${EXTRA[@]}"
(
  source /opt/ros/humble/setup.bash
  /usr/bin/python3 -c 'from launch.substitutions import PathSubstitution; from launch_ros.actions import Node; print("ROS launch imports OK")'
)
python3 -m pip install --target "$ROOT/.deps-readonly" -r "$ROOT/config/requirements-readonly.lock.txt"
