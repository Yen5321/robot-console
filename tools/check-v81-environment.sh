#!/usr/bin/env bash
set -eo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
source /opt/ros/humble/setup.bash
export PYTHONPATH="$ROOT/.deps-readonly:$ROOT/vendor/pyAgxArm${PYTHONPATH:+:$PYTHONPATH}"
exec /usr/bin/python3 "$ROOT/tools/check-v81-environment.py"
