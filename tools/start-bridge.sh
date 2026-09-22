#!/usr/bin/env bash
set -eo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
MODE="${1:-sim}"
case "$MODE" in sim|readonly|real) ;; *) echo 'Allowed: sim, readonly, real' >&2; exit 2;; esac
export PYTHONPATH="$ROOT/bridge${PYTHONPATH:+:$PYTHONPATH}"
cd "$ROOT"
exec "${BRIDGE_PYTHON:-python3}" -m robot_bridge --config "${2:-config/bridge.$MODE.yaml}"
