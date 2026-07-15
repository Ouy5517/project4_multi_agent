#!/usr/bin/env bash
set -euo pipefail

cd "$(dirname "$0")/.."
PYTHON_BIN="${PYTHON_BIN:-./coop_env/bin/python}"
if [[ ! -x "$PYTHON_BIN" ]]; then
  PYTHON_BIN="${PYTHON_BIN_FALLBACK:-/usr/bin/python3}"
fi

"$PYTHON_BIN" -m mujoco_soccer.diagnostics.stutter_diagnosis "$@"
