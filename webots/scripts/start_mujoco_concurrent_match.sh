#!/usr/bin/env bash
set -euo pipefail

MODE="view"
SEED="42"
DURATION=""

while [[ $# -gt 0 ]]; do
  case "$1" in
    --view) MODE="view"; shift ;;
    --record) MODE="record"; shift ;;
    --no-render) MODE="no-render"; shift ;;
    --seed) SEED="${2:-42}"; shift 2 ;;
    --duration) DURATION="${2:-60}"; shift 2 ;;
    *)
      echo "Unknown argument: $1" >&2
      exit 2
      ;;
  esac
done

cd "$(dirname "$0")/.."
PYTHON_BIN="${PYTHON_BIN:-/usr/bin/python3}"
"$PYTHON_BIN" -m mujoco_soccer.tools_generate_proxy_model >/dev/null

DURATION_ARGS=()
if [[ -n "$DURATION" ]]; then
  DURATION_ARGS=(--duration "$DURATION")
fi

case "$MODE" in
  view)
    "$PYTHON_BIN" -m mujoco_soccer.run_demo --mode concurrent-match --view --seed "$SEED" "${DURATION_ARGS[@]}"
    ;;
  record)
    "$PYTHON_BIN" -m mujoco_soccer.run_demo --mode concurrent-match --seed "$SEED" "${DURATION_ARGS[@]}"
    ;;
  no-render)
    "$PYTHON_BIN" -m mujoco_soccer.run_demo --mode concurrent-match --no-render --seed "$SEED" "${DURATION_ARGS[@]}"
    ;;
esac
