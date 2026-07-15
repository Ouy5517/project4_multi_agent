#!/usr/bin/env bash
set -euo pipefail

MODE="view"
DURATION=""
SPEED="1.0"

while [[ $# -gt 0 ]]; do
  case "$1" in
    --view) MODE="view"; shift ;;
    --record) MODE="record"; shift ;;
    --replay) MODE="replay"; shift ;;
    --duration) DURATION="${2:-}"; shift 2 ;;
    --speed) SPEED="${2:-1.0}"; shift 2 ;;
    *)
      echo "Unknown argument: $1" >&2
      exit 2
      ;;
  esac
done

cd "$(dirname "$0")/.."
PYTHON_BIN="${PYTHON_BIN:-/usr/bin/python3}"

"$PYTHON_BIN" - <<'PY'
import importlib.util
import sys
if importlib.util.find_spec("mujoco") is None:
    print("MuJoCo Python package is not available", file=sys.stderr)
    sys.exit(1)
PY

"$PYTHON_BIN" -m mujoco_soccer.tools_generate_proxy_model >/dev/null
MODEL="mujoco_soccer/models/t1_2v2_soccer_visual_v3.xml"
if [[ ! -f "$MODEL" ]]; then
  echo "Visual V3 model missing: $MODEL" >&2
  exit 1
fi

if [[ "$MODE" == "replay" ]]; then
  VIDEO="$(ls -t results/mujoco_four_robot_demo/visual_v3_*/demo_visual_v3.mp4 2>/dev/null | head -n 1 || true)"
  if [[ -z "$VIDEO" ]]; then
    echo "No Visual V3 video found." >&2
    exit 1
  fi
  echo "PLAYBACK OF RECORDED PHYSICAL SIMULATION"
  echo "$VIDEO"
  if command -v xdg-open >/dev/null 2>&1; then
    xdg-open "$VIDEO" >/dev/null 2>&1 || true
  fi
  exit 0
fi

if [[ -n "${DISPLAY:-}" ]]; then
  export MUJOCO_GL="${MUJOCO_GL:-glfw}"
else
  export MUJOCO_GL="${MUJOCO_GL:-egl}"
  if [[ "$MODE" == "view" ]]; then
    echo "DISPLAY is not set; VIEW mode will run offscreen timing without opening a GUI."
  fi
fi

RUN_ID="visual_v3_$(date +%Y%m%d_%H%M%S)"
DURATION_ARGS=()
if [[ -n "$DURATION" ]]; then
  DURATION_ARGS=(--duration "$DURATION")
fi

case "$MODE" in
  view)
    echo "Starting Visual V3 VIEW: $RUN_ID"
    RUN_MODE="visual-v3-demo"
    if [[ -n "$DURATION" ]]; then
      RUN_MODE="visual-check-v3"
    fi
    "$PYTHON_BIN" -m mujoco_soccer.run_demo \
      --mode "$RUN_MODE" \
      --model "$MODEL" \
      --camera broadcast_wide \
      --playback-speed "$SPEED" \
      --visual \
      --fast-viewer \
      --no-record \
      --run-id "$RUN_ID" \
      "${DURATION_ARGS[@]}"
    ;;
  record)
    echo "Starting Visual V3 RECORD: $RUN_ID"
    "$PYTHON_BIN" -m mujoco_soccer.run_demo \
      --mode visual-v3-demo \
      --model "$MODEL" \
      --camera broadcast_wide \
      --playback-speed 1.15 \
      --run-id "$RUN_ID"
    ;;
esac

echo "Visual V3 results: results/mujoco_four_robot_demo/$RUN_ID"
