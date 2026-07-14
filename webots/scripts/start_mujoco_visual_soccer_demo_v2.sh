#!/usr/bin/env bash
set -euo pipefail

MODE="normal"
RECORD_ARGS=()
for arg in "$@"; do
  case "$arg" in
    --slow) MODE="slow" ;;
    --normal) MODE="normal" ;;
    --fast) MODE="fast" ;;
    --no-record) RECORD_ARGS+=(--no-record) ;;
    *)
      echo "Unknown argument: $arg" >&2
      exit 2
      ;;
  esac
done

case "$MODE" in
  slow) PLAYBACK_SPEED="0.85" ;;
  normal) PLAYBACK_SPEED="1.15" ;;
  fast) PLAYBACK_SPEED="1.35" ;;
esac

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

MODEL="mujoco_soccer/models/t1_2v2_soccer_visual_v2.xml"
if [[ ! -f "$MODEL" ]]; then
  echo "Visual V2 model missing: $MODEL" >&2
  exit 1
fi

if [[ -n "${DISPLAY:-}" ]]; then
  export MUJOCO_GL="${MUJOCO_GL:-glfw}"
else
  export MUJOCO_GL="${MUJOCO_GL:-egl}"
  echo "DISPLAY is not set; running clean offscreen recorder with passive viewer fallback disabled by EGL."
fi

RUN_ID="visual_v2_$(date +%Y%m%d_%H%M%S)"
echo "Starting MuJoCo Visual V2 demo: $RUN_ID"
echo "Playback speed: ${PLAYBACK_SPEED}x"

"$PYTHON_BIN" -m mujoco_soccer.run_demo \
  --mode visual-v2-demo \
  --model "$MODEL" \
  --camera broadcast \
  --playback-speed "$PLAYBACK_SPEED" \
  --clean-viewer \
  --run-id "$RUN_ID" \
  "${RECORD_ARGS[@]}"

RESULT_DIR="results/mujoco_four_robot_demo/$RUN_ID"
echo "Visual V2 results: $RESULT_DIR"
