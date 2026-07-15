#!/usr/bin/env bash
set -euo pipefail

MODE="match"
SEED=""
DURATION=""
TARGET_FPS=""
VIDEO_FPS=""
FRONTEND="auto"

while [[ $# -gt 0 ]]; do
  case "$1" in
    --match) MODE="match"; shift ;;
    --showcase) MODE="showcase"; shift ;;
    --record) MODE="record"; shift ;;
    --replay) MODE="replay"; shift ;;
    --acceptance) MODE="acceptance"; shift ;;
    --benchmark) MODE="benchmark"; shift ;;
    --seed) SEED="$2"; shift 2 ;;
    --duration) DURATION="$2"; shift 2 ;;
    --target-fps) TARGET_FPS="$2"; shift 2 ;;
    --video-fps) VIDEO_FPS="$2"; shift 2 ;;
    --frontend) FRONTEND="$2"; shift 2 ;;
    *)
      echo "Unknown argument: $1" >&2
      exit 2
      ;;
esac
done

case "$FRONTEND" in
  auto|native|opencv) ;;
  *)
    echo "Unknown frontend: $FRONTEND" >&2
    exit 2
    ;;
esac

cd "$(dirname "$0")/.."
PYTHON_BIN="${PYTHON_BIN:-./coop_env/bin/python}"
if [[ ! -x "$PYTHON_BIN" ]]; then
  PYTHON_BIN="${PYTHON_BIN_FALLBACK:-/usr/bin/python3}"
fi
CONFIG="mujoco_soccer/config/final_release.yaml"

read_config() {
  "$PYTHON_BIN" - "$CONFIG" "$1" <<'PY'
import sys
from pathlib import Path
path, dotted = Path(sys.argv[1]), sys.argv[2]
try:
    import yaml
    data = yaml.safe_load(path.read_text())
    value = data
    for part in dotted.split("."):
        value = value[part]
    print(value)
except Exception:
    defaults = {
        "simulation.default_seed": 42,
        "simulation.default_duration_seconds": 60,
        "viewer.target_fps": 60,
        "video.output_fps": 60,
    }
    print(defaults[dotted])
PY
}

SEED="${SEED:-$(read_config simulation.default_seed)}"
TARGET_FPS="${TARGET_FPS:-$(read_config viewer.target_fps)}"
VIDEO_FPS="${VIDEO_FPS:-$(read_config video.output_fps)}"

DURATION_ARGS=()
if [[ -n "$DURATION" ]]; then
  DURATION_ARGS=(--duration "$DURATION")
fi

echo "[final-demo] mode=$MODE"
echo "[final-demo] frontend=$FRONTEND"
echo "[final-demo] python=$PYTHON_BIN"
"$PYTHON_BIN" - <<'PY'
import mujoco
print("[final-demo] mujoco=" + mujoco.__version__)
PY
if [[ "${DISPLAY:-}" == "" ]]; then
  echo "[final-demo] DISPLAY is not set; --match/--benchmark may not open a viewer." >&2
else
  echo "[final-demo] DISPLAY=$DISPLAY"
fi
if command -v glxinfo >/dev/null 2>&1; then
  glxinfo -B 2>/dev/null | awk '/OpenGL renderer string|direct rendering/ {print "[final-demo] " $0}' || true
fi
"$PYTHON_BIN" -m mujoco_soccer.tools_generate_proxy_model >/dev/null
test -f mujoco_soccer/models/t1_2v2_soccer_visual_v3.xml
echo "[final-demo] Webots/mck/RPC are not started by this script."

case "$MODE" in
  match)
    if [[ "$FRONTEND" == "opencv" ]]; then
      echo "[final-demo] OpenCV realtime viewer is unavailable unless cv2 is installed; using native MuJoCo viewer." >&2
    fi
    ./scripts/start_mujoco_concurrent_match_smooth.sh --view --seed "$SEED" --target-fps "$TARGET_FPS" "${DURATION_ARGS[@]}"
    ;;
  showcase)
    ./scripts/start_mujoco_visual_soccer_demo_v3.sh --view
    ;;
  record)
    ./scripts/start_mujoco_concurrent_match_smooth.sh --record --seed "$SEED" --video-fps "$VIDEO_FPS" "${DURATION_ARGS[@]}"
    ;;
  replay)
    ./scripts/start_mujoco_concurrent_match_smooth.sh --replay
    ;;
  acceptance)
    ./scripts/run_final_acceptance.sh
    ;;
  benchmark)
    if [[ -z "$DURATION" ]]; then
      DURATION_ARGS=(--duration 15)
    fi
    ./scripts/start_mujoco_concurrent_match_smooth.sh --benchmark --seed "$SEED" --target-fps "$TARGET_FPS" "${DURATION_ARGS[@]}"
    ;;
esac
