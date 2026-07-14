#!/usr/bin/env bash
set -euo pipefail

MODE="view"
SEED="42"
DURATION=""
TARGET_FPS="60"
VIDEO_FPS="60"
VERBOSE=()

while [[ $# -gt 0 ]]; do
  case "$1" in
    --view) MODE="view"; shift ;;
    --record) MODE="record"; shift ;;
    --benchmark) MODE="benchmark"; shift ;;
    --replay) MODE="replay"; shift ;;
    --verbose) VERBOSE=(--verbose); shift ;;
    --seed) SEED="${2:-42}"; shift 2 ;;
    --duration) DURATION="${2:-60}"; shift 2 ;;
    --target-fps) TARGET_FPS="${2:-60}"; shift 2 ;;
    --video-fps) VIDEO_FPS="${2:-60}"; shift 2 ;;
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
    "$PYTHON_BIN" -m mujoco_soccer.run_demo --mode concurrent-match --view --smooth-frontend --seed "$SEED" --target-fps "$TARGET_FPS" "${DURATION_ARGS[@]}" "${VERBOSE[@]}"
    ;;
  record)
    "$PYTHON_BIN" -m mujoco_soccer.run_demo --mode concurrent-match --smooth-frontend --no-render --seed "$SEED" --video-fps "$VIDEO_FPS" "${DURATION_ARGS[@]}" "${VERBOSE[@]}"
    RUN_DIR="$(find results/mujoco_concurrent_match -maxdepth 1 -type d -name 'concurrent_*' -printf '%T@ %p\n' | sort -nr | head -n 1 | cut -d' ' -f2-)"
    "$PYTHON_BIN" scripts/render_concurrent_log_video_60fps.py "$RUN_DIR" --fps "$VIDEO_FPS"
    ;;
  benchmark)
    if [[ -z "$DURATION" ]]; then
      DURATION_ARGS=(--duration 20)
    fi
    "$PYTHON_BIN" -m mujoco_soccer.run_demo --mode concurrent-match --view --smooth-frontend --benchmark --seed "$SEED" --target-fps "$TARGET_FPS" "${DURATION_ARGS[@]}" "${VERBOSE[@]}"
    ;;
  replay)
    VIDEO="$(find results/mujoco_concurrent_match -maxdepth 2 -name demo.mp4 -printf '%T@ %p\n' | sort -nr | head -n 1 | cut -d' ' -f2-)"
    if [[ -z "$VIDEO" ]]; then
      echo "No recorded demo.mp4 found under results/mujoco_concurrent_match" >&2
      exit 1
    fi
    if command -v ffplay >/dev/null 2>&1; then
      ffplay -autoexit "$VIDEO"
    else
      echo "$VIDEO"
    fi
    ;;
esac
