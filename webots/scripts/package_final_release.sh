#!/usr/bin/env bash
set -euo pipefail

cd "$(dirname "$0")/.."
TS="$(date +%Y%m%d_%H%M%S)"
OUT="dist/booster_soccer_final_release_${TS}.tar.gz"
mkdir -p dist

LATEST_VIDEO_RUN="$(find results/mujoco_concurrent_match -maxdepth 2 -name demo_60fps.mp4 -printf '%T@ %h\n' 2>/dev/null | sort -nr | head -n 1 | cut -d' ' -f2-)"
LATEST_ACCEPTANCE="$(find results/final_acceptance -maxdepth 1 -type d -name 'final_release_*' -printf '%T@ %p\n' 2>/dev/null | sort -nr | head -n 1 | cut -d' ' -f2- || true)"

INCLUDES=(
  README.md
  PROJECT_PROGRESS.md
  TEST_RESULTS.md
  mujoco_soccer
  scripts
  tests
  docs
)
if [[ -n "$LATEST_VIDEO_RUN" ]]; then
  INCLUDES+=("$LATEST_VIDEO_RUN")
fi
if [[ -n "$LATEST_ACCEPTANCE" ]]; then
  INCLUDES+=("$LATEST_ACCEPTANCE")
fi

tar \
  --exclude='.git' \
  --exclude='__pycache__' \
  --exclude='.pytest_cache' \
  --exclude='*.prof' \
  --exclude='backups' \
  --exclude='results/mujoco_concurrent_match/concurrent_20260713_153709' \
  -czf "$OUT" "${INCLUDES[@]}"

gzip -t "$OUT"
tar -tzf "$OUT" >/dev/null
sha256sum "$OUT" > "$OUT.sha256"
echo "$OUT"
echo "$OUT.sha256"
