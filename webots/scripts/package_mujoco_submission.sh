#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"
TS="$(date +%Y%m%d_%H%M%S)"
OUT="dist/booster_soccer_mujoco_four_robot_${TS}.tar.gz"
LATEST="${1:-}"
if [[ -z "$LATEST" ]]; then
  LATEST="$(find results/mujoco_four_robot_demo -mindepth 2 -maxdepth 2 -name summary.json -printf '%T@ %h\n' | sort -n | tail -n 1 | cut -d' ' -f2- || true)"
fi
tar --exclude='__pycache__' --exclude='.pytest_cache' --exclude='coop_env' --exclude='runner_instances' \
  -czf "$OUT" \
  mujoco_soccer strategy integration tests scripts docs README.md PROJECT_PROGRESS.md \
  ${LATEST:+$LATEST}
cat > dist/MUJOCO_SUBMISSION_MANIFEST.md <<EOF
# MuJoCo Submission Manifest

- package: $OUT
- latest_run: ${LATEST:-none}
- generated_at: $TS
EOF
echo "$OUT"
