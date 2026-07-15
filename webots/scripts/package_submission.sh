#!/usr/bin/env bash
set -euo pipefail

PROJECT="/home/plon/Workspace/booster_soccer_project"
cd "$PROJECT"

TS="$(date +%Y%m%d_%H%M%S)"
mkdir -p dist
ARCHIVE="dist/booster_soccer_four_robot_final_${TS}.tar.gz"
MANIFEST="dist/FOUR_ROBOT_DEMO_MANIFEST.md"

tar \
  --exclude='./dist' \
  --exclude='./coop_env' \
  --exclude='./__pycache__' \
  --exclude='*/__pycache__' \
  --exclude='./.pytest_cache' \
  --exclude='*/.pytest_cache' \
  --exclude='*/node_modules' \
  --exclude='*/record_*.lcm' \
  --exclude='*/core' \
  --exclude='*/core.*' \
  --exclude='*/runner_instances' \
  --exclude='*/mck' \
  --exclude='*.lcm' \
  --exclude='*.log' \
  --exclude='./results/startup_runs' \
  -czf "$ARCHIVE" \
  README.md PROJECT_PROGRESS.md common strategy robot_adapter integration demos scripts docs tests scenarios config configs worlds controllers pytest.ini requirements.txt results/final_submission results/native_physical_kick results/four_robot_physical_demo outputs 2>/dev/null

cat > "$MANIFEST" <<EOF
# Booster Soccer Four Robot Final Demo Manifest

Archive: $ARCHIVE
Created: $(date -Iseconds)

## Start

- Four robot assisted physical demo: ./scripts/start_four_robot_physical_demo.sh
- Stop four robot demo: ./scripts/stop_four_robot_physical_demo.sh
- Check four robot demo: ./scripts/check_four_robot_physical_demo.sh
- Legacy real mode: ./scripts/start_final_submission_demo.sh real
- Legacy mock mode: ./scripts/start_final_submission_demo.sh mock
- Native kick: ./scripts/start_native_physical_kick.sh kick
- Native assisted kick: ./scripts/start_native_physical_kick.sh assisted-kick
- Native check: ./scripts/check_native_physical_kick.sh

## Included

- Source code: common, strategy, robot_adapter, integration, demos
- Scripts: scripts
- Tests: tests, scenarios, config, configs, pytest.ini
- Webots assets: worlds, controllers, including T1_2v2_assisted_physical_soccer.wbt
- Documentation: README.md, PROJECT_PROGRESS.md, docs
- Evidence: results/final_submission, results/native_physical_kick, results/four_robot_physical_demo
- Screenshots directory: outputs

## Excluded

- runner_instances
- mck binary copies
- core files
- record_*.lcm
- Python caches and pytest cache
- virtual environments
- temporary startup run logs
EOF

echo "$ARCHIVE"
echo "$MANIFEST"
