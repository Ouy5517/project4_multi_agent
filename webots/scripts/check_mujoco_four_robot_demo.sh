#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
if [[ $# -gt 0 ]]; then
  LATEST="$1"
else
  LATEST="$(find "$ROOT/results/mujoco_four_robot_demo" -mindepth 2 -maxdepth 2 -name summary.json -printf '%T@ %h\n' | sort -n | tail -n 1 | cut -d' ' -f2-)"
fi
python3 - "$LATEST/summary.json" <<'PY'
import json, sys
path = sys.argv[1]
data = json.load(open(path, encoding="utf-8"))
print("summary:", path)
print("demo_success:", data.get("demo_success"))
print("failure_reason:", data.get("failure_reason"))
print("total_contacts:", data.get("total_contacts"))
print("simulation_time:", data.get("simulation_time"))
PY
