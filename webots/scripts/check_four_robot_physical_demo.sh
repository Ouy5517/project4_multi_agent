#!/usr/bin/env bash
set -uo pipefail

PROJECT="/home/plon/Workspace/booster_soccer_project"
latest="$(find "$PROJECT/results/four_robot_physical_demo" -maxdepth 2 -name pids.env -type f 2>/dev/null | sort | tail -1)"
echo "=== Four Robot Physical Demo Status ==="
ps -ef | grep -E 'webots-bin|webots-controller|mck|rpc_service_node' | grep -v grep || true
if [ -z "$latest" ]; then
  echo "current_run_id="
  exit 0
fi
# shellcheck disable=SC1090
source "$latest"
echo "current_run_id=${RUN_ID:-}"
echo "run_dir=${RUN_DIR:-}"
summary="${RUN_DIR:-}/summary.json"
if [ -f "$summary" ]; then
  python3 - "$summary" <<'PY'
import json, sys
d=json.load(open(sys.argv[1]))
print("stage=FINISHED")
for key in ["blue1_path_length","blue2_path_length","red1_path_length","red2_path_length","blue1_contact_count","blue2_contact_count","red1_contact_count","red2_contact_count","ball_total_distance","demo_success"]:
    print(f"{key}={d.get(key)}")
print("last_strategy=see decisions.jsonl")
PY
else
  echo "summary=pending"
  [ -f "${RUN_DIR:-}/events.jsonl" ] && tail -5 "${RUN_DIR:-}/events.jsonl"
fi
