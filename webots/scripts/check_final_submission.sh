#!/usr/bin/env bash
set -uo pipefail

PROJECT="/home/plon/Workspace/booster_soccer_project"
RUNS_DIR="$PROJECT/results/final_submission"
LATEST="$(find "$RUNS_DIR" -maxdepth 2 \( -name real_summary.json -o -name summary.json \) -printf '%T@ %h\n' 2>/dev/null | sort -nr | awk 'NR==1 {print $2}')"

echo "=== Final Submission Check ==="
echo "[processes]"
ps -ef | grep -E 'webots-bin|webots-controller|mck|rpc_service_node' | grep -v grep || true
echo "[port 1234]"
ss -ltnp 2>/dev/null | grep ':1234' || true

if [ -z "$LATEST" ]; then
    echo "No final_submission result directory found."
    exit 1
fi

echo "[latest]"
echo "path=$LATEST"
if [ -f "$LATEST/metadata.json" ]; then
    python3 - "$LATEST/metadata.json" <<'PY'
import json, sys
d=json.load(open(sys.argv[1]))
print(f"current_run_id={d.get('run_id')}")
print(f"current_mode={d.get('mode')}")
PY
fi

echo "[status]"
python3 - "$LATEST" <<'PY'
import json, pathlib, sys
run=pathlib.Path(sys.argv[1])
summary_file = run / "real_summary.json"
if not summary_file.exists():
    summary_file = run / "summary.json"
summary=json.load(open(summary_file))
print(f"mck_status={summary.get('mck_ready', 'not_used')}")
print(f"rpc_status={summary.get('processes_alive', {}).get('rpc', 'not_used')}")
print(f"webots_status={summary.get('processes_alive', {}).get('webots', 'stopped_or_not_recorded')}")
print(f"latest_log_path={run}")
print(f"latest_ball_displacement={summary.get('ball_displacement')}")
print(f"latest_strategy={summary.get('selected_strategy') or summary.get('scenario_b_strategy')}")
print(f"dribble_success={summary.get('dribble_success')}")
print(f"mock_all_scenarios_passed={summary.get('all_scenarios_passed')}")
PY

echo "[pytest]"
pytest -q
