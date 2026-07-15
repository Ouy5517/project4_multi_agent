#!/usr/bin/env bash
set -uo pipefail

PROJECT="/home/plon/Workspace/booster_soccer_project"
RUNS_DIR="$PROJECT/results/native_physical_kick"
LATEST="$(find "$RUNS_DIR" -maxdepth 2 -name summary.json -printf '%T@ %h\n' 2>/dev/null | sort -nr | awk 'NR==1 {print $2}')"

echo "=== Native Physical Kick Check ==="
echo "[processes]"
ps -ef | grep -E 'webots-bin|webots-controller|mck|rpc_service_node' | grep -v grep || true
echo "[port 1234]"
ss -ltnp 2>/dev/null | grep ':1234' || true

if [ -z "$LATEST" ]; then
    echo "No native summary found."
    exit 1
fi

echo "latest=$LATEST"
python3 - "$LATEST" <<'PY'
import json, pathlib, sys
run=pathlib.Path(sys.argv[1])
d=json.load(open(run/'summary.json'))
print(f"run_id={d.get('run_id')}")
print(f"assisted_mode={d.get('assisted_mode')}")
print(f"strategy={d.get('strategy')}")
print(f"ball_horizontal_displacement={d.get('ball_horizontal_displacement')}")
print(f"kick_success={d.get('kick_success')}")
print(f"dribble_touch_count={d.get('dribble_touch_count')}")
print(f"dribble_total_displacement={d.get('dribble_total_displacement')}")
print(f"dribble_success={d.get('dribble_success')}")
print(f"robot_fallen={d.get('robot_fallen')}")
print(f"joint_limit_violation={d.get('joint_limit_violation')}")
print(f"failure_reason={d.get('failure_reason')}")
inv=run/'device_inventory.json'
if inv.exists():
    items=json.load(open(inv))
    print(f"motor_count={sum(1 for x in items if x.get('is_motor'))}")
PY

echo "[pytest]"
pytest -q
