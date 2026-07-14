#!/usr/bin/env bash
set -uo pipefail

PROJECT="/home/plon/Workspace/booster_soccer_project"
RUNS_DIR="$PROJECT/results/final_submission"

find_latest_with_pids() {
    if [ "${1:-}" ]; then
        if [ -f "$RUNS_DIR/$1/pids.env" ]; then
            printf '%s\n' "$RUNS_DIR/$1"
        elif [ -f "$RUNS_DIR/mock_$1/pids.env" ]; then
            printf '%s\n' "$RUNS_DIR/mock_$1"
        fi
        return
    fi
    find "$RUNS_DIR" -maxdepth 2 -name pids.env -printf '%T@ %h\n' 2>/dev/null | sort -nr | awk 'NR==1 {print $2}'
}

TARGET_DIR="$(find_latest_with_pids "${1:-}")"
if [ -z "$TARGET_DIR" ]; then
    echo "No final submission run with pids.env found."
    exit 0
fi

PID_FILE="$TARGET_DIR/pids.env"
echo "=== Stopping run: $(basename "$TARGET_DIR") ==="

# shellcheck source=/dev/null
. "$PID_FILE"

WEBOTS_PID="${WEBOTS_PID:-${W_PID:-0}}"
WEBOTS_BIN_PID="${WEBOTS_BIN_PID:-0}"
RPC_PID="${RPC_PID:-${R_PID:-0}}"
RPC_CHILD_PID="${RPC_CHILD_PID:-${R_CHILD_PID:-0}}"
CONTROLLER_PID="${CONTROLLER_PID:-0}"
MCK_PID="${MCK_PID:-${M_PID:-0}}"

echo "Tracked PIDs: webots=$WEBOTS_PID webots_bin=$WEBOTS_BIN_PID rpc=$RPC_PID rpc_child=$RPC_CHILD_PID controller=$CONTROLLER_PID mck=$MCK_PID"

for pid in "$MCK_PID" "$CONTROLLER_PID" "$RPC_CHILD_PID" "$RPC_PID" "$WEBOTS_BIN_PID" "$WEBOTS_PID"; do
    if [ "$pid" -gt 1 ] 2>/dev/null && kill -0 "$pid" 2>/dev/null; then
        echo "TERM PID $pid"
        kill -TERM "$pid" 2>/dev/null || true
    fi
done

waited=0
while [ "$waited" -lt 5 ]; do
    any=0
    for pid in "$MCK_PID" "$CONTROLLER_PID" "$RPC_CHILD_PID" "$RPC_PID" "$WEBOTS_BIN_PID" "$WEBOTS_PID"; do
        [ "$pid" -gt 1 ] 2>/dev/null && kill -0 "$pid" 2>/dev/null && any=1
    done
    [ "$any" -eq 0 ] && break
    sleep 1
    waited=$((waited + 1))
done

survivors=0
for pid in "$MCK_PID" "$CONTROLLER_PID" "$RPC_CHILD_PID" "$RPC_PID" "$WEBOTS_BIN_PID" "$WEBOTS_PID"; do
    if [ "$pid" -gt 1 ] 2>/dev/null && kill -0 "$pid" 2>/dev/null; then
        echo "KILL PID $pid"
        kill -KILL "$pid" 2>/dev/null || true
        survivors=$((survivors + 1))
    fi
done

rm -f "$PROJECT/runtime"/*.pid "$PROJECT/runtime"/*.ready 2>/dev/null || true

python3 - "$TARGET_DIR" "$survivors" <<'PY' 2>/dev/null || true
import json, sys
from datetime import datetime
path = f"{sys.argv[1]}/metadata.json"
try:
    with open(path, "r", encoding="utf-8") as f:
        data = json.load(f)
except Exception:
    data = {}
data["status"] = "stopped"
data["survivors_after_kill"] = int(sys.argv[2])
data["stopped_wall_time"] = datetime.now().isoformat()
with open(path, "w", encoding="utf-8") as f:
    json.dump(data, f, indent=2)
    f.write("\n")
PY

echo "=== STOP DONE ==="
echo "survivors_after_kill=$survivors"
