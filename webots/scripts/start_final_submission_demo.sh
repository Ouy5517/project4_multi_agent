#!/usr/bin/env bash
set -uo pipefail

MODE="${1:-real}"
PROJECT="/home/plon/Workspace/booster_soccer_project"
WEBOTS_HOME="/home/plon/Workspace/webots_updated"
WEBOTS_PROJECT="/home/plon/Workspace/booster_t1_webots"
WORLD_DIR="$WEBOTS_PROJECT/simulation/webots_simulation/worlds"
RUNNER_SRC="$WEBOTS_PROJECT/runner_extracted"
RUNNER_BASE="$WEBOTS_PROJECT/runner_instances"

case "$MODE" in
    real)
        WORLD="$WORLD_DIR/T1_submission_demo.wbt"
        DEMO_SCRIPT="$PROJECT/demos/final_real_dribble_demo.py"
        RUN_PREFIX=""
        ;;
    mock)
        WORLD="$WORLD_DIR/T1_submission_demo.wbt"
        DEMO_SCRIPT="$PROJECT/demos/final_mock_2v2_demo.py"
        RUN_PREFIX="mock_"
        ;;
    *)
        echo "Usage: $0 [real|mock]"
        exit 2
        ;;
esac

TS="$(date +%Y%m%d_%H%M%S)"
RAND_SUFFIX="$(od -An -N3 -tx1 /dev/urandom | tr -d ' \n')"
RUN_ID="${TS}_${RAND_SUFFIX}"
RUN_DIR="$PROJECT/results/final_submission/${RUN_PREFIX}${RUN_ID}"
RUNNER_INSTANCE=""

WEBOTS_PID=0
WEBOTS_BIN_PID=0
RPC_PID=0
RPC_CHILD_PID=0
CONTROLLER_PID=0
MCK_PID=0
MCK_READY=0
MCK_READY_SECONDS=0
DEMO_RC=99

mkdir -p "$PROJECT/results/final_submission" "$PROJECT/outputs/screenshots"
mkdir "$RUN_DIR" || { echo "FATAL: run dir exists: $RUN_DIR"; exit 1; }
exec > >(tee "$RUN_DIR/console.log") 2>&1

write_pids() {
    cat > "$RUN_DIR/pids.env" <<EOF
RUN_ID=$RUN_ID
MODE=$MODE
WEBOTS_PID=$WEBOTS_PID
WEBOTS_BIN_PID=$WEBOTS_BIN_PID
RPC_PID=$RPC_PID
RPC_CHILD_PID=$RPC_CHILD_PID
CONTROLLER_PID=$CONTROLLER_PID
MCK_PID=$MCK_PID
RUN_DIR=$RUN_DIR
RUNNER_INSTANCE=$RUNNER_INSTANCE
EOF
}

cleanup() {
    rc=$?
    echo "=== CLEANUP start rc=$rc ==="
    write_pids
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
    for pid in "$MCK_PID" "$CONTROLLER_PID" "$RPC_CHILD_PID" "$RPC_PID" "$WEBOTS_BIN_PID" "$WEBOTS_PID"; do
        if [ "$pid" -gt 1 ] 2>/dev/null && kill -0 "$pid" 2>/dev/null; then
            echo "KILL PID $pid"
            kill -KILL "$pid" 2>/dev/null || true
        fi
    done
    python3 - "$RUN_DIR" "$RUN_ID" "$MODE" "$WORLD" "$RUNNER_INSTANCE" "$rc" <<'PY' || true
import json, sys
from datetime import datetime
run_dir, run_id, mode, world, runner, rc = sys.argv[1:7]
data = {
    "run_id": run_id,
    "mode": mode,
    "world": world,
    "runner_instance": runner,
    "exit_code": int(rc),
    "cleanup_wall_time": datetime.now().isoformat(),
}
with open(f"{run_dir}/metadata.json", "w", encoding="utf-8") as f:
    json.dump(data, f, indent=2)
    f.write("\n")
PY
    echo "=== CLEANUP done ==="
    exit "$rc"
}
trap cleanup EXIT INT TERM HUP

echo "=== FINAL SUBMISSION DEMO ($MODE) ==="
echo "RUN_ID=$RUN_ID"
echo "RUN_DIR=$RUN_DIR"
echo "world=$(realpath "$WORLD")"

"$PROJECT/scripts/stop_final_submission_demo.sh" 2>/dev/null || true
find /dev/shm -maxdepth 1 -user "$USER" \( -name 'fast_*' -o -name 'fastrtps*' \) -print -delete 2>/dev/null || true

if ps -ef | grep -E 'webots-bin|webots-controller|mck|rpc_service_node' | grep -v grep; then
    echo "FATAL: tracked processes still running before cold start"
    exit 1
fi
if ss -ltnp 2>/dev/null | grep ':1234'; then
    echo "FATAL: port 1234 is occupied"
    exit 1
fi

export RUN_ID
export FINAL_SUBMISSION_RUN_DIR="$RUN_DIR"
export MATCH_STATE_RUN_ID="$RUN_ID"
export MATCH_STATE_FILE="$RUN_DIR/match_state.jsonl"
export WEBOTS_WORLD="$WORLD"
export DEMO_MODE="$MODE"

if [ "$MODE" = "real" ]; then
    RUNNER_INSTANCE="$RUNNER_BASE/final_$RUN_ID"
    mkdir -p "$RUNNER_BASE"
    mkdir "$RUNNER_INSTANCE"
    rsync -a --exclude 'record_*.lcm' --exclude 'core' --exclude 'core.*' "$RUNNER_SRC/" "$RUNNER_INSTANCE/"
    mkdir -p "$RUNNER_INSTANCE/home" "$RUNNER_INSTANCE/tmp/xdg" "$RUNNER_INSTANCE/logs" "$RUNNER_INSTANCE/records"
    chmod 700 "$RUNNER_INSTANCE/tmp/xdg"
    CONFIG="$RUNNER_INSTANCE/configs/config_final_no_record.lua"
    OPTIONS="$RUNNER_INSTANCE/configs/common_module_options_final_no_record.lua"
    export RUNNER_INSTANCE MCK_CONFIG="$CONFIG"
    export RECORDING_FLAGS_JSON='{"record_data":false,"record_data_":false,"record_traj_data_":false,"record_backends":[]}'
    echo "RUNNER_INSTANCE=$RUNNER_INSTANCE"
    echo "config=$(realpath "$CONFIG")"
    echo "options=$(realpath "$OPTIONS")"
    sha256sum "$CONFIG" "$OPTIONS"
    grep -nE 'record_data\s*=|record_data_\s*=|record_traj_data_\s*=|record_backends' "$OPTIONS" | sed -n '1,80p'
fi

write_pids

echo "[1] Starting Webots"
"$WEBOTS_HOME/webots" "$WORLD" > "$RUN_DIR/webots.log" 2>&1 &
WEBOTS_PID=$!
sleep 2
WEBOTS_BIN_PID="$(pgrep -P "$WEBOTS_PID" -f webots-bin 2>/dev/null | head -1 || true)"
WEBOTS_BIN_PID="${WEBOTS_BIN_PID:-0}"
write_pids

for _ in $(seq 1 40); do
    ss -ltn 2>/dev/null | grep -q ':1234' && break
    kill -0 "$WEBOTS_PID" 2>/dev/null || { echo "FATAL: Webots died"; tail -80 "$RUN_DIR/webots.log"; exit 1; }
    sleep 1
done
ss -ltn 2>/dev/null | grep -q ':1234' || { echo "FATAL: Webots port 1234 not ready"; exit 1; }
echo "Webots ready on port 1234"

export LD_LIBRARY_PATH="${RUNNER_INSTANCE:-$RUNNER_SRC}/lib:${RUNNER_INSTANCE:-$RUNNER_SRC}/lib-usr-local:${RUNNER_INSTANCE:-$RUNNER_SRC}/lib-x86_64-linux-gnu:$WEBOTS_HOME/lib/controller:$WEBOTS_HOME/lib/webots:${LD_LIBRARY_PATH:-}"
export FASTRTPS_DEFAULT_PROFILES_FILE="${RUNNER_INSTANCE:-$RUNNER_SRC}/fastdds_profile.xml"
export BOOSTER_ROS2="${RUNNER_INSTANCE:-$RUNNER_SRC}/booster_ros2"
export PROJECT_SITE_PACKAGES="$PROJECT/coop_env/lib/python3.10/site-packages"
set +u
source /opt/ros/humble/setup.bash 2>/dev/null || true
source "$BOOSTER_ROS2/install/setup.bash" 2>/dev/null || true
set -u
export PYTHONPATH="$PROJECT_SITE_PACKAGES:${PYTHONPATH:-}"

if [ "$MODE" = "real" ]; then
    echo "[2] Starting RPC"
    ros2 run booster_rpc_service rpc_service_node > "$RUN_DIR/rpc.log" 2>&1 &
    RPC_PID=$!
    sleep 2
    RPC_CHILD_PID="$(pgrep -P "$RPC_PID" -f rpc_service_node 2>/dev/null | head -1 || true)"
    RPC_CHILD_PID="${RPC_CHILD_PID:-0}"
    kill -0 "$RPC_PID" 2>/dev/null || { echo "FATAL: RPC died"; tail -80 "$RUN_DIR/rpc.log"; exit 1; }
    write_pids

    echo "[3] Starting single mck for T1_BLUE_1"
    (
        cd "$RUNNER_INSTANCE"
        export HOME="$RUNNER_INSTANCE/home"
        export TMPDIR="$RUNNER_INSTANCE/tmp"
        export XDG_RUNTIME_DIR="$RUNNER_INSTANCE/tmp/xdg"
        "$WEBOTS_HOME/webots-controller" --protocol=tcp --ip-address=127.0.0.1 --port=1234 --robot-name=T1_BLUE_1 ./mck configs/config_final_no_record.lua
    ) > "$RUN_DIR/mck.log" 2>&1 &
    MCK_PID=$!
    CONTROLLER_PID=$MCK_PID
    write_pids

    START_WAIT="$(date +%s)"
    last_status=0
    while true; do
        sleep 5
        elapsed=$(($(date +%s) - START_WAIT))
        if ! kill -0 "$MCK_PID" 2>/dev/null; then
            echo "mck initialization crashed after repeated recording reentrancy warnings."
            tail -300 "$RUN_DIR/mck.log" > "$RUN_DIR/mck_last_300.log" 2>/dev/null || true
            dmesg | tail -n 100 > "$RUN_DIR/dmesg_tail.txt" 2>&1 || true
            find "$RUNNER_INSTANCE" -type f \( -name core -o -name 'core.*' \) > "$RUN_DIR/core_files.txt" 2>/dev/null || true
            python3 - "$RUN_DIR" "$RUN_ID" "$WORLD" "$CONFIG" "$OPTIONS" "$RUNNER_INSTANCE" <<'PY'
import json, sys
run_dir, run_id, world, config, options, runner = sys.argv[1:7]
summary = {
  "run_id": run_id, "world": world, "config": config, "options": options,
  "runner_instance": runner, "recording_flags": {"record_data": False, "record_data_": False, "record_traj_data_": False, "record_backends": []},
  "mck_ready": False, "mck_ready_seconds": 0, "mck_segfault": True,
  "real_mck_robot": "T1_BLUE_1", "active_robot_count": 1, "participating_robot_count": 4,
  "dribble_success": False, "failure_reason": "mck initialization crashed after repeated recording reentrancy warnings."
}
with open(f"{run_dir}/real_summary.json", "w", encoding="utf-8") as f:
    json.dump(summary, f, indent=2)
    f.write("\n")
PY
            exit 1
        fi
        modules=$(grep -c "Loading '" "$RUN_DIR/mck.log" 2>/dev/null || true)
        rejected=$(grep -c "Recording rejected" "$RUN_DIR/mck.log" 2>/dev/null || true)
        estab=$(ss -tn 2>/dev/null | grep ':1234' | grep -c ESTAB || true)
        if grep -qiE 'State machine initialized|mck ready' "$RUN_DIR/mck.log" && [ "$modules" -ge 15 ] 2>/dev/null; then
            MCK_READY=1
            MCK_READY_SECONDS=$elapsed
            export MCK_READY=true MCK_READY_SECONDS
            echo "mck ready after ${elapsed}s modules=$modules rejected=$rejected"
            break
        fi
        if [ $((elapsed - last_status)) -ge 60 ]; then
            echo "status ${elapsed}s: mck=alive webots=$(kill -0 "$WEBOTS_PID" 2>/dev/null && echo alive || echo dead) rpc=$(kill -0 "$RPC_PID" 2>/dev/null && echo alive || echo dead) estab=$estab modules=$modules recording_rejected=$rejected"
            last_status=$elapsed
        fi
        [ "$elapsed" -lt 420 ] || { echo "FATAL: mck not ready after 420s"; exit 1; }
    done
else
    touch "$RUN_DIR/rpc.log" "$RUN_DIR/mck.log" "$RUN_DIR/controller.log"
fi

echo "[4] Running demo script"
/usr/bin/python3.10 "$DEMO_SCRIPT"
DEMO_RC=$?
echo "DEMO_RC=$DEMO_RC"

python3 - "$RUN_DIR" "$MODE" "$DEMO_RC" <<'PY'
import json, pathlib, sys
run_dir, mode, rc = pathlib.Path(sys.argv[1]), sys.argv[2], int(sys.argv[3])
status = {"mode": mode, "demo_rc": rc}
for name in ("webots", "rpc", "mck"):
    status[f"{name}_log_exists"] = (run_dir / f"{name}.log").exists()
with open(run_dir / "process_status.json", "w", encoding="utf-8") as f:
    json.dump(status, f, indent=2)
    f.write("\n")
with open(run_dir / "result.json", "w", encoding="utf-8") as f:
    json.dump({"success": rc == 0, "mode": mode}, f, indent=2)
    f.write("\n")
PY

exit "$DEMO_RC"
