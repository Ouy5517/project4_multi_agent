#!/usr/bin/env bash
# Phase 2: Run T1_release baseline test - step by step
set -uo pipefail

ROOT="/home/plon/Workspace/booster_t1_webots"
RUNNER="$ROOT/runner_extracted"
WEBOTS_SCRIPT="/home/plon/Workspace/webots_updated/webots"
WEBOTS_HOME="/home/plon/Workspace/webots_updated"
WORLD="$ROOT/simulation/webots_simulation/worlds/T1_release.wbt"
LOG_DIR="$ROOT/logs"
ROBOT_NAME="T1_release"
PROJECT="/home/plon/Workspace/booster_soccer_project"

# ------------------------------------------------------------------
# fresh run_id: YYYYMMDD_HHMMSS_<6 random hex>
# ------------------------------------------------------------------
TS=$(date +%Y%m%d_%H%M%S)
RAND_SUFFIX=$(od -An -N3 -tx1 /dev/urandom | tr -d ' ')
RUN_ID="${TS}_${RAND_SUFFIX}"
RUN_DIR="$PROJECT/results/startup_runs/$RUN_ID"

# ------------------------------------------------------------------
# PID tracking (only for this round)
# ------------------------------------------------------------------
WEBOTS_PID=""
RPC_PID=""
RPC_CHILD_PID=""
MCK_PID=""

# ------------------------------------------------------------------
# cleanup() – unified, only touches tracked PIDs
# ------------------------------------------------------------------
cleanup() {
    local rc=${?}
    echo ""
    echo "=== CLEANUP (exit=${rc}) ==="

    # 1) TERM to all tracked PIDs (reverse order: mck → rpc → webots)
    local pid
    for pid in $MCK_PID $RPC_CHILD_PID $RPC_PID $WEBOTS_PID; do
        if [ -n "$pid" ] && kill -0 "$pid" 2>/dev/null; then
            echo "  TERM → PID $pid"
            kill -TERM "$pid" 2>/dev/null || true
        fi
    done

    # 2) Wait up to 5 seconds
    local waited=0
    while [ $waited -lt 5 ]; do
        local any=0
        for pid in $MCK_PID $RPC_CHILD_PID $RPC_PID $WEBOTS_PID; do
            [ -n "$pid" ] && kill -0 "$pid" 2>/dev/null && any=1
        done
        [ $any -eq 0 ] && break
        sleep 1
        waited=$((waited + 1))
    done

    # 3) KILL any survivors
    for pid in $MCK_PID $RPC_CHILD_PID $RPC_PID $WEBOTS_PID; do
        if [ -n "$pid" ] && kill -0 "$pid" 2>/dev/null; then
            echo "  KILL → PID $pid"
            kill -KILL "$pid" 2>/dev/null || true
        fi
    done

    echo "=== CLEANUP DONE ==="

    # Write/update metadata even on failure
    if [ -n "${RUN_DIR:-}" ] && [ -d "$RUN_DIR" ]; then
        cat > "$RUN_DIR/metadata.json" << METADATA_EOF
{
  "run_id": "$RUN_ID",
  "ts": "$TS",
  "world_file": "$WORLD",
  "robot_name": "$ROBOT_NAME",
  "runner": "$RUNNER",
  "webots_home": "$WEBOTS_HOME",
  "webots_pid": "${WEBOTS_PID:-0}",
  "rpc_pid": "${RPC_PID:-0}",
  "rpc_child_pid": "${RPC_CHILD_PID:-0}",
  "mck_pid": "${MCK_PID:-0}",
  "exit_code": $rc,
  "cleanup_wall_time": "$(date -Iseconds)"
}
METADATA_EOF
    fi

    exit $rc
}

trap cleanup EXIT INT TERM HUP

# ------------------------------------------------------------------
# Create run directory – MUST be fresh
# ------------------------------------------------------------------
mkdir "$RUN_DIR" || {
    echo "FATAL: run directory already exists: $RUN_DIR"
    exit 1
}

# log dir may already exist
mkdir -p "$LOG_DIR"

echo "============================================================"
echo "Phase 2: T1_release Baseline Test"
echo "Run ID: $RUN_ID"
echo "Run Dir: $RUN_DIR"
echo "============================================================"

# Step 1: Start Webots
echo ""
echo "[Step 1] Starting Webots..."
"$WEBOTS_SCRIPT" "$WORLD" > "$LOG_DIR/webots_$TS.log" 2>&1 &
WEBOTS_PID=$!
echo "  Webots PID: $WEBOTS_PID"

# Step 2: Wait for port 1234
echo "[Step 2] Waiting for Webots port 1234..."
for i in $(seq 1 30); do
    sleep 2
    if ss -ltn 2>/dev/null | grep -q ':1234'; then
        echo "  Webots listening on 1234 ($((i*2))s)"
        break
    fi
    if ! kill -0 $WEBOTS_PID 2>/dev/null; then
        echo "  FAIL: Webots exited prematurely"
        tail -30 "$LOG_DIR/webots_$TS.log"
        exit 1
    fi
done

if ! ss -ltn 2>/dev/null | grep -q ':1234'; then
    echo "  FAIL: Webots port 1234 not up after 60s"
    exit 1
fi

# Step 3: Set up environment
export LD_LIBRARY_PATH="$RUNNER/lib:$RUNNER/lib-usr-local:$RUNNER/lib-x86_64-linux-gnu:$WEBOTS_HOME/lib/controller:$WEBOTS_HOME/lib/webots:${LD_LIBRARY_PATH:-}"
export FASTRTPS_DEFAULT_PROFILES_FILE="$RUNNER/fastdds_profile.xml"
export BOOSTER_ROS2="$RUNNER/booster_ros2"

# Source ROS2
set +u
source /opt/ros/humble/setup.bash
source "$BOOSTER_ROS2/install/setup.bash"
set -u

echo "[Step 3] Environment configured"
echo "  ros2: $(which ros2 2>/dev/null || echo 'NOT FOUND')"

# Step 4: Start RPC
echo "[Step 4] Starting rpc_service_node..."
ros2 run booster_rpc_service rpc_service_node > "$LOG_DIR/rpc_$TS.log" 2>&1 &
RPC_PID=$!
echo "  RPC PID: $RPC_PID"
sleep 2

# Capture child PID (the actual Python rpc_service_node)
RPC_CHILD_PID=$(pgrep -P "$RPC_PID" -f rpc_service_node 2>/dev/null | head -1 || true)
if [ -n "$RPC_CHILD_PID" ]; then
    echo "  RPC child PID: $RPC_CHILD_PID"
else
    echo "  (could not capture RPC child PID)"
fi

sleep 1

if ! kill -0 $RPC_PID 2>/dev/null; then
    echo "  FAIL: RPC died immediately"
    echo "  RPC log:"
    tail -20 "$LOG_DIR/rpc_$TS.log"
    exit 1
fi
echo "  RPC running OK"

# Step 5: Start mck
echo "[Step 5] Starting mck via webots-controller..."
cd "$RUNNER"
"$WEBOTS_HOME/webots-controller" \
    --protocol=tcp \
    --ip-address=127.0.0.1 \
    --port=1234 \
    --robot-name="$ROBOT_NAME" \
    ./mck configs/config.lua \
    > "$LOG_DIR/mck_$TS.log" 2>&1 &
MCK_PID=$!
echo "  mck PID: $MCK_PID"
sleep 3

if ! kill -0 $MCK_PID 2>/dev/null; then
    echo "  FAIL: mck died immediately"
    echo "  mck log:"
    tail -30 "$LOG_DIR/mck_$TS.log"
    exit 1
fi
echo "  mck running OK"

# Step 6: Wait for ESTAB connection
echo "[Step 6] Waiting for mck to connect to Webots (up to 480s)..."
START_WAIT=$(date +%s)
ESTAB=0
for i in $(seq 1 32); do
    sleep 15
    ELAPSED=$(($(date +%s) - START_WAIT))

    if ! kill -0 $MCK_PID 2>/dev/null; then
        echo "  FAIL: mck exited after ${ELAPSED}s"
        echo "  Last 80 lines of mck log:"
        tail -80 "$LOG_DIR/mck_$TS.log"
        exit 1
    fi

    if ss -tn 2>/dev/null | grep ':1234' | grep -q 'ESTAB'; then
        echo "  mck connected after ~${ELAPSED}s"
        ESTAB=1
        break
    fi

    if [ $((i % 4)) -eq 0 ]; then
        echo "  ... waiting (${ELAPSED}s) mck=$(kill -0 $MCK_PID 2>/dev/null && echo alive || echo dead) rpc=$(kill -0 $RPC_PID 2>/dev/null && echo alive || echo dead)"
    fi
done

if [ "$ESTAB" -eq 0 ]; then
    echo "  FAIL: mck did not connect within 480s"
    tail -80 "$LOG_DIR/mck_$TS.log"
    exit 1
fi

echo ""
echo "============================================================"
echo "ALL PROCESSES RUNNING"
echo "  run_id:     $RUN_ID"
echo "  webots-bin: PID=$WEBOTS_PID $(kill -0 $WEBOTS_PID 2>/dev/null && echo alive || echo dead)"
echo "  mck:        PID=$MCK_PID $(kill -0 $MCK_PID 2>/dev/null && echo alive || echo dead)"
echo "  rpc:        PID=$RPC_PID (child=$RPC_CHILD_PID) $(kill -0 $RPC_PID 2>/dev/null && echo alive || echo dead)"
echo "  1234 ESTAB: $(ss -tn 2>/dev/null | grep ':1234' | grep ESTAB | head -1 || echo MISSING)"
echo "============================================================"

# Write initial metadata (cleanup will overwrite on exit)
cat > "$RUN_DIR/metadata.json" << METADATA_EOF
{
  "run_id": "$RUN_ID",
  "ts": "$TS",
  "world_file": "$WORLD",
  "robot_name": "$ROBOT_NAME",
  "runner": "$RUNNER",
  "webots_home": "$WEBOTS_HOME",
  "webots_pid": $WEBOTS_PID,
  "rpc_pid": $RPC_PID,
  "rpc_child_pid": "${RPC_CHILD_PID:-0}",
  "mck_pid": $MCK_PID,
  "start_wall_time": "$(date -Iseconds)",
  "status": "running"
}
METADATA_EOF

# Also write a shell-sourceable PID file for stop scripts
cat > "$RUN_DIR/pids.env" << PID_EOF
WEBOTS_PID=$WEBOTS_PID
RPC_PID=$RPC_PID
RPC_CHILD_PID=${RPC_CHILD_PID:-0}
MCK_PID=$MCK_PID
RUN_ID=$RUN_ID
PID_EOF

echo ""
echo "Run information saved to:"
echo "  $RUN_DIR/metadata.json"
echo "  $RUN_DIR/pids.env"
echo ""
echo "Press Ctrl+C to stop all processes, or run:"
echo "  $PROJECT/scripts/stop_final_submission_demo.sh $RUN_ID"
