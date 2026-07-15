#!/usr/bin/env bash
# Phase 2: Re-validate original T1_release.wbt baseline
# Phase 3: Test T1_green_single_baseline.wbt
set -euo pipefail

ROOT="/home/plon/Workspace/booster_t1_webots"
RUNNER="$ROOT/runner_extracted"
WEBOTS="/home/plon/Workspace/webots_updated/webots"
PROJECT="/home/plon/Workspace/booster_soccer_project"
SCRIPTS="$ROOT/scripts"

WORLD_NAME="${1:-T1_release.wbt}"
WORLD_DIR="$ROOT/simulation/webots_simulation/worlds"
WORLD="$WORLD_DIR/$WORLD_NAME"
LOG_DIR="$ROOT/logs"

WEBOTS_HOME="/home/plon/Workspace/webots_updated"

USAGE="Usage: $0 <world_file> [--baseline|--green]

  Worlds:
    T1_release.wbt            Original release baseline
    T1_green_single_baseline.wbt  Green-field single-robot baseline

  Modes:
    --baseline  Standard baseline test (Prepare + Walk + Stand 60s)
    --green     Green-field single-robot test
"

MODE="${2:---baseline}"

# Validate world exists
if [ ! -f "$WORLD" ]; then
    echo "ERROR: World file not found: $WORLD"
    echo "$USAGE"
    echo ""
    echo "Available worlds:"
    ls -1 "$WORLD_DIR"/*.wbt 2>/dev/null | while read w; do
        echo "  $(basename "$w")"
    done
    exit 1
fi

echo "============================================================"
echo "Phase 2/3: T1 Baseline Test"
echo "World: $WORLD_NAME"
echo "Mode:  $MODE"
echo "============================================================"

# ── Stop previous run ──
"$SCRIPTS/stop_all.sh" 2>/dev/null || true
sleep 2

# Pre-flight
check_file() { [ -s "$1" ] || { echo "MISSING: $1"; exit 1; }; }
check_file "$WEBOTS"
check_file "$WORLD"
check_file "$RUNNER/mck"

mkdir -p "$LOG_DIR"
TS=$(date +%Y%m%d_%H%M%S)

# ── Extract robot name from world ──
ROBOT_NAME=$(grep 'name "' "$WORLD" | grep -i 'robot\|T1' | grep -v 'match_state\|Supervisor\|solid\|field' | head -1 | sed 's/.*name "//' | sed 's/".*//')
if [ -z "$ROBOT_NAME" ]; then
    echo "WARNING: Could not detect robot name from world file, defaulting to T1_release"
    ROBOT_NAME="T1_release"
fi
echo "Detected robot name: $ROBOT_NAME"

# ── Start Webots ──
echo ""
echo "[Step 1] Starting Webots with $WORLD_NAME..."
"$WEBOTS" "$WORLD" > "$LOG_DIR/webots_baseline_$TS.log" 2>&1 &
WEBOTS_PID=$!

echo "[Step 2] Waiting for Webots port 1234..."
for i in $(seq 1 30); do
    sleep 2
    if ss -ltn 2>/dev/null | grep -q ':1234'; then
        echo "  Webots listening on 1234 ($((i*2))s)."
        break
    fi
    if ! kill -0 $WEBOTS_PID 2>/dev/null; then
        echo "FAIL: Webots exited. Check log: $LOG_DIR/webots_baseline_$TS.log"
        tail -40 "$LOG_DIR/webots_baseline_$TS.log"
        exit 1
    fi
done

# ── Start RPC ──
echo "[Step 3] Starting rpc_service_node..."

export WEBOTS_HOME="$WEBOTS_HOME"
export LD_LIBRARY_PATH="$RUNNER/lib:$RUNNER/lib-usr-local:$RUNNER/lib-x86_64-linux-gnu:$WEBOTS_HOME/lib/controller:${LD_LIBRARY_PATH:-}"
export FASTRTPS_DEFAULT_PROFILES_FILE="$RUNNER/fastdds_profile.xml"
export BOOSTER_ROS2="$RUNNER/booster_ros2"
export COLCON_CURRENT_PREFIX="$BOOSTER_ROS2/install"

source /opt/ros/humble/setup.bash 2>/dev/null || true
source "$BOOSTER_ROS2/install/setup.bash" 2>/dev/null || true

ros2 run booster_rpc_service rpc_service_node > "$LOG_DIR/rpc_baseline_$TS.log" 2>&1 &
RPC_PID=$!
echo "  RPC PID: $RPC_PID"
sleep 3

# ── Start mck ──
echo "[Step 4] Starting mck via webots-controller..."
$WEBOTS_HOME/webots-controller \
    --protocol=tcp \
    --ip-address=127.0.0.1 \
    --port=1234 \
    --robot-name="$ROBOT_NAME" \
    ./mck configs/config.lua \
    > "$LOG_DIR/mck_baseline_$TS.log" 2>&1 &
MCK_PID=$!
echo "  mck PID: $MCK_PID"

# ── Wait for mck ready ──
echo "[Step 5] Waiting for mck ready (up to 480s)..."
"$SCRIPTS/wait_mck_ready.sh" "$ROBOT_NAME" "$LOG_DIR/mck_baseline_$TS.log" 480

echo ""
echo "[READY] mck initialized."

# ── Run baseline test sequence ──
CLIENT="$PROJECT/tools/t1_ros2_control_client.py"
export FASTRTPS_DEFAULT_PROFILES_FILE="$RUNNER/fastdds_profile.xml"
export ROS_LOCALHOST_ONLY=0
export LD_LIBRARY_PATH="$RUNNER/booster_ros2/install/booster_msgs/lib:$RUNNER/booster_ros2/install/booster_interface/lib:$RUNNER/lib:$RUNNER/lib-usr-local:$RUNNER/lib-x86_64-linux-gnu:$WEBOTS_HOME/lib/controller:/opt/ros/humble/lib/x86_64-linux-gnu:/opt/ros/humble/lib:${LD_LIBRARY_PATH:-}"
export AMENT_PREFIX_PATH="$BOOSTER_ROS2/install/booster_rpc_service:$BOOSTER_ROS2/install/booster_msgs:$BOOSTER_ROS2/install/booster_interface:/opt/ros/humble"
export PYTHONPATH="$BOOSTER_ROS2/install/booster_msgs/local/lib/python3.10/dist-packages:$BOOSTER_ROS2/install/booster_interface/local/lib/python3.10/dist-packages:/opt/ros/humble/lib/python3.10/site-packages:/opt/ros/humble/local/lib/python3.10/dist-packages"

echo ""
echo "============================================================"
echo "[Test 1] GetMode (before Prepare)"
echo "============================================================"
/usr/bin/python3.10 "$CLIENT" mode

echo ""
echo "============================================================"
echo "[Test 2] Prepare"
echo "============================================================"
/usr/bin/python3.10 "$CLIENT" prepare

sleep 5

echo ""
echo "============================================================"
echo "[Test 3] GetMode (after Prepare)"
echo "============================================================"
/usr/bin/python3.10 "$CLIENT" mode

echo ""
echo "============================================================"
echo "[Test 4] Walking"
echo "============================================================"
/usr/bin/python3.10 "$CLIENT" stand  # stand = walking mode
sleep 3

echo ""
echo "============================================================"
echo "[Test 5] GetMode (in Walking)"
echo "============================================================"
/usr/bin/python3.10 "$CLIENT" mode

echo ""
echo "============================================================"
echo "[Test 6] Stand 60 seconds (no Move commands)"
echo "============================================================"
echo "Robot should now be standing. Observe GUI."
echo "Waiting 60 seconds..."
sleep 60

echo ""
echo "============================================================"
echo "[Test 7] Small Move (vx=0.02, 0.2s)"
echo "============================================================"
/usr/bin/python3.10 "$CLIENT" move
sleep 2

echo ""
echo "============================================================"
echo "[Test 8] Stop"
echo "============================================================"
/usr/bin/python3.10 "$CLIENT" stop

echo ""
echo "============================================================"
echo "[Test 9] Final Status"
echo "============================================================"
/usr/bin/python3.10 "$CLIENT" status
/usr/bin/python3.10 "$CLIENT" mode

echo ""
echo "============================================================"
echo "TEST SEQUENCE COMPLETE"
echo "World: $WORLD_NAME"
echo "Robot: $ROBOT_NAME"
echo "Logs:  $LOG_DIR/*_baseline_$TS.log"
echo "============================================================"
echo ""
echo "Key observations to record:"
echo "  1. mck cold-start wall time"
echo "  2. Simulation time advanced during cold start"
echo "  3. Robot initial position (check Webots GUI)"
echo "  4. Robot position at mck ready"
echo "  5. Pre-Prepare posture"
echo "  6. Post-Prepare posture"
echo "  7. Post-Walking posture"
echo "  8. Whether robot stood for 60s"
echo "  9. Whether small Move was visible"
echo "  10. Whether Stop was effective"

# Keep running for observation
echo ""
echo "Press Enter to stop all processes... (or Ctrl+C)"
read -r

# Cleanup
echo "Stopping..."
/usr/bin/python3.10 "$CLIENT" stop 2>/dev/null || true
"$SCRIPTS/stop_all.sh" 2>/dev/null || true
echo "Done."
