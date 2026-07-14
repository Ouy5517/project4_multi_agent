#!/usr/bin/env bash
# Start four mck instances in parallel for T1_2v2_today.wbt
set +u

ROOT="/home/plon/Workspace/booster_t1_webots"
RUNNER="$ROOT/runner_extracted"
INSTANCES="$ROOT/runner_instances"
WEBOTS="/home/plon/Workspace/webots_updated/webots"
WEBOTS_HOME="/home/plon/Workspace/webots_updated"
WORLD="$ROOT/simulation/webots_simulation/worlds/T1_2v2_today.wbt"
LOG_DIR="$ROOT/logs"
TS=$(date +%Y%m%d_%H%M%S)
READY_DIR="$ROOT/runtime"
mkdir -p "$LOG_DIR" "$READY_DIR"
rm -f "$READY_DIR"/*.ready

# Robot mapping: instance -> robot name
declare -A ROBOTS
ROBOTS[blue1]="T1_BLUE_1"
ROBOTS[blue2]="T1_BLUE_2"
ROBOTS[red1]="T1_RED_1"
ROBOTS[red2]="T1_RED_2"

echo "=== Four MCK Startup ==="
echo "TS: $TS"

# Check prerequisites
for f in "$WEBOTS" "$WORLD" "$RUNNER/mck"; do
    [ -s "$f" ] || { echo "FATAL: missing $f"; exit 1; }
done

# Clean previous
"$ROOT/scripts/stop_all.sh" 2>/dev/null || true
pkill -9 -x webots-bin 2>/dev/null; pkill -9 -x mck 2>/dev/null; pkill -f '[r]pc_service_node' 2>/dev/null
sleep 2

# Step 1: Start Webots
echo "[1] Starting Webots..."
"$WEBOTS" "$WORLD" > "$LOG_DIR/wb4_$TS.log" 2>&1 &
W_PID=$!
for i in $(seq 1 30); do
    sleep 2
    if ss -ltn 2>/dev/null | grep -q ':1234'; then
        echo "  Webots up ($((i*2))s) PID=$W_PID"
        break
    fi
    if ! kill -0 $W_PID 2>/dev/null; then
        echo "FATAL: Webots died"
        exit 1
    fi
done

# Step 2: Setup environment
export LD_LIBRARY_PATH="$RUNNER/lib:$RUNNER/lib-usr-local:$RUNNER/lib-x86_64-linux-gnu:$WEBOTS_HOME/lib/controller:$WEBOTS_HOME/lib/webots"
export FASTRTPS_DEFAULT_PROFILES_FILE="$RUNNER/fastdds_profile.xml"
export BOOSTER_ROS2="$RUNNER/booster_ros2"
source /opt/ros/humble/setup.bash 2>/dev/null
source "$BOOSTER_ROS2/install/setup.bash" 2>/dev/null

# Step 3: Start RPC (single rpc_service_node handles all 4 via DDS)
echo "[2] Starting rpc_service_node..."
ros2 run booster_rpc_service rpc_service_node > "$LOG_DIR/rpc4_$TS.log" 2>&1 &
R_PID=$!
sleep 3
echo "  RPC PID=$R_PID"

# Step 4: Launch 4 mck in parallel
echo "[3] Launching 4 mck instances..."
MCK_PIDS=()

for inst in blue1 blue2 red1 red2; do
    ROBOT_NAME="${ROBOTS[$inst]}"
    INST_DIR="$INSTANCES/$inst"

    # Launch in background
    (
        cd "$INST_DIR"
        export TMPDIR="$INST_DIR/tmp"
        export FASTRTPS_DEFAULT_PROFILES_FILE="$RUNNER/fastdds_profile.xml"

        "$WEBOTS_HOME/webots-controller" \
            --protocol=tcp \
            --ip-address=127.0.0.1 \
            --port=1234 \
            --robot-name="$ROBOT_NAME" \
            ./mck configs/config_soccer.lua \
            > "$INST_DIR/logs/mck_$TS.log" 2>&1
    ) &
    PID=$!
    MCK_PIDS+=($PID)
    echo "  $inst ($ROBOT_NAME): PID=$PID"
    sleep 0.5  # Stagger to avoid port contention
done

echo ""
echo "All 4 mck launched. PIDs: ${MCK_PIDS[*]}"

# Step 5: Wait for all ESTAB (up to 600s)
echo "[4] Waiting for all 4 ESTAB (max 600s)..."
START=$(date +%s)
ALL_READY=0

for round in $(seq 1 40); do
    sleep 15
    ELAPSED=$(($(date +%s) - START))

    # Count ESTAB connections
    ESTAB_COUNT=$(ss -tn 2>/dev/null | grep ':1234' | grep -c 'ESTAB')

    # Check mck processes
    ALIVE=0; DEAD=0
    for P in "${MCK_PIDS[@]}"; do
        kill -0 $P 2>/dev/null && ALIVE=$((ALIVE+1)) || DEAD=$((DEAD+1))
    done

    echo "  ${ELAPSED}s: ESTAB=$ESTAB_COUNT mck=$ALIVE/$((ALIVE+DEAD))"

    if [ $DEAD -gt 0 ]; then
        echo "  WARNING: $DEAD mck died"
        for i in "${!MCK_PIDS[@]}"; do
            if ! kill -0 ${MCK_PIDS[$i]} 2>/dev/null; then
                echo "  DEAD: $inst (${ROBOTS[$inst]})"
            fi
        done
        break
    fi

    # Check if all are ready (ESTAB >= 4 means at least webots + 4 controllers)
    if [ "$ESTAB_COUNT" -ge 5 ] && [ "$ALIVE" -eq 4 ]; then
        # Verify modules loaded
        READY=0
        for inst in blue1 blue2 red1 red2; do
            MOD=$(grep -c "Loading '" "$INSTANCES/$inst/logs/mck_$TS.log" 2>/dev/null || echo 0)
            SM=$(grep -c "State machine initialized" "$INSTANCES/$inst/logs/mck_$TS.log" 2>/dev/null || echo 0)
            if [ "$MOD" -ge 15 ] && [ "$SM" -ge 1 ]; then
                READY=$((READY+1))
                touch "$READY_DIR/${ROBOTS[$inst]}.ready"
            fi
        done
        echo "  Modules+SM ready: $READY/4"

        if [ "$READY" -eq 4 ]; then
            echo "  ALL 4 READY after ${ELAPSED}s!"
            ALL_READY=1
            break
        fi
    fi

    if [ "$ELAPSED" -ge 600 ]; then
        echo "  TIMEOUT at 600s"
        break
    fi
done

echo ""
echo "=== Final Status ==="
for inst in blue1 blue2 red1 red2; do
    ROBOT_NAME="${ROBOTS[$inst]}"
    READY_FILE="$READY_DIR/${ROBOT_NAME}.ready"
    STATUS="NOT_READY"
    [ -f "$READY_FILE" ] && STATUS="READY"
    echo "  $inst ($ROBOT_NAME): $STATUS"
done
echo "webots-bin: $(pgrep -x webots-bin || echo DEAD)"
echo "mck count:  $(pgrep -x mck | wc -l)"
echo "ESTAB:      $(ss -tn 2>/dev/null | grep ':1234' | grep -c ESTAB)"

# Save info
echo "W_PID=$W_PID" > /tmp/four_mck_pids.txt
echo "R_PID=$R_PID" >> /tmp/four_mck_pids.txt
echo "MCK_PIDS=${MCK_PIDS[*]}" >> /tmp/four_mck_pids.txt
echo "TS=$TS" >> /tmp/four_mck_pids.txt
echo "ALL_READY=$ALL_READY" >> /tmp/four_mck_pids.txt

if [ "$ALL_READY" -eq 1 ]; then
    echo "SUCCESS: All 4 mck ready"
else
    echo "PARTIAL: Not all ready yet"
fi