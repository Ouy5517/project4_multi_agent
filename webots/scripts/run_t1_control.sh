#!/usr/bin/env bash
set -euo pipefail
# Allow unbound in sourced ROS scripts
set +u

ROOT="/home/plon/Workspace/booster_t1_webots/runner_extracted"
PROJECT="/home/plon/Workspace/booster_soccer_project"
CLIENT="$PROJECT/tools/t1_ros2_control_client.py"

# ── Pre-flight checks ──
fail() { echo "[RUN_T1_CONTROL] FAIL: $*"; exit 1; }

pgrep -x webots-bin > /dev/null || fail "webots-bin not running. Start with scripts/start_all.sh first."
pgrep -x mck > /dev/null          || fail "mck not running. Start with scripts/start_all.sh first."
pgrep -f '[r]pc_service_node' > /dev/null || fail "rpc_service_node not running. Start with scripts/start_all.sh first."

if ! ss -tn 2>/dev/null | grep ':1234' | grep -q 'ESTAB'; then
    fail "Port 1234 not ESTABLISHED between mck and Webots."
fi

[ -s "$CLIENT" ] || fail "Client script not found: $CLIENT"

# ── Environment ──
export FASTRTPS_DEFAULT_PROFILES_FILE="$ROOT/fastdds_profile.xml"
export ROS_LOCALHOST_ONLY=0
export LD_LIBRARY_PATH="$ROOT/booster_ros2/install/booster_msgs/lib:$ROOT/booster_ros2/install/booster_interface/lib:$ROOT/lib:$ROOT/lib-usr-local:$ROOT/lib-x86_64-linux-gnu:/home/plon/Workspace/webots_updated/lib/controller:/opt/ros/humble/lib/x86_64-linux-gnu:/opt/ros/humble/lib"
export AMENT_PREFIX_PATH="$ROOT/booster_ros2/install/booster_rpc_service:$ROOT/booster_ros2/install/booster_msgs:$ROOT/booster_ros2/install/booster_interface:/opt/ros/humble"
export PYTHONPATH="$ROOT/booster_ros2/install/booster_msgs/local/lib/python3.10/dist-packages:$ROOT/booster_ros2/install/booster_interface/local/lib/python3.10/dist-packages:/opt/ros/humble/lib/python3.10/site-packages:/opt/ros/humble/local/lib/python3.10/dist-packages"

source /opt/ros/humble/setup.bash
source "$ROOT/booster_ros2/install/setup.bash"

# ── Run client ──
exec /usr/bin/python3.10 "$CLIENT" "$@"
