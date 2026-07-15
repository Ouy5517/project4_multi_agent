#!/usr/bin/env bash
set -uo pipefail

MODE="${1:-kick}"
PROJECT="/home/plon/Workspace/booster_soccer_project"
WEBOTS_HOME="/home/plon/Workspace/webots_updated"
WORLD="/home/plon/Workspace/booster_t1_webots/simulation/webots_simulation/worlds/T1_native_physical_kick.wbt"
RUN_ID="$(date +%Y%m%d_%H%M%S)_$(od -An -N3 -tx1 /dev/urandom | tr -d ' \n')"
RUN_DIR="$PROJECT/results/native_physical_kick/$RUN_ID"
WEBOTS_PID=0
WEBOTS_BIN_PID=0
NATIVE_KICK_TIMEOUT_SECONDS="${NATIVE_KICK_TIMEOUT_SECONDS:-600}"

case "$MODE" in
    kick|dribble|guided-dribble|shoot|assisted-kick|assisted-node-test|support-check|frame-check|geometry-check|collision-audit|settle-check) ;;
    *) echo "Usage: $0 [kick|dribble|guided-dribble|shoot|assisted-kick|assisted-node-test|support-check|frame-check|geometry-check|collision-audit|settle-check]"; exit 2 ;;
esac
if [ "$MODE" = "assisted-kick" ] || [ "$MODE" = "assisted-node-test" ] || [ "$MODE" = "support-check" ] || [ "$MODE" = "frame-check" ] || [ "$MODE" = "geometry-check" ] || [ "$MODE" = "collision-audit" ] || [ "$MODE" = "settle-check" ]; then
    WORLD="/home/plon/Workspace/booster_t1_webots/simulation/webots_simulation/worlds/T1_native_assisted_kick.wbt"
fi

mkdir -p "$PROJECT/results/native_physical_kick" "$PROJECT/outputs/screenshots"
mkdir "$RUN_DIR" || { echo "FATAL: run dir exists: $RUN_DIR"; exit 1; }
exec > >(tee "$RUN_DIR/console.log") 2>&1

write_pids() {
    cat > "$RUN_DIR/pids.env" <<EOF
RUN_ID=$RUN_ID
MODE=$MODE
WEBOTS_PID=$WEBOTS_PID
WEBOTS_BIN_PID=$WEBOTS_BIN_PID
RUN_DIR=$RUN_DIR
EOF
}

cleanup() {
    rc=$?
    echo "=== native cleanup rc=$rc ==="
    write_pids
    for pid in "$WEBOTS_BIN_PID" "$WEBOTS_PID"; do
        if [ "$pid" -gt 1 ] 2>/dev/null && kill -0 "$pid" 2>/dev/null; then
            echo "TERM PID $pid"
            kill -TERM "$pid" 2>/dev/null || true
        fi
    done
    waited=0
    while [ "$waited" -lt 5 ]; do
        any=0
        for pid in "$WEBOTS_BIN_PID" "$WEBOTS_PID"; do
            [ "$pid" -gt 1 ] 2>/dev/null && kill -0 "$pid" 2>/dev/null && any=1
        done
        [ "$any" -eq 0 ] && break
        sleep 1
        waited=$((waited + 1))
    done
    for pid in "$WEBOTS_BIN_PID" "$WEBOTS_PID"; do
        if [ "$pid" -gt 1 ] 2>/dev/null && kill -0 "$pid" 2>/dev/null; then
            echo "KILL PID $pid"
            kill -KILL "$pid" 2>/dev/null || true
        fi
    done
    exit "$rc"
}
trap cleanup EXIT INT TERM HUP

if ps -ef | grep -E 'webots-bin|webots-controller|mck|rpc_service_node' | grep -v grep; then
    echo "FATAL: Webots/mck/RPC process already running; stop it first."
    exit 1
fi
if ss -ltnp 2>/dev/null | grep ':1234'; then
    echo "FATAL: port 1234 occupied."
    exit 1
fi

export RUN_ID
export NATIVE_KICK_MODE="$MODE"
export NATIVE_KICK_RUN_DIR="$RUN_DIR"
export FINAL_SUBMISSION_RUN_DIR="$RUN_DIR"
export MATCH_STATE_RUN_ID="$RUN_ID"
export MATCH_STATE_FILE="$RUN_DIR/match_state.jsonl"
export WEBOTS_WORLD="$WORLD"
export PYTHONPATH="$PROJECT/coop_env/lib/python3.10/site-packages:$PROJECT:${PYTHONPATH:-}"

cat > "$RUN_DIR/metadata.json" <<EOF
{
  "run_id": "$RUN_ID",
  "mode": "$MODE",
  "world": "$WORLD",
  "assisted_mode": $( { [[ "$MODE" == assisted-* ]] || [ "$MODE" = "support-check" ] || [ "$MODE" = "frame-check" ] || [ "$MODE" = "geometry-check" ] || [ "$MODE" = "collision-audit" ] || [ "$MODE" = "settle-check" ]; } && echo true || echo false ),
  "mck_used": false,
  "rpc_used": false,
  "supervisor_moves_ball": false
}
EOF

echo "=== Native Physical Kick ==="
echo "RUN_ID=$RUN_ID"
echo "RUN_DIR=$RUN_DIR"
echo "MODE=$MODE"
echo "WORLD=$WORLD"
echo "WORLD_REALPATH=$(realpath "$WORLD")"
echo "WORLD_SHA256=$(sha256sum "$WORLD" | awk '{print $1}')"

"$WEBOTS_HOME/webots" "$WORLD" > "$RUN_DIR/webots.log" 2>&1 &
WEBOTS_PID=$!
sleep 2
WEBOTS_BIN_PID="$(pgrep -P "$WEBOTS_PID" -f webots-bin 2>/dev/null | head -1 || true)"
WEBOTS_BIN_PID="${WEBOTS_BIN_PID:-0}"
write_pids

deadline=$((SECONDS + NATIVE_KICK_TIMEOUT_SECONDS))
while [ "$SECONDS" -lt "$deadline" ]; do
    if [ -f "$RUN_DIR/summary.json" ]; then
        echo "Summary: $RUN_DIR/summary.json"
        python3 - "$RUN_DIR/summary.json" <<'PY'
import json, sys
d=json.load(open(sys.argv[1]))
print(json.dumps(d, indent=2, ensure_ascii=False))
PY
        if python3 - "$RUN_DIR/summary.json" <<'PY'
import json, sys
d=json.load(open(sys.argv[1]))
if d.get("mode") == "assisted-node-test":
    raise SystemExit(0 if d.get("node_resolution_success") else 1)
if d.get("mode") == "support-check":
    raise SystemExit(0 if d.get("support_check_success") else 1)
if d.get("mode") == "frame-check":
    raise SystemExit(0 if d.get("coordinate_frame_valid") else 1)
if d.get("mode") == "geometry-check":
    raise SystemExit(0 if d.get("geometry_check_success") else 1)
if d.get("mode") == "collision-audit":
    raise SystemExit(0 if d.get("collision_audit_success") else 1)
if d.get("mode") == "settle-check":
    raise SystemExit(0 if d.get("settle_check_success") else 1)
raise SystemExit(0 if d.get("kick_success") else 1)
PY
        then
            if [ "$MODE" = "assisted-node-test" ] || [ "$MODE" = "support-check" ] || [ "$MODE" = "frame-check" ] || [ "$MODE" = "geometry-check" ] || [ "$MODE" = "collision-audit" ] || [ "$MODE" = "settle-check" ]; then
                exit 0
            fi
            echo "ACTION REQUIRED:"
            echo "请使用 Win+Shift+S 截取T1脚部真实接触足球的Webots画面，保存为："
            echo "outputs/screenshots/native_physical_kick.png"
            sleep 20
            exit 0
        fi
        exit 1
    fi
    if ! kill -0 "$WEBOTS_PID" 2>/dev/null; then
        echo "FATAL: Webots exited before summary."
        tail -80 "$RUN_DIR/webots.log" || true
        exit 1
    fi
    sleep 1
done

echo "FATAL: native controller did not produce summary within ${NATIVE_KICK_TIMEOUT_SECONDS}s."
tail -120 "$RUN_DIR/webots.log" || true
exit 1
