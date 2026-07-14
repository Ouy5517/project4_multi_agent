#!/usr/bin/env bash
set -uo pipefail

PROJECT="/home/plon/Workspace/booster_soccer_project"
WEBOTS_HOME="/home/plon/Workspace/webots_updated"
WORLD="/home/plon/Workspace/booster_t1_webots/simulation/webots_simulation/worlds/T1_2v2_assisted_physical_soccer.wbt"
MODE="${1:-full}"
TIMEOUT_SECONDS="${FOUR_ROBOT_DEMO_TIMEOUT_SECONDS:-360}"

case "$MODE" in
  full|single-contact|motion-check) ;;
  *) echo "Usage: $0 [full|single-contact|motion-check]"; exit 2 ;;
esac

RUN_ID="$(date +%Y%m%d_%H%M%S)_$(od -An -N3 -tx1 /dev/urandom | tr -d ' \n')"
RUN_DIR="$PROJECT/results/four_robot_physical_demo/$RUN_ID"
WEBOTS_PID=0
WEBOTS_BIN_PID=0

mkdir -p "$PROJECT/results/four_robot_physical_demo" "$PROJECT/outputs/screenshots"
mkdir "$RUN_DIR" || { echo "FATAL: run dir exists: $RUN_DIR"; exit 1; }
exec > >(tee "$RUN_DIR/console.log") 2>&1

write_pids() {
  cat > "$RUN_DIR/pids.env" <<EOF
RUN_ID=$RUN_ID
MODE=$MODE
WEBOTS_PID=$WEBOTS_PID
WEBOTS_BIN_PID=$WEBOTS_BIN_PID
RUN_DIR=$RUN_DIR
WORLD=$WORLD
EOF
}

cleanup() {
  rc=$?
  echo "=== four_robot cleanup rc=$rc ==="
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

cd "$PROJECT"

if ps -ef | grep -E 'webots-bin|webots-controller|mck|rpc_service_node' | grep -v grep; then
  echo "FATAL: Webots/mck/RPC process already running; stop it first."
  exit 1
fi
if ss -ltnp 2>/dev/null | grep ':1234'; then
  echo "FATAL: port 1234 occupied."
  exit 1
fi
if ps -ef | grep -E 'mck|rpc_service_node' | grep -v grep; then
  echo "FATAL: mck/RPC must not be running for assisted physical demo."
  exit 1
fi

export RUN_ID
export FOUR_ROBOT_DEMO_MODE="$MODE"
export FOUR_ROBOT_DEMO_RUN_DIR="$RUN_DIR"
export FINAL_SUBMISSION_RUN_DIR="$RUN_DIR"
export MATCH_STATE_RUN_ID="$RUN_ID"
export MATCH_STATE_FILE="$RUN_DIR/match_state.jsonl"
export WEBOTS_WORLD="$WORLD"
export PYTHONPATH="$PROJECT/coop_env/lib/python3.10/site-packages:$PROJECT:${PYTHONPATH:-}"

cat > "$RUN_DIR/metadata.json" <<EOF
{
  "run_id": "$RUN_ID",
  "mode": "ASSISTED_2V2_PHYSICAL",
  "script_mode": "$MODE",
  "world": "$WORLD",
  "mck_used": false,
  "rpc_used": false,
  "supervisor_moves_ball": false
}
EOF

echo "=== ASSISTED 2v2 PHYSICAL SOCCER DEMO ==="
echo "RUN_ID=$RUN_ID"
echo "RUN_DIR=$RUN_DIR"
echo "MODE=$MODE"
echo "WORLD=$WORLD"
echo "WORLD_SHA256=$(sha256sum "$WORLD" | awk '{print $1}')"
echo "ACTION REQUIRED:"
echo "开始录制Webots窗口。"
echo "四机器人将在5秒后开始移动。"

"$WEBOTS_HOME/webots" "$WORLD" > "$RUN_DIR/webots.log" 2>&1 &
WEBOTS_PID=$!
sleep 2
WEBOTS_BIN_PID="$(pgrep -P "$WEBOTS_PID" -f webots-bin 2>/dev/null | head -1 || true)"
WEBOTS_BIN_PID="${WEBOTS_BIN_PID:-0}"
write_pids

deadline=$((SECONDS + TIMEOUT_SECONDS))
while [ "$SECONDS" -lt "$deadline" ]; do
  if [ -f "$RUN_DIR/summary.json" ]; then
    echo "Summary: $RUN_DIR/summary.json"
    python3 - "$RUN_DIR/summary.json" "$MODE" <<'PY'
import json, sys
d=json.load(open(sys.argv[1]))
mode=sys.argv[2]
print(json.dumps(d, indent=2, ensure_ascii=False))
if mode != "full" and d.get("mode") == "ASSISTED_2V2_PHYSICAL":
    raise SystemExit(0)
raise SystemExit(0 if d.get("demo_success") else 1)
PY
    rc=$?
    echo "DEMO FINISHED"
    echo "请保存录屏和截图。"
    sleep 2
    exit "$rc"
  fi
  if ! kill -0 "$WEBOTS_PID" 2>/dev/null; then
    echo "FATAL: Webots exited before summary."
    tail -120 "$RUN_DIR/webots.log" || true
    exit 1
  fi
  sleep 1
done

echo "FATAL: four robot demo did not produce summary within ${TIMEOUT_SECONDS}s."
tail -160 "$RUN_DIR/webots.log" || true
exit 1
