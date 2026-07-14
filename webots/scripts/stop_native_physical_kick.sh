#!/usr/bin/env bash
set -uo pipefail

PROJECT="/home/plon/Workspace/booster_soccer_project"
RUNS_DIR="$PROJECT/results/native_physical_kick"
TARGET="$(find "$RUNS_DIR" -maxdepth 2 -name pids.env -printf '%T@ %h\n' 2>/dev/null | sort -nr | awk 'NR==1 {print $2}')"

if [ -z "$TARGET" ]; then
    echo "No native physical kick run with pids.env found."
    exit 0
fi

# shellcheck source=/dev/null
. "$TARGET/pids.env"
WEBOTS_PID="${WEBOTS_PID:-0}"
WEBOTS_BIN_PID="${WEBOTS_BIN_PID:-0}"

echo "Stopping native run $(basename "$TARGET")"
for pid in "$WEBOTS_BIN_PID" "$WEBOTS_PID"; do
    if [ "$pid" -gt 1 ] 2>/dev/null && kill -0 "$pid" 2>/dev/null; then
        echo "TERM PID $pid"
        kill -TERM "$pid" 2>/dev/null || true
    fi
done
sleep 5
for pid in "$WEBOTS_BIN_PID" "$WEBOTS_PID"; do
    if [ "$pid" -gt 1 ] 2>/dev/null && kill -0 "$pid" 2>/dev/null; then
        echo "KILL PID $pid"
        kill -KILL "$pid" 2>/dev/null || true
    fi
done
