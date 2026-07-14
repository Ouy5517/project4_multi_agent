#!/usr/bin/env bash
set -uo pipefail

PROJECT="/home/plon/Workspace/booster_soccer_project"
latest="$(find "$PROJECT/results/four_robot_physical_demo" -maxdepth 2 -name pids.env -type f 2>/dev/null | sort | tail -1)"
if [ -z "$latest" ]; then
  echo "No four robot demo PID file found."
  exit 0
fi

RUN_ID=""
WEBOTS_PID=0
WEBOTS_BIN_PID=0
# shellcheck disable=SC1090
source "$latest"
echo "Stopping four robot demo run: ${RUN_ID:-unknown}"
for pid in "${WEBOTS_BIN_PID:-0}" "${WEBOTS_PID:-0}"; do
  if [ "$pid" -gt 1 ] 2>/dev/null && kill -0 "$pid" 2>/dev/null; then
    echo "TERM PID $pid"
    kill -TERM "$pid" 2>/dev/null || true
  fi
done
sleep 2
for pid in "${WEBOTS_BIN_PID:-0}" "${WEBOTS_PID:-0}"; do
  if [ "$pid" -gt 1 ] 2>/dev/null && kill -0 "$pid" 2>/dev/null; then
    echo "KILL PID $pid"
    kill -KILL "$pid" 2>/dev/null || true
  fi
done
echo "STOP DONE"
