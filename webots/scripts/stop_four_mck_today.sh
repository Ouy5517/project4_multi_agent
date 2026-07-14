#!/usr/bin/env bash
set +u
echo "Stopping four mck..."
pkill -x mck 2>/dev/null || true
pkill -f '[r]pc_service_node' 2>/dev/null || true
sleep 2
pkill -9 -x webots-bin 2>/dev/null || true
pkill -9 -x mck 2>/dev/null || true
rm -f /home/plon/Workspace/booster_t1_webots/runtime/*.ready 2>/dev/null
sleep 1
echo "Done. webots=$(pgrep -x webots-bin|xargs echo||echo 0) mck=$(pgrep -x mck|xargs echo||echo 0) rpc=$(pgrep -f '[r]pc_service_node'|xargs echo||echo 0)"