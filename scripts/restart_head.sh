#!/usr/bin/env bash
# 重啟頭部橋接（XR 已連、controller_data/head_pose 穩定後救援用）。
#   用法：bash /tmp/restart_head.sh [real|isaac] [headpose|stick]
pkill -9 -f head_bridge.py 2>/dev/null || true
sleep 1
MODE="${1:-real}"
SOURCE="${2:-headpose}"
setsid bash /tmp/head_run.sh "$MODE" "$SOURCE" > /tmp/head.log 2>&1 < /dev/null &
disown || true
sleep 4
echo "===PROC==="
pgrep -af head_bridge.py | grep -v restart_head
echo "===LOG==="
cat /tmp/head.log