#!/usr/bin/env bash
# 停止全部遙操 bridge（不動機器人控制堆疊；真機請另外用 run_real_stop.sh 收 torque）。
echo "== 停止遙操 bridge =="
for p in pink_arm_ik.py gripper_bridge.py engage_button.py head_bridge.py; do
  pkill -9 -f "$p" 2>/dev/null && echo "  killed $p" || echo "  $p not running"
done
echo "done."