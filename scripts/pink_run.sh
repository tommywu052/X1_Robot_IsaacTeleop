#!/usr/bin/env bash
# 雙臂差分 IK 主節點（Pinocchio + Pink）。預設 disengage，按左手 Y 才接手。
# 可用環境變數覆寫首步安定時間：READY_SETTLE_S（真機建議 3.5，模擬 1.6）。
source /tmp/teleop_env.sh
exec "$TELEOP_PY" /tmp/pink_arm_ik.py --ros-args \
  -p start_engaged:=false \
  -p ready_settle_s:="${READY_SETTLE_S:-1.6}"