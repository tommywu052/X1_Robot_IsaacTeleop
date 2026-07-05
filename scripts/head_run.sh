#!/usr/bin/env bash
# 頭部橋接：
#   MODE   = real  (JointTrajectory -> /head_controller，真機) | isaac (JointState -> /isaac_joint_commands，模擬)
#   SOURCE = headpose (頭顯朝向絕對鏡像，頭往哪看機器人往哪轉) | stick (右搖桿速率控制)
# 本套件預設 headpose。pitch 方向依來源不同：headpose 需 -1（下看=下看），stick 用 +1。
source /tmp/teleop_env.sh
MODE="${1:-real}"
SOURCE="${2:-headpose}"
if [ "$SOURCE" = "headpose" ]; then
  PITCH_SIGN="${3:--1.0}"
else
  PITCH_SIGN="${3:-1.0}"
fi
exec "$TELEOP_PY" /tmp/head_bridge.py --ros-args \
  -p mode:="$MODE" -p input_source:="$SOURCE" -p pitch_sign:="$PITCH_SIGN"