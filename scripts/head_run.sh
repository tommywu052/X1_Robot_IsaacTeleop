#!/usr/bin/env bash
# 頭部橋接：
#   MODE   = real  (JointTrajectory -> /head_controller，真機) | isaac (JointState -> /isaac_joint_commands，模擬)
#   SOURCE = headpose (頭顯朝向絕對鏡像，頭往哪看機器人往哪轉) | stick (右搖桿速率控制)
# 本套件預設 headpose。方向依來源不同（Quest3 實測校正）：headpose 需 yaw+pitch 都 +1
# （頭往左/下看 = 機器人往左/下轉，1:1 鏡像），stick 用自己調好的號。$3=pitch $4=yaw 可覆寫。
source /tmp/teleop_env.sh
MODE="${1:-real}"
SOURCE="${2:-headpose}"
if [ "$SOURCE" = "headpose" ]; then
  PITCH_SIGN="${3:-1.0}"
  YAW_SIGN="${4:-1.0}"
else
  PITCH_SIGN="${3:-1.0}"
  YAW_SIGN="${4:--1.0}"
fi
exec "$TELEOP_PY" /tmp/head_bridge.py --ros-args \
  -p mode:="$MODE" -p input_source:="$SOURCE" \
  -p pitch_sign:="$PITCH_SIGN" -p yaw_sign:="$YAW_SIGN"