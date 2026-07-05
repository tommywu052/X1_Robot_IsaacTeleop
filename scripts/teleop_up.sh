#!/usr/bin/env bash
# ---------------------------------------------------------------------------
# 一鍵啟動 X1 Quest3 遙操的四個 bridge（雙臂 + 夾爪 + engage 按鈕 + 頭部）。
#
#   用法：bash /tmp/teleop_up.sh [isaac|real] [headpose|stick]
#   預設：real headpose
#
# 前置條件（本腳本【不】負責，需先就緒）：
#   - 機器人控制堆疊已在跑：
#       模擬 -> Isaac Sim 播放中 + run_isaac_moveit（有 /joint_states、*_controller）
#       真機 -> bash /tmp/run_real_robot.sh（ros2_control + 各控制器 spawn 完成）
#   - 前端 CloudXR / teleop_ros2_node 在 horde（$HORDE_IP）上運作。
# ---------------------------------------------------------------------------
source /tmp/teleop_env.sh
MODE="${1:-real}"
SOURCE="${2:-headpose}"

if [ "$MODE" = "real" ]; then
  export READY_SETTLE_S="${READY_SETTLE_S:-3.5}"   # 真機首步放緩
else
  export READY_SETTLE_S="${READY_SETTLE_S:-1.6}"
fi

echo "=== teleop_up: mode=$MODE  head_source=$SOURCE  ready_settle=${READY_SETTLE_S}s ==="

echo "== 1) 擺 ready pose =="
bash /tmp/ready_run.sh

echo "== 2) 雙臂 IK（disengage）=="
pkill -9 -f pink_arm_ik.py 2>/dev/null || true
sleep 1
nohup bash /tmp/pink_run.sh > /tmp/pink.log 2>&1 &
disown

echo "== 3) 夾爪 =="
pkill -9 -f gripper_bridge.py 2>/dev/null || true
sleep 1
nohup bash /tmp/gripper_run.sh > /tmp/gripper.log 2>&1 &
disown

echo "== 4) engage 按鈕（左手 Y）=="
pkill -9 -f engage_button.py 2>/dev/null || true
sleep 1
nohup bash /tmp/engage_run.sh > /tmp/engage_btn.log 2>&1 &
disown

echo "== 5) 頭部（$SOURCE）=="
pkill -9 -f head_bridge.py 2>/dev/null || true
sleep 1
setsid bash /tmp/head_run.sh "$MODE" "$SOURCE" > /tmp/head.log 2>&1 < /dev/null &
disown

sleep 4
echo "=== 狀態檢查 ==="
echo "-- pink:";    pgrep -af pink_arm_ik.py    || echo "  (pink NOT up!)"
echo "-- gripper:"; pgrep -af gripper_bridge.py || echo "  (gripper NOT up!)"
echo "-- engage:";  pgrep -af engage_button.py  || echo "  (engage NOT up!)"
echo "-- head:";    pgrep -af head_bridge.py | grep -v teleop_up || echo "  (head NOT up!)"
echo "--- pink.log ---";    tail -4 /tmp/pink.log    2>/dev/null
echo "--- head.log ---";    tail -4 /tmp/head.log    2>/dev/null
echo
echo "OK。戴上/連上 Quest3 -> 按左手 Y 讓手臂接手；轉頭驅動機器人頭部；擠壓扳機控夾爪。"
echo "停止：bash /tmp/teleop_down.sh"