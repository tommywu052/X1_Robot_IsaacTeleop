#!/usr/bin/env bash
# Prepare cam WSL for REAL-robot teleop (does NOT touch the Feetech servos):
#   - stop the SIM stack (moveit_isaac + its ros2_control/move_group/rviz)
#   - stop head_bridge (it targets Isaac-only /isaac_joint_commands; useless on real)
#   - restart pink DISENGAGED with a gentler first-move (ready_settle_s=3.5)
#   - (re)start engage_button (Y button -> engage) since it was not running
#   - keep gripper_bridge as-is
# Real motion only happens later, when YOU launch moveit_real and press engage.
source /opt/ros/jazzy/setup.bash
source ~/xiaobei_X1_ws/install/setup.bash 2>/dev/null
export ROS_DOMAIN_ID=0
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
# 前端主機 IP（同機用 127.0.0.1，跨機改成前端 IP；或先 source /tmp/teleop_env.sh）
export ROS_STATIC_PEERS="${ROS_STATIC_PEERS:-127.0.0.1}"

echo "== stop SIM stack =="
pkill -9 -f moveit_isaac        2>/dev/null || true
pkill -9 -f move_group          2>/dev/null || true
pkill -9 -f ros2_control_node   2>/dev/null || true
pkill -9 -f 'lib/rviz2/rviz2'   2>/dev/null || true
pkill -9 -f robot_state_publisher 2>/dev/null || true
pkill -9 -f spawner             2>/dev/null || true

echo "== stop head_bridge (no-op on real) =="
pkill -9 -f head_bridge.py      2>/dev/null || true

echo "== restart pink DISENGAGED, gentle first move (ready_settle_s=3.5) =="
pkill -9 -f pink_arm_ik.py      2>/dev/null || true
sleep 2
nohup ~/pink_venv/bin/python3 /tmp/pink_arm_ik.py \
  --ros-args -p ready_settle_s:=3.5 -p start_engaged:=false \
  > /tmp/pink.log 2>&1 &

echo "== (re)start engage_button (Y = engage) =="
pkill -9 -f engage_button.py    2>/dev/null || true
sleep 1
nohup ~/pink_venv/bin/python3 /tmp/engage_button.py \
  --ros-args -p button_field:=left_secondary_click -p start_engaged:=false \
  > /tmp/engage_btn.log 2>&1 &

sleep 4
echo "== verify =="
echo "-- pink:";    pgrep -af pink_arm_ik.py    || echo "(pink NOT up!)"
echo "-- gripper:"; pgrep -af gripper_bridge.py || echo "(gripper down)"
echo "-- engage:";  pgrep -af engage_button.py  || echo "(engage down)"
echo "-- sim leftovers (should be clean):"; pgrep -af 'move_group|ros2_control_node|moveit_isaac' || echo "(clean)"
echo "-- ttyACM:";  ls -l /dev/ttyACM0
echo "-- pink.log:"; tail -6 /tmp/pink.log
