#!/usr/bin/env bash
source /opt/ros/jazzy/setup.bash
source ~/xiaobei_X1_ws/install/setup.bash 2>/dev/null
export ROS_DOMAIN_ID=0
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
# 前端主機 IP（同機用 127.0.0.1，跨機改成前端 IP；或先 source /tmp/teleop_env.sh）
export ROS_STATIC_PEERS="${ROS_STATIC_PEERS:-127.0.0.1}"

echo "== controllers =="
timeout 15 ros2 control list_controllers 2>&1 || echo "(list_controllers CLI timeout - will rely on topic check)"
echo "== hardware components =="
timeout 12 ros2 control list_hardware_components 2>&1 || true
echo "== /joint_states probe (reliable, rclpy) =="
~/pink_venv/bin/python3 /tmp/joints_probe.py 2>&1 || true
echo "== hardware log (servo activation) =="
grep -iE "Opening Port|Activating|torque|responding|Failed|error|Discrepancy" /tmp/moveit_real.log 2>/dev/null | tail -25 || echo "(no /tmp/moveit_real.log - launched in foreground terminal)"
