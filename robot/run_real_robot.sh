#!/usr/bin/env bash
# Launch MoveIt against the REAL Xiaobei X1 robot (Feetech servos over /dev/ttyACM0).
# WARNING: this ENABLES TORQUE and allows MoveIt to execute trajectories.
# Make sure the workspace is clear and the e-stop / power switch is within reach.
source /opt/ros/jazzy/setup.bash
source ~/xiaobei_X1_ws/install/setup.bash
export DISPLAY=127.0.0.1:0.0
unset WAYLAND_DISPLAY
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_DOMAIN_ID=0
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
unset FASTRTPS_DEFAULT_PROFILES_FILE
unset FASTDDS_DEFAULT_PROFILES_FILE

if [ ! -e /dev/ttyACM0 ]; then
  echo "ERROR: /dev/ttyACM0 not found."
  echo "Attach the robot USB from Windows (admin PowerShell):"
  echo '  & "C:\Program Files\usbipd-win\usbipd.exe" attach --wsl --busid 9-3'
  exit 1
fi
if [ ! -w /dev/ttyACM0 ]; then
  echo "Fixing /dev/ttyACM0 permission (will prompt for sudo password)..."
  sudo chmod 666 /dev/ttyACM0
fi
echo "DISPLAY=$DISPLAY  /dev/ttyACM0 ready (real robot)"

pkill -9 -f move_group 2>/dev/null || true
pkill -9 -f rviz2 2>/dev/null || true
pkill -9 -f ros2_control_node 2>/dev/null || true
pkill -9 -f robot_state_publisher 2>/dev/null || true
pkill -9 -f spawner 2>/dev/null || true
timeout 5 ros2 daemon stop 2>/dev/null || true
sleep 2
echo "=== launching moveit_real.launch.py (REAL ROBOT) ==="
exec ros2 launch pkg_robot_model moveit_real.launch.py
