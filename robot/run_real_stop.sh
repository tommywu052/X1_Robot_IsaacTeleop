#!/usr/bin/env bash
# Gracefully stop the real-robot MoveIt stack. Deactivating ros2_control will
# call on_deactivate (torque OFF) on the hardware before processes are killed.
pkill -INT -f 'ros2 launch' 2>/dev/null || true
sleep 3
pkill -9 -f move_group 2>/dev/null || true
pkill -9 -f rviz2 2>/dev/null || true
pkill -9 -f ros2_control_node 2>/dev/null || true
pkill -9 -f robot_state_publisher 2>/dev/null || true
pkill -9 -f spawner 2>/dev/null || true
ros2 daemon stop 2>/dev/null || true
echo "stopped."
