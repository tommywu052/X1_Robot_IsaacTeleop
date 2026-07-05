#!/usr/bin/env python3
"""Gripper bridge: map XR controller finger curl -> robot gripper open/close.

Frontend (Isaac Teleop controller_teleop) publishes /xr_teleop/finger_joints
(sensor_msgs/JointState with TriHand finger joints). The controller
trigger/grip drives finger curl. We take a representative curl per hand
(index+middle proximal average) and command the simple 1-DOF grippers:
  left  -> /left_gripper_controller/joint_trajectory   (joint j_7)
  right -> /right_gripper_controller/joint_trajectory  (joint j_57)

Squeeze grip/trigger => fingers curl => gripper closes. All ranges are ROS
params so the mapping/direction can be tuned live without editing code.
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSPresetProfiles
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration

LEFT_CURL = ["left_index_proximal", "left_middle_proximal"]
RIGHT_CURL = ["right_index_proximal", "right_middle_proximal"]


class GripperBridge(Node):
    def __init__(self):
        super().__init__("gripper_bridge")
        # finger curl value that maps to fully-open / fully-closed gripper
        self.declare_parameter("curl_open", 0.05)
        self.declare_parameter("curl_close", 1.2)
        # gripper joint positions for open / closed (flip these to invert)
        self.declare_parameter("gripper_open", 0.0)
        self.declare_parameter("gripper_close", 1.0)
        self.declare_parameter("rate_hz", 30.0)

        best = QoSPresetProfiles.SENSOR_DATA.value
        self.pub_l = self.create_publisher(JointTrajectory, "/left_gripper_controller/joint_trajectory", 10)
        self.pub_r = self.create_publisher(JointTrajectory, "/right_gripper_controller/joint_trajectory", 10)
        self.create_subscription(JointState, "/xr_teleop/finger_joints", self._on_fingers, best)
        self.curl_l = 0.0
        self.curl_r = 0.0
        self.have = False
        rate = float(self.get_parameter("rate_hz").value)
        self.create_timer(1.0 / rate, self._tick)
        self.get_logger().info("gripper_bridge up (finger curl -> j_7/j_57)")

    def _on_fingers(self, msg: JointState):
        idx = {n: i for i, n in enumerate(msg.name)}

        def curl(names):
            vals = [abs(msg.position[idx[n]]) for n in names if n in idx]
            return sum(vals) / len(vals) if vals else 0.0

        self.curl_l = curl(LEFT_CURL)
        self.curl_r = curl(RIGHT_CURL)
        self.have = True

    def _map(self, curl):
        c0 = float(self.get_parameter("curl_open").value)
        c1 = float(self.get_parameter("curl_close").value)
        g0 = float(self.get_parameter("gripper_open").value)
        g1 = float(self.get_parameter("gripper_close").value)
        t = (curl - c0) / (c1 - c0) if abs(c1 - c0) > 1e-6 else 0.0
        t = max(0.0, min(1.0, t))
        return g0 + t * (g1 - g0)

    def _tick(self):
        if not self.have:
            return
        for curl, joint, pub in ((self.curl_l, "j_7", self.pub_l),
                                 (self.curl_r, "j_57", self.pub_r)):
            traj = JointTrajectory()
            traj.joint_names = [joint]
            pt = JointTrajectoryPoint()
            pt.positions = [self._map(curl)]
            pt.time_from_start = Duration(sec=0, nanosec=100000000)  # 0.1s
            traj.points = [pt]
            pub.publish(traj)


def main():
    rclpy.init()
    node = GripperBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
