#!/usr/bin/env python3
"""Teleop ready pose the user likes: arms down, both elbows bent up.

Both arms use the same joint values (visually symmetric per user feedback).
Servo singularity thresholds are relaxed so this near-nominal pose does not
trigger an emergency stop; motion is just velocity-scaled near singularities.
"""
import time
import rclpy
from rclpy.node import Node
from builtin_interfaces.msg import Duration
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

LEFT = ["j_1", "j_2", "j_3", "j_4", "j_5", "j_6"]
RIGHT = ["j_51", "j_52", "j_53", "j_54", "j_55", "j_56"]

# Natural ready pose (ready_search.py): hands in FRONT (+x), elbow only ~51deg
# bent (j_4=-0.895, vs the old -1.785/102deg that looked contorted), chest height.
# Well-conditioned (cond~34, far from singularity). Identical joint values on both
# arms give a clean y-mirror of the EE position, so the pose is left/right symmetric.
# Left/right differ ONLY in the wrist roll (j_6 vs j_56 = -pi - j_6) so the
# grippers point INWARD symmetrically (mirror_ik.py: exact y-mirror of position
# AND orientation, least-contorted solution).
# Arm (j_1..j_5) identical on both arms -> symmetric shape, elbow unchanged.
# ONLY the wrist roll j_6 is sign-flipped (pin_mirror.py: rotating just the L6
# end points the grippers inward symmetrically; orierr~0, poserr~0).
LEFT_POSE = [0.067, -0.359, -0.27, 0.889, 1.145, 0.767]
RIGHT_POSE = [0.067, -0.359, -0.27, 0.889, 1.145, -0.767]


def traj(names, pos, sec):
    t = JointTrajectory()
    t.joint_names = names
    p = JointTrajectoryPoint()
    p.positions = pos
    p.time_from_start = Duration(sec=sec, nanosec=0)
    t.points = [p]
    return t


def main():
    rclpy.init()
    n = Node("ready_pose")
    pl = n.create_publisher(JointTrajectory, "/left_arm_controller/joint_trajectory", 10)
    pr = n.create_publisher(JointTrajectory, "/right_arm_controller/joint_trajectory", 10)
    time.sleep(1.5)
    for _ in range(5):
        pl.publish(traj(LEFT, LEFT_POSE, 3))
        pr.publish(traj(RIGHT, RIGHT_POSE, 3))
        rclpy.spin_once(n, timeout_sec=0.1)
        time.sleep(0.2)
    time.sleep(4.0)
    print("ready pose sent  L=%s  R=%s" % (LEFT_POSE, RIGHT_POSE))
    n.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
