#!/usr/bin/env python3
"""List all DDS topics as seen by a live rclpy node (same env as the working
pink/gripper nodes). More reliable than the ros2 CLI daemon on this host."""
import time
import rclpy
from rclpy.node import Node


def main():
    rclpy.init()
    n = Node("topic_probe")
    # let discovery settle
    end = time.time() + 6.0
    while time.time() < end:
        rclpy.spin_once(n, timeout_sec=0.2)
    seen = n.get_topic_names_and_types()
    print("== TOPICS (%d) ==" % len(seen))
    for name, types in sorted(seen):
        print("%s  %s" % (name, ",".join(types)))
    n.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
