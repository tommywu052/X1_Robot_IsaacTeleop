#!/usr/bin/env python3
"""Reliable controller-state readout via the controller_manager service
(bypasses the flaky `ros2 control` CLI / daemon)."""
import rclpy
from rclpy.node import Node
from controller_manager_msgs.srv import ListControllers

def main():
    rclpy.init()
    n = Node("list_ctrl_probe")
    cli = n.create_client(ListControllers, "/controller_manager/list_controllers")
    if not cli.wait_for_service(timeout_sec=8.0):
        print("controller_manager service NOT available (CM down?)")
        return
    fut = cli.call_async(ListControllers.Request())
    rclpy.spin_until_future_complete(n, fut, timeout_sec=8.0)
    res = fut.result()
    if res is None:
        print("service call timed out")
        return
    if not res.controller:
        print("NO controllers loaded")
    for c in res.controller:
        print("%-26s state=%-12s type=%s" % (c.name, c.state, c.type))
    n.destroy_node(); rclpy.shutdown()

if __name__ == "__main__":
    main()
