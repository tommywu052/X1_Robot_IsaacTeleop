#!/usr/bin/env python3
"""Decisive local check for the head path:
 - who publishes / subscribes /head_controller/joint_trajectory
 - retry controller_manager list with a generous timeout
"""
import rclpy
from rclpy.node import Node
from controller_manager_msgs.srv import ListControllers

TOPIC = "/head_controller/joint_trajectory"

def main():
    rclpy.init()
    n = Node("head_probe")
    # let discovery settle
    import time
    end = time.time() + 3
    while time.time() < end:
        rclpy.spin_once(n, timeout_sec=0.2)

    pubs = n.get_publishers_info_by_topic(TOPIC)
    subs = n.get_subscriptions_info_by_topic(TOPIC)
    print("=== %s ===" % TOPIC)
    print("publishers  (%d):" % len(pubs))
    for p in pubs:
        print("   node=%s type=%s" % (p.node_name, p.topic_type))
    print("subscribers (%d):" % len(subs))
    for s in subs:
        print("   node=%s type=%s" % (s.node_name, s.topic_type))
    if len(subs) == 0:
        print(">>> NO subscriber -> head_controller is NOT active/loaded")
    else:
        print(">>> head_controller IS subscribed (active)")

    # try controller list (best effort)
    cli = n.create_client(ListControllers, "/controller_manager/list_controllers")
    if cli.wait_for_service(timeout_sec=5.0):
        fut = cli.call_async(ListControllers.Request())
        rclpy.spin_until_future_complete(n, fut, timeout_sec=12.0)
        res = fut.result()
        if res is not None:
            print("=== controllers ===")
            for c in res.controller:
                print("   %-26s %s" % (c.name, c.state))
        else:
            print("(list_controllers call timed out)")
    else:
        print("(controller_manager service not available)")
    n.destroy_node(); rclpy.shutdown()

if __name__ == "__main__":
    main()
