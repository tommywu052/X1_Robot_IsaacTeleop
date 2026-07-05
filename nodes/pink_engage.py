import os, sys
os.environ.setdefault('ROS_DOMAIN_ID', '0')
os.environ['RMW_IMPLEMENTATION'] = 'rmw_fastrtps_cpp'
os.environ['ROS_AUTOMATIC_DISCOVERY_RANGE'] = 'LOCALHOST'
import rclpy
from rclpy.node import Node
from std_srvs.srv import SetBool

val = (len(sys.argv) > 1 and sys.argv[1].lower() in ('1', 'true', 'on'))
rclpy.init()
n = Node('pink_engage')
cli = n.create_client(SetBool, '/pink_arm_ik/engage')
if not cli.wait_for_service(timeout_sec=5.0):
    print('engage service not available')
    raise SystemExit(1)
req = SetBool.Request()
req.data = val
fut = cli.call_async(req)
rclpy.spin_until_future_complete(n, fut, timeout_sec=5.0)
r = fut.result()
print('engage(%s) -> success=%s msg=%s' % (val, r.success, r.message))
n.destroy_node()
rclpy.shutdown()
