#!/usr/bin/env python3
"""Count controller_data vs ee_poses over 5s to confirm the controllers are
asleep while hand/pose tracking is alive. Move the RIGHT controller now."""
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSPresetProfiles
from std_msgs.msg import ByteMultiArray
from geometry_msgs.msg import PoseArray

class R(Node):
    def __init__(self):
        super().__init__("rate_probe")
        self.cd=0; self.ee=0
        best=QoSPresetProfiles.SENSOR_DATA.value
        self.create_subscription(ByteMultiArray,"/xr_teleop/controller_data",self._cd,best)
        self.create_subscription(PoseArray,"/xr_teleop/ee_poses",self._ee,best)
    def _cd(self,m): self.cd+=1
    def _ee(self,m): self.ee+=1

def main():
    rclpy.init(); n=R()
    end=time.time()+5
    while time.time()<end:
        rclpy.spin_once(n,timeout_sec=0.05)
    print("controller_data: %d msgs in 5s  (%.1f Hz)" % (n.cd, n.cd/5.0))
    print("ee_poses       : %d msgs in 5s  (%.1f Hz)" % (n.ee, n.ee/5.0))
    if n.cd==0 and n.ee>0:
        print(">>> controllers ASLEEP / hand-tracking: poses alive, stick dead")
    elif n.cd==0 and n.ee==0:
        print(">>> whole XR input dead (CloudXR session likely dropped)")
    else:
        print(">>> controller_data IS flowing now")
    n.destroy_node(); rclpy.shutdown()

if __name__=="__main__":
    main()
