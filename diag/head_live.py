#!/usr/bin/env python3
"""15s live causality trace: print stick changes AND head-command changes in real
time so we can see whether head_bridge converts stick -> head cmd. Push the RIGHT
stick continuously while this runs."""
import time
import msgpack
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSPresetProfiles
from std_msgs.msg import ByteMultiArray
from trajectory_msgs.msg import JointTrajectory

class T(Node):
    def __init__(self):
        super().__init__("head_live")
        self.t0=time.time()
        self.pstick=(0.0,0.0)
        self.pcmd=None
        self.ncd=0; self.ncmd=0
        best=QoSPresetProfiles.SENSOR_DATA.value
        self.create_subscription(ByteMultiArray,"/xr_teleop/controller_data",self._c,best)
        self.create_subscription(JointTrajectory,"/head_controller/joint_trajectory",self._t,10)
    def _c(self,m):
        raw=b"".join(x if isinstance(x,(bytes,bytearray)) else bytes([x&0xFF]) for x in m.data)
        try: d=msgpack.unpackb(raw,raw=False)
        except Exception: return
        self.ncd+=1
        v=d.get("right_thumbstick")
        if isinstance(v,(list,tuple)) and len(v)>=2:
            cur=(round(float(v[0]),2),round(float(v[1]),2))
            if abs(cur[0]-self.pstick[0])>0.1 or abs(cur[1]-self.pstick[1])>0.1:
                print("[t=%5.1f] STICK %s" % (time.time()-self.t0, cur))
                self.pstick=cur
    def _t(self,m):
        if not m.points: return
        self.ncmd+=1
        cur=tuple(round(p,3) for p in m.points[0].positions)
        if self.pcmd is None: self.pcmd=cur; return
        if any(abs(a-b)>0.005 for a,b in zip(cur,self.pcmd)):
            print("[t=%5.1f]        HEAD_CMD yaw=%.3f pitch=%.3f" % (time.time()-self.t0, cur[0], cur[1]))
            self.pcmd=cur

def main():
    rclpy.init(); n=T()
    end=time.time()+30
    last_hb=time.time()
    while time.time()<end:
        rclpy.spin_once(n,timeout_sec=0.02)
        if time.time()-last_hb>=5:
            last_hb=time.time()
            print("[t=%5.1f] .. alive, cd=%d cmd=%d (push the stick!)" % (time.time()-n.t0, n.ncd, n.ncmd))
    print("--- controller_data msgs=%d, head_cmd msgs=%d ---" % (n.ncd, n.ncmd))
    n.destroy_node(); rclpy.shutdown()

if __name__=="__main__":
    main()
