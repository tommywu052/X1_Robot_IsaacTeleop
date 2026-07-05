#!/usr/bin/env python3
"""30s causality trace for PATH B: print gaze yaw/pitch (from head_pose) changes
AND head-command changes, so we can confirm headset orientation drives the head.
Rotate your view/head in the emulator while this runs."""
import time, math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSPresetProfiles
from geometry_msgs.msg import PoseStamped
from trajectory_msgs.msg import JointTrajectory

def gaze(x, y, z, w):
    dx = -2.0*(x*z + w*y); dy = -2.0*(y*z - w*x); dz = 2.0*(x*x + y*y) - 1.0
    return math.degrees(math.atan2(dx, -dz)), math.degrees(math.asin(max(-1.0, min(1.0, dy))))

class T(Node):
    def __init__(self):
        super().__init__("headpose_live")
        self.t0=time.time(); self.pg=None; self.pc=None; self.ng=0; self.nc=0
        best=QoSPresetProfiles.SENSOR_DATA.value
        self.create_subscription(PoseStamped,"/xr_teleop/head_pose",self._h,best)
        self.create_subscription(JointTrajectory,"/head_controller/joint_trajectory",self._t,10)
    def _h(self,m):
        self.ng+=1
        o=m.pose.orientation; gy,gp=gaze(o.x,o.y,o.z,o.w)
        cur=(round(gy,1),round(gp,1))
        if self.pg is None: self.pg=cur; return
        if abs(cur[0]-self.pg[0])>2 or abs(cur[1]-self.pg[1])>2:
            print("[t=%5.1f] GAZE yaw=%6.1f pitch=%6.1f (deg)" % (time.time()-self.t0,cur[0],cur[1]))
            self.pg=cur
    def _t(self,m):
        if not m.points: return
        self.nc+=1
        cur=tuple(round(math.degrees(p),1) for p in m.points[0].positions)
        if self.pc is None: self.pc=cur; return
        if any(abs(a-b)>1.0 for a,b in zip(cur,self.pc)):
            print("[t=%5.1f]        HEAD_CMD yaw=%6.1f pitch=%6.1f (deg)" % (time.time()-self.t0,cur[0],cur[1]))
            self.pc=cur

def main():
    rclpy.init(); n=T(); end=time.time()+30; hb=time.time()
    while time.time()<end:
        rclpy.spin_once(n,timeout_sec=0.02)
        if time.time()-hb>=6:
            hb=time.time(); print("[t=%5.1f] .. alive gaze_msgs=%d cmd_msgs=%d (rotate your head!)" % (time.time()-n.t0,n.ng,n.nc))
    print("--- head_pose msgs=%d, head_cmd msgs=%d ---" % (n.ng,n.nc))
    n.destroy_node(); rclpy.shutdown()

if __name__=="__main__":
    main()
