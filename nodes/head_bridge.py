#!/usr/bin/env python3
"""XR -> X1 head (j_101 yaw / j_102 pitch).

Two INPUT sources (param `input_source`):

  input_source=stick (default): RIGHT thumbstick, RATE control. Stick deflection =
    angular velocity; head turns while held, holds when released. Reads
    /xr_teleop/controller_data (MessagePack). Good when you want the head
    independent of where you are looking.

  input_source=headpose: the headset ORIENTATION drives the head, POSITION
    (absolute) control -- the robot head mirrors where you look. Reads
    /xr_teleop/head_pose (PoseStamped). This is NOT iterative IK: a 2-DOF
    yaw+pitch neck is a closed-form decomposition of the gaze direction into two
    angles. We rotate the forward vector by the headset quaternion to get the
    gaze direction, then yaw=atan2(dx,-dz), pitch=asin(dy). On the first sample
    (after seeding joints from /joint_states) we capture a reference gaze so the
    current look direction maps to the current head pose (clutch/rebase, no
    jump), then track deltas with light EMA smoothing.

Two output modes (param `mode`):

  mode=isaac (sim): publish a JointState carrying ONLY the two head joints to
    /isaac_joint_commands. The sim stack has no head_controller, so the
    topic_based hardware publishes NaN for j_101/j_102 and Isaac ignores them
    (head holds); our by-name JointState drives them without touching the arm
    command. No resource conflict.

  mode=real (robot): the real ros2_control stack now spawns a JointTrajectory
    head_controller that claims j_101/j_102, so we CANNOT poke /joint_states
    style commands anymore -- we publish a trajectory_msgs/JointTrajectory to
    /<head_controller>/joint_trajectory. Each tick sends a single point at the
    integrated target with a short time_from_start so the JTC interpolates
    smoothly between our rate-limited setpoints.

Control model: RATE control. Stick deflection = angular velocity, so the head
turns while you hold the stick and stops (holds) when you release. We seed the
current head angles from /joint_states on startup so there is no jump, then
integrate. Signs / rates / soft range are all parameters so directions can be
flipped after a live test.

  right_thumbstick[0] (x, left/right) -> j_101 yaw
  right_thumbstick[1] (y, up/down)    -> j_102 pitch
"""
import math
import time

import msgpack
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSPresetProfiles
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
from std_msgs.msg import ByteMultiArray
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


def _clamp(v, lo, hi):
    return lo if v < lo else hi if v > hi else v


def _wrap(a):
    """Normalize an angle difference to (-pi, pi]."""
    return math.atan2(math.sin(a), math.cos(a))


def _gaze_yaw_pitch(x, y, z, w):
    """Gaze yaw/pitch from a headset quaternion: rotate forward (0,0,-1) by q,
    then decompose. Frame/axis sign quirks are absorbed by yaw_sign/pitch_sign
    and the reference rebase, so this need not match a specific convention."""
    dx = -2.0 * (x * z + w * y)
    dy = -2.0 * (y * z - w * x)
    dz = 2.0 * (x * x + y * y) - 1.0
    yaw = math.atan2(dx, -dz)
    pitch = math.asin(_clamp(dy, -1.0, 1.0))
    return yaw, pitch


class HeadBridge(Node):
    def __init__(self):
        super().__init__("head_bridge")
        self.declare_parameter("yaw_joint", "j_101")
        self.declare_parameter("pitch_joint", "j_102")
        # "isaac" -> JointState to cmd_topic ; "real" -> JointTrajectory to head_controller
        self.declare_parameter("mode", "isaac")
        self.declare_parameter("cmd_topic", "/isaac_joint_commands")
        self.declare_parameter("head_controller", "head_controller")
        # JTC interpolation horizon per point (s); a few ticks ahead = smooth
        self.declare_parameter("traj_time", 0.10)
        self.declare_parameter("stick_field", "right_thumbstick")
        # velocity at full stick (rad/s)
        self.declare_parameter("yaw_rate", 1.2)
        self.declare_parameter("pitch_rate", 0.9)
        # direction: flip if a live test shows the head moving the wrong way
        self.declare_parameter("yaw_sign", -1.0)
        self.declare_parameter("pitch_sign", 1.0)
        self.declare_parameter("deadzone", 0.15)
        # soft travel limits (rad); continuous joints, so keep these sane
        self.declare_parameter("yaw_min", -1.6)
        self.declare_parameter("yaw_max", 1.6)
        self.declare_parameter("pitch_min", -0.7)
        self.declare_parameter("pitch_max", 0.7)
        self.declare_parameter("rate_hz", 50.0)
        # self-heal: if NO controller_data at all for this long, the subscription
        # never matched (e.g. head_bridge started before the XR publisher existed,
        # a common race when auto-started by run_real_robot.sh). Recreate the sub
        # to force fresh DDS discovery. Safe because an idle-but-matched stream
        # still delivers ~33 Hz zero-value msgs, so this only fires when truly
        # unmatched, never merely because the stick is centered.
        self.declare_parameter("resub_timeout_s", 4.0)
        # input source: "stick" (rate) or "headpose" (absolute, mirror your gaze)
        self.declare_parameter("input_source", "stick")
        self.declare_parameter("headpose_topic", "/xr_teleop/head_pose")
        # headpose: map scale (1.0 = 1:1 mirror) and EMA smoothing (0..1, higher=snappier)
        self.declare_parameter("yaw_scale", 1.0)
        self.declare_parameter("pitch_scale", 1.0)
        self.declare_parameter("ema_alpha", 0.25)

        self.yj = self.get_parameter("yaw_joint").value
        self.pj = self.get_parameter("pitch_joint").value
        self.field = self.get_parameter("stick_field").value
        self.yaw_rate = float(self.get_parameter("yaw_rate").value)
        self.pitch_rate = float(self.get_parameter("pitch_rate").value)
        self.yaw_sign = float(self.get_parameter("yaw_sign").value)
        self.pitch_sign = float(self.get_parameter("pitch_sign").value)
        self.dz = float(self.get_parameter("deadzone").value)
        self.yaw_min = float(self.get_parameter("yaw_min").value)
        self.yaw_max = float(self.get_parameter("yaw_max").value)
        self.pitch_min = float(self.get_parameter("pitch_min").value)
        self.pitch_max = float(self.get_parameter("pitch_max").value)
        hz = float(self.get_parameter("rate_hz").value)
        self.mode = str(self.get_parameter("mode").value).lower()
        self.traj_time = float(self.get_parameter("traj_time").value)
        head_ctrl = self.get_parameter("head_controller").value
        self.source = str(self.get_parameter("input_source").value).lower()
        self.yaw_scale = float(self.get_parameter("yaw_scale").value)
        self.pitch_scale = float(self.get_parameter("pitch_scale").value)
        self.ema = float(self.get_parameter("ema_alpha").value)

        self.sx = 0.0
        self.sy = 0.0
        self.yaw = None   # seeded from /joint_states
        self.pitch = None
        self.base_yaw = None    # head joint angles captured at seed time (headpose base)
        self.base_pitch = None
        self.ref_yaw = None     # gaze reference captured on first headpose (clutch)
        self.ref_pitch = None
        self.warned = False
        self.resub_timeout = float(self.get_parameter("resub_timeout_s").value)
        self._last_rx = None       # wall-clock of last controller_data msg
        self._ctrl_sub = None

        self.create_subscription(JointState, "/joint_states", self._on_js, 10)
        if self.source == "headpose":
            best = QoSPresetProfiles.SENSOR_DATA.value
            self.create_subscription(PoseStamped, self.get_parameter("headpose_topic").value,
                                     self._on_headpose, best)
        else:
            self._make_ctrl_sub()
        if self.mode == "real":
            self.traj_topic = "/%s/joint_trajectory" % head_ctrl
            self.pub = self.create_publisher(JointTrajectory, self.traj_topic, 10)
            out = "JointTrajectory -> %s" % self.traj_topic
        else:
            self.cmd_topic = self.get_parameter("cmd_topic").value
            self.pub = self.create_publisher(JointState, self.cmd_topic, 10)
            out = "JointState -> %s" % self.cmd_topic
        self.dt = 1.0 / hz
        self.create_timer(self.dt, self._tick)
        src = ("headpose(%s)" % self.get_parameter("headpose_topic").value
               if self.source == "headpose" else "stick(%s)" % self.field)
        self.get_logger().info(
            "head_bridge up [source=%s mode=%s]: %s(yaw)/%s(pitch) <- %s, rate_hz=%.0f, %s"
            % (self.source, self.mode, self.yj, self.pj, src, hz, out))

    def _make_ctrl_sub(self):
        best = QoSPresetProfiles.SENSOR_DATA.value
        if self._ctrl_sub is not None:
            self.destroy_subscription(self._ctrl_sub)
        self._ctrl_sub = self.create_subscription(
            ByteMultiArray, "/xr_teleop/controller_data", self._on_ctrl, best)
        self._last_rx = time.monotonic()

    def _maybe_resub(self):
        # never received (or stopped) for too long -> subscription likely unmatched
        if self._last_rx is None:
            return
        if time.monotonic() - self._last_rx > self.resub_timeout:
            self.get_logger().warn(
                "no controller_data for %.1fs -> recreating subscription (re-discovery)"
                % self.resub_timeout)
            self._make_ctrl_sub()

    def _on_js(self, msg):
        if self.yaw is not None:
            return
        try:
            iy = list(msg.name).index(self.yj)
            ip = list(msg.name).index(self.pj)
        except ValueError:
            return
        y = msg.position[iy]
        p = msg.position[ip]
        if math.isnan(y) or math.isnan(p):
            return
        self.yaw = _clamp(y, self.yaw_min, self.yaw_max)
        self.pitch = _clamp(p, self.pitch_min, self.pitch_max)
        self.base_yaw = self.yaw
        self.base_pitch = self.pitch
        self.get_logger().info("seeded head: yaw=%.3f pitch=%.3f" % (self.yaw, self.pitch))

    def _on_headpose(self, msg):
        if self.base_yaw is None:   # wait until joints are seeded
            return
        o = msg.pose.orientation
        gy, gp = _gaze_yaw_pitch(o.x, o.y, o.z, o.w)
        if self.ref_yaw is None:    # clutch: current gaze = current head pose
            self.ref_yaw = gy
            self.ref_pitch = gp
            self.get_logger().info("headpose reference captured: yaw0=%.3f pitch0=%.3f" % (gy, gp))
            return
        tgt_yaw = _clamp(self.base_yaw + self.yaw_sign * self.yaw_scale * _wrap(gy - self.ref_yaw),
                         self.yaw_min, self.yaw_max)
        tgt_pitch = _clamp(self.base_pitch + self.pitch_sign * self.pitch_scale * _wrap(gp - self.ref_pitch),
                           self.pitch_min, self.pitch_max)
        a = self.ema
        self.yaw = (1.0 - a) * self.yaw + a * tgt_yaw
        self.pitch = (1.0 - a) * self.pitch + a * tgt_pitch

    def _on_ctrl(self, msg):
        raw = b"".join(x if isinstance(x, (bytes, bytearray)) else bytes([x & 0xFF])
                       for x in msg.data)
        try:
            d = msgpack.unpackb(raw, raw=False)
        except Exception:  # noqa: BLE001
            return
        self._last_rx = time.monotonic()
        v = d.get(self.field)
        if not isinstance(v, (list, tuple)) or len(v) < 2:
            if not self.warned:
                self.get_logger().warn("no '%s' [x,y] in controller_data; keys=%s"
                                       % (self.field, list(d.keys())))
                self.warned = True
            return
        try:
            self.sx = float(v[0])
            self.sy = float(v[1])
        except (TypeError, ValueError):
            self.sx = self.sy = 0.0

    def _dead(self, v):
        return 0.0 if abs(v) < self.dz else v

    def _tick(self):
        if self.source == "stick":
            self._maybe_resub()
        if self.yaw is None:
            return
        if self.source == "stick":
            # RATE control: integrate stick deflection into the target angle
            x = self._dead(self.sx)
            y = self._dead(self.sy)
            self.yaw = _clamp(self.yaw + self.yaw_sign * x * self.yaw_rate * self.dt,
                              self.yaw_min, self.yaw_max)
            self.pitch = _clamp(self.pitch + self.pitch_sign * y * self.pitch_rate * self.dt,
                                self.pitch_min, self.pitch_max)
        # headpose: self.yaw/self.pitch are set absolutely in _on_headpose
        if self.mode == "real":
            jt = JointTrajectory()
            jt.joint_names = [self.yj, self.pj]
            pt = JointTrajectoryPoint()
            pt.positions = [self.yaw, self.pitch]
            pt.time_from_start.sec = int(self.traj_time)
            pt.time_from_start.nanosec = int((self.traj_time % 1.0) * 1e9)
            jt.points = [pt]
            self.pub.publish(jt)
        else:
            js = JointState()
            js.header.stamp = self.get_clock().now().to_msg()
            js.name = [self.yj, self.pj]
            js.position = [self.yaw, self.pitch]
            self.pub.publish(js)


def main():
    rclpy.init()
    node = HeadBridge()
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
