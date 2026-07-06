#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""Isaac Teleop -> BESEK X1 dual-arm bridge using PINK differential IK.

Replaces the MoveIt-Servo (Jacobian-velocity) path. Pink solves a QP each cycle
(frame tasks for both hands + posture regularization) with damped least squares,
so it degrades GRACEFULLY near singularities (slows down) instead of the hard
emergency-stops that MoveIt Servo produced.

Pipeline:
  /xr_teleop/ee_poses (PoseArray [left,right])
    -> clutch (rebase XR target onto current EE at engage, no jump)
    -> Pink FrameTask targets (L6, R56)
    -> solve_ik -> integrate -> joint positions
    -> /{left,right}_arm_controller/joint_trajectory

Run with the venv python that has pinocchio+pink:  ~/pink_venv/bin/python3
Needs ROS env sourced so rclpy + message packages import.
"""
import math
import os
import time
from typing import Optional

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSPresetProfiles
from geometry_msgs.msg import PoseArray
from sensor_msgs.msg import JointState
from std_srvs.srv import SetBool
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration

import pinocchio as pin
from pink import solve_ik
from pink import Configuration
from pink.tasks import FrameTask, PostureTask

URDF = os.path.expanduser("~/xiaobei_X1_ws/src/pkg_robot_model/urdf/pkg_robot_model.urdf")
LEFT_JOINTS = ["j_1", "j_2", "j_3", "j_4", "j_5", "j_6"]
RIGHT_JOINTS = ["j_51", "j_52", "j_53", "j_54", "j_55", "j_56"]
LEFT_EE = "L6"
RIGHT_EE = "R56"
# Nominal posture for redundancy resolution == the ready pose (ready_search.py):
# hands in FRONT, elbow only ~51deg bent (j_4=-0.895), well-conditioned (cond~34).
# Identical values on both arms give a clean y-mirror EE (left/right symmetric).
# Left/right differ ONLY in wrist roll (j_56 = -pi - j_6) so grippers point
# INWARD symmetrically (mirror_ik.py exact y-mirror of position AND orientation).
NOMINAL = {
    "j_1": 0.067, "j_2": -0.359, "j_3": -0.27, "j_4": 0.889, "j_5": 1.145, "j_6": 0.767,
    "j_51": 0.067, "j_52": -0.359, "j_53": -0.27, "j_54": 0.889, "j_55": 1.145, "j_56": -0.767,
}


def q_normalize(q):
    n = np.linalg.norm(q)
    return q / n if n > 1e-12 else np.array([0.0, 0.0, 0.0, 1.0])


def quat_xyzw_to_R(q):
    x, y, z, w = q_normalize(q)
    return np.array([
        [1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
        [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
        [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)],
    ])


def R_to_quat_xyzw(R):
    t = np.trace(R)
    if t > 0:
        s = math.sqrt(t + 1.0) * 2
        w, x, y, z = 0.25 * s, (R[2, 1]-R[1, 2])/s, (R[0, 2]-R[2, 0])/s, (R[1, 0]-R[0, 1])/s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2
        w, x, y, z = (R[2, 1]-R[1, 2])/s, 0.25*s, (R[0, 1]+R[1, 0])/s, (R[0, 2]+R[2, 0])/s
    elif R[1, 1] > R[2, 2]:
        s = math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2
        w, x, y, z = (R[0, 2]-R[2, 0])/s, (R[0, 1]+R[1, 0])/s, 0.25*s, (R[1, 2]+R[2, 1])/s
    else:
        s = math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2
        w, x, y, z = (R[1, 0]-R[0, 1])/s, (R[0, 2]+R[2, 0])/s, (R[1, 2]+R[2, 1])/s, 0.25*s
    return q_normalize(np.array([x, y, z, w]))


def quat_nlerp(qa, qb, a):
    """Normalized lerp toward qb by fraction a (hemisphere-aligned)."""
    if np.dot(qa, qb) < 0.0:
        qb = -qb
    return q_normalize(qa + a * (qb - qa))


class ArmClutch:
    def __init__(self, ee):
        self.ee = ee
        self.R_off = np.eye(3)
        self.t_off = np.zeros(3)
        self.have = False
        # smoothed (filtered) target used by IK
        self.xr_pos: Optional[np.ndarray] = None
        self.xr_R: Optional[np.ndarray] = None
        self.xr_quat: Optional[np.ndarray] = None
        # latest raw sample from the frontend
        self.raw_pos: Optional[np.ndarray] = None
        self.raw_quat: Optional[np.ndarray] = None
        self.last_used: Optional[np.ndarray] = None  # xr_pos processed last cycle


class PinkArmIK(Node):
    def __init__(self):
        super().__init__("pink_arm_ik")
        self.declare_parameter("control_rate_hz", 50.0)
        self.declare_parameter("input_timeout_s", 0.5)
        self.declare_parameter("start_engaged", False)
        self.declare_parameter("pos_cost", 1.0)
        self.declare_parameter("ori_cost", 0.5)
        self.declare_parameter("posture_cost", 2e-2)
        self.declare_parameter("lm_damping", 1.0)
        self.declare_parameter("jump_relatch_m", 0.15)  # XR jump -> re-clutch
        self.declare_parameter("pos_lpf_alpha", 0.35)  # EMA on XR position (0..1)
        self.declare_parameter("ori_lpf_alpha", 0.12)  # EMA on XR rotation (stronger)
        self.declare_parameter("ready_settle_s", 1.6)  # on engage: go to ready, then latch

        self.model = pin.buildModelFromUrdf(URDF)
        self.data = self.model.createData()
        self.q = pin.neutral(self.model)
        # joint name -> (idx_q, nq, idx_v)
        self.jmap = {}
        for jid in range(1, self.model.njoints):
            nm = self.model.names[jid]
            self.jmap[nm] = (self.model.idx_qs[jid], self.model.nqs[jid], self.model.idx_vs[jid])
        for nm in LEFT_JOINTS + RIGHT_JOINTS:
            if nm not in self.jmap:
                raise RuntimeError("joint %s not in model. names=%s" % (nm, list(self.jmap)))
        for f in (LEFT_EE, RIGHT_EE):
            if not self.model.existFrame(f):
                raise RuntimeError("frame %s not in model" % f)

        # nominal q for posture target
        self.q_nominal = pin.neutral(self.model)
        for nm, a in NOMINAL.items():
            self._set_angle(self.q_nominal, nm, a)

        self.left = ArmClutch(LEFT_EE)
        self.right = ArmClutch(RIGHT_EE)
        self.engaged = bool(self.get_parameter("start_engaged").value)
        self.have_js = False
        self.last_input = self.get_clock().now()
        self.ready_until_ns = 0  # while now < this, hold the ready move (no latch)

        pc = float(self.get_parameter("pos_cost").value)
        oc = float(self.get_parameter("ori_cost").value)
        lm = float(self.get_parameter("lm_damping").value)
        self.task_l = FrameTask(LEFT_EE, position_cost=pc, orientation_cost=oc, lm_damping=lm)
        self.task_r = FrameTask(RIGHT_EE, position_cost=pc, orientation_cost=oc, lm_damping=lm)
        self.posture = PostureTask(cost=float(self.get_parameter("posture_cost").value))
        self.posture.set_target(self.q_nominal)

        best = QoSPresetProfiles.SENSOR_DATA.value
        self.pub_l = self.create_publisher(JointTrajectory, "/left_arm_controller/joint_trajectory", 10)
        self.pub_r = self.create_publisher(JointTrajectory, "/right_arm_controller/joint_trajectory", 10)
        self.create_subscription(PoseArray, "/xr_teleop/ee_poses", self._on_ee, best)
        self.create_subscription(JointState, "/joint_states", self._on_js, best)
        self.create_service(SetBool, "~/engage", self._on_engage)

        self.rate = float(self.get_parameter("control_rate_hz").value)
        self.dt = 1.0 / self.rate
        self.timer = self.create_timer(self.dt, self._step)
        self.get_logger().info("pink_arm_ik up. engaged=%s. model nq=%d nv=%d" %
                               (self.engaged, self.model.nq, self.model.nv))

    def _set_angle(self, q, name, angle):
        qi, nq, _ = self.jmap[name]
        if nq == 1:
            q[qi] = angle
        else:  # continuous joint: (cos, sin)
            q[qi] = math.cos(angle)
            q[qi + 1] = math.sin(angle)

    def _get_angle(self, q, name):
        qi, nq, _ = self.jmap[name]
        if nq == 1:
            return float(q[qi])
        return math.atan2(q[qi + 1], q[qi])

    def _on_js(self, msg: JointState):
        # CLOSED-LOOP: resync IK state from the real robot each cycle so it
        # self-corrects (safe with input EMA: the filter smooths the TARGET, not
        # the command output, so it does not fight this feedback).
        idx = {n: i for i, n in enumerate(msg.name)}
        for nm in self.jmap:
            if nm in idx:
                self._set_angle(self.q, nm, msg.position[idx[nm]])
        self.have_js = True

    def _on_ee(self, msg: PoseArray):
        self.last_input = self.get_clock().now()
        if len(msg.poses) >= 1:
            self._store(self.left, msg.poses[0])
        if len(msg.poses) >= 2:
            self._store(self.right, msg.poses[1])

    # OpenXR frame (x-right, y-up, z-back) -> robot base (x-forward, y-left, z-up)
    XR2ROBOT = np.array([[0.0, 0.0, -1.0],
                         [-1.0, 0.0, 0.0],
                         [0.0, 1.0, 0.0]])

    def _store(self, arm: ArmClutch, pose):
        p = np.array([pose.position.x, pose.position.y, pose.position.z])
        R = quat_xyzw_to_R([pose.orientation.x, pose.orientation.y,
                            pose.orientation.z, pose.orientation.w])
        arm.raw_pos = self.XR2ROBOT @ p
        # Orientation must be a proper CHANGE OF BASIS (similarity transform):
        # R_robot = C @ R_xr @ C.T, C = XR2ROBOT. Left-multiplying only (C @ R_xr)
        # cancels in the clutch delta (R0.T@R1) and leaves the controller rotation
        # in XR axes -> rotations about 2 of 3 axes felt inverted. Conjugation
        # re-expresses rotation in robot axes, consistent with the position map.
        arm.raw_quat = R_to_quat_xyzw(self.XR2ROBOT @ R @ self.XR2ROBOT.T)

    def _on_engage(self, req, resp):
        self.engaged = bool(req.data)
        self.left.have = False
        self.right.have = False
        if self.engaged:
            # Always start teleop from the ready pose: command it now, and block
            # clutch latching until the arm has settled there (see _step).
            settle = float(self.get_parameter("ready_settle_s").value)
            self._send_ready(settle)
            self.ready_until_ns = self.get_clock().now().nanoseconds + int(settle * 1e9)
        resp.success = True
        resp.message = "engaged=%s" % self.engaged
        self.get_logger().info(resp.message)
        return resp

    def _send_ready(self, settle):
        """One-shot: move both arms to the NOMINAL (ready) pose over `settle` s."""
        dur = int(max(0.2, settle) * 1e9)
        for names, pub in ((LEFT_JOINTS, self.pub_l), (RIGHT_JOINTS, self.pub_r)):
            traj = JointTrajectory()
            traj.joint_names = list(names)
            pt = JointTrajectoryPoint()
            pt.positions = [float(NOMINAL[n]) for n in names]
            pt.time_from_start = Duration(sec=dur // 1000000000, nanosec=dur % 1000000000)
            traj.points = [pt]
            pub.publish(traj)

    def _cur_ee(self, config, ee):
        T = config.get_transform_frame_to_world(ee)
        return T.rotation.copy(), T.translation.copy()

    def _latch(self, arm, config):
        if arm.xr_pos is None:
            return False
        R_cur, t_cur = self._cur_ee(config, arm.ee)
        # Orientation: track the controller with a fixed offset (no jump at engage).
        arm.R_off = R_cur @ arm.xr_R.T
        # Position: PURE translation offset. The hand follows XR 1:1 in the base
        # frame, so "controller forward" == "hand forward". (The old code rotated
        # the XR delta by R_off, which coupled hand orientation into translation
        # direction and made motion feel reversed / hard to control.)
        arm.t_off = t_cur - arm.xr_pos
        arm.have = True
        return True

    def _target(self, arm):
        R = arm.R_off @ arm.xr_R
        t = arm.xr_pos + arm.t_off
        return pin.SE3(R, t)

    def _update_filter(self, arm, a_pos, a_ori):
        """Low-pass the raw XR sample into the smoothed target (arm.xr_pos/xr_R).
        Position and rotation use separate EMA gains (rotation is noisier -> the
        gripper visibly shakes if the FrameTask chases every jitter). Returns
        False for invalid/idle input (missing or all-zero) so the caller holds
        still and re-clutches when valid data returns."""
        if arm.raw_pos is None or np.linalg.norm(arm.raw_pos) < 1e-6:
            arm.xr_pos = None
            arm.xr_quat = None
            return False
        if arm.xr_pos is None or arm.xr_quat is None:
            arm.xr_pos = arm.raw_pos.copy()
            arm.xr_quat = arm.raw_quat.copy()
        else:
            arm.xr_pos = arm.xr_pos + a_pos * (arm.raw_pos - arm.xr_pos)
            arm.xr_quat = quat_nlerp(arm.xr_quat, arm.raw_quat, a_ori)
        arm.xr_R = quat_xyzw_to_R(arm.xr_quat)
        return True

    def _step(self):
        if not self.have_js:
            return
        dt_in = (self.get_clock().now() - self.last_input).nanoseconds * 1e-9
        active = self.engaged and dt_in < float(self.get_parameter("input_timeout_s").value)
        config = Configuration(self.model, self.data, self.q)
        if not active:
            return
        # Settling to ready pose after engage: hold off IK/latch so the arm
        # reaches ready before it starts tracking (first latch => target=ready).
        if self.get_clock().now().nanoseconds < self.ready_until_ns:
            self.left.have = self.right.have = False
            return
        jump = float(self.get_parameter("jump_relatch_m").value)
        a_pos = float(self.get_parameter("pos_lpf_alpha").value)
        a_ori = float(self.get_parameter("ori_lpf_alpha").value)
        ok = True
        for arm, task in ((self.left, self.task_l), (self.right, self.task_r)):
            # Low-pass the XR stream; invalid/idle (missing or all-zero) -> hold
            # still and re-clutch when valid data returns (no idle->live lurch).
            if not self._update_filter(arm, a_pos, a_ori):
                arm.have = False
                arm.last_used = None
                ok = False
                break
            if not arm.have:
                if not self._latch(arm, config):
                    ok = False
                    break
            elif arm.last_used is not None and \
                    np.linalg.norm(arm.xr_pos - arm.last_used) > jump:
                # Large discontinuity (idle->live, teleport): re-clutch so the
                # target snaps to the current EE instead of commanding the jump.
                self._latch(arm, config)
            arm.last_used = arm.xr_pos.copy()
            task.set_target(self._target(arm))
        if not ok:
            return
        try:
            v = solve_ik(config, [self.task_l, self.task_r, self.posture],
                         self.dt, solver="quadprog")
        except Exception as e:  # noqa: BLE001
            self.get_logger().warn("solve_ik failed: %s" % e, throttle_duration_sec=1.0)
            return
        q_new = pin.integrate(self.model, self.q, v * self.dt)
        # keep the open-loop state inside joint limits (avoids driving bounded
        # joints out of range, which would make solve_ik throw next cycle)
        q_new = np.clip(q_new, self.model.lowerPositionLimit, self.model.upperPositionLimit)
        self.q = q_new
        self._publish(q_new)

    def _publish(self, q):
        for names, pub in ((LEFT_JOINTS, self.pub_l), (RIGHT_JOINTS, self.pub_r)):
            traj = JointTrajectory()
            traj.joint_names = list(names)
            pt = JointTrajectoryPoint()
            pt.positions = [self._get_angle(q, n) for n in names]
            pt.time_from_start = Duration(sec=0, nanosec=int(2 * self.dt * 1e9))
            traj.points = [pt]
            pub.publish(traj)


def main():
    rclpy.init()
    node = PinkArmIK()
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
