#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Play a LabanotationSuite gesture JSON on X1 via JointTrajectoryController.

Works for Isaac (use_isaac stack) and the real robot -- same topics as ready_pose.

Examples (WSL, after sourcing ROS + workspace):

  # dry-run: print keyframes, no ROS
  python3 laban/nodes/laban_player.py \\
    --gesture laban/gestures/Ges01_wavehand.total.json --dry-run

  # play once on Isaac / real stack
  python3 laban/nodes/laban_player.py \\
    --gesture laban/gestures/Ges01_wavehand.total.json --speed 1.0

  # loop
  python3 laban/nodes/laban_player.py \\
    --gesture laban/gestures/Ges01_wavehand.total.json --loop
"""
from __future__ import annotations

import argparse
import os
import sys
import time
from collections import namedtuple
from pathlib import Path

# Allow `python laban/nodes/laban_player.py` without installing a package.
_ROOT = Path(__file__).resolve().parents[1]
if str(_ROOT) not in sys.path:
    sys.path.insert(0, str(_ROOT))

from decoder import Gesture, load_gesture  # noqa: E402
from x1_mapper import (  # noqa: E402
    BASE_LEFT,
    BASE_RIGHT,
    HEAD_BASE,
    HEAD_NAMES,
    LEFT_NAMES,
    RIGHT_NAMES,
    map_head,
    map_limbs_to_arms,
)


def _duration_msg(sec: float):
    from builtin_interfaces.msg import Duration

    sec = max(0.0, float(sec))
    s = int(sec)
    ns = int(round((sec - s) * 1e9))
    if ns >= 1_000_000_000:
        s += 1
        ns -= 1_000_000_000
    return Duration(sec=s, nanosec=ns)


def build_arm_trajectory(names, points_q, times_sec, with_velocities=True):
    """Position waypoints plus finite-difference velocities.

    A position-only trajectory leaves the JTC to guess the velocity profile, which
    shows up on the real robot as a lurch at every waypoint. MoveIt trajectories carry
    velocities (AddTimeOptimalParameterization) and that is why OMPL/cuMotion look smooth.
    """
    from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

    traj = JointTrajectory()
    traj.joint_names = list(names)
    n = len(points_q)
    for i, (q, t) in enumerate(zip(points_q, times_sec)):
        pt = JointTrajectoryPoint()
        pt.positions = [float(v) for v in q]
        if with_velocities:
            if i == 0 or i == n - 1:
                pt.velocities = [0.0] * len(q)
            else:
                dt = float(times_sec[i + 1]) - float(times_sec[i - 1])
                if dt <= 0:
                    pt.velocities = [0.0] * len(q)
                else:
                    pt.velocities = [
                        (float(nxt) - float(prv)) / dt
                        for prv, nxt in zip(points_q[i - 1], points_q[i + 1])
                    ]
        pt.time_from_start = _duration_msg(t)
        traj.points.append(pt)
    return traj


# Joint velocity ceiling for playback. joint_limits.yaml allows 3.14 rad/s but also
# scales requests to 0.1 by default, so MoveIt effectively moves far slower than that.
# Gestures only need to look natural, and a low ceiling is what keeps the torso from
# rocking, so cap well below the hardware limit and stretch time when a segment exceeds it.
MAX_JOINT_VEL = 1.0  # rad/s
# The head has its own ceiling: its two joints carry far less inertia than an arm,
# so holding them to the arm limit makes every nod look laboured, but the URDF
# declares them "continuous" with no limits of their own, so an unchecked keyframe
# pair can ask for any rate at all (shake head swings 0.7 rad in 500 ms).
MAX_HEAD_VEL = float(os.environ.get("X1_MAX_HEAD_VEL", "1.5"))  # rad/s
# Shortest lead-in worth publishing. Anything less reads as a snap even when the
# arms are already sitting on the opening pose.
MIN_APPROACH_S = 0.4
# Amplitudes to fall back through when full-amplitude IK leaves a joint limit.
IK_GAIN_FALLBACKS = (0.85, 0.7, 0.55, 0.4)


def _smoothstep(s: float) -> float:
    """Ease in/out so each keyframe segment starts and ends at zero velocity."""
    s = min(1.0, max(0.0, s))
    return s * s * (3.0 - 2.0 * s)


def keyframe_poses(gesture: Gesture, mapper_name: str = "lut", ik_gain=None):
    """Joint pose per Laban keyframe: (time_ms, left_q, right_q, head_q).

    With the IK mapper, retries the whole gesture at reduced amplitude when full
    amplitude is unreachable.  Retrying per keyframe does not work: the branch is
    chosen at the first raised keyframe and every later solve inherits it, so the
    gain has to be constant across the gesture.
    """
    if mapper_name != "ik":
        return _keyframe_poses_once(gesture, mapper_name, ik_gain)

    requested = ik_gain if ik_gain is not None else float(
        os.environ.get("LABAN_IK_GAIN", "1.00")
    )
    gains = [requested] + [g for g in IK_GAIN_FALLBACKS if g < requested]
    last_error = None
    for gain in gains:
        try:
            return _keyframe_poses_once(gesture, mapper_name, gain)
        except RuntimeError as exc:
            last_error = exc
            print("[laban] IK gain %.2f unusable: %s" % (gain, exc))
    raise RuntimeError(
        "ik mapper failed at every amplitude down to %.2f: %s" % (gains[-1], last_error)
    )


def _keyframe_poses_once(gesture: Gesture, mapper_name: str, ik_gain):
    if mapper_name == "ik":
        try:
            from x1_ik_mapper import X1PinkLabanMapper
        except ImportError as exc:
            raise RuntimeError(
                "IK mapper needs Pinocchio + Pink. Run through laban/run_player.sh "
                "(it selects ~/pink_venv/bin/python3) or set X1_PYTHON."
            ) from exc
        ik_mapper = X1PinkLabanMapper(retarget_gain=ik_gain)
        map_limbs = ik_mapper.map_limbs
        print("[laban] IK retarget gain = %.2f" % ik_mapper.retarget_gain)
    elif mapper_name == "lut":
        map_limbs = map_limbs_to_arms
    else:
        raise ValueError("unknown mapper: %s" % mapper_name)

    poses = []
    for kf in gesture.keyframes:
        try:
            left_q, right_q = map_limbs(kf.limbs)
        except Exception as exc:
            raise RuntimeError(
                "%s mapper failed at keyframe %s (t=%.0f ms): %s"
                % (mapper_name, kf.name, kf.time_ms, exc)
            ) from exc
        poses.append(
            (kf.time_ms, left_q, right_q, map_head(kf.head, kf.rotation))
        )
    if mapper_name == "ik":
        names = list(LEFT_NAMES) + list(RIGHT_NAMES)
        for frame, prev, cur in zip(gesture.keyframes[1:], poses, poses[1:]):
            deltas = [
                abs(b - a)
                for a, b in zip(prev[1] + prev[2], cur[1] + cur[2])
            ]
            largest = max(deltas)
            if largest > 1.0:
                index = deltas.index(largest)
                print(
                    "[laban] IK keyframe jump %s: %.3f rad (%s)"
                    % (frame.name, largest, names[index])
                )
            # Rest-to-raised is a legitimate ~1.5 rad step (the sampler stretches it
            # to respect MAX_JOINT_VEL).  An IK branch flip, the failure this guard
            # exists for, showed up as 2.4-5.0 rad, so reject only above that.
            if largest > 2.2:
                raise RuntimeError(
                    "unsafe IK discontinuity at %s: %s changes %.3f rad "
                    "(likely an IK branch flip)" % (frame.name, names[index], largest)
                )
    return poses


LeadIn = namedtuple(
    "LeadIn", "times left_qs right_qs head_qs ramp gap needed"
)


def apply_lead_in(cur_left, cur_right, times, left_qs, right_qs, head_qs, approach_s,
                  cur_head=None):
    """Prepend the measured pose so the arms ease into the opening pose.

    The first gesture point sits at t=0, and a JTC point at time_from_start=0 means
    "be there instantly" -- a violent snap from wherever the arms are. Spending a
    flat --approach regardless of distance is the other extreme, and it is what
    makes a short spoken reply finish before the gesture has begun, so the ramp is
    what MAX_JOINT_VEL requires for the actual distance, with --approach as the cap.

    cur_head matters when one gesture supersedes another: nearly every gesture in
    the library opens on head Forward/Normal, so without the measured head pose the
    first point would snap the head back to centre from wherever the last gesture
    left it. Absent (not every stack publishes head states) it falls back to
    starting at the gesture's own opening head pose, which is the old behaviour.
    """
    start_head = list(cur_head) if cur_head is not None else list(head_qs[0])
    needed = _segment_duration(
        (0.0, cur_left, cur_right, start_head),
        (0.0, left_qs[0], right_qs[0], head_qs[0]),
        MIN_APPROACH_S,
    )
    ramp = min(approach_s, needed)
    gap = max(
        (abs(b - a) for a, b in zip(cur_left + cur_right, left_qs[0] + right_qs[0])),
        default=0.0,
    )
    return LeadIn(
        times=[0.0] + [t + ramp for t in times],
        left_qs=[cur_left] + left_qs,
        right_qs=[cur_right] + right_qs,
        head_qs=[start_head] + head_qs,
        ramp=ramp,
        gap=gap,
        needed=needed,
    )


LeadOut = namedtuple("LeadOut", "times left_qs right_qs head_qs hold ramp gap")

# How long the closing pose is held before the arms open back up. Returning the
# instant the last keyframe lands throws away the pose the gesture was building to.
RETURN_HOLD_S = float(os.environ.get("X1_RETURN_HOLD", "0.6"))
MIN_RETURN_S = 0.6


def apply_lead_out(times, left_qs, right_qs, head_qs, dt_s, include_head=True,
                   hold_s=RETURN_HOLD_S):
    """Append the ready pose so the arms do not park in the closing pose.

    A gesture's last keyframe is chosen for expression, not for resting: several
    of the library's two-handed gestures end with the grippers a few millimetres
    apart, and the JTC holds that until something else arrives. Measured on cam,
    the reference pose left behind by thanks.json has 4 mm of clearance between
    the hands versus 231 mm at the ready pose, so every following gesture also
    started from there. Sampled at dt like the rest of the timeline so the Isaac
    mirror, which interpolates between samples, follows it too.
    """
    ready_left = list(BASE_LEFT)
    ready_right = list(BASE_RIGHT)
    ready_head = list(HEAD_BASE) if include_head else list(head_qs[-1])

    last_left = list(left_qs[-1])
    last_right = list(right_qs[-1])
    last_head = list(head_qs[-1])

    gap = max(
        (abs(b - a) for a, b in zip(last_left + last_right, ready_left + ready_right)),
        default=0.0,
    )
    ramp = _segment_duration(
        (0.0, last_left, last_right, last_head),
        (0.0, ready_left, ready_right, ready_head),
        MIN_RETURN_S,
    )

    out_t = list(times)
    out_l = [list(q) for q in left_qs]
    out_r = [list(q) for q in right_qs]
    out_h = [list(q) for q in head_qs]

    t = out_t[-1]
    if hold_s > 0:
        t += hold_s
        out_t.append(t)
        out_l.append(list(last_left))
        out_r.append(list(last_right))
        out_h.append(list(last_head))

    steps = max(1, int(round(ramp / dt_s)))
    for k in range(1, steps + 1):
        s = _smoothstep(k / float(steps))
        out_t.append(t + k * dt_s)
        out_l.append([a + (b - a) * s for a, b in zip(last_left, ready_left)])
        out_r.append([a + (b - a) * s for a, b in zip(last_right, ready_right)])
        out_h.append([a + (b - a) * s for a, b in zip(last_head, ready_head)])

    return LeadOut(
        times=out_t,
        left_qs=out_l,
        right_qs=out_r,
        head_qs=out_h,
        hold=hold_s,
        ramp=steps * dt_s,
        gap=gap,
    )


def _segment_duration(prev_pose, next_pose, nominal_s: float,
                      head_vel: float | None = MAX_HEAD_VEL) -> float:
    """Stretch a segment until no joint has to exceed its velocity ceiling.

    head_vel=None ignores the head, which is what a caller playing with --no-head
    wants: the head poses are still in the samples, and letting them stretch the
    timeline would slow a gesture down for motion that is never commanded.
    """
    span = 0.0
    for a, b in zip(prev_pose[1] + prev_pose[2], next_pose[1] + next_pose[2]):
        span = max(span, abs(b - a))
    needed = span / MAX_JOINT_VEL if MAX_JOINT_VEL > 0 else 0.0

    if head_vel:
        head_span = 0.0
        for a, b in zip(prev_pose[3], next_pose[3]):
            head_span = max(head_span, abs(b - a))
        needed = max(needed, head_span / head_vel)

    # smoothstep peaks at 1.5x the average rate, so allow for that headroom.
    return max(nominal_s, needed * 1.5)


def resample_gesture(
    gesture: Gesture,
    dt_s: float,
    speed: float,
    mapper_name: str = "lut",
    ik_gain=None,
    include_head: bool = True,
):
    """Uniform time samples -> list of (t_sec, left_q, right_q, head_q).

    Interpolation happens in joint space between keyframe poses with smoothstep easing,
    which is what makes the motion continuous instead of stepping at every keyframe.
    """
    if speed <= 0:
        raise ValueError("speed must be > 0")

    poses = keyframe_poses(gesture, mapper_name=mapper_name, ik_gain=ik_gain)
    return resample_from_poses(poses, dt_s, speed, include_head=include_head)


def resample_from_poses(poses, dt_s: float, speed: float, include_head: bool = True):
    """Sampling half of resample_gesture, for callers that already have the poses.

    The daemon caches solved poses across gestures, so it needs the interpolation
    without paying for the IK again.
    """
    if speed <= 0:
        raise ValueError("speed must be > 0")
    if len(poses) == 1:
        _, left_q, right_q, head_q = poses[0]
        return [(0.0, left_q, right_q, head_q), (dt_s, left_q, right_q, head_q)]

    head_vel = MAX_HEAD_VEL if include_head else None
    samples = []
    t = 0.0
    for prev, nxt in zip(poses, poses[1:]):
        nominal = (nxt[0] - prev[0]) / 1000.0 / speed
        seg = _segment_duration(prev, nxt, max(nominal, dt_s), head_vel=head_vel)
        steps = max(1, int(round(seg / dt_s)))
        for k in range(steps):
            s = _smoothstep(k / float(steps))
            left_q = [a + (b - a) * s for a, b in zip(prev[1], nxt[1])]
            right_q = [a + (b - a) * s for a, b in zip(prev[2], nxt[2])]
            head_q = [a + (b - a) * s for a, b in zip(prev[3], nxt[3])]
            samples.append((t + k * dt_s, left_q, right_q, head_q))
        t += steps * dt_s

    last = poses[-1]
    samples.append((t, list(last[1]), list(last[2]), list(last[3])))
    return samples


def dry_run(gesture: Gesture, samples) -> None:
    print("gesture: %s" % gesture.name)
    print("source:  %s" % gesture.source_path)
    print(
        "frames:  %d  t=[%.0f .. %.0f] ms  duration=%.0f ms"
        % (len(gesture.keyframes), gesture.start_ms, gesture.end_ms, gesture.duration_ms)
    )
    print("samples: %d" % len(samples))
    print("--- keyframes (symbols) ---")
    for kf in gesture.keyframes:
        re = kf.limbs.get("right elbow")
        rw = kf.limbs.get("right wrist")
        le = kf.limbs.get("left elbow")
        lw = kf.limbs.get("left wrist")
        print(
            "  %-12s t=%7.0fms  RE=%s/%s  RW=%s/%s  LE=%s/%s  LW=%s/%s"
            % (
                kf.name,
                kf.time_ms,
                re.direction if re else "?",
                re.level if re else "?",
                rw.direction if rw else "?",
                rw.level if rw else "?",
                le.direction if le else "?",
                le.level if le else "?",
                lw.direction if lw else "?",
                lw.level if lw else "?",
            )
        )
    print("--- trajectory samples (first/last) ---")
    for idx in (0, len(samples) - 1):
        t, lq, rq, hq = samples[idx]
        print("  [%d] t=%.2fs" % (idx, t))
        print("       L=%s" % (["%.3f" % v for v in lq],))
        print("       R=%s" % (["%.3f" % v for v in rq],))
        print("       H=%s" % (["%.3f" % v for v in hq],))


# isaac_x1_ros2.py wires ROS2SubscribeJointState -> IsaacArticulationController, so
# the Isaac twin takes joint *positions*, not trajectories: it never subscribes to
# /left_arm_controller/joint_trajectory. Streaming the same samples here lets one
# process drive the real controllers and the twin off a single timeline.
ISAAC_CMD_TOPIC = "/isaac_joint_commands"
ISAAC_CMD_HZ = 50.0
# Isaac's participant is a late joiner for every player process, and matching it
# takes anywhere from ~1s to over 5s here, so the wait is tunable.
ISAAC_DISCOVERY_S = float(os.environ.get("X1_ISAAC_WAIT", "3.0"))
# When the robot is being driven too, only this much of that wait is spent before
# publishing; a twin that shows up later joins the stream in progress.
ISAAC_GRACE_S = float(os.environ.get("X1_ISAAC_GRACE", "0.8"))

# No sign correction here on purpose. besek_x1.usd once had j_6, j_7 and j_51..j_56
# authored with axes opposite to pkg_robot_model.urdf, which is what
# besek_x1_patch/joint_state_mirror_relay.py compensated for. fix_usd_axes.py has
# since flipped those joints inside the USD itself, so joint values now transfer
# 1:1 and negating them again reverses the right arm.


def _interpolate(times, series, t):
    """Linear interpolation of a list of joint vectors sampled at `times`."""
    if t <= times[0]:
        return list(series[0])
    if t >= times[-1]:
        return list(series[-1])
    hi = 1
    while hi < len(times) and times[hi] < t:
        hi += 1
    lo = hi - 1
    span = times[hi] - times[lo]
    frac = 0.0 if span <= 0 else (t - times[lo]) / span
    return [a + (b - a) * frac for a, b in zip(series[lo], series[hi])]


def _isaac_command(node, pub, times, left_qs, right_qs, head_qs, include_head, t):
    from sensor_msgs.msg import JointState

    names = list(LEFT_NAMES) + list(RIGHT_NAMES)
    positions = _interpolate(times, left_qs, t) + _interpolate(times, right_qs, t)
    if include_head:
        names += list(HEAD_NAMES)
        positions += _interpolate(times, head_qs, t)

    msg = JointState()
    msg.header.stamp = node.get_clock().now().to_msg()
    msg.name = names
    msg.position = positions
    pub.publish(msg)


def wait_for_subscribers(node, pubs, timeout_s: float):
    """Return {topic: count}. A JTC that is not discovered accepts nothing silently."""
    import rclpy

    deadline = time.time() + timeout_s
    counts = {}
    while True:
        counts = {p.topic_name: p.get_subscription_count() for p in pubs}
        if all(c > 0 for c in counts.values()) or time.time() >= deadline:
            return counts
        rclpy.spin_once(node, timeout_sec=0.1)


def _capture_positions(node, names, timeout_s):
    """Block until one /joint_states with all `names` arrives; return {name: pos} or None."""
    import rclpy
    from sensor_msgs.msg import JointState

    latest = {}

    def cb(msg):
        for name, pos in zip(msg.name, msg.position):
            if name in names:
                latest[name] = pos

    sub = node.create_subscription(JointState, "/joint_states", cb, 10)
    deadline = time.time() + timeout_s
    while time.time() < deadline and not all(n in latest for n in names):
        rclpy.spin_once(node, timeout_sec=0.1)
    node.destroy_subscription(sub)
    if all(n in latest for n in names):
        return latest
    return None


def play_once(
    samples,
    include_head: bool,
    settle_s: float,
    check_only: bool = False,
    approach_s: float = 3.0,
    isaac_mirror: bool = False,
    isaac_only: bool = False,
    return_ready: bool = True,
    dt_s: float = 0.1,
) -> int:
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import JointState
    from trajectory_msgs.msg import JointTrajectory

    times = [s[0] for s in samples]
    left_qs = [s[1] for s in samples]
    right_qs = [s[2] for s in samples]
    head_qs = [s[3] for s in samples]

    rclpy.init()
    node = Node("laban_player")
    pub_l = node.create_publisher(JointTrajectory, "/left_arm_controller/joint_trajectory", 10)
    pub_r = node.create_publisher(JointTrajectory, "/right_arm_controller/joint_trajectory", 10)
    pub_h = None
    if include_head:
        pub_h = node.create_publisher(JointTrajectory, "/head_controller/joint_trajectory", 10)

    # Watching /joint_states makes "did anything actually move?" answerable: a trajectory
    # can be delivered and accepted and still leave the robot where it was.
    watched = list(LEFT_NAMES) + list(RIGHT_NAMES)
    seen: dict = {}

    def on_joint_states(msg):
        for name, pos in zip(msg.name, msg.position):
            if name in watched:
                lo, hi = seen.get(name, (pos, pos))
                seen[name] = (min(lo, pos), max(hi, pos))

    node.create_subscription(JointState, "/joint_states", on_joint_states, 10)

    pub_isaac = None
    if isaac_mirror:
        pub_isaac = node.create_publisher(JointState, ISAAC_CMD_TOPIC, 10)

    pubs = [pub_l, pub_r] + ([pub_h] if pub_h is not None else [])
    counts = wait_for_subscribers(node, pubs, max(settle_s, 3.0))
    for topic, count in counts.items():
        node.get_logger().info("%s: %d subscriber(s)" % (topic, count))

    missing = [t for t, c in counts.items() if c == 0]
    if missing and isaac_only:
        node.get_logger().info(
            "isaac-only: ignoring %s with no subscriber" % ", ".join(missing)
        )
        missing = []
    if missing:
        node.get_logger().error(
            "no controller subscribed to %s -- the trajectory would go nowhere. "
            "Usually a DDS mismatch: the real stack needs X1_DDS=real (fastdds_mtu.xml + "
            "SUBNET), Isaac needs X1_DDS=isaac (LOCALHOST, no profile)." % ", ".join(missing)
        )
        node.destroy_node()
        rclpy.shutdown()
        return 1

    if pub_isaac is not None:
        # wait_for_subscribers returns as soon as the arm controllers are matched,
        # which is well before Isaac's participant shows up. Waiting the full
        # discovery window here would delay the robot too, so only a run that has
        # nothing else to drive waits it out; otherwise the stream starts on time
        # and a late twin joins mid-gesture.
        wait_s = ISAAC_DISCOVERY_S if isaac_only else ISAAC_GRACE_S
        isaac_deadline = time.time() + wait_s
        while pub_isaac.get_subscription_count() == 0 and time.time() < isaac_deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
        if pub_isaac.get_subscription_count() > 0:
            node.get_logger().info("%s: Isaac twin discovered" % ISAAC_CMD_TOPIC)
        else:
            node.get_logger().warn(
                "no subscriber on %s after %.1fs; continuing. If the twin never "
                "joins, start it with run_isaac.bat isaac_x1_ros2.py."
                % (ISAAC_CMD_TOPIC, wait_s)
            )

    if check_only:
        node.get_logger().info("check only: all controllers discovered, nothing published")
        node.destroy_node()
        rclpy.shutdown()
        return 0

    if approach_s > 0:
        cur = _capture_positions(node, watched, timeout_s=3.0)
        if cur is None:
            node.get_logger().warn(
                "no /joint_states for lead-in; playing without approach ramp (may snap)"
            )
        else:
            # _capture_positions stores every name each message carries, so the head
            # is already here when the stack publishes it.
            cur_head = (
                [cur[n] for n in HEAD_NAMES]
                if include_head and all(n in cur for n in HEAD_NAMES)
                else None
            )
            lead = apply_lead_in(
                [cur[n] for n in LEFT_NAMES],
                [cur[n] for n in RIGHT_NAMES],
                times, left_qs, right_qs, head_qs, approach_s,
                cur_head=cur_head,
            )
            node.get_logger().info(
                "lead-in %.2fs for %.2f rad (needs %.2fs, cap %.2fs)"
                % (lead.ramp, lead.gap, lead.needed, approach_s)
            )
            times, left_qs = lead.times, lead.left_qs
            right_qs, head_qs = lead.right_qs, lead.head_qs

    if return_ready:
        out = apply_lead_out(
            times, left_qs, right_qs, head_qs, dt_s, include_head=include_head
        )
        node.get_logger().info(
            "return to ready: hold %.2fs then %.2fs for %.2f rad"
            % (out.hold, out.ramp, out.gap)
        )
        times, left_qs = out.times, out.left_qs
        right_qs, head_qs = out.right_qs, out.head_qs

    left_traj = build_arm_trajectory(LEFT_NAMES, left_qs, times)
    right_traj = build_arm_trajectory(RIGHT_NAMES, right_qs, times)
    head_traj = build_arm_trajectory(HEAD_NAMES, head_qs, times) if include_head else None

    # The JTC starts executing the moment it accepts the first message, so the
    # timeline the twin has to follow starts here, not after the repeat loop.
    t0 = time.time()
    for _ in range(5):
        if not isaac_only:
            pub_l.publish(left_traj)
            pub_r.publish(right_traj)
            if pub_h is not None and head_traj is not None:
                pub_h.publish(head_traj)
        if pub_isaac is not None:
            _isaac_command(
                node, pub_isaac, times, left_qs, right_qs, head_qs,
                include_head, time.time() - t0,
            )
        rclpy.spin_once(node, timeout_sec=0.05)
        time.sleep(0.05)

    total = times[-1] if times else 0.0
    if isaac_only:
        node.get_logger().info(
            "streaming Laban gesture (%d pts, %.2fs) -> isaac twin only"
            % (len(samples), total)
        )
    else:
        node.get_logger().info(
            "published Laban trajectory (%d pts, %.2fs) -> left/right_arm_controller%s%s"
            % (
                len(samples),
                total,
                " + head_controller" if include_head else "",
                " + isaac twin" if pub_isaac is not None else "",
            )
        )

    seen.clear()
    end = t0 + total + 0.5
    period = 1.0 / ISAAC_CMD_HZ
    next_cmd = time.time()
    isaac_seen = pub_isaac is not None and pub_isaac.get_subscription_count() > 0
    while time.time() < end:
        now = time.time()
        if pub_isaac is not None and not isaac_seen and pub_isaac.get_subscription_count():
            isaac_seen = True
            node.get_logger().info(
                "Isaac twin joined %.1fs into the gesture" % (now - t0)
            )
        if pub_isaac is not None and now >= next_cmd:
            _isaac_command(
                node, pub_isaac, times, left_qs, right_qs, head_qs,
                include_head, now - t0,
            )
            next_cmd = now + period
        rclpy.spin_once(node, timeout_sec=0.02 if pub_isaac is not None else 0.05)

    if seen:
        moved = {n: hi - lo for n, (lo, hi) in seen.items()}
        biggest = sorted(moved.items(), key=lambda kv: -kv[1])[:4]
        node.get_logger().info(
            "measured travel (rad): " + ", ".join("%s=%.3f" % kv for kv in biggest)
        )
        if max(moved.values()) < 0.01:
            node.get_logger().warn(
                "joints barely moved -- controller took the trajectory but the robot "
                "stayed put (check torque / controller is active / another node is "
                "streaming setpoints to the same controller)"
            )
    else:
        node.get_logger().warn("no /joint_states received; cannot confirm motion")

    node.destroy_node()
    rclpy.shutdown()
    return 0


def resolve_gesture_path(arg: str) -> Path:
    p = Path(arg)
    if p.is_file():
        return p
    tried = []
    for cand in (_ROOT / "gestures" / Path(arg).name, _ROOT / arg):
        if cand.is_file():
            return cand
        tried.append(str(cand))
    raise FileNotFoundError("gesture not found: %s (also tried %s)" % (arg, ", ".join(tried)))


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description="X1 Laban gesture player (Isaac / real)")
    parser.add_argument(
        "--gesture",
        default="Ges01_wavehand.total.json",
        help="path or filename under laban/gestures/",
    )
    parser.add_argument("--speed", type=float, default=1.0, help="playback speed (>1 = faster)")
    parser.add_argument("--dt", type=float, default=0.1, help="sample period seconds")
    parser.add_argument("--loop", action="store_true", help="repeat until Ctrl-C")
    parser.add_argument("--dry-run", action="store_true", help="print plan only, no ROS")
    parser.add_argument("--no-head", action="store_true", help="do not command head_controller")
    parser.add_argument(
        "--mapper",
        choices=("lut", "ik"),
        default="lut",
        help="symbol lookup table or URDF-based Pink geometric IK",
    )
    parser.add_argument(
        "--ik-gain",
        type=float,
        default=None,
        help="IK amplitude: 1.0 follows the Laban direction fully, lower stays "
        "closer to the ready pose (default 1.0, or LABAN_IK_GAIN)",
    )
    parser.add_argument(
        "--allow-ik-real",
        action="store_true",
        help="allow the experimental IK mapper on real hardware after Isaac validation",
    )
    parser.add_argument("--settle", type=float, default=1.0, help="seconds to wait before publish")
    parser.add_argument(
        "--approach",
        type=float,
        default=3.0,
        help="seconds to ease from the current pose into the first gesture pose (0 = off)",
    )
    parser.add_argument(
        "--check",
        action="store_true",
        help="report controller subscriber counts and exit without moving anything",
    )
    parser.add_argument(
        "--isaac-mirror",
        action="store_true",
        help="also stream the same samples to %s so the Isaac twin performs the "
        "gesture alongside the robot (needs run_isaac.bat isaac_x1_ros2.py)"
        % ISAAC_CMD_TOPIC,
    )
    parser.add_argument(
        "--no-return",
        action="store_true",
        help="leave the arms in the gesture's closing pose instead of opening them "
        "back to the ready pose (BASE_LEFT/BASE_RIGHT) when it ends",
    )
    parser.add_argument(
        "--isaac-only",
        action="store_true",
        help="drive the Isaac twin and nothing else: publish no trajectory, so the "
        "mirror can be validated while the real robot stays still",
    )
    args = parser.parse_args(argv)

    if (
        args.mapper == "ik"
        and os.environ.get("X1_DDS", "isaac").lower() == "real"
        and not args.allow_ik_real
        and not args.dry_run
    ):
        parser.error(
            "--mapper ik is currently simulation-first. Validate this gesture in "
            "Isaac, then add --allow-ik-real explicitly."
        )

    os.environ.setdefault("ROS_DOMAIN_ID", "0")
    os.environ.setdefault("RMW_IMPLEMENTATION", "rmw_fastrtps_cpp")
    os.environ.setdefault("ROS_AUTOMATIC_DISCOVERY_RANGE", "LOCALHOST")

    path = resolve_gesture_path(args.gesture)
    gesture = load_gesture(path)
    include_head = not args.no_head
    samples = resample_gesture(
        gesture,
        dt_s=args.dt,
        speed=args.speed,
        mapper_name=args.mapper,
        ik_gain=args.ik_gain,
        include_head=include_head,
    )

    if args.dry_run:
        dry_run(gesture, samples)
        return 0

    try:
        if args.check:
            return play_once(
                samples,
                include_head=include_head,
                settle_s=args.settle,
                check_only=True,
                approach_s=args.approach,
                isaac_mirror=args.isaac_mirror or args.isaac_only,
                isaac_only=args.isaac_only,
            )
        if args.loop:
            while True:
                rc = play_once(
                    samples,
                    include_head=include_head,
                    settle_s=args.settle,
                    approach_s=args.approach,
                    isaac_mirror=args.isaac_mirror or args.isaac_only,
                    isaac_only=args.isaac_only,
                    # Looping back to the ready pose between repetitions would break
                    # the cycle the gesture was authored as.
                    return_ready=False,
                    dt_s=args.dt,
                )
                if rc != 0:
                    return rc
        return play_once(
            samples,
            include_head=include_head,
            settle_s=args.settle,
            approach_s=args.approach,
            isaac_mirror=args.isaac_mirror or args.isaac_only,
            isaac_only=args.isaac_only,
            return_ready=not args.no_return,
            dt_s=args.dt,
        )
    except KeyboardInterrupt:
        print("stopped")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
