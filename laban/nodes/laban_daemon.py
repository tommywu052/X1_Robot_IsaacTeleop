#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Resident Laban player: keeps DDS and the IK model warm between gestures.

Spawning laban_player.py per gesture costs 2.3s before anything moves (measured
with --check on cam): bash and the venv python, loading Pinocchio, rclpy init, and
rediscovering the controllers every time. In conversation that delay lands the
gesture after a short reply has already finished. This process pays it once.

Requests arrive as one JSON object per line on a Unix socket, and the reply carries
the timing the caller needs to line the gesture up with speech:

  {"cmd": "play", "gesture": "/path/to/hello.json", "speed": 1.0}
    -> {"ok": true, "gesture": "hello", "ramp_s": 0.9, "duration_s": 12.9, ...}
  {"cmd": "stop"}     cancel by publishing an empty trajectory (killing a player
                      process does not stop the robot: the JTC already has the
                      whole trajectory)
  {"cmd": "status"}
  {"cmd": "ping"}

Start it through run_player.sh so it inherits the same DDS setup as a one-shot
play:

  X1_DDS=both bash laban/run_player.sh --daemon
"""
from __future__ import annotations

import argparse
import json
import os
import socket
import sys
import time
from pathlib import Path

_HERE = Path(__file__).resolve().parent
_ROOT = _HERE.parent
for _p in (str(_ROOT), str(_HERE)):
    if _p not in sys.path:
        sys.path.insert(0, _p)

import laban_player as lp  # noqa: E402
from decoder import load_gesture  # noqa: E402
from gesture_policy import GesturePolicy  # noqa: E402
from x1_mapper import HEAD_NAMES, LEFT_NAMES, RIGHT_NAMES, clamp_head  # noqa: E402

DEFAULT_SOCKET = "/tmp/x1_laban_%s.sock"
SPIN_S = 0.02


def socket_path(mode: str) -> str:
    """One socket per DDS mode, so a real and an isaac daemon can coexist."""
    return os.environ.get("X1_LABAN_SOCKET") or DEFAULT_SOCKET % mode


def already_running(path: str) -> bool:
    """True if a live daemon owns this socket.

    Two daemons on one set of controllers would each publish a full trajectory and
    fight over the arms, so starting the second one has to fail rather than take
    the socket over. A socket left behind by a crash answers nothing and is removed.
    """
    if not os.path.exists(path):
        return False
    try:
        with socket.socket(socket.AF_UNIX, socket.SOCK_STREAM) as sock:
            sock.settimeout(2.0)
            sock.connect(path)
            sock.sendall(b'{"cmd": "ping"}\n')
            return bool(json.loads(sock.recv(4096).decode("utf-8")).get("ok"))
    except (OSError, ValueError):
        os.unlink(path)
        return False


def _symbol_key(gesture) -> tuple:
    """Cache key covering everything the IK solve depends on.

    Deliberately excludes keyframe times: a gesture stretched to fit an utterance
    has the same poses, so the stretched copies the chat bridge writes per reply
    all hit the same entry.
    """
    frames = []
    for kf in gesture.keyframes:
        limbs = tuple(
            (name, limb.direction, limb.level)
            for name, limb in sorted(kf.limbs.items())
        )
        frames.append((limbs, kf.head, kf.rotation))
    return tuple(frames)


class LabanDaemon:
    def __init__(self, args):
        import rclpy
        from rclpy.node import Node
        from sensor_msgs.msg import JointState
        from trajectory_msgs.msg import JointTrajectory

        self.args = args
        self.rclpy = rclpy
        self.JointTrajectory = JointTrajectory

        rclpy.init()
        self.node = Node("laban_daemon")
        self.log = self.node.get_logger()

        self.pub_l = self.node.create_publisher(
            JointTrajectory, "/left_arm_controller/joint_trajectory", 10
        )
        self.pub_r = self.node.create_publisher(
            JointTrajectory, "/right_arm_controller/joint_trajectory", 10
        )
        self.pub_h = self.node.create_publisher(
            JointTrajectory, "/head_controller/joint_trajectory", 10
        )
        self.pub_isaac = (
            self.node.create_publisher(JointState, lp.ISAAC_CMD_TOPIC, 10)
            if args.isaac_mirror
            else None
        )

        self.watched = list(LEFT_NAMES) + list(RIGHT_NAMES)
        # Head states are recorded but stay out of the readiness gate below: not every
        # stack publishes them, and a missing head must not cost the arms their lead-in.
        self.tracked = self.watched + list(HEAD_NAMES)
        self.current = {}
        self.node.create_subscription(
            JointState, "/joint_states", self._on_joint_states, 10
        )

        # Warm state: the whole point of the daemon.
        self._mappers = {}
        self._poses = {}
        self._play = None  # active timeline for the Isaac mirror
        self._isaac_seen = False
        self._quit = False
        self.played = 0
        self.refused = 0

        # Every entry point ends here, and a socket request carries an absolute
        # path rather than a library name, so a filter applied where the catalogue
        # is built cannot see it. This is the only place a hard block covers every
        # caller.
        self.policy = GesturePolicy(profile="chat", warn=self.log.warning)

    # ------------------------------------------------------------------ ROS
    def _on_joint_states(self, msg):
        for name, pos in zip(msg.name, msg.position):
            if name in self.tracked:
                self.current[name] = pos

    def warmup(self) -> None:
        """Discover controllers and load the IK model before the first request."""
        deadline = time.time() + self.args.settle
        pubs = [self.pub_l, self.pub_r]
        while time.time() < deadline:
            if all(p.get_subscription_count() for p in pubs):
                break
            self.rclpy.spin_once(self.node, timeout_sec=0.1)
        for topic, pub in (
            ("/left_arm_controller/joint_trajectory", self.pub_l),
            ("/right_arm_controller/joint_trajectory", self.pub_r),
            ("/head_controller/joint_trajectory", self.pub_h),
        ):
            self.log.info("%s: %d subscriber(s)" % (topic, pub.get_subscription_count()))
        if self.pub_isaac is not None:
            while time.time() < deadline and not self.pub_isaac.get_subscription_count():
                self.rclpy.spin_once(self.node, timeout_sec=0.1)
            self._isaac_seen = bool(self.pub_isaac.get_subscription_count())
            self.log.info(
                "%s: %s"
                % (
                    lp.ISAAC_CMD_TOPIC,
                    "twin discovered" if self._isaac_seen else "no twin yet",
                )
            )

        if self.args.mapper == "ik":
            started = time.time()
            self._mapper_for(float(os.environ.get("LABAN_IK_GAIN", "1.00")))
            self.log.info("IK model loaded in %.2fs" % (time.time() - started))

        while not all(n in self.current for n in self.watched) and time.time() < deadline + 2:
            self.rclpy.spin_once(self.node, timeout_sec=0.1)
        self.log.info(
            "warm: /joint_states %s"
            % ("ok" if all(n in self.current for n in self.watched) else "missing")
        )

    def _mapper_for(self, gain: float):
        key = round(gain, 3)
        if key not in self._mappers:
            from x1_ik_mapper import X1PinkLabanMapper

            self._mappers[key] = X1PinkLabanMapper(retarget_gain=key)
        return self._mappers[key]

    # -------------------------------------------------------------- planning
    def _keyframe_poses(self, gesture):
        """Cached IK. Returns poses with this gesture's own keyframe times."""
        key = (_symbol_key(gesture), self.args.mapper)
        cached = self._poses.get(key)
        if cached is None:
            if self.args.mapper == "ik":
                cached = self._solve_ik(gesture)
            else:
                cached = [
                    (p[1], p[2], p[3])
                    for p in lp._keyframe_poses_once(gesture, self.args.mapper, None)
                ]
            self._poses[key] = cached
        return [
            (kf.time_ms, left, right, head)
            for kf, (left, right, head) in zip(gesture.keyframes, cached)
        ]

    def _solve_ik(self, gesture):
        """Same amplitude fallback as laban_player, but on warm mapper instances."""
        requested = float(os.environ.get("LABAN_IK_GAIN", "1.00"))
        last_error = None
        for gain in [requested] + [g for g in lp.IK_GAIN_FALLBACKS if g < requested]:
            mapper = self._mapper_for(gain)
            # These instances are warm on purpose (loading the model costs 0.35s),
            # but their continuity state must not leak between gestures or attempts:
            # the one-shot player gets a fresh mapper every time, and a seed from
            # the previous gesture's final pose can select a different IK branch.
            mapper.reset()
            try:
                poses = []
                for kf in gesture.keyframes:
                    left_q, right_q = mapper.map_limbs(kf.limbs)
                    poses.append(
                        (left_q, right_q, lp.map_head(kf.head, kf.rotation))
                    )
                self._check_continuity(poses)
                if gain != requested:
                    self.log.info("IK amplitude reduced to %.2f" % gain)
                return poses
            except Exception as exc:  # noqa: BLE001 - try the next amplitude
                last_error = exc
        raise RuntimeError("IK failed at every amplitude: %s" % last_error)

    def _check_continuity(self, poses) -> None:
        names = list(LEFT_NAMES) + list(RIGHT_NAMES)
        for prev, cur in zip(poses, poses[1:]):
            deltas = [abs(b - a) for a, b in zip(prev[0] + prev[1], cur[0] + cur[1])]
            largest = max(deltas)
            if largest > 2.2:
                raise RuntimeError(
                    "unsafe IK discontinuity: %s changes %.3f rad (branch flip)"
                    % (names[deltas.index(largest)], largest)
                )

    # ------------------------------------------------------------- commands
    @staticmethod
    def _library_stem(path: Path) -> str:
        """Which library sample this file is, taken from the file rather than its name.

        A caller that stretches a gesture to fit a length writes the copy to a
        temporary file whose name says nothing about which gesture it holds. The
        Laban format's top-level key is the library name ("think d high l") and
        survives that rewrite, so the content answers what the path cannot.
        """
        try:
            with path.open(encoding="utf-8") as fh:
                data = json.load(fh)
            if isinstance(data, dict) and data:
                name = next(iter(data))
                if isinstance(name, str) and name.strip():
                    return name.strip()
        except (OSError, ValueError, StopIteration):
            pass
        return path.stem

    def cmd_play(self, req: dict) -> dict:
        path = Path(req.get("gesture", ""))
        if not path.is_file():
            return {"ok": False, "error": "gesture not found: %s" % path}
        speed = float(req.get("speed", self.args.speed))
        approach = float(req.get("approach", self.args.approach))
        include_head = not bool(req.get("no_head", self.args.no_head))
        # Per-request, so the twin can be checked without moving the robot.
        isaac_only = bool(req.get("isaac_only", self.args.isaac_only))

        # A gesture whose retarget puts the arms inside each other is refused on
        # hardware and allowed on the twin. The block protects the robot, and
        # validating in Isaac first is the documented workflow for exactly the
        # gestures that need it.
        stem = self._library_stem(path)
        if self.policy.tier(stem) == "blocked":
            if not isaac_only:
                self.refused += 1
                return {
                    "ok": False,
                    "error": "gesture %r refused: %s. Play it with "
                             '"isaac_only": true to see it on the twin.'
                             % (stem, self.policy.reason(stem)),
                    "gesture": stem,
                    "tier": "blocked",
                }
            self.log.warning(
                "playing blocked gesture %r on the twin only: %s"
                % (stem, self.policy.reason(stem))
            )

        gesture = load_gesture(path)
        poses = self._keyframe_poses(gesture)
        samples = lp.resample_from_poses(
            poses, dt_s=self.args.dt, speed=speed, include_head=include_head
        )

        times = [s[0] for s in samples]
        left_qs = [list(s[1]) for s in samples]
        right_qs = [list(s[2]) for s in samples]
        head_qs = [list(s[3]) for s in samples]

        return_ready = bool(req.get("return_ready", not self.args.no_return))

        ramp = 0.0
        if approach > 0 and all(n in self.current for n in self.watched):
            cur_head = (
                [self.current[n] for n in HEAD_NAMES]
                if include_head and all(n in self.current for n in HEAD_NAMES)
                else None
            )
            lead = lp.apply_lead_in(
                [self.current[n] for n in LEFT_NAMES],
                [self.current[n] for n in RIGHT_NAMES],
                times, left_qs, right_qs, head_qs, approach,
                cur_head=cur_head,
            )
            times, left_qs = lead.times, lead.left_qs
            right_qs, head_qs = lead.right_qs, lead.head_qs
            ramp = lead.ramp
            self.log.info(
                "lead-in %.2fs for %.2f rad (needs %.2fs, cap %.2fs)"
                % (lead.ramp, lead.gap, lead.needed, approach)
            )

        # Where the expressive part ends. The bridge lines speech up against this,
        # not against the end of the timeline, so opening the arms back up afterwards
        # does not make a reply look like it still has gesture left to run.
        motion_s = times[-1]
        if return_ready:
            out = lp.apply_lead_out(
                times, left_qs, right_qs, head_qs, self.args.dt,
                include_head=include_head,
            )
            times, left_qs = out.times, out.left_qs
            right_qs, head_qs = out.right_qs, out.head_qs
            self.log.info(
                "return to ready: hold %.2fs then %.2fs for %.2f rad"
                % (out.hold, out.ramp, out.gap)
            )

        if not isaac_only:
            if not (
                self.pub_l.get_subscription_count()
                and self.pub_r.get_subscription_count()
            ):
                # Publishing into the void looks exactly like "the robot ignored the
                # gesture", so say it instead. Usually a DDS mismatch or a stack that
                # restarted under the daemon.
                return {
                    "ok": False,
                    "error": "no controller subscribed to the arm trajectory topics",
                }
            # One publish is enough here, unlike the one-shot player which shotguns
            # five copies to survive DDS matching: these publishers are already
            # matched, and a repeat would restart the JTC goal.
            self.pub_l.publish(lp.build_arm_trajectory(LEFT_NAMES, left_qs, times))
            self.pub_r.publish(lp.build_arm_trajectory(RIGHT_NAMES, right_qs, times))
            if include_head:
                self.pub_h.publish(
                    lp.build_arm_trajectory(HEAD_NAMES, head_qs, times)
                )

        self._play = {
            "t0": time.time(),
            "times": times,
            "left": left_qs,
            "right": right_qs,
            "head": head_qs,
            "head_on": include_head,
            "next_cmd": time.time(),
        }
        self.played += 1
        self.log.info(
            "playing %s (%d pts, %.2fs%s)"
            % (
                path.stem,
                len(times),
                times[-1],
                " -> twin only" if isaac_only else (" + twin" if self.pub_isaac else ""),
            )
        )
        return {
            "ok": True,
            "gesture": path.stem,
            "ramp_s": round(ramp, 3),
            "motion_s": round(motion_s, 3),
            "duration_s": round(times[-1], 3),
            "returns_to_ready": return_ready,
            "points": len(times),
            "isaac_mirror": self.pub_isaac is not None,
        }

    # ------------------------------------------------------- emergency stop
    #
    # Three levels, because they are not interchangeable and picking the wrong one
    # is expensive in both directions.
    #
    #   cancel      empty trajectory. The JTC drops what it was given; the servos
    #               stay powered and hold position. Instant, recoverable, and the
    #               only one a watchdog should ever use.
    #   freeze      deactivate the arm/head controllers. Nothing can command the
    #               joints afterwards, including a second stack, which is the one
    #               thing `cancel` cannot promise. Recoverable with unfreeze.
    #   torque_off  deactivate the XiaobeiSystem hardware component, whose
    #               on_deactivate() calls EnableTorque(id, 0) on every servo.
    #
    # torque_off is a real e-stop and it is a ONE-WAY TRIP. The plugin opens the
    # serial port in on_init() and closes it in on_deactivate() (st.end()), while
    # on_activate() only re-enables torque -- so reactivating talks to a closed
    # port. Recovery means restarting ros2_control_node, which on this machine has
    # been up for days. It also drops the arms: torque off means limp, and limp
    # means gravity. That is correct for an emergency and wrong for a timeout,
    # which is exactly why the watchdog is wired to `cancel`.
    ARM_CONTROLLERS = ("left_arm_controller", "right_arm_controller", "head_controller")
    HARDWARE_COMPONENT = "XiaobeiSystem"
    CONFIRM_TOKEN = "TORQUE_OFF"

    def _client(self, kind: str):
        """Service clients made on demand, so a stack without controller_manager still boots."""
        if not hasattr(self, "_clients"):
            self._clients = {}
        if kind in self._clients:
            return self._clients[kind]
        from controller_manager_msgs.srv import (
            ListHardwareComponents, SetHardwareComponentState, SwitchController,
        )

        spec = {
            "list_hardware": (ListHardwareComponents,
                              "/controller_manager/list_hardware_components"),
            "set_hardware": (SetHardwareComponentState,
                             "/controller_manager/set_hardware_component_state"),
            "switch": (SwitchController, "/controller_manager/switch_controller"),
        }[kind]
        client = self.node.create_client(spec[0], spec[1])
        self._clients[kind] = client
        return client

    def _call(self, kind: str, request, timeout: float = 3.0):
        """Blocking service call. Safe here: handle() runs in the accept loop, not
        in an rclpy callback, so spinning from inside it cannot deadlock."""
        client = self._client(kind)
        if not client.wait_for_service(timeout_sec=timeout):
            raise RuntimeError("%s service not available" % kind)
        future = client.call_async(request)
        self.rclpy.spin_until_future_complete(self.node, future, timeout_sec=timeout)
        if not future.done():
            raise RuntimeError("%s service did not answer in %.1fs" % (kind, timeout))
        return future.result()

    def cmd_hardware(self, req: dict) -> dict:
        """Hardware components and their lifecycle states. Read-only."""
        from controller_manager_msgs.srv import ListHardwareComponents

        try:
            result = self._call("list_hardware", ListHardwareComponents.Request())
        except Exception as exc:  # noqa: BLE001
            return {"ok": False, "error": str(exc)}
        return {
            "ok": True,
            "components": [
                {
                    "name": c.name,
                    "type": c.type,
                    "state": c.state.label,
                    "command_interfaces": len(c.command_interfaces),
                }
                for c in result.component
            ],
        }

    def cmd_estop(self, req: dict) -> dict:
        level = str(req.get("level", "cancel"))

        if level == "cancel":
            out = self.cmd_stop()
            out["level"] = "cancel"
            out["note"] = "trajectory cancelled; servos still powered and holding"
            return out

        if level in ("freeze", "unfreeze"):
            from controller_manager_msgs.srv import SwitchController

            request = SwitchController.Request()
            names = list(req.get("controllers") or self.ARM_CONTROLLERS)
            if level == "freeze":
                # Cancel first. Deactivating a controller that still holds a goal
                # leaves the goal to resume the moment it is reactivated.
                self.cmd_stop()
                request.deactivate_controllers = names
            else:
                request.activate_controllers = names
            request.strictness = SwitchController.Request.BEST_EFFORT
            try:
                result = self._call("switch", request, timeout=5.0)
            except Exception as exc:  # noqa: BLE001
                return {"ok": False, "error": str(exc), "level": level}
            self.log.warning("estop %s: %s -> ok=%s" % (level, names, result.ok))
            return {
                "ok": bool(result.ok), "level": level, "controllers": names,
                "note": "recoverable: estop level=%s"
                        % ("unfreeze" if level == "freeze" else "freeze"),
            }

        if level == "torque_off":
            if req.get("confirm") != self.CONFIRM_TOKEN:
                return {
                    "ok": False,
                    "level": "torque_off",
                    "error": 'refused: pass confirm="%s". This disables torque on '
                             "every servo, so the arms go limp and fall, and the "
                             "plugin closes the serial port on the way out -- "
                             "recovery needs ros2_control_node restarted, not just "
                             "reactivated." % self.CONFIRM_TOKEN,
                }
            from controller_manager_msgs.srv import SetHardwareComponentState
            from lifecycle_msgs.msg import State

            self.cmd_stop()
            request = SetHardwareComponentState.Request()
            request.name = str(req.get("component") or self.HARDWARE_COMPONENT)
            request.target_state.id = State.PRIMARY_STATE_INACTIVE
            request.target_state.label = "inactive"
            self.log.error(
                "ESTOP torque_off: deactivating %s; every servo loses torque and "
                "the arms will fall" % request.name
            )
            try:
                result = self._call("set_hardware", request, timeout=8.0)
            except Exception as exc:  # noqa: BLE001
                return {"ok": False, "error": str(exc), "level": "torque_off"}
            return {
                "ok": bool(result.ok),
                "level": "torque_off",
                "component": request.name,
                "note": "torque disabled on every servo. Serial port closed by "
                        "on_deactivate; restart ros2_control_node to recover.",
            }

        return {
            "ok": False,
            "error": "unknown estop level %r; expected cancel, freeze, unfreeze "
                     "or torque_off" % level,
        }

    def cmd_joints(self, req: dict) -> dict:
        """Current angles for the joints this node tracks.

        `status` only reports whether joint states are arriving, which is not
        enough to answer the question that matters after a gesture: did anything
        move. `play` returns as soon as the trajectory is published, and a JTC
        accepts a trajectory whether or not it can follow one, so a caller that
        wants to distinguish "played" from "accepted and ignored" has to compare
        angles before and after. This is that comparison's data source.

        Read-only. Cheaper than a second /joint_states subscriber elsewhere, and
        much cheaper than a cold rclpy participant, which on this DDS setup can
        take minutes to discover anything.
        """
        names = req.get("names") or list(self.tracked)
        have = {n: round(float(self.current[n]), 6) for n in names if n in self.current}
        return {
            "ok": bool(have),
            "positions": have,
            "missing": [n for n in names if n not in self.current],
            "stamp": round(time.time(), 3),
        }

    def _busy(self) -> float:
        """Seconds left on the active timeline, 0 when nothing is running.

        Not read off `_play is not None`: the mirror tick is the only thing that
        clears `_play`, so on a daemon started without an Isaac publisher it stays
        set forever after the first gesture. `cmd_status`'s `playing` field has
        that behaviour and the bridge already reads it, so it is left alone; this
        computes the answer from the timeline instead.
        """
        if self._play is None:
            return 0.0
        left = self._play["t0"] + self._play["times"][-1] - time.time()
        return max(0.0, left)

    def cmd_look_at(self, req: dict) -> dict:
        """Point the head at an absolute or relative yaw/pitch.

        The head is the most trustworthy axis on the robot -- driving every corner
        of HEAD_LIMITS held at most 0.012 rad of error and 0.007 rad of tracking
        error, against the arms' 0.019-0.117 rad -- so a
        Phase 1 task that needs to change viewpoint should move this and not the
        arms.

        It lives here rather than in a separate node for the same reason cmd_play
        does: this process already holds a warm, matched publisher on
        /head_controller/joint_trajectory, and a second publisher on the same
        controller would fight it.

        HEAD_LIMITS is the only bound there is; the URDF sets none on j_101/j_102.
        A target outside it is clamped and the reply says so, because a silently
        clamped look reports success while pointing somewhere else -- the same
        class of lie as a head-only gesture returning ok with the head off.
        """
        if not all(n in self.current for n in HEAD_NAMES):
            return {
                "ok": False,
                "error": "no joint states for %s, so the current head pose is unknown"
                         % ", ".join(HEAD_NAMES),
            }
        remaining = self._busy()
        if remaining > 0.0:
            return {
                "ok": False,
                "error": "a gesture is still running for %.2fs; stop it first or wait"
                         % remaining,
            }

        cur = [self.current[n] for n in HEAD_NAMES]
        relative = bool(req.get("relative", False))
        requested = [
            float(req.get("yaw", 0.0 if relative else cur[0])),
            float(req.get("pitch", 0.0 if relative else cur[1])),
        ]
        if relative:
            requested = [cur[0] + requested[0], cur[1] + requested[1]]

        target = clamp_head(requested)
        clamped = [
            HEAD_NAMES[i] for i in range(2)
            if abs(target[i] - requested[i]) > 1e-6
        ]

        travel = max(abs(target[i] - cur[i]) for i in range(2))
        # Same ceiling the player rate-limits gestures with, so a look and a
        # gesture move the head at the same speed.
        needed = travel / lp.MAX_HEAD_VEL if lp.MAX_HEAD_VEL > 0 else 0.0
        duration = max(float(req.get("duration", 0.0)), needed, self.args.dt)

        steps = max(2, int(round(duration / self.args.dt)) + 1)
        times = [i * duration / (steps - 1) for i in range(steps)]
        head_qs = [
            [cur[j] + (target[j] - cur[j]) * (i / (steps - 1)) for j in range(2)]
            for i in range(steps)
        ]
        # The arms hold station. Sending their current pose rather than nothing is
        # what lets the Isaac mirror below reuse the gesture timeline unchanged.
        left_hold = [self.current.get(n, 0.0) for n in LEFT_NAMES]
        right_hold = [self.current.get(n, 0.0) for n in RIGHT_NAMES]
        left_qs = [list(left_hold) for _ in times]
        right_qs = [list(right_hold) for _ in times]

        isaac_only = bool(req.get("isaac_only", self.args.isaac_only))
        if not isaac_only:
            if not self.pub_h.get_subscription_count():
                return {
                    "ok": False,
                    "error": "no controller subscribed to /head_controller/joint_trajectory",
                }
            self.pub_h.publish(lp.build_arm_trajectory(HEAD_NAMES, head_qs, times))

        self._play = {
            "t0": time.time(),
            "times": times,
            "left": left_qs,
            "right": right_qs,
            "head": head_qs,
            "head_on": True,
            "next_cmd": time.time(),
        }

        reply = {
            "ok": True,
            "yaw": round(target[0], 4),
            "pitch": round(target[1], 4),
            "from": [round(v, 4) for v in cur],
            "travel_rad": round(travel, 4),
            "duration_s": round(duration, 3),
            "points": len(times),
            "isaac_mirror": self.pub_isaac is not None,
        }
        if clamped:
            reply["clamped"] = clamped
            reply["requested"] = [round(v, 4) for v in requested]
            self.log.warning(
                "look_at clamped %s: asked %s, sending %s"
                % (", ".join(clamped), [round(v, 3) for v in requested],
                   [round(v, 3) for v in target])
            )
        if travel <= 1e-4:
            # Zero travel is a no-op, and a no-op that returns ok is exactly the
            # trap NO_OP_HEAD_ONLY exists for. Say it in the reply so the trace
            # layer does not have to infer it.
            reply["no_op"] = True
        return reply

    def cmd_stop(self) -> dict:
        was = self._play is not None
        self._play = None
        # An empty trajectory is the JTC's cancel: without it the controller keeps
        # executing whatever it was already given.
        for pub in (self.pub_l, self.pub_r, self.pub_h):
            pub.publish(self.JointTrajectory())
        return {"ok": True, "stopped": was}

    def cmd_status(self) -> dict:
        return {
            "ok": True,
            "playing": self._play is not None,
            # Lets a launcher tell a real-only daemon from one mirroring to Isaac,
            # since both live on the same socket.
            "isaac_mirror": self.pub_isaac is not None,
            "isaac_only": bool(self.args.isaac_only),
            "returns_to_ready": not self.args.no_return,
            "mapper": self.args.mapper,
            "left_subs": self.pub_l.get_subscription_count(),
            "right_subs": self.pub_r.get_subscription_count(),
            "isaac_subs": (
                self.pub_isaac.get_subscription_count() if self.pub_isaac else 0
            ),
            "cached_gestures": len(self._poses),
            "played": self.played,
            "refused": self.refused,
            "policy": self.policy.summary(),
            "joint_states": all(n in self.current for n in self.watched),
        }

    def cmd_services(self, req: dict) -> dict:
        """Service names this node can see, optionally filtered by substring.

        Exists because the ros2 CLI cannot answer this here. With the real DDS
        settings (SUBNET discovery plus fastdds_mtu.xml) `ros2 service list` hangs
        past two minutes and returns nothing, while this node's publishers and
        subscriptions work fine -- measured, not assumed. The difference is that a
        CLI invocation is a brand-new participant that has to discover the graph
        from cold, and this one discovered it at warmup and kept it.

        Read-only, and the reason it is worth having: the ASPIRE preflight has to
        establish whether a torque-off path exists before it will allow autonomous
        motion, and a check that cannot complete is a check that gets switched off.
        """
        needle = str(req.get("match", "")).lower()
        try:
            names = self.node.get_service_names_and_types()
        except Exception as exc:  # noqa: BLE001
            return {"ok": False, "error": "service introspection failed: %r" % exc}
        found = [n for n, _t in names if not needle or needle in n.lower()]
        return {
            "ok": True,
            "total": len(names),
            "matched": sorted(found),
            "nodes": sorted(
                "%s%s" % (ns.rstrip("/"), name) if ns != "/" else "/%s" % name
                for name, ns in self.node.get_node_names_and_namespaces()
            ),
        }

    def handle(self, req: dict) -> dict:
        cmd = req.get("cmd", "")
        if cmd == "ping":
            return {"ok": True, "pong": True}
        if cmd == "status":
            return self.cmd_status()
        if cmd == "services":
            return self.cmd_services(req)
        if cmd == "joints":
            return self.cmd_joints(req)
        if cmd == "hardware":
            return self.cmd_hardware(req)
        if cmd == "estop":
            try:
                return self.cmd_estop(req)
            except Exception as exc:  # noqa: BLE001 - an e-stop must always answer
                self.log.error("estop failed: %r" % exc)
                return {"ok": False, "error": str(exc)}
        if cmd == "look_at":
            try:
                return self.cmd_look_at(req)
            except Exception as exc:  # noqa: BLE001 - a bad request must not kill us
                self.log.error("look_at failed: %r" % exc)
                return {"ok": False, "error": str(exc)}
        if cmd == "stop":
            return self.cmd_stop()
        if cmd == "quit":
            self.cmd_stop()
            self._quit = True
            return {"ok": True, "quitting": True}
        if cmd == "play":
            try:
                return self.cmd_play(req)
            except Exception as exc:  # noqa: BLE001 - a bad gesture must not kill us
                self.log.error("play failed: %r" % exc)
                return {"ok": False, "error": str(exc)}
        return {"ok": False, "error": "unknown cmd: %r" % cmd}

    # ------------------------------------------------------------ main loop
    def _mirror_tick(self) -> None:
        if self.pub_isaac is None or self._play is None:
            return
        now = time.time()
        if not self._isaac_seen and self.pub_isaac.get_subscription_count():
            self._isaac_seen = True
            self.log.info("Isaac twin joined")
        if now < self._play["next_cmd"]:
            return
        elapsed = now - self._play["t0"]
        if elapsed > self._play["times"][-1] + 0.5:
            self._play = None
            return
        lp._isaac_command(
            self.node,
            self.pub_isaac,
            self._play["times"],
            self._play["left"],
            self._play["right"],
            self._play["head"],
            self._play["head_on"],
            elapsed,
        )
        self._play["next_cmd"] = now + 1.0 / lp.ISAAC_CMD_HZ

    def serve(self, path: str) -> int:
        pid_path = path + ".pid"
        srv = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        srv.bind(path)
        srv.listen(4)
        srv.settimeout(SPIN_S)
        os.chmod(path, 0o666)
        Path(pid_path).write_text("%d\n" % os.getpid())
        self.log.info("listening on %s (pid %d)" % (path, os.getpid()))
        print("[laban-daemon] ready on %s (pid %d)" % (path, os.getpid()), flush=True)

        try:
            while not self._quit:
                try:
                    conn, _ = srv.accept()
                except socket.timeout:
                    conn = None
                except OSError as exc:
                    self.log.error("accept failed: %r" % exc)
                    conn = None
                if conn is not None:
                    with conn:
                        conn.settimeout(5.0)
                        try:
                            data = conn.recv(65536).decode("utf-8").strip()
                            req = json.loads(data) if data else {}
                        except Exception as exc:  # noqa: BLE001
                            req = None
                            reply = {"ok": False, "error": "bad request: %s" % exc}
                        if req is not None:
                            reply = self.handle(req)
                        try:
                            conn.sendall(
                                (json.dumps(reply, ensure_ascii=False) + "\n").encode(
                                    "utf-8"
                                )
                            )
                        except OSError:
                            pass
                self.rclpy.spin_once(self.node, timeout_sec=0.0)
                self._mirror_tick()
        except KeyboardInterrupt:
            print("\n[laban-daemon] stopped", flush=True)
        finally:
            srv.close()
            for leftover in (path, pid_path):
                if os.path.exists(leftover):
                    os.unlink(leftover)
            self.node.destroy_node()
            self.rclpy.shutdown()
        return 0


def main(argv=None) -> int:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--mapper", choices=("lut", "ik"), default="ik")
    p.add_argument("--speed", type=float, default=1.0, help="default for a request")
    p.add_argument("--approach", type=float, default=2.0, help="lead-in cap")
    p.add_argument("--dt", type=float, default=0.1)
    p.add_argument("--no-head", action="store_true")
    p.add_argument(
        "--no-return",
        action="store_true",
        help="leave the arms in each gesture's closing pose instead of opening them "
        "back to the ready pose (can be overridden per request)",
    )
    p.add_argument("--settle", type=float, default=5.0, help="warmup discovery window")
    p.add_argument("--isaac-mirror", action="store_true")
    p.add_argument(
        "--isaac-only",
        action="store_true",
        help="publish no trajectory, so the twin can be driven while the robot stays still",
    )
    args = p.parse_args(argv)
    if args.isaac_only:
        args.isaac_mirror = True

    os.environ.setdefault("ROS_DOMAIN_ID", "0")
    os.environ.setdefault("RMW_IMPLEMENTATION", "rmw_fastrtps_cpp")
    os.environ.setdefault("ROS_AUTOMATIC_DISCOVERY_RANGE", "LOCALHOST")

    path = socket_path(os.environ.get("X1_DDS", "real"))
    if already_running(path):
        print(
            "[laban-daemon] one is already listening on %s -- "
            "stop it first (laban/laban_ctl.py quit)" % path,
            file=sys.stderr,
        )
        return 1

    daemon = LabanDaemon(args)
    daemon.warmup()
    return daemon.serve(path)


if __name__ == "__main__":
    raise SystemExit(main())
