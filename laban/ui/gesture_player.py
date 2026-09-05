# -*- coding: utf-8 -*-
"""Fire-and-forget X1 Laban playback from the chat voice bridge.

Talks to the resident player (laban/nodes/laban_daemon.py) when its socket is
there, and otherwise spawns laban/run_player.sh, so neither path duplicates DDS or
IK logic. Only one gesture runs at a time; a new request preempts the previous one.

The difference matters for co-speech timing: a spawn costs 2.3s before the arms
can move, while the daemon already holds rclpy, the controllers, and the IK model.
"""
from __future__ import annotations

import json
import os
import socket
import subprocess
import threading
import time
from pathlib import Path
from typing import Optional


ROOT = Path(__file__).resolve().parents[2]
PLAYER = ROOT / "laban" / "run_player.sh"
LOG_DIR = Path(__file__).resolve().parent / "data"
LOG_PATH = LOG_DIR / "last_gesture.log"

# Spawn -> ready to publish, measured on cam with run_player.sh --check: 2.3s for
# the real stack (1.0s of it is bash + venv python + IK, the rest is rclpy init and
# controller discovery). The caller needs this to line a gesture up with speech
# that is already playing.
SPAWN_S = 2.3
# --isaac-mirror spends a short grace period on Isaac discovery (X1_ISAAC_GRACE)
# before publishing; a twin that appears later joins the stream in progress.
ISAAC_WAIT_S = 0.8

# Resident player: a socket round trip plus one IK solve, and nothing at all once
# the gesture's poses are cached. Must match laban_daemon.socket_path().
DAEMON_SOCKET = "/tmp/x1_laban_%s.sock"
DAEMON_CALL_S = 0.4
DAEMON_TIMEOUT_S = 8.0


class GesturePlayer:
    def __init__(
        self,
        dds_mode: str = "real",
        mapper: str = "ik",
        speed: float = 1.0,
        approach: float = 2.0,
        allow_ik_real: bool = True,
        no_head: bool = False,
    ):
        self.dds_mode = dds_mode
        # "both" = real robot plus the Isaac twin, from one player: the twin takes
        # JointState on /isaac_joint_commands and is reachable from the real
        # stack's DDS settings, so a single timeline drives them together.
        self.isaac_mirror = dds_mode == "both"
        self.launch_mode = "real" if self.isaac_mirror else dds_mode
        self.mapper = mapper
        self.speed = speed
        self.approach = approach
        self.allow_ik_real = allow_ik_real
        self.no_head = no_head
        self._procs: list = []
        self._logs: list = []
        # Reentrant: play() holds the lock and then calls stop().
        self._lock = threading.RLock()
        self.last_gesture: Optional[str] = None
        self.last_started = 0.0
        self.socket_path = (
            os.environ.get("X1_LABAN_SOCKET") or DAEMON_SOCKET % self.launch_mode
        )
        self.daemon_ok = self._request({"cmd": "ping"}, timeout=1.0) is not None

    @property
    def lead_s(self) -> float:
        """Delay between play() and the gesture's first keyframe.

        Startup, then the approach ramp that eases the arms into the opening pose.
        Only an estimate for scheduling; play() reports what actually happened.
        """
        if self.daemon_ok:
            return DAEMON_CALL_S + min(self.approach, 1.0)
        return SPAWN_S + (ISAAC_WAIT_S if self.isaac_mirror else 0.0) + self.approach

    def _request(self, payload: dict, timeout: float = DAEMON_TIMEOUT_S):
        """One request/response with the resident player, or None if it is not up."""
        try:
            with socket.socket(socket.AF_UNIX, socket.SOCK_STREAM) as sock:
                sock.settimeout(timeout)
                sock.connect(self.socket_path)
                sock.sendall((json.dumps(payload) + "\n").encode("utf-8"))
                data = sock.recv(65536).decode("utf-8").strip()
            return json.loads(data) if data else None
        except (OSError, ValueError):
            return None

    def stop(self, cancel: bool = True) -> None:
        with self._lock:
            # Terminating a spawned player does not stop the robot -- the JTC already
            # holds the whole trajectory -- so only the daemon can really cancel.
            # play() skips it: the new trajectory preempts the old one on its own.
            if cancel and self.daemon_ok:
                self._request({"cmd": "stop"}, timeout=2.0)
            for proc in self._procs:
                try:
                    proc.terminate()
                    proc.wait(timeout=2.0)
                except Exception:
                    try:
                        proc.kill()
                    except Exception:
                        pass
            self._procs = []
            for log in self._logs:
                try:
                    log.close()
                except Exception:
                    pass
            self._logs = []

    def _play_via_daemon(self, gesture_path: Path, speech_ms) -> Optional[dict]:
        """Play through the resident player, or None to fall back to a spawn."""
        started = time.time()
        reply = self._request(
            {
                "cmd": "play",
                "gesture": str(gesture_path),
                "speed": self.speed,
                "approach": self.approach,
                "no_head": self.no_head,
            }
        )
        if reply is None:
            if self.daemon_ok:
                self.daemon_ok = False
            return None
        call_s = time.time() - started
        self.daemon_ok = True
        if not reply.get("ok"):
            return {
                "ok": False,
                "error": reply.get("error", "daemon refused"),
                "via": "daemon",
            }
        self.last_gesture = gesture_path.name
        self.last_started = started
        return {
            "ok": True,
            "gesture": gesture_path.name,
            "via": "daemon",
            "dds": self.dds_mode,
            "isaac_mirror": bool(reply.get("isaac_mirror")),
            "mapper": self.mapper,
            "speech_ms": speech_ms,
            # Measured, not estimated. lead_s is when the arms start moving, ends_in_s
            # when the timeline (ramp included) runs out, both from the play() call.
            # motion_s excludes the return to the ready pose that the daemon appends:
            # that is the arms tidying up, not gesture still to come, so speech should
            # not be held against it.
            "lead_s": round(call_s + float(reply.get("ramp_s", 0.0)), 3),
            "ends_in_s": round(
                call_s
                + float(reply.get("motion_s", reply.get("duration_s", 0.0))),
                3,
            ),
        }

    def play(self, gesture_path: Path, speech_ms: Optional[int] = None) -> dict:
        gesture_path = Path(gesture_path)
        if not gesture_path.is_file():
            return {"ok": False, "error": "gesture not found: %s" % gesture_path}
        if not PLAYER.is_file():
            return {"ok": False, "error": "laban/run_player.sh missing"}

        cmd = [
            "bash",
            str(PLAYER),
            str(gesture_path),
            "--mapper",
            self.mapper,
            "--speed",
            str(self.speed),
            "--approach",
            str(self.approach),
        ]
        if self.no_head:
            cmd.append("--no-head")
        if self.mapper == "ik" and self.allow_ik_real:
            cmd.append("--allow-ik-real")
        if self.isaac_mirror:
            cmd.append("--isaac-mirror")

        env = os.environ.copy()
        env["X1_DDS"] = self.launch_mode

        with self._lock:
            self.stop(cancel=False)
            reply = self._play_via_daemon(gesture_path, speech_ms)
            if reply is not None:
                return reply
            # Keep the last run's output: a silent DEVNULL hides IK/DDS errors
            # that would otherwise look like "the robot just did not move".
            LOG_PATH.parent.mkdir(parents=True, exist_ok=True)
            log = open(LOG_PATH, "wb")
            self._logs.append(log)
            proc = subprocess.Popen(
                cmd,
                cwd=str(ROOT),
                env=env,
                stdout=log,
                stderr=subprocess.STDOUT,
            )
            self._procs.append(proc)
            self.last_gesture = gesture_path.name
            self.last_started = time.time()
            return {
                "ok": True,
                "gesture": gesture_path.name,
                "pid": proc.pid,
                "dds": self.dds_mode,
                "isaac_mirror": self.isaac_mirror,
                "mapper": self.mapper,
                "speech_ms": speech_ms,
            }
