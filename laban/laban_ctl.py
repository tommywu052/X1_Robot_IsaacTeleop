#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Talk to the resident Laban player (laban/nodes/laban_daemon.py).

Pure sockets, no rclpy, so it runs under any python3:

  python3 laban/laban_ctl.py status
  python3 laban/laban_ctl.py play laban/gestures/library/hello.json --speed 1.0
  python3 laban/laban_ctl.py stop

The round trip it prints is what the chat bridge pays before the arms move, so it
is the number to watch when checking co-speech timing.
"""
from __future__ import annotations

import argparse
import json
import os
import socket
import sys
import time
from pathlib import Path

DEFAULT_SOCKET = "/tmp/x1_laban_%s.sock"


def socket_path(mode: str) -> str:
    return os.environ.get("X1_LABAN_SOCKET") or DEFAULT_SOCKET % mode


def request(path: str, payload: dict, timeout: float = 15.0) -> dict:
    with socket.socket(socket.AF_UNIX, socket.SOCK_STREAM) as sock:
        sock.settimeout(timeout)
        sock.connect(path)
        sock.sendall((json.dumps(payload) + "\n").encode("utf-8"))
        data = sock.recv(65536).decode("utf-8").strip()
    return json.loads(data) if data else {}


def main(argv=None) -> int:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("cmd", choices=("status", "ping", "play", "stop", "quit"))
    p.add_argument("gesture", nargs="?", help="gesture JSON, for play")
    p.add_argument("--speed", type=float)
    p.add_argument("--approach", type=float)
    p.add_argument("--head", action="store_true", help="also drive the head")
    p.add_argument(
        "--isaac-only",
        action="store_true",
        help="drive only the Isaac twin, leaving the robot still",
    )
    p.add_argument(
        "--no-return",
        action="store_true",
        help="leave the arms in the gesture's closing pose instead of opening them "
        "back to the ready pose (overrides the daemon's default for this play)",
    )
    p.add_argument(
        "--mode",
        default=os.environ.get("X1_DDS", "real"),
        help="which daemon socket to use ('both' shares the real one)",
    )
    p.add_argument(
        "--field",
        help="print only this reply field, for scripts (empty output = no daemon)",
    )
    args = p.parse_args(argv)

    mode = "real" if args.mode == "both" else args.mode
    path = socket_path(mode)
    if not os.path.exists(path):
        if not args.field:
            print(
                "no daemon on %s -- start one with X1_DDS=%s bash laban/run_player.sh "
                "--daemon --mapper ik" % (path, args.mode),
                file=sys.stderr,
            )
        return 2

    payload = {"cmd": args.cmd}
    if args.cmd == "play":
        if not args.gesture:
            print("play needs a gesture file", file=sys.stderr)
            return 2
        payload["gesture"] = str(Path(args.gesture).resolve())
        payload["no_head"] = not args.head
        if args.isaac_only:
            payload["isaac_only"] = True
        if args.no_return:
            payload["return_ready"] = False
        if args.speed is not None:
            payload["speed"] = args.speed
        if args.approach is not None:
            payload["approach"] = args.approach

    started = time.time()
    try:
        reply = request(path, payload)
    except OSError as exc:
        if not args.field:
            print("daemon unreachable: %s" % exc, file=sys.stderr)
        return 1
    if args.field:
        value = reply.get(args.field)
        if value is None:
            return 1
        print(json.dumps(value))
        return 0
    print("round trip %.3fs" % (time.time() - started))
    print(json.dumps(reply, ensure_ascii=False, indent=2))
    return 0 if reply.get("ok") else 1


if __name__ == "__main__":
    raise SystemExit(main())
