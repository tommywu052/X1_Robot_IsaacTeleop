#!/usr/bin/env bash
# 夾爪橋接：扳機/握把手指彎曲 -> j_7 / j_57。擠壓=閉合，放開=張開。免 engage。
source /tmp/teleop_env.sh
exec "$TELEOP_PY" /tmp/gripper_bridge.py