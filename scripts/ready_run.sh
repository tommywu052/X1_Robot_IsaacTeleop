#!/usr/bin/env bash
# 把雙臂擺到 ready pose（engage 前先擺好，避免從歪姿接手）。
source /tmp/teleop_env.sh
exec "$TELEOP_PY" /tmp/ready_pose.py