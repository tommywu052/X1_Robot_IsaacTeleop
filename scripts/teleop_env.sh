#!/usr/bin/env bash
# ---------------------------------------------------------------------------
# X1 Quest3 遙操 —— 共用環境設定
# 所有 run 腳本都會 `source /tmp/teleop_env.sh`。
# 若你的機器路徑 / 前端 IP 不同，改這一個檔就好（部署後在 cam:/tmp/teleop_env.sh）。
# ---------------------------------------------------------------------------

# ROS 2 安裝與工作區（依你的 cam WSL 環境調整）
ROS_SETUP="${ROS_SETUP:-/opt/ros/jazzy/setup.bash}"
WS_SETUP="${WS_SETUP:-$HOME/xiaobei_X1_ws/install/setup.bash}"

# 跑 bridge 節點用的 python（pink_venv 內含 pinocchio/pink/msgpack）
export TELEOP_PY="${TELEOP_PY:-$HOME/pink_venv/bin/python3}"

# 前端 CloudXR / teleop_ros2_node 所在主機（跨網段 DDS static peer）
# 同機部署 -> 127.0.0.1；前端在另一台 -> 改成那台的 IP。
export HORDE_IP="${HORDE_IP:-127.0.0.1}"

# ---- DDS / ROS 網路（前後端一致，勿設自訂 FastDDS profile）----
export ROS_DOMAIN_ID=0
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
export ROS_STATIC_PEERS="$HORDE_IP"
unset FASTRTPS_DEFAULT_PROFILES_FILE 2>/dev/null || true
unset FASTDDS_DEFAULT_PROFILES_FILE 2>/dev/null || true

# shellcheck disable=SC1090
[ -f "$ROS_SETUP" ] && source "$ROS_SETUP"
# shellcheck disable=SC1090
[ -f "$WS_SETUP" ] && source "$WS_SETUP"