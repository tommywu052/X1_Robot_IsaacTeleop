#!/usr/bin/env bash
# Play a Laban gesture on the running X1 stack (Isaac or real).
# Usage: bash laban/run_player.sh [gesture_file] [extra args...]
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
# --daemon starts the resident player instead of playing one file, so a gesture can
# start without paying process start and DDS discovery again (see laban_daemon.py).
DAEMON=0
if [[ "${1:-}" == "--daemon" ]]; then
  DAEMON=1
  shift
  GESTURE=""
else
  GESTURE="${1:-Ges01_wavehand.total.json}"
  if [[ $# -gt 0 ]]; then shift; fi
fi

# The real-robot stack (cam_relaunch_ts.sh) runs with useBuiltinTransports=false via
# fastdds_mtu.xml and SUBNET discovery, so a player using the Isaac settings (LOCALHOST,
# no profile) discovers nothing: it publishes into the void with no error. Pick the same
# DDS config as the stack being driven.
MODE="${X1_DDS:-isaac}"

# "both" = real robot + Isaac twin from one player. Running a second player in the
# isaac DDS config does NOT work here: the real controllers live in this same WSL,
# so a LOCALHOST-scoped player finds the real JTC again and the robot is commanded
# twice while the twin never moves. The twin does not take trajectories at all --
# isaac_x1_ros2.py subscribes to /isaac_joint_commands (JointState) -- and it is
# reachable from the real stack's DDS settings, so
# one process streams both off a single timeline.
if [[ "$MODE" == "both" ]]; then
  MODE=real
  set -- "$@" --isaac-mirror
fi

# ROS setup.bash reads AMENT_TRACE_SETUP_FILES et al. unguarded, so nounset has to
# be off while sourcing or every run dies with "unbound variable".
set +u
case "$MODE" in
  real)
    # /tmp/teleop_env.sh is the cam override (teleop_env_cam.sh): SUBNET + fastdds_mtu.xml,
    # i.e. exactly what the teleop bridges use to reach the real controllers.
    if [[ ! -f /tmp/teleop_env.sh ]]; then
      echo "X1_DDS=real needs /tmp/teleop_env.sh (deploy_cam.ps1 puts it there)." >&2
      exit 1
    fi
    # shellcheck disable=SC1091
    source /tmp/teleop_env.sh
    ;;
  isaac)
    export ROS_DOMAIN_ID=0
    export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
    export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
    unset FASTRTPS_DEFAULT_PROFILES_FILE FASTDDS_DEFAULT_PROFILES_FILE
    [[ -f /opt/ros/jazzy/setup.bash ]] && source /opt/ros/jazzy/setup.bash
    [[ -f "$HOME/xiaobei_X1_ws/install/setup.bash" ]] && source "$HOME/xiaobei_X1_ws/install/setup.bash"
    ;;
  *)
    echo "X1_DDS must be 'isaac' or 'real' (got '$MODE')." >&2
    exit 1
    ;;
esac
set -u

echo "[laban] X1_DDS=$MODE range=${ROS_AUTOMATIC_DISCOVERY_RANGE:-unset} profile=${FASTRTPS_DEFAULT_PROFILES_FILE:-none}"

PYTHON="${X1_PYTHON:-python3}"
for arg in "$@"; do
  if [[ "$arg" == "ik" && -z "${X1_PYTHON:-}" && -x "$HOME/pink_venv/bin/python3" ]]; then
    PYTHON="$HOME/pink_venv/bin/python3"
  fi
done
echo "[laban] python=$PYTHON"

if [[ "$DAEMON" == "1" ]]; then
  export X1_DDS="$MODE"
  exec "$PYTHON" "$ROOT/laban/nodes/laban_daemon.py" "$@"
fi

exec "$PYTHON" "$ROOT/laban/nodes/laban_player.py" --gesture "$GESTURE" "$@"
