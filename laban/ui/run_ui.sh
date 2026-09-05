#!/usr/bin/env bash
# Start the X1 gesture control web UI.
#
#   bash laban/ui/run_ui.sh              # drive the real robot
#   X1_DDS=isaac bash laban/ui/run_ui.sh # drive the Isaac twin only
#   X1_DDS=both  bash laban/ui/run_ui.sh # real robot + Isaac twin together
#
# A resident daemon is optional but makes playback start ~2s sooner:
#   X1_DDS=real bash laban/run_player.sh --daemon --mapper ik
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
cd "$ROOT/laban/ui"

export X1_DDS="${X1_DDS:-real}"
export X1_MAPPER="${X1_MAPPER:-ik}"
export X1_UI_PORT="${X1_UI_PORT:-9200}"

# No ROS environment is set up here on purpose: this process only ever talks to
# the daemon over a unix socket or spawns run_player.sh, and that script does its
# own DDS setup. Sourcing ROS here would also need `set +u`, because setup.bash
# reads AMENT_TRACE_SETUP_FILES unguarded.
if [[ "$X1_DDS" != "isaac" && ! -f /tmp/teleop_env.sh ]]; then
  echo "[x1-gesture] warning: no /tmp/teleop_env.sh, so a spawned player cannot" >&2
  echo "             reach the real controllers. Run deploy/deploy_teleop.ps1," >&2
  echo "             or use X1_DDS=isaac for the twin." >&2
fi

# Same interpreter as the player: system python3 on Debian/Ubuntu is externally
# managed (PEP 668), so the deps live in the venv that also holds pinocchio.
PYTHON="${X1_PYTHON:-$HOME/pink_venv/bin/python3}"
[[ -x "$PYTHON" ]] || PYTHON=python3
if ! "$PYTHON" -c 'import uvicorn, fastapi' 2>/dev/null; then
  echo "[x1-gesture] missing deps for $PYTHON, install with:" >&2
  echo "  $PYTHON -m pip install -r laban/ui/requirements.txt" >&2
  exit 1
fi

echo "[x1-gesture] UI http://0.0.0.0:${X1_UI_PORT}  dds=${X1_DDS} mapper=${X1_MAPPER}"
exec "$PYTHON" -m uvicorn server:app --host 0.0.0.0 --port "$X1_UI_PORT"
