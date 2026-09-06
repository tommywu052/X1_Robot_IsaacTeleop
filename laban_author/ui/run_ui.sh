#!/usr/bin/env bash
# Start the X1 gesture authoring web UI.
#
#   bash laban_author/ui/run_ui.sh                  # author for the real robot
#   X1_DDS=isaac bash laban_author/ui/run_ui.sh     # preview on the twin only
#
# Runs alongside the playback UI on 9200 rather than replacing it: this one
# makes gestures, that one plays the whole library.
set -euo pipefail
AUTHOR="$(cd "$(dirname "$0")/.." && pwd)"
cd "$AUTHOR/ui"

export X1_DDS="${X1_DDS:-real}"
export X1_MAPPER="${X1_MAPPER:-ik}"
export X1_AUTHOR_PORT="${X1_AUTHOR_PORT:-9300}"

# The playback deployment supplies both the library a score is saved into and
# the player it is played through, so it is one setting rather than two.
#
# Probed in the same order as pipeline.find_laban_dir, because this tool sits
# beside teleop_share in the development tree and inside it in a checkout, and
# a shell default that disagreed with the python one would start a server whose
# banner named a different library from the one it wrote to.
if [[ -z "${X1_LABAN_DIR:-}" ]]; then
  for candidate in "$AUTHOR/../laban" "$AUTHOR/../teleop_share/laban"; do
    if [[ -f "$candidate/ui/gesture_player.py" ]]; then
      X1_LABAN_DIR="$(cd "$candidate" && pwd)"
      break
    fi
  done
fi
export X1_LABAN_DIR="${X1_LABAN_DIR:-$(cd "$AUTHOR/.." && pwd)/laban}"

# No ROS environment here for the same reason as the playback UI: this process
# only talks to the daemon over a unix socket or spawns run_player.sh, and that
# script does its own DDS setup.
if [[ "$X1_DDS" != "isaac" && ! -f /tmp/teleop_env.sh ]]; then
  echo "[x1-author] warning: no /tmp/teleop_env.sh, so a spawned player cannot" >&2
  echo "            reach the real controllers." >&2
fi

if [[ ! -d "$X1_LABAN_DIR/ui" ]]; then
  echo "[x1-author] no playback UI at $X1_LABAN_DIR/ui; saving will work but" >&2
  echo "            playing will not. Set X1_LABAN_DIR." >&2
fi

# Estimation runs in the V2D reconstruction venv, which then starts its own
# container. Both are checked here rather than failing halfway through a job.
V2D_ROOT="${V2D_ROOT:-$HOME/nvidia/video_to_data}"
if [[ ! -x "$V2D_ROOT/reconstruction/.venv/bin/python" ]]; then
  echo "[x1-author] no V2D venv under $V2D_ROOT; set V2D_ROOT" >&2
  exit 1
fi
if ! docker image inspect "${SAM3D_IMAGE:-v2d_sam3d_body}" >/dev/null 2>&1; then
  echo "[x1-author] the ${SAM3D_IMAGE:-v2d_sam3d_body} image is not built" >&2
  exit 1
fi

# Same interpreter as the player and the playback UI: system python3 is
# externally managed (PEP 668), so the deps live in the venv with pinocchio.
PYTHON="${X1_PYTHON:-$HOME/pink_venv/bin/python3}"
[[ -x "$PYTHON" ]] || PYTHON=python3
if ! "$PYTHON" -c 'import uvicorn, fastapi, numpy, multipart' 2>/dev/null; then
  echo "[x1-author] missing deps for $PYTHON, install with:" >&2
  echo "  $PYTHON -m pip install -r laban_author/ui/requirements.txt" >&2
  exit 1
fi

echo "[x1-author] UI http://0.0.0.0:${X1_AUTHOR_PORT}  dds=${X1_DDS} mapper=${X1_MAPPER}"
echo "[x1-author] library ${X1_LABAN_DIR}/gestures/library"
exec "$PYTHON" -m uvicorn server:app --host 0.0.0.0 --port "$X1_AUTHOR_PORT"
