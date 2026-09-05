# -*- coding: utf-8 -*-
"""Map Laban limb symbols / spherical state to X1 arm joint positions.

Skeleton mapper: starts from the known ready_pose baseline and applies coarse
offsets keyed by (direction, level). Approximate on purpose so Isaac playback
works before a calibrated symbol LUT exists.

Joint order (matches ready_pose / JointTrajectoryController):
  left:  j_1 .. j_6
  right: j_51 .. j_56
"""
from __future__ import annotations

import math
import os
from typing import Dict, List, Sequence, Tuple

from decoder import LimbSpherical

# Natural ready pose, same values as nodes/ready_pose.py in this repo.
BASE_LEFT: List[float] = [0.067, -0.359, -0.27, 0.889, 1.145, 0.767]
BASE_RIGHT: List[float] = [0.067, -0.359, -0.27, 0.889, 1.145, -0.767]

LEFT_NAMES = ["j_1", "j_2", "j_3", "j_4", "j_5", "j_6"]
RIGHT_NAMES = ["j_51", "j_52", "j_53", "j_54", "j_55", "j_56"]
HEAD_NAMES = ["j_101", "j_102"]
HEAD_BASE = [0.0, 0.0]

_JOINT_CLAMP = (-2.8, 2.8)

# Measured by taking the rotation axis of each head joint in base coordinates by
# finite difference through the URDF: +j_101 turns the head to the
# robot's RIGHT and +j_102 lifts the chin.  Neither is obvious from the URDF (both
# joints carry a non-trivial rpy) and both are the opposite of what reading the
# joint names suggests, so the signs below are deliberate.  Confirmed on the Isaac
# twin, which renders the same directions, so one set of numbers serves both.
#
# The URDF declares both head joints "continuous", i.e. with no limits at all, so
# nothing downstream will refuse a bad command.  This is the only bound.
HEAD_LIMITS = {"j_101": (-1.0, 1.0), "j_102": (-0.40, 0.40)}

# Laban only ever says "Left Forward" / "Right Forward" for the head in this
# library, never a full side turn, so yaw stays a glance rather than a look-away.
HEAD_YAW = 0.35
HEAD_PITCH_UP = 0.20
HEAD_PITCH_DOWN = 0.25

# The library's "rotation" field is MSRAbot turning its torso, which X1 cannot do at
# all, so it is expressed as extra head yaw instead. Scaled down rather than taken
# literally: 60 degrees of torso is the common value and following it 1:1 would both
# exceed HEAD_LIMITS and read as a startled snap. At 0.5 it becomes a 30 degree look,
# which still leaves room for the head's own glance inside the limit. This changes
# the meaning for pointing gestures ("turn to face that" becomes "look at that") and
# suits the think * family, which is most of the users of the field.
HEAD_ROTATION_SCALE = float(os.environ.get("X1_HEAD_ROTATION_SCALE", "0.5"))


def _clamp(q: Sequence[float]) -> List[float]:
    lo, hi = _JOINT_CLAMP
    return [max(lo, min(hi, float(v))) for v in q]


def _add(a: Sequence[float], b: Sequence[float]) -> List[float]:
    return [x + y for x, y in zip(a, b)]


def _key(direction: str, level: str) -> str:
    return "%s|%s" % (direction.strip().lower(), level.strip().lower())


# Elbow symbol -> offset on [j1..j6] authored for the RIGHT arm frame.
# Left arm reuses the same numbers after mirroring left/right labels
# (ready_pose already uses identical j1..j5 on both sides).
_ELBOW_OFFSET_RIGHT: Dict[str, List[float]] = {
    "place|low": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    "place|normal": [0.05, -0.25, 0.0, 0.15, 0.0, 0.0],
    "place|high": [0.1, -0.85, 0.05, 0.35, 0.1, 0.0],
    "forward|low": [0.05, -0.1, 0.15, 0.05, 0.0, 0.0],
    "forward|normal": [0.1, -0.35, 0.2, 0.2, 0.0, 0.0],
    "forward|high": [0.15, -0.75, 0.25, 0.35, 0.1, 0.0],
    "right|low": [0.15, -0.2, -0.35, 0.1, 0.0, 0.0],
    "right|normal": [0.25, -0.45, -0.55, 0.2, 0.05, 0.0],
    "right|high": [0.35, -0.95, -0.65, 0.35, 0.15, 0.0],
    "right forward|low": [0.12, -0.15, -0.2, 0.1, 0.0, 0.0],
    "right forward|normal": [0.2, -0.4, -0.35, 0.2, 0.05, 0.0],
    "right forward|high": [0.3, -0.9, -0.45, 0.35, 0.12, 0.0],
    "left|low": [0.1, -0.15, 0.35, 0.1, 0.0, 0.0],
    "left|normal": [0.18, -0.4, 0.5, 0.2, 0.05, 0.0],
    "left|high": [0.25, -0.85, 0.55, 0.3, 0.1, 0.0],
    "left forward|low": [0.08, -0.12, 0.2, 0.08, 0.0, 0.0],
    "left forward|normal": [0.15, -0.35, 0.3, 0.18, 0.05, 0.0],
    "left forward|high": [0.22, -0.8, 0.35, 0.3, 0.1, 0.0],
    "backward|low": [-0.05, -0.1, 0.0, 0.05, 0.0, 0.0],
    "backward|normal": [-0.1, -0.3, 0.0, 0.15, 0.0, 0.0],
    "backward|high": [-0.1, -0.55, 0.0, 0.25, 0.0, 0.0],
}

_WRIST_OFFSET_RIGHT: Dict[str, List[float]] = {
    "forward|low": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    "forward|normal": [0.0, 0.0, 0.0, 0.1, 0.15, 0.0],
    "forward|high": [0.0, -0.05, 0.0, 0.15, 0.35, 0.1],
    "place|low": [0.0, 0.0, 0.0, -0.1, -0.2, 0.0],
    "place|high": [0.0, -0.05, 0.0, 0.2, 0.45, 0.0],
    "right|low": [0.0, 0.0, -0.05, 0.05, 0.1, -0.25],
    "right|normal": [0.0, 0.0, -0.08, 0.1, 0.2, -0.4],
    "right|high": [0.0, -0.05, -0.1, 0.15, 0.35, -0.55],
    "right forward|low": [0.0, 0.0, -0.04, 0.05, 0.1, -0.15],
    "right forward|normal": [0.0, 0.0, -0.06, 0.1, 0.2, -0.3],
    "right forward|high": [0.0, -0.05, -0.08, 0.15, 0.4, -0.45],
    "left|low": [0.0, 0.0, 0.05, 0.05, 0.1, 0.25],
    "left|normal": [0.0, 0.0, 0.08, 0.1, 0.2, 0.4],
    "left|high": [0.0, -0.05, 0.1, 0.15, 0.35, 0.55],
    "left forward|low": [0.0, 0.0, 0.04, 0.05, 0.1, 0.15],
    "left forward|normal": [0.0, 0.0, 0.06, 0.1, 0.2, 0.3],
    "left forward|high": [0.0, -0.05, 0.08, 0.15, 0.35, 0.4],
}


def _lookup_offset(table: Dict[str, List[float]], direction: str, level: str) -> List[float]:
    k = _key(direction, level)
    if k in table:
        return list(table[k])
    for lv in ("normal", "low", "high"):
        alt = _key(direction, lv)
        if alt in table:
            return list(table[alt])
    return [0.0] * 6


def _spherical_nudge(limb: LimbSpherical, scale: float = 0.15) -> List[float]:
    """Small continuous nudge from theta/phi so interpolated frames are smooth."""
    x, y, z = limb.vector
    return [
        scale * x * 0.3,
        scale * -(1.0 - abs(y)) * 0.5,
        scale * -z * 0.4,
        scale * (0.5 - abs(y)) * 0.3,
        scale * x * 0.2,
        scale * z * 0.25,
    ]


def _mirror_dir_label(direction: str) -> str:
    d = direction.strip().lower()
    swaps = {
        "left": "right",
        "right": "left",
        "left forward": "right forward",
        "right forward": "left forward",
        "left backward": "right backward",
        "right backward": "left backward",
    }
    return swaps.get(d, d)


def map_arm(side: str, elbow: LimbSpherical, wrist: LimbSpherical) -> List[float]:
    """Compose one arm's 6 joint positions from elbow + wrist Laban state."""
    side = side.lower()
    base = list(BASE_LEFT if side == "left" else BASE_RIGHT)

    e_dir, e_lvl = elbow.direction, elbow.level
    w_dir, w_lvl = wrist.direction, wrist.level
    if side == "left":
        # "Left/High" on the left arm = ipsilateral raise (= right-arm "Right/High").
        e_dir = _mirror_dir_label(e_dir)
        w_dir = _mirror_dir_label(w_dir)

    q = _add(base, _lookup_offset(_ELBOW_OFFSET_RIGHT, e_dir, e_lvl))
    q = _add(q, _lookup_offset(_WRIST_OFFSET_RIGHT, w_dir, w_lvl))
    q = _add(q, _spherical_nudge(elbow, 0.12))
    q = _add(q, _spherical_nudge(wrist, 0.08))

    # Wrist-roll offsets use the right-arm sign; mirror roll onto BASE_LEFT.
    if side == "left":
        offset_roll = q[5] - BASE_LEFT[5]
        q[5] = BASE_LEFT[5] - offset_roll

    return _clamp(q)


def map_limbs_to_arms(limbs: Dict[str, LimbSpherical]) -> Tuple[List[float], List[float]]:
    """Return (left_q, right_q) from a sampled limb dict."""
    rest_e = LimbSpherical("Place", "Low", math.radians(175), 0.0, (0.0, -1.0, 0.0))
    rest_w = LimbSpherical("Forward", "Low", math.radians(135), 0.0, (0.0, -0.7, 0.7))

    le = limbs.get("left elbow", rest_e)
    lw = limbs.get("left wrist", rest_w)
    re = limbs.get("right elbow", rest_e)
    rw = limbs.get("right wrist", rest_w)

    return map_arm("left", le, lw), map_arm("right", re, rw)


def clamp_head(q: Sequence[float]) -> List[float]:
    """Clamp to HEAD_LIMITS rather than the arms' +/-2.8, which is no bound at all here."""
    out = []
    for name, value in zip(HEAD_NAMES, q):
        lo, hi = HEAD_LIMITS[name]
        out.append(max(lo, min(hi, float(value))))
    return out


def map_head(head: Tuple[str, str] | None, rotation_deg: float = 0.0) -> List[float]:
    """Laban head symbol (+ torso rotation) -> [j_101 yaw, j_102 pitch].

    Signs follow the measured axes: +yaw turns right, +pitch lifts the chin.  So
    "Left Forward" needs a negative yaw and level "Low" (looking down) a negative
    pitch -- both the reverse of what the words suggest.

    rotation_deg is the torso rotation X1 has no joint for, added to yaw at
    HEAD_ROTATION_SCALE.  It is summed rather than taking the larger of the two so
    "look left while turned right" partly cancels, as it would on a body.
    """
    yaw = 0.0
    pitch = 0.0
    if head:
        direction, level = head
        d = direction.lower()
        if "left" in d:
            yaw = -HEAD_YAW
        elif "right" in d:
            yaw = HEAD_YAW
        lv = level.lower()
        if lv == "high":
            pitch = HEAD_PITCH_UP
        elif lv == "low":
            pitch = -HEAD_PITCH_DOWN

    if rotation_deg:
        yaw += math.radians(rotation_deg) * HEAD_ROTATION_SCALE

    if not head and not rotation_deg:
        return list(HEAD_BASE)
    return clamp_head([yaw, pitch])
