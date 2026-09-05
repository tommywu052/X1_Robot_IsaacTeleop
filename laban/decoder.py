# -*- coding: utf-8 -*-
"""Parse MSRAbot / LabanotationSuite gesture JSON and decode dir+lvl to theta/phi.

Mirrors microsoft/LabanotationSuite MSRAbotSimulation/js/labanotation.js so
keyframe timing and spherical directions stay compatible with official samples.
"""
from __future__ import annotations

import json
import math
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

LIMB_KEYS = ("right elbow", "right wrist", "left elbow", "left wrist")
HEAD_KEY = "head"
# MSRAbot turns its whole torso; X1 has no torso yaw joint, so this ends up folded
# into head yaw by x1_mapper.map_head. 16 of the 92 gestures use it, all eight
# "think *" among them at ToRight/60.
ROTATION_KEY = "rotation"


def _unwrap(value: Any) -> Any:
    """Kinect/LabanEditor exports often wrap scalars as single-element lists."""
    if isinstance(value, list) and len(value) == 1:
        return value[0]
    return value


def _as_float(value: Any) -> float:
    return float(_unwrap(value))


def _as_dir_lvl(value: Any) -> Tuple[str, str]:
    raw = value
    if isinstance(raw, list) and len(raw) >= 2:
        return str(raw[0]).strip(), str(raw[1]).strip()
    if isinstance(raw, dict):
        return str(raw.get("dir", "Place")).strip(), str(raw.get("lvl", "Low")).strip()
    raise ValueError("expected [dir, lvl], got %r" % (value,))


def _as_rotation(value: Any) -> float:
    """["ToRight"|"ToLeft", degrees] -> signed degrees, positive to the robot's right.

    The amount already carries its own sign in the library (ToRight/-60 occurs), so
    the direction word only flips it, and a missing or unreadable field is no
    rotation rather than an error: it is present in all 92 files and 0 in most.
    """
    if not isinstance(value, list) or len(value) < 2:
        return 0.0
    try:
        amount = float(value[1])
    except (TypeError, ValueError):
        return 0.0
    return -amount if str(value[0]).strip().lower() == "toleft" else amount


@dataclass
class LimbSpherical:
    direction: str
    level: str
    theta: float  # rad
    phi: float  # rad
    vector: Tuple[float, float, float]  # (x, y, z)


@dataclass
class Keyframe:
    name: str
    time_ms: float
    limbs: Dict[str, LimbSpherical] = field(default_factory=dict)
    head: Optional[Tuple[str, str]] = None
    # ("ToRight" | "ToLeft", degrees). Signed the same way as head yaw: positive
    # is to the robot's right.
    rotation: float = 0.0


@dataclass
class Gesture:
    name: str
    keyframes: List[Keyframe]
    source_path: Optional[str] = None

    @property
    def start_ms(self) -> float:
        return self.keyframes[0].time_ms if self.keyframes else 0.0

    @property
    def end_ms(self) -> float:
        return self.keyframes[-1].time_ms if self.keyframes else 0.0

    @property
    def duration_ms(self) -> float:
        return self.end_ms - self.start_ms


def dir_lvl_to_spherical(direction: str, level: str) -> LimbSpherical:
    """Port of Labanotation.convertLabanotation2Vec."""
    dir_l = direction.lower()
    lv = level.lower()

    theta_deg = 180.0
    if lv == "high":
        theta_deg = 45.0
    elif lv == "normal":
        theta_deg = 90.0
    elif lv == "low":
        theta_deg = 135.0

    phi_deg = 0.0
    if dir_l == "forward":
        phi_deg = 0.0
    elif dir_l == "right forward":
        phi_deg = -45.0
    elif dir_l == "right":
        phi_deg = -90.0
    elif dir_l == "right backward":
        phi_deg = -135.0
    elif dir_l == "backward":
        phi_deg = 180.0
    elif dir_l == "left backward":
        phi_deg = 135.0
    elif dir_l == "left":
        phi_deg = 90.0
    elif dir_l == "left forward":
        phi_deg = 45.0
    elif dir_l == "place":
        if lv == "high":
            theta_deg, phi_deg = 5.0, 0.0
        elif lv == "low":
            theta_deg, phi_deg = 175.0, 0.0
        else:
            theta_deg, phi_deg = 180.0, 0.0
    else:
        phi_deg = 0.0

    theta = math.radians(theta_deg)
    phi = math.radians(phi_deg)
    y = math.cos(theta)
    x = math.sin(theta) * math.sin(phi)
    z = math.sin(theta) * math.cos(phi)
    return LimbSpherical(direction, level, theta, phi, (x, y, z))


def load_gesture(path: str | Path) -> Gesture:
    path = Path(path)
    data = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(data, dict) or not data:
        raise ValueError("empty or invalid gesture file: %s" % path)

    # Official MSRAbot samples: { "Ges01_wavehand": { "Position0": {...}, ... } }
    gesture_name, body = next(iter(data.items()))
    if not isinstance(body, dict):
        raise ValueError("expected nested position map under %r" % gesture_name)

    frames: List[Keyframe] = []
    for pos_name, obj in body.items():
        if not isinstance(obj, dict):
            continue
        if "start time" not in obj:
            continue
        t_ms = _as_float(obj["start time"])
        limbs: Dict[str, LimbSpherical] = {}
        for key in LIMB_KEYS:
            if key not in obj:
                continue
            d, lv = _as_dir_lvl(obj[key])
            limbs[key] = dir_lvl_to_spherical(d, lv)
        head = None
        if HEAD_KEY in obj:
            head = _as_dir_lvl(obj[HEAD_KEY])
        frames.append(
            Keyframe(
                name=pos_name,
                time_ms=t_ms,
                limbs=limbs,
                head=head,
                rotation=_as_rotation(obj.get(ROTATION_KEY)),
            )
        )

    frames.sort(key=lambda f: f.time_ms)
    if not frames:
        raise ValueError("no keyframes found in %s" % path)
    return Gesture(name=gesture_name, keyframes=frames, source_path=str(path))


def _lerp(a: float, b: float, s: float) -> float:
    return a + (b - a) * s


def interpolate_limbs(a: Keyframe, b: Keyframe, s: float) -> Dict[str, LimbSpherical]:
    """Lerp theta/phi between two keyframes (same as JS interpolateRotation)."""
    out: Dict[str, LimbSpherical] = {}
    keys = set(a.limbs) | set(b.limbs)
    for key in keys:
        la = a.limbs.get(key)
        lb = b.limbs.get(key)
        if la is None and lb is None:
            continue
        if la is None:
            out[key] = lb  # type: ignore[assignment]
            continue
        if lb is None:
            out[key] = la
            continue
        theta = _lerp(la.theta, lb.theta, s)
        phi = _lerp(la.phi, lb.phi, s)
        y = math.cos(theta)
        x = math.sin(theta) * math.sin(phi)
        z = math.sin(theta) * math.cos(phi)
        out[key] = LimbSpherical(lb.direction, lb.level, theta, phi, (x, y, z))
    return out


def sample_gesture(gesture: Gesture, time_ms: float) -> Dict[str, LimbSpherical]:
    """Sample limb spherical state at absolute gesture time (ms)."""
    frames = gesture.keyframes
    if time_ms <= frames[0].time_ms:
        return dict(frames[0].limbs)
    if time_ms >= frames[-1].time_ms:
        return dict(frames[-1].limbs)

    prev = frames[0]
    for cur in frames[1:]:
        if prev.time_ms <= time_ms <= cur.time_ms:
            span = cur.time_ms - prev.time_ms
            s = 0.0 if span <= 0 else (time_ms - prev.time_ms) / span
            return interpolate_limbs(prev, cur, s)
        prev = cur
    return dict(frames[-1].limbs)
