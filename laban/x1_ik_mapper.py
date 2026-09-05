"""Geometry-based Laban -> X1 mapper using Pinocchio and Pink.

Laban describes the upper-arm (shoulder to elbow) and forearm (elbow to wrist)
as directions.  This mapper turns those directions into Cartesian elbow/wrist
targets using the X1 URDF, then solves both targets instead of guessing joint
offsets from a gesture-specific lookup table.

Run this module with the existing Pink environment (~/pink_venv/bin/python3).
It deliberately has no ROS dependency, so all gesture keyframes can be solved
before a JointTrajectory is published.
"""
from __future__ import annotations

import math
import os
from pathlib import Path
from typing import Dict, List, Sequence, Tuple

import numpy as np
import pinocchio as pin
from pink import Configuration, solve_ik
from pink.tasks import FrameTask, PostureTask

from decoder import LimbSpherical
from x1_mapper import BASE_LEFT, BASE_RIGHT, LEFT_NAMES, RIGHT_NAMES


# The URDF marks the arm joints continuous with +/-3.14 soft limits, so stay a
# little inside that to keep commands away from the servo boundary.
JOINT_SAFETY_RAD = 2.9

# Matches the player's discontinuity guard: anything larger is a branch flip
# rather than a big-but-legitimate reach.
MAX_KEYFRAME_TRAVEL_RAD = 2.2

DEFAULT_URDF = Path(
    os.path.expanduser("~/xiaobei_X1_ws/src/pkg_robot_model/urdf/pkg_robot_model.urdf")
)

# j_4 / j_54 are the elbow flexion joints.  Their child-frame origins are the
# elbow locations; L6 / R56 are the arm end frames used by MoveIt.
ARM = {
    "left": {
        "names": LEFT_NAMES,
        "base": BASE_LEFT,
        "shoulder": "L1",
        "elbow": "L4",
        "wrist": "L6",
    },
    "right": {
        "names": RIGHT_NAMES,
        "base": BASE_RIGHT,
        "shoulder": "R51",
        "elbow": "R54",
        "wrist": "R56",
    },
}


def laban_vector_to_robot(vector: Sequence[float]) -> np.ndarray:
    """Laban (left, up, forward) -> X1 base (forward, left, up).

    decoder.py encodes Right as negative Laban x and Left as positive x, hence
    that first component maps directly to the robot's +y (left) axis.
    """
    left, up, forward = map(float, vector)
    return np.array([forward, left, up], dtype=float)


class X1PinkLabanMapper:
    """IK solver biased to the same safe, ready-pose branch at every keyframe."""

    def __init__(
        self,
        urdf_path: str | Path | None = None,
        max_iterations: int = 300,
        # Targets come from human-proportioned Laban directions, so even a good
        # solve keeps a residual: X1's ready pose does not hang straight down, and
        # elbow+wrist positions together over-constrain a 6-DOF arm.  This limit
        # only rejects wild solutions; joint safety is checked separately.
        tolerance_m: float = 0.250,
        retarget_gain: float | None = None,
    ) -> None:
        path = Path(urdf_path or os.environ.get("X1_URDF", DEFAULT_URDF)).expanduser()
        if not path.is_file():
            raise FileNotFoundError(
                "X1 URDF not found at %s (set X1_URDF to override)" % path
            )

        self.model = pin.buildModelFromUrdf(str(path))
        self.data = self.model.createData()
        self.jmap = {
            self.model.names[jid]: (
                self.model.idx_qs[jid],
                self.model.nqs[jid],
            )
            for jid in range(1, self.model.njoints)
        }
        self.max_iterations = int(max_iterations)
        self.tolerance_m = float(os.environ.get("LABAN_IK_TOLERANCE_M", tolerance_m))
        if retarget_gain is None:
            retarget_gain = float(os.environ.get("LABAN_IK_GAIN", "1.00"))
        self.retarget_gain = float(retarget_gain)
        if not 0.0 < self.retarget_gain <= 1.0:
            raise ValueError("LABAN_IK_GAIN must be in (0, 1]")

        self.q_nominal = pin.neutral(self.model)
        for name, angle in zip(LEFT_NAMES, BASE_LEFT):
            self._set_angle(self.q_nominal, name, angle)
        for name, angle in zip(RIGHT_NAMES, BASE_RIGHT):
            self._set_angle(self.q_nominal, name, angle)
        self.q = self.q_nominal.copy()
        self.last_gain = self.retarget_gain

        # Position-only tasks are intentional: Laban describes limb directions,
        # not a full six-DOF palm orientation.
        self.tasks: Dict[str, Tuple[FrameTask, FrameTask]] = {}
        for side, cfg in ARM.items():
            elbow = FrameTask(
                cfg["elbow"],
                position_cost=1.0,
                orientation_cost=0.0,
                lm_damping=1.0,
            )
            wrist = FrameTask(
                cfg["wrist"],
                position_cost=1.0,
                orientation_cost=0.0,
                lm_damping=1.0,
            )
            self.tasks[side] = (elbow, wrist)

        continuity_cost = float(os.environ.get("LABAN_IK_POSTURE_COST", "0.20"))
        self.continuity_posture = PostureTask(cost=continuity_cost)
        self.continuity_posture.set_target(self.q_nominal)
        self.nominal_posture = PostureTask(cost=2e-2)
        self.nominal_posture.set_target(self.q_nominal)
        self.geometry = self._measure_geometry()

    def _set_angle(self, q: np.ndarray, name: str, angle: float) -> None:
        if name not in self.jmap:
            raise RuntimeError("joint %s is absent from the X1 URDF" % name)
        qi, nq = self.jmap[name]
        if nq == 1:
            q[qi] = angle
        else:
            q[qi] = math.cos(angle)
            q[qi + 1] = math.sin(angle)

    def _get_angle(self, q: np.ndarray, name: str) -> float:
        qi, nq = self.jmap[name]
        if nq == 1:
            return float(q[qi])
        return math.atan2(float(q[qi + 1]), float(q[qi]))

    def _frame_position(self, config: Configuration, frame: str) -> np.ndarray:
        return config.get_transform_frame_to_world(frame).translation.copy()

    def _measure_geometry(self) -> Dict[str, Dict[str, np.ndarray | float]]:
        """Measure shoulder locations and robot-specific limb lengths from FK."""
        config = Configuration(self.model, self.data, self.q_nominal)
        out: Dict[str, Dict[str, np.ndarray | float]] = {}
        for side, cfg in ARM.items():
            shoulder = self._frame_position(config, cfg["shoulder"])
            elbow = self._frame_position(config, cfg["elbow"])
            wrist = self._frame_position(config, cfg["wrist"])
            out[side] = {
                "shoulder": shoulder,
                "upper_length": float(np.linalg.norm(elbow - shoulder)),
                "fore_length": float(np.linalg.norm(wrist - elbow)),
                "upper_neutral": (elbow - shoulder) / np.linalg.norm(elbow - shoulder),
                "fore_neutral": (wrist - elbow) / np.linalg.norm(wrist - elbow),
            }
        return out

    def _blend_direction(
        self, neutral: np.ndarray, laban: np.ndarray, gain: float | None = None
    ) -> np.ndarray:
        g = self.retarget_gain if gain is None else float(gain)
        direction = (1.0 - g) * neutral + g * laban
        norm = float(np.linalg.norm(direction))
        if norm < 1e-9:
            return neutral.copy()
        return direction / norm

    def _targets(
        self,
        side: str,
        elbow_laban: LimbSpherical,
        wrist_laban: LimbSpherical,
        gain: float | None = None,
    ) -> Tuple[np.ndarray, np.ndarray]:
        geom = self.geometry[side]
        shoulder = np.asarray(geom["shoulder"])
        upper = self._blend_direction(
            np.asarray(geom["upper_neutral"]),
            laban_vector_to_robot(elbow_laban.vector),
            gain,
        )
        fore = self._blend_direction(
            np.asarray(geom["fore_neutral"]),
            laban_vector_to_robot(wrist_laban.vector),
            gain,
        )
        elbow = shoulder + float(geom["upper_length"]) * upper
        wrist = elbow + float(geom["fore_length"]) * fore
        return elbow, wrist

    def _residual(self, q: np.ndarray, targets) -> float:
        config = Configuration(self.model, self.data, q)
        errors = []
        for side in ("left", "right"):
            cfg = ARM[side]
            target_e, target_w = targets[side]
            errors.append(
                float(np.linalg.norm(self._frame_position(config, cfg["elbow"]) - target_e))
            )
            errors.append(
                float(np.linalg.norm(self._frame_position(config, cfg["wrist"]) - target_w))
            )
        return max(errors)

    def reset(self) -> None:
        """Forget the previous solution and start from the ready pose again.

        map_limbs seeds from self.q and also pulls toward it through
        continuity_posture, which is what keeps one gesture continuous. Across
        gestures it is wrong: a mapper reused for a second gesture starts from the
        first one's final pose and can settle on a different IK branch (measured
        0.38 rad on the left elbow for away -> hello, and the arm can end up
        reaching across the body). The one-shot player never hit this because it
        builds a mapper per gesture and per amplitude attempt; a resident player
        has to ask for the same fresh start.
        """
        self.q = self.q_nominal.copy()
        self.last_gain = self.retarget_gain

    def _seeds(self):
        """Previous pose, ready pose, then elbow-biased variants.

        The first two cover continuity and branch resets.  The variants exist
        because both can converge onto the same near-limit shoulder branch
        (j_2 = -2.95 on `hello.json`); nudging the elbow joints first sends the
        solver down a different, in-range branch.
        """
        seeds = [self.q.copy(), self.q_nominal.copy()]
        for elbow_bias in (0.9, -0.9):
            q = self.q_nominal.copy()
            for name in ("j_4", "j_54"):
                if name in self.jmap:
                    self._set_angle(q, name, elbow_bias)
            seeds.append(q)
        return seeds

    def _solve_from(self, seed: np.ndarray, targets, tasks):
        q = seed.copy()
        dt = 0.05
        residual = self._residual(q, targets)
        for _ in range(self.max_iterations):
            if residual <= 0.020:
                break
            config = Configuration(self.model, self.data, q)
            velocity = solve_ik(config, tasks, dt, solver="quadprog")
            q = pin.integrate(self.model, q, velocity * dt)
            residual = self._residual(q, targets)
            if float(np.linalg.norm(velocity)) < 1e-4:
                break
        return q, residual

    def map_limbs(
        self, limbs: Dict[str, LimbSpherical]
    ) -> Tuple[List[float], List[float]]:
        rest_e = LimbSpherical(
            "Place", "Low", math.radians(175), 0.0, (0.0, -1.0, 0.0)
        )
        rest_w = LimbSpherical(
            "Forward", "Low", math.radians(135), 0.0, (0.0, -0.7, 0.7)
        )

        names = list(LEFT_NAMES) + list(RIGHT_NAMES)
        previous = [self._get_angle(self.q, name) for name in names]

        targets = {}
        for side in ("left", "right"):
            elbow_laban = limbs.get("%s elbow" % side, rest_e)
            wrist_laban = limbs.get("%s wrist" % side, rest_w)
            targets[side] = self._targets(side, elbow_laban, wrist_laban)

        all_tasks = []
        for side in ("left", "right"):
            elbow_task, wrist_task = self.tasks[side]
            elbow_target, wrist_target = targets[side]
            elbow_task.set_target(pin.SE3(np.eye(3), elbow_target))
            wrist_task.set_target(pin.SE3(np.eye(3), wrist_target))
            all_tasks.extend((elbow_task, wrist_task))
        # The preceding solution is both the seed and a soft posture target.  This
        # selects a continuous IK branch.  A weaker nominal target prevents gradual
        # drift over long gestures without forcing every pose back toward ready.
        self.continuity_posture.set_target(self.q.copy())
        all_tasks.extend((self.continuity_posture, self.nominal_posture))

        candidates = []
        for seed in self._seeds():
            q, residual = self._solve_from(seed, targets, all_tasks)
            angles = [self._get_angle(q, name) for name in names]
            worst = max(abs(value) for value in angles)
            travel = max(abs(a - b) for a, b in zip(angles, previous))
            candidates.append((q, residual, worst, travel, angles))

        usable = [
            c
            for c in candidates
            if c[2] <= JOINT_SAFETY_RAD and c[1] <= self.tolerance_m
        ]
        if not usable:
            best = min(candidates, key=lambda c: c[1])
            if best[1] > self.tolerance_m:
                raise RuntimeError(
                    "Laban IK did not converge: position residual %.3f m (limit %.3f m)"
                    % (best[1], self.tolerance_m)
                )
            worst_name, worst_value = max(
                zip(names, best[4]), key=lambda item: abs(item[1])
            )
            raise RuntimeError(
                "IK selected %s=%.3f rad outside the conservative +/-%.1f rad range"
                % (worst_name, worst_value, JOINT_SAFETY_RAD)
            )

        # Continuity beats amplitude: a solution reached by flipping the shoulder
        # to the opposite branch is in range but unplayable.
        continuous = [c for c in usable if c[3] <= MAX_KEYFRAME_TRAVEL_RAD]
        chosen = min(continuous or usable, key=lambda c: c[3])
        self.q = chosen[0]
        angles = chosen[4]
        return angles[: len(LEFT_NAMES)], angles[len(LEFT_NAMES) :]
