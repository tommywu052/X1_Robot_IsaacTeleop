# -*- coding: utf-8 -*-
"""Which of the 92 gestures are safe to play, and for whom.

At full amplitude the retarget commands genuine self-collisions in 13 of the 92
library samples: the URDF collision meshes interpenetrate, min gap 0.000 m, and
`thanks` does it in 6 of its 8 keyframes. The arms only clear each other on the
real robot because gravity opens the shoulders. Measured against the MoveIt
SRDF's surviving collision pairs and documented in `laban/README.md`.

A caller that reaches a gesture by keyword, prefix or string similarity -- or
that falls back to a random pick when nothing matches -- can land on any of the
13. Gating one of those paths would leave the others open, which is why this is
applied where the catalogue is built rather than where a gesture is chosen, and
again in the daemon, which is the one place every caller has to pass through.

Two profiles, because callers are not equally supervised:

    chat     ok + caution     a human is picking gestures and watching the robot
    aspire   ok only          an agent is choosing autonomously

`caution` is the tier for gestures whose computed clearance is smaller than the
shoulders' standing tracking error (0.019 rad at the ready pose, 0.117 rad in a
closed two-handed pose). 4 mm of clearance is not a clearance the hardware can be
trusted to keep, but most of the 0.004 m entries are the head-only gestures --
`nod`, `shake head`, `laugh head` -- where the arms hold one pose throughout and
which have never been observed to touch. Removing those from conversation would
be a large behaviour regression to fix a risk that has not materialised, so they
stay available to `chat` and are kept out of the agent's search space.

BLOCKED_SELF_COLLISION is a compiled-in constant on purpose. The tiers and the
measured gaps live in `gesture_allowlist.yaml` next to this file, but a safety
property that disappears when a config file is absent is not a safety property.
If the config and this constant ever disagree, the union is blocked and the
mismatch is reported rather than one of them silently winning.
"""
from __future__ import annotations

import sys
from pathlib import Path
from typing import Dict, Iterable, List, Optional, Sequence

# Measured 2026-09-01: min gap 0.000 m against the MoveIt SRDF's 169 surviving
# collision pairs.
BLOCKED_SELF_COLLISION = frozenset({
    "bad",
    "help4",
    "howareyou4",
    "please4",
    "sorry",
    "sorry2",
    "thanks",
    "think d high l",
    "think d high s",
    "think d low l",
    "think d low s",
    "tired",
    "youandme4",
})

CONFIG_SEARCH: Sequence[Path] = (
    Path(__file__).resolve().parent / "gesture_allowlist.yaml",
)

PROFILES: Dict[str, frozenset] = {
    "chat": frozenset({"ok", "caution", "unknown"}),
    "aspire": frozenset({"ok"}),
}


class GesturePolicy:
    """Tier lookup for library stems, with or without the measured config.

    Without the config every non-blocked gesture reads as `unknown`, which
    `chat` allows and `aspire` does not. That asymmetry is deliberate: a human
    may play a gesture nobody has measured, an autonomous search may not.
    """

    def __init__(
        self,
        profile: str = "chat",
        config_path: Path | str | None = None,
        warn=None,
    ):
        if profile not in PROFILES:
            raise ValueError(
                "unknown gesture profile %r; expected one of %s"
                % (profile, ", ".join(sorted(PROFILES)))
            )
        self.profile = profile
        self._allowed_tiers = PROFILES[profile]
        self._warn = warn or (lambda msg: sys.stderr.write("[gesture-policy] %s\n" % msg))

        self.blocked: Dict[str, str] = {
            stem: "self-collision at full amplitude (min gap 0.000 m)"
            for stem in BLOCKED_SELF_COLLISION
        }
        self.caution: Dict[str, float] = {}
        self.ok: Dict[str, float] = {}
        self.head_only: frozenset = frozenset()
        self.reduced_amplitude: Dict[str, float] = {}
        self.config_path: Optional[Path] = None

        data = self._load(config_path)
        if data:
            self._apply(data)

    # ------------------------------------------------------------------ config

    def _load(self, config_path):
        candidates = [Path(config_path)] if config_path else list(CONFIG_SEARCH)
        for path in candidates:
            if not path.is_file():
                continue
            try:
                import yaml
            except ImportError:
                self._warn(
                    "PyYAML missing, so %s was not read: only the compiled-in "
                    "self-collision block is in force" % path.name
                )
                return None
            try:
                data = yaml.safe_load(path.read_text(encoding="utf-8")) or {}
            except Exception as exc:  # a malformed config must not open the gate
                self._warn("could not parse %s (%s); keeping the compiled-in block" % (path, exc))
                return None
            self.config_path = path
            return data
        if config_path:
            self._warn("no gesture allow-list at %s" % config_path)
        else:
            self._warn(
                "no gesture allow-list found (looked in %s); the %d compiled-in "
                "self-collision blocks still apply, tiers do not"
                % (", ".join(str(p) for p in CONFIG_SEARCH), len(BLOCKED_SELF_COLLISION))
            )
        return None

    def _apply(self, data: dict) -> None:
        for stem, detail in (data.get("blocked") or {}).items():
            gap = detail.get("min_gap_m") if isinstance(detail, dict) else detail
            frames = detail.get("colliding_keyframes") if isinstance(detail, dict) else None
            reason = "self-collision, min gap %.3f m" % float(gap or 0.0)
            if frames:
                reason += " in %d keyframe%s" % (frames, "" if frames == 1 else "s")
            self.blocked[str(stem)] = reason

        self.caution = {str(k): float(v) for k, v in (data.get("caution") or {}).items()}
        self.ok = {str(k): float(v) for k, v in (data.get("ok") or {}).items()}
        self.head_only = frozenset(str(x) for x in (data.get("head_only") or []))
        self.reduced_amplitude = {
            str(k): float(v) for k, v in (data.get("reduced_amplitude") or {}).items()
        }

        # A config that disagrees with the compiled-in constant means one of the
        # two is describing a different robot. Say so rather than picking one.
        measured_block = set(data.get("blocked") or {})
        if measured_block and measured_block != set(BLOCKED_SELF_COLLISION):
            only_config = sorted(measured_block - set(BLOCKED_SELF_COLLISION))
            only_code = sorted(set(BLOCKED_SELF_COLLISION) - measured_block)
            self._warn(
                "allow-list and BLOCKED_SELF_COLLISION disagree; blocking the union. "
                "config only: %s | code only: %s. Re-measure and update the constant."
                % (only_config or "-", only_code or "-")
            )

    # ------------------------------------------------------------------- query

    def tier(self, stem: str) -> str:
        stem = str(stem)
        if stem in self.blocked:
            return "blocked"
        if stem in self.caution:
            return "caution"
        if stem in self.ok:
            return "ok"
        return "unknown"

    def allowed(self, stem: str) -> bool:
        return self.tier(stem) in self._allowed_tiers

    def reason(self, stem: str) -> str:
        tier = self.tier(stem)
        if tier == "blocked":
            return self.blocked[str(stem)]
        if tier == "caution":
            return "clearance %.3f m, inside the shoulders' tracking error" % self.caution[str(stem)]
        if tier == "unknown":
            return "not in the measured allow-list"
        return "clearance %.3f m" % self.ok[str(stem)]

    def filter(self, names: Iterable[str]) -> List[str]:
        """Keep the file or stem names this profile may play."""
        return [n for n in names if self.allowed(Path(str(n)).stem)]

    def rejected(self, names: Iterable[str]) -> List[tuple]:
        """(name, tier, reason) for everything filter() would drop."""
        out = []
        for name in names:
            stem = Path(str(name)).stem
            if not self.allowed(stem):
                out.append((stem, self.tier(stem), self.reason(stem)))
        return out

    def summary(self) -> str:
        return (
            "profile=%s tiers=%s config=%s blocked=%d caution=%d ok=%d"
            % (
                self.profile,
                "+".join(sorted(self._allowed_tiers)),
                self.config_path.name if self.config_path else "none",
                len(self.blocked),
                len(self.caution),
                len(self.ok),
            )
        )


def default_policy(profile: str = "chat", **kwargs) -> GesturePolicy:
    return GesturePolicy(profile=profile, **kwargs)


if __name__ == "__main__":
    import argparse

    ap = argparse.ArgumentParser(description="Report the gesture policy for a library.")
    ap.add_argument("--profile", default="chat", choices=sorted(PROFILES))
    ap.add_argument("--library", default=str(Path(__file__).resolve().parent / "gestures" / "library"))
    ap.add_argument("--config", default=None)
    args = ap.parse_args()

    policy = GesturePolicy(profile=args.profile, config_path=args.config)
    names = sorted(p.name for p in Path(args.library).glob("*.json"))
    kept = policy.filter(names)
    print(policy.summary())
    print("library %d -> playable %d" % (len(names), len(kept)))
    for stem, tier, reason in policy.rejected(names):
        print("  %-9s %-26s %s" % (tier, stem, reason))
