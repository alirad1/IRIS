"""Hand point -> smooth joint targets. Pure logic, unit tested, no I/O.

Pipeline per frame:
  hand pixel -> one-euro filter -> pixel deadband -> aim map (clamped to the
  calibrated area) -> clamp to joint limits -> target.
Hand lost for longer than a short grace period -> one "hold" (the motor service
brakes and stays), then nothing until the hand is back.
The motor service adds its own speed/accel limits on top.
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import List, Optional, Sequence, Tuple


class OneEuro:
    """One-euro filter (Casiez et al. 2012) on a 2D point. Strong smoothing when
    still, less lag when moving fast."""

    def __init__(self, min_cutoff: float = 0.4, beta: float = 0.005, d_cutoff: float = 1.0):
        self.min_cutoff, self.beta, self.d_cutoff = min_cutoff, beta, d_cutoff
        self.reset()

    def reset(self) -> None:
        self.x: Optional[Tuple[float, float]] = None
        self.dx = (0.0, 0.0)
        self.t: Optional[float] = None

    @staticmethod
    def _alpha(cutoff: float, dt: float) -> float:
        tau = 1.0 / (2 * math.pi * cutoff)
        return 1.0 / (1.0 + tau / dt)

    def __call__(self, p: Sequence[float], t: float) -> Tuple[float, float]:
        p = (float(p[0]), float(p[1]))
        if self.x is None or self.t is None or t <= self.t:
            self.x, self.t = p, t
            return p
        dt = t - self.t
        self.t = t
        raw_dx = ((p[0] - self.x[0]) / dt, (p[1] - self.x[1]) / dt)
        ad = self._alpha(self.d_cutoff, dt)
        self.dx = (self.dx[0] + ad * (raw_dx[0] - self.dx[0]),
                   self.dx[1] + ad * (raw_dx[1] - self.dx[1]))
        speed = math.hypot(*self.dx)
        a = self._alpha(self.min_cutoff + self.beta * speed, dt)
        self.x = (self.x[0] + a * (p[0] - self.x[0]), self.x[1] + a * (p[1] - self.x[1]))
        return self.x


@dataclass
class Decision:
    kind: str                      # "target", "hold" or "none"
    targets: Optional[List[float]] = None
    aim_px: Optional[Tuple[float, float]] = None
    clamped: bool = False
    filtered_px: Optional[Tuple[float, float]] = None
    state: str = ""                # FOLLOW / OUT OF REACH / LOST / PAUSED


class Follower:
    def __init__(self, aim_map, limits: Sequence[Tuple[float, float]],
                 deadband_px: float = 10.0, lost_grace_s: float = 0.4,
                 min_cutoff: float = 0.4, beta: float = 0.005) -> None:
        self.map = aim_map
        self.limits = list(limits)
        self.deadband_px = deadband_px
        self.lost_grace_s = lost_grace_s
        self.filter = OneEuro(min_cutoff, beta)
        self.committed: Optional[Tuple[float, float]] = None
        self.target: Optional[List[float]] = None
        self.aim_px: Optional[Tuple[float, float]] = None
        self.clamped = False
        self.last_seen: Optional[float] = None
        self.hold_sent = True  # starts holding; nothing sent until a hand shows up
        self.paused = False

    def pause(self, paused: bool) -> Decision:
        self.paused = paused
        if paused:
            self._lose()
            return Decision("hold", state="PAUSED")
        return Decision("none", state="LOST")

    def _lose(self) -> None:
        self.hold_sent = True
        self.target = None
        self.committed = None
        self.filter.reset()

    def update(self, hand_px: Optional[Sequence[float]], t: float) -> Decision:
        if self.paused:
            return Decision("none", state="PAUSED")
        if hand_px is None:
            if (self.target is not None and self.last_seen is not None
                    and t - self.last_seen <= self.lost_grace_s):
                return Decision("target", list(self.target), self.aim_px, self.clamped,
                                None, "FOLLOW")
            if not self.hold_sent:
                self._lose()
                return Decision("hold", state="LOST")
            return Decision("none", state="LOST")

        self.last_seen = t
        f = self.filter(hand_px, t)
        if (self.committed is None
                or math.hypot(f[0] - self.committed[0], f[1] - self.committed[1]) > self.deadband_px):
            self.committed = f
            joints, aim, clamped = self.map.joints_for(f)
            self.target = [min(max(x, lo), hi) for x, (lo, hi) in zip(joints, self.limits)]
            self.aim_px, self.clamped = aim, clamped
        self.hold_sent = False
        return Decision("target", list(self.target), self.aim_px, self.clamped, f,
                        "OUT OF REACH" if self.clamped else "FOLLOW")
