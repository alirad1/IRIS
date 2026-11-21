"""Pure motor logic: no I/O, no hardware. Everything here is unit tested.

THE PAIR RULE: CH1 and CH2 are one logical joint, `pair`. servos[0] gets
+pair and servos[1] gets -pair, each around 135 plus its own trim. Nothing in
this module (or anywhere else) lets a caller address a channel directly; the
only way to move servos is ServoBus.apply() with all 4 logical joints.
"""
from __future__ import annotations

import math
from typing import Dict, List, Optional, Sequence, Tuple

from iris.config import (JOINT_NAMES, SERVO_CENTER_DEG, SERVO_RANGE_DEG,
                         Config, JointCfg)

N_JOINTS = len(JOINT_NAMES)
PAIR_SIGNS = (+1, -1)
_PAIR_TOL = 1e-6


class CommandError(ValueError):
    """A target command was rejected. Nothing moves because of it."""


def servo_signs(joint_name: str) -> Tuple[int, ...]:
    return PAIR_SIGNS if joint_name == "pair" else (+1,)


def physical_envelope(joint: JointCfg) -> Tuple[float, float]:
    """Logical range in which every servo of this joint stays inside 0..270.

    For the pair this is the intersection over both servos, so the pair is
    always limited as one value and both sides shrink together.
    """
    lo, hi = -math.inf, math.inf
    for s, sign in zip(joint.servos, servo_signs(joint.name)):
        # 0 <= 135 + trim + sign * x <= 270
        a = (0.0 - SERVO_CENTER_DEG - s.trim) / sign
        b = (SERVO_RANGE_DEG - SERVO_CENTER_DEG - s.trim) / sign
        lo, hi = max(lo, min(a, b)), min(hi, max(a, b))
    return lo, hi


def validate_targets(targets: object) -> List[float]:
    """Exactly 4 finite numbers [j0, pair, j3, j4], or CommandError."""
    if not isinstance(targets, (list, tuple)) or len(targets) != N_JOINTS:
        raise CommandError(f"targets must be a list of {N_JOINTS} numbers [j0, pair, j3, j4]")
    out = []
    for name, v in zip(JOINT_NAMES, targets):
        if isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(v):
            raise CommandError(f"target for {name} must be a finite number, got {v!r}")
        out.append(float(v))
    return out


def _clamp(x: float, lo: float, hi: float) -> float:
    return lo if x < lo else hi if x > hi else x


class ServoBus:
    """Maps the 4 logical joints to servo angles and writes them all at once.

    `driver` must provide write_all({channel: angle}) for every configured
    channel, release_all() and deinit(). Only this class calls write_all.
    """

    def __init__(self, cfg: Config, driver) -> None:
        self._joints = cfg.joints
        self._driver = driver
        self._envelopes = [physical_envelope(j) for j in cfg.joints]

    def servo_angles(self, logical: Sequence[float]) -> Dict[int, float]:
        values = validate_targets(list(logical))
        angles: Dict[int, float] = {}
        for joint, env, x in zip(self._joints, self._envelopes, values):
            x = _clamp(x, *env)  # one value per joint, pair included
            for s, sign in zip(joint.servos, servo_signs(joint.name)):
                angles[s.channel] = SERVO_CENTER_DEG + s.trim + sign * x
        self._check_pair(angles)
        return angles

    def _check_pair(self, angles: Dict[int, float]) -> None:
        pair = self._joints[JOINT_NAMES.index("pair")]
        s1, s2 = pair.servos
        d1 = angles[s1.channel] - SERVO_CENTER_DEG - s1.trim
        d2 = angles[s2.channel] - SERVO_CENTER_DEG - s2.trim
        if abs(d1 + d2) > _PAIR_TOL:
            raise AssertionError(f"pair rule violated: {d1} vs {d2}; refusing to write")
        for ch, a in angles.items():
            if not 0.0 <= a <= SERVO_RANGE_DEG:
                raise AssertionError(f"channel {ch} angle {a} outside 0..270; refusing to write")

    def apply(self, logical: Sequence[float]) -> Dict[int, float]:
        angles = self.servo_angles(logical)
        self._driver.write_all(angles)
        return angles

    def release(self) -> None:
        self._driver.release_all()


class MotionController:
    """Clamp, speed/accel limit and command timeout for the 4 logical joints.

    Times are seconds from any monotonic clock; the caller passes them in so
    tests can use a fake clock.
    """

    def __init__(self, cfg: Config, calibrate: bool = False) -> None:
        self.calibrate = calibrate
        if calibrate:
            # Physical servo range, but tiny steps and slow speed.
            self.limits = [physical_envelope(j) for j in cfg.joints]
            self.max_speed = min(cfg.motor.max_speed_dps, cfg.calibration.max_speed_dps)
            self.max_step: Optional[float] = cfg.calibration.max_step_deg
        else:
            self.limits = [(j.min, j.max) for j in cfg.joints]
            self.max_speed = cfg.motor.max_speed_dps
            self.max_step = None
        self.max_accel = cfg.motor.max_accel_dps2
        self.timeout_s = cfg.motor.timeout_s
        self.home = [j.home for j in cfg.joints]
        self.pos = list(self.home)
        self.vel = [0.0] * N_JOINTS
        self.target = list(self.home)
        self.holding = True
        self._last_cmd_t: Optional[float] = None
        self._last_step_t: Optional[float] = None

    def reset_to_home(self, t: float) -> List[float]:
        """Startup: position is unknown, so we assume and command home."""
        self.pos = list(self.home)
        self.vel = [0.0] * N_JOINTS
        self.target = list(self.home)
        self.holding = True
        self._last_cmd_t = None
        self._last_step_t = t
        return list(self.pos)

    def clamp(self, targets: Sequence[float]) -> List[float]:
        return [_clamp(x, lo, hi) for x, (lo, hi) in zip(targets, self.limits)]

    def set_target(self, targets: object, t: float) -> List[float]:
        values = self.clamp(validate_targets(targets))
        if self.max_step is not None:
            # Re-sending the already accepted target is a keep-alive, not a new step,
            # even if braking carried the joint a little past it.
            for name, x, p, cur in zip(JOINT_NAMES, values, self.pos, self.target):
                if abs(x - p) > self.max_step + 1e-9 and abs(x - cur) > 1e-9:
                    raise CommandError(
                        f"calibration mode: {name} target {x:.2f} is more than "
                        f"{self.max_step} deg from position {p:.2f}")
        self.target = values
        self.holding = False
        self._last_cmd_t = t
        return list(values)

    def _stopping_point(self, i: int) -> float:
        v = self.vel[i]
        x = self.pos[i] + math.copysign(v * v / (2.0 * self.max_accel), v)
        lo, hi = self.limits[i]
        return _clamp(x, lo, hi)

    def hold(self) -> None:
        """Brake to a stop along the current direction, then stay there."""
        self.target = [self._stopping_point(i) for i in range(N_JOINTS)]
        self.holding = True

    def step(self, t: float) -> List[float]:
        if self._last_step_t is None:
            self._last_step_t = t
        dt = t - self._last_step_t
        self._last_step_t = t
        if dt <= 0:
            return list(self.pos)
        dt = min(dt, 0.1)  # a stalled loop never turns into one big jump

        if (not self.holding and self._last_cmd_t is not None
                and t - self._last_cmd_t > self.timeout_s):
            self.hold()

        a, vmax = self.max_accel, self.max_speed
        for i in range(N_JOINTS):
            err = self.target[i] - self.pos[i]
            v = self.vel[i]
            if err == 0.0 and v == 0.0:
                continue
            # Fastest speed from which we can still stop exactly at the target.
            v_des = math.copysign(min(vmax, math.sqrt(2.0 * a * abs(err))), err) if err else 0.0
            dv = _clamp(v_des - v, -a * dt, a * dt)
            v_new = v + dv
            move = v_new * dt
            if err != 0.0 and ((err > 0 and move >= err) or (err < 0 and move <= err)):
                self.pos[i] = self.target[i]
                self.vel[i] = 0.0
            else:
                self.pos[i] += move
                self.vel[i] = v_new
            lo, hi = self.limits[i]
            if not lo <= self.pos[i] <= hi:
                self.pos[i] = _clamp(self.pos[i], lo, hi)
                self.vel[i] = 0.0
        return list(self.pos)

    def state(self) -> dict:
        return {
            "pos": [round(x, 3) for x in self.pos],
            "target": [round(x, 3) for x in self.target],
            "holding": self.holding,
            "limits": [[round(lo, 3), round(hi, 3)] for lo, hi in self.limits],
            "calibrate": self.calibrate,
        }
