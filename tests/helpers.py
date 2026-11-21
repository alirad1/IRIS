from __future__ import annotations

import copy

from iris import config as config_mod
from iris.motor.hw import FakePCA9685
from iris.motor.service import MotorService

# Tests use their own fixed config (uncalibrated: -10..10, home 0, no invert) so
# they don't change when the real config/iris.json is calibrated.
_real = config_mod.load_raw()
BASE = copy.deepcopy(_real)
for _name, _j in BASE["joints"].items():
    _j.pop("invert", None)
    _j.update({"min": -10, "max": 10, "home": 0})
    for _s in _j["servos"]:
        _s["trim"] = 0


def raw(**joint_overrides):
    """A copy of the repo config with per-joint overrides merged in."""
    d = copy.deepcopy(BASE)
    for name, over in joint_overrides.items():
        d["joints"][name].update(over)
    return d


def cfg(**joint_overrides):
    return config_mod.from_dict(raw(**joint_overrides))


def wide_cfg(speed=10.0, accel=40.0, **joint_overrides):
    """Test-only config with wide limits so motion tests have room."""
    d = raw(**joint_overrides)
    for j in d["joints"].values():
        j.setdefault("min", -10)
        j["min"], j["max"] = min(j["min"], -90), max(j["max"], 90)
    d["motor"]["max_speed_dps"] = speed
    d["motor"]["max_accel_dps2"] = accel
    return config_mod.from_dict(d)


def service(c=None, calibrate=False, t=0.0):
    c = c or cfg()
    drv = FakePCA9685(c.pca, c.channels)
    svc = MotorService(c, drv, calibrate=calibrate)
    svc.start(t)
    return svc, drv


class Clock:
    def __init__(self, svc, t=0.0, dt=0.02):
        self.svc, self.t, self.dt = svc, t, dt

    def run(self, seconds):
        positions = []
        for _ in range(int(round(seconds / self.dt))):
            self.t += self.dt
            self.svc.tick(self.t)
            positions.append(list(self.svc.controller.pos))
        return positions
