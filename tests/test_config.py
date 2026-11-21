from __future__ import annotations

import copy
import unittest

from iris import config as config_mod
from iris.config import ConfigError

from tests.helpers import BASE, raw


def bad(d):
    with unittest.TestCase().assertRaises(ConfigError):
        config_mod.from_dict(d)


class ConfigTests(unittest.TestCase):
    def test_repo_config_loads_with_uncalibrated_limits(self):
        c = config_mod.load()
        for j in c.joints:
            self.assertEqual((j.min, j.max), (-10, 10), j.name)
        self.assertEqual(c.channels, (0, 1, 2, 3, 4))
        self.assertEqual(c.motor.timeout_s, 0.5)

    def test_sign_key_rejected(self):
        d = raw(pair={"servos": [{"channel": 1, "trim": 0, "sign": 1},
                                 {"channel": 2, "trim": 0, "sign": 1}]})
        bad(d)

    def test_pair_needs_exactly_two_servos(self):
        bad(raw(pair={"servos": [{"channel": 1, "trim": 0}]}))
        bad(raw(pair={"servos": [{"channel": 1, "trim": 0}, {"channel": 2, "trim": 0},
                                 {"channel": 5, "trim": 0}]}))

    def test_single_joints_need_one_servo(self):
        bad(raw(j0={"servos": [{"channel": 0, "trim": 0}, {"channel": 5, "trim": 0}]}))

    def test_duplicate_channel_rejected(self):
        bad(raw(j3={"servos": [{"channel": 1, "trim": 0}]}))
        bad(raw(pair={"servos": [{"channel": 1, "trim": 0}, {"channel": 1, "trim": 0}]}))

    def test_home_must_be_inside_limits(self):
        bad(raw(j0={"home": 11}))
        bad(raw(j0={"min": 5, "max": 2}))

    def test_limits_must_fit_servo_range_with_trim(self):
        bad(raw(pair={"min": -10, "max": 120,
                      "servos": [{"channel": 1, "trim": 20}, {"channel": 2, "trim": 0}]}))

    def test_hard_caps(self):
        for key, val in (("timeout_s", 0.6), ("max_speed_dps", 31), ("max_accel_dps2", 500),
                         ("timeout_s", 0)):
            d = copy.deepcopy(BASE)
            d["motor"][key] = val
            bad(d)
        d = copy.deepcopy(BASE)
        d["calibration"]["max_step_deg"] = 5
        bad(d)
        d = copy.deepcopy(BASE)
        d["calibration"]["max_speed_dps"] = 10
        bad(d)

    def test_unknown_and_missing_keys(self):
        d = copy.deepcopy(BASE)
        d["joints"]["j5"] = d["joints"]["j0"]
        bad(d)
        d = copy.deepcopy(BASE)
        del d["joints"]["j4"]
        bad(d)
        d = copy.deepcopy(BASE)
        d["motor"]["oops"] = 1
        bad(d)

    def test_non_numbers(self):
        bad(raw(j0={"min": "a"}))
        bad(raw(j0={"max": True}))
        bad(raw(j0={"servos": [{"channel": 0.0, "trim": 0}]}))
        bad(raw(j0={"servos": [{"channel": 16, "trim": 0}]}))


if __name__ == "__main__":
    unittest.main()
