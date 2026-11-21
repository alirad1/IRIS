"""Pair rule, clamping, rate limiting, timeout, startup and release.

All on FakePCA9685. Run: python -m unittest
"""
from __future__ import annotations

import inspect
import json
import math
import unittest

from iris import protocol
from iris.config import SERVO_CENTER_DEG
from iris.motor import core, hw
from iris.motor.core import CommandError, MotionController, ServoBus
from iris.motor.hw import FakePCA9685

from tests.helpers import Clock, cfg, service, wide_cfg

J0, PAIR, J3, J4 = range(4)


def pair_offsets(c, angles):
    s1, s2 = c.joint("pair").servos
    return (angles[s1.channel] - SERVO_CENTER_DEG - s1.trim,
            angles[s2.channel] - SERVO_CENTER_DEG - s2.trim)


def target(svc, t, values):
    return json.loads(svc.handle(protocol.encode_target(values), t))


class PairRule(unittest.TestCase):
    def test_ch1_plus_ch2_minus(self):
        c = cfg()
        bus = ServoBus(c, FakePCA9685(c.pca, c.channels))
        a = bus.servo_angles([0, 7.5, 0, 0])
        self.assertAlmostEqual(a[1], 135 + 7.5)
        self.assertAlmostEqual(a[2], 135 - 7.5)

    def test_every_write_is_mirrored_during_motion(self):
        c = wide_cfg(pair={"servos": [{"channel": 1, "trim": 4}, {"channel": 2, "trim": -6}]})
        svc, drv = service(c)
        clk = Clock(svc)
        for goal in (40, -25, 60, 0):
            target(svc, clk.t, [5, goal, -5, 10])
            for _ in range(200):
                clk.run(0.02)
                target(svc, clk.t, [5, goal, -5, 10])
        self.assertGreater(len(drv.history), 500)
        for w in drv.history:
            d1, d2 = pair_offsets(c, w)
            self.assertAlmostEqual(d1, -d2, places=9)

    def test_every_write_covers_both_pair_channels(self):
        svc, drv = service()
        Clock(svc).run(0.5)
        for w in drv.history:
            self.assertIn(1, w)
            self.assertIn(2, w)

    def test_trims_shrink_pair_envelope_for_both_sides(self):
        # CH1 trim +25: CH1 = 160 + x <= 270 -> x <= 110. CH2 = 135 - x >= 0 -> x <= 135.
        c = cfg(pair={"servos": [{"channel": 1, "trim": 25}, {"channel": 2, "trim": 0}]})
        lo, hi = core.physical_envelope(c.joint("pair"))
        self.assertAlmostEqual(hi, 110)
        self.assertAlmostEqual(lo, -135)
        bus = ServoBus(c, FakePCA9685(c.pca, c.channels))
        a = bus.servo_angles([0, 500, 0, 0])  # far past the end
        self.assertAlmostEqual(a[1], 270)
        self.assertAlmostEqual(a[2], 135 - 110)  # moved with CH1, not clamped alone
        d1, d2 = pair_offsets(c, a)
        self.assertAlmostEqual(d1, -d2)

    def test_driver_rejects_partial_write(self):
        c = cfg()
        drv = FakePCA9685(c.pca, c.channels)
        with self.assertRaises(AssertionError):
            drv.write_all({1: 140.0})
        with self.assertRaises(AssertionError):
            drv.write_all({0: 135.0, 1: 140.0, 3: 135.0, 4: 135.0})
        self.assertEqual(drv.history, [])

    def test_bus_refuses_to_write_if_mapping_breaks_rule(self):
        c = cfg()
        drv = FakePCA9685(c.pca, c.channels)
        bus = ServoBus(c, drv)
        original = core.servo_signs
        core.servo_signs = lambda name: (+1, +1) if name == "pair" else (+1,)
        try:
            with self.assertRaises(AssertionError):
                bus.apply([0, 5, 0, 0])
        finally:
            core.servo_signs = original
        self.assertEqual(drv.history, [])

    def test_protocol_has_no_way_to_name_a_channel(self):
        bad = [
            {"cmd": "target", "targets": [0, 0, 0, 0, 0]},
            {"cmd": "target", "targets": [0, 0, 0]},
            {"cmd": "target", "targets": [0, 0, 0, 0], "channel": 1},
            {"cmd": "target", "targets": {"ch1": 5, "ch2": -5}},
            {"cmd": "channel", "channel": 1, "angle": 140},
            {"cmd": "servo", "targets": [0, 0, 0, 0]},
        ]
        svc, drv = service()
        for msg in bad:
            r = json.loads(svc.handle(json.dumps(msg).encode(), 0.01))
            self.assertFalse(r["ok"], msg)
            self.assertEqual(svc.controller.target, [0, 0, 0, 0])

    def test_no_public_api_takes_a_channel(self):
        for cls in (ServoBus, MotionController):
            for name, fn in inspect.getmembers(cls, inspect.isfunction):
                if name.startswith("_"):
                    continue
                params = inspect.signature(fn).parameters
                self.assertFalse(any("channel" in p or p == "ch" for p in params),
                                 f"{cls.__name__}.{name}{tuple(params)}")


class Clamping(unittest.TestCase):
    def test_targets_clamped_to_config_limits(self):
        svc, _ = service()
        r = target(svc, 0.01, [50, -50, 10.5, -999])
        self.assertTrue(r["ok"])
        self.assertEqual(r["target"], [10, -10, 10, -10])

    def test_position_never_leaves_limits(self):
        svc, drv = service()
        clk = Clock(svc)
        for _ in range(300):
            target(svc, clk.t, [1e6, -1e6, 1e6, -1e6])
            clk.run(0.02)
        for w in drv.history:
            for ch, a in w.items():
                self.assertLessEqual(abs(a - 135), 10 + 1e-9)

    def test_non_finite_rejected_and_nothing_changes(self):
        svc, _ = service()
        for bad in ([float("nan"), 0, 0, 0], [0, float("inf"), 0, 0], [0, 0, True, 0],
                    [0, 0, "5", 0], None):
            msg = json.dumps({"cmd": "target", "targets": bad}).encode()
            r = json.loads(svc.handle(msg, 0.01))
            self.assertFalse(r["ok"], bad)
        self.assertEqual(svc.controller.target, [0, 0, 0, 0])

    def test_garbage_rejected(self):
        svc, _ = service()
        for data in (b"", b"\xff\xfe", b"[]", b"{", b"x" * 5000, b'{"cmd":"target"}'):
            r = json.loads(svc.handle(data, 0.01))
            self.assertFalse(r["ok"], data[:20])

    def test_calibrate_mode_uses_physical_range_but_small_steps(self):
        c = cfg()
        svc, _ = service(c, calibrate=True)
        r = target(svc, 0.01, [0, 0, 2.5, 0])
        self.assertTrue(r["ok"])
        r = target(svc, 0.02, [0, 0, 4.0, 0])  # 4 deg from position 0
        self.assertFalse(r["ok"])
        self.assertEqual(svc.controller.target[J3], 2.5)
        # Walk j3 past the normal +10 limit in 3-deg steps.
        clk = Clock(svc, t=0.02)
        goal = 0.0
        while goal < 15:
            goal += 3
            r = target(svc, clk.t, [0, 0, goal, 0])
            self.assertTrue(r["ok"], r)
            for _ in range(60):
                clk.run(0.02)
                target(svc, clk.t, [0, 0, goal, 0])
        self.assertAlmostEqual(svc.controller.pos[J3], 15.0)

    def test_calibrate_mode_speed_capped(self):
        svc, _ = service(wide_cfg(speed=30), calibrate=True)
        self.assertLessEqual(svc.controller.max_speed, 5.0)


class RateLimit(unittest.TestCase):
    def run_move(self, speed, accel, goal, seconds):
        c = wide_cfg(speed=speed, accel=accel)
        svc, _ = service(c)
        clk = Clock(svc)
        traj = []
        for _ in range(int(seconds / 0.02)):
            target(svc, clk.t, [0, goal, 0, 0])
            traj += clk.run(0.02)
        return [p[PAIR] for p in traj], clk.dt

    def test_speed_limit(self):
        xs, dt = self.run_move(10, 40, 80, 10)
        steps = [b - a for a, b in zip(xs, xs[1:])]
        self.assertLessEqual(max(abs(s) for s in steps), 10 * dt + 1e-9)
        self.assertAlmostEqual(xs[-1], 80)

    def test_accel_limit(self):
        xs, dt = self.run_move(10, 40, 30, 6)
        v = [(b - a) / dt for a, b in zip(xs, xs[1:])]
        dvs = [abs(b - a) for a, b in zip(v, v[1:])]
        # ignore the single arrival tick, where the last tiny bit is snapped
        arrive = next(i for i, x in enumerate(xs) if x == 30)
        self.assertLessEqual(max(dvs[:arrive - 2]), 40 * dt + 1e-6)

    def test_no_overshoot(self):
        for goal in (3, -3, 30, -45):
            xs, _ = self.run_move(10, 40, goal, 12)
            if goal > 0:
                self.assertLessEqual(max(xs), goal + 1e-9)
            else:
                self.assertGreaterEqual(min(xs), goal - 1e-9)
            self.assertAlmostEqual(xs[-1], goal)

    def test_monotonic_approach(self):
        xs, _ = self.run_move(10, 40, 20, 5)
        self.assertTrue(all(b >= a - 1e-12 for a, b in zip(xs, xs[1:])))

    def test_stalled_loop_does_not_jump(self):
        svc, _ = service(wide_cfg())
        target(svc, 0.0, [0, 50, 0, 0])
        svc.tick(5.0)  # 5 s gap (also past the timeout)
        self.assertLessEqual(abs(svc.controller.pos[PAIR]), 10 * 0.1 + 1e-9)


class Timeout(unittest.TestCase):
    def test_holds_after_timeout_without_reversing(self):
        svc, _ = service(wide_cfg())
        clk = Clock(svc)
        for _ in range(100):  # 2 s of commands toward +80: moving at 10 deg/s
            target(svc, clk.t, [0, 80, 0, 0])
            clk.run(0.02)
        self.assertFalse(svc.controller.holding)
        v_before = svc.controller.vel[PAIR]
        self.assertGreater(v_before, 9)
        x_last_cmd = svc.controller.pos[PAIR]
        xs = [p[PAIR] for p in clk.run(3.0)]  # no more commands
        self.assertTrue(svc.controller.holding)
        self.assertTrue(all(b >= a - 1e-12 for a, b in zip(xs, xs[1:])), "reversed")
        # Runs on for at most 0.5 s at speed, then brakes over v^2/2a.
        self.assertLessEqual(xs[-1] - x_last_cmd, 10 * 0.52 + 10 ** 2 / (2 * 40) + 0.3)
        self.assertEqual(svc.controller.vel[PAIR], 0.0)
        self.assertLess(xs[-1], 80)
        self.assertEqual(xs[-1], xs[-50])

    def test_still_moving_until_timeout(self):
        svc, _ = service(wide_cfg())
        clk = Clock(svc)
        target(svc, clk.t, [0, 80, 0, 0])
        clk.run(0.48)
        self.assertFalse(svc.controller.holding)
        clk.run(0.06)
        self.assertTrue(svc.controller.holding)

    def test_status_does_not_keep_it_alive(self):
        svc, _ = service(wide_cfg())
        clk = Clock(svc)
        target(svc, clk.t, [0, 80, 0, 0])
        for _ in range(50):
            clk.run(0.02)
            svc.handle(protocol.encode_status(), clk.t)
        self.assertTrue(svc.controller.holding)

    def test_rejected_command_does_not_keep_it_alive(self):
        svc, _ = service(wide_cfg())
        clk = Clock(svc)
        target(svc, clk.t, [0, 80, 0, 0])
        for _ in range(50):
            clk.run(0.02)
            target(svc, clk.t, [0, float("nan"), 0, 0])
        self.assertTrue(svc.controller.holding)

    def test_new_command_resumes(self):
        svc, _ = service(wide_cfg())
        clk = Clock(svc)
        target(svc, clk.t, [0, 20, 0, 0])
        clk.run(2.0)
        held = svc.controller.pos[PAIR]
        self.assertTrue(svc.controller.holding)
        for _ in range(400):
            target(svc, clk.t, [0, 20, 0, 0])
            clk.run(0.02)
        self.assertGreater(svc.controller.pos[PAIR], held)
        self.assertAlmostEqual(svc.controller.pos[PAIR], 20)

    def test_idle_at_startup_holds_home(self):
        svc, drv = service()
        Clock(svc).run(2.0)
        self.assertEqual(svc.controller.pos, [0, 0, 0, 0])
        for w in drv.history:
            self.assertEqual(set(w.values()), {135.0})


class StartupAndExit(unittest.TestCase):
    def test_first_write_is_home(self):
        c = cfg(j0={"home": 4}, pair={"home": -3}, j3={"home": 2}, j4={"home": -7})
        svc, drv = service(c)
        first = drv.history[0]
        self.assertEqual(len(drv.history), 1)
        self.assertAlmostEqual(first[0], 139)
        self.assertAlmostEqual(first[1], 132)
        self.assertAlmostEqual(first[2], 138)
        self.assertAlmostEqual(first[3], 137)
        self.assertAlmostEqual(first[4], 128)

    def test_no_write_before_start(self):
        c = cfg()
        drv = FakePCA9685(c.pca, c.channels)
        from iris.motor.service import MotorService
        MotorService(c, drv)
        self.assertEqual(drv.history, [])

    def test_stop_releases_everything(self):
        svc, drv = service()
        target(svc, 0.01, [5, 5, 5, 5])
        Clock(svc).run(0.5)
        svc.stop()
        self.assertTrue(drv.released)
        self.assertTrue(drv.deinited)
        self.assertTrue(all(v is None for v in drv.angles.values()))

    def test_real_driver_not_imported_by_dry_run_path(self):
        # RealPCA9685 imports board/adafruit lazily, inside __init__ only.
        src = inspect.getsource(hw)
        top = src.split("class FakePCA9685")[0]
        self.assertNotIn("import board", top)
        self.assertNotIn("adafruit", top.replace("Adafruit", ""))


if __name__ == "__main__":
    unittest.main()
