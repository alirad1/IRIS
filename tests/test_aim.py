from __future__ import annotations

import math
import random
import tempfile
import unittest
from pathlib import Path

from iris.aim import mapping
from iris.aim.follow_logic import Follower, OneEuro
from iris.aim.mapping import AimMap, AimMapError, AimPoint


def lin(u, v):
    """A made-up linear relation from pixel to joints, so interpolation is exact."""
    return [0.1 * u - 20, -0.05 * v + 30, 0.0, 0.0]


def grid(n=3, x0=100, x1=500, y0=80, y1=400, f=lin):
    pts = []
    for i in range(n):
        for k in range(n):
            u = x0 + (x1 - x0) * i / (n - 1)
            v = y0 + (y1 - y0) * k / (n - 1)
            pts.append(AimPoint((u, v), f(u, v)))
    return pts


class MapTests(unittest.TestCase):
    def test_exact_at_points(self):
        pts = grid()
        m = AimMap(pts)
        for p in pts:
            j, aim, clamped = m.joints_for(p.px)
            self.assertFalse(clamped)
            for a, b in zip(j, p.joints):
                self.assertAlmostEqual(a, b, places=6)

    def test_linear_inside(self):
        m = AimMap(grid())
        rng = random.Random(0)
        for _ in range(200):
            u, v = rng.uniform(100, 500), rng.uniform(80, 400)
            j, aim, clamped = m.joints_for((u, v))
            self.assertFalse(clamped)
            for a, b in zip(j, lin(u, v)):
                self.assertAlmostEqual(a, b, places=6)

    def test_irregular_points_no_overshoot(self):
        rng = random.Random(1)
        pts = [AimPoint((rng.uniform(50, 600), rng.uniform(50, 430)),
                        [rng.uniform(-50, 50), rng.uniform(-20, 50), 0, 0]) for _ in range(9)]
        m = AimMap(pts)
        lo = [min(p.joints[i] for p in pts) for i in range(4)]
        hi = [max(p.joints[i] for p in pts) for i in range(4)]
        for _ in range(500):
            j, _, _ = m.joints_for((rng.uniform(0, 640), rng.uniform(0, 480)))
            for i in range(4):
                self.assertGreaterEqual(j[i], lo[i] - 1e-9)
                self.assertLessEqual(j[i], hi[i] + 1e-9)

    def test_outside_clamps_to_nearest_edge(self):
        m = AimMap(grid())
        j, aim, clamped = m.joints_for((300, 10))      # above the top edge (v=80)
        self.assertTrue(clamped)
        self.assertAlmostEqual(aim[0], 300)
        self.assertAlmostEqual(aim[1], 80)
        for a, b in zip(j, lin(300, 80)):
            self.assertAlmostEqual(a, b, places=6)
        j, aim, clamped = m.joints_for((900, 900))     # past a corner
        self.assertTrue(clamped)
        self.assertAlmostEqual(aim[0], 500)
        self.assertAlmostEqual(aim[1], 400)

    def test_continuous_across_edge(self):
        m = AimMap(grid())
        a, _, _ = m.joints_for((300, 80.0001))
        b, _, _ = m.joints_for((300, 79.9999))
        for x, y in zip(a, b):
            self.assertAlmostEqual(x, y, places=3)

    def test_bad_inputs(self):
        with self.assertRaises(AimMapError):
            AimMap(grid()[:2])
        with self.assertRaises(AimMapError):
            AimMap([AimPoint((i * 50.0, i * 50.0), [0, 0, 0, 0]) for i in range(5)])  # a line

    def test_save_load_and_config_check(self):
        with tempfile.TemporaryDirectory() as d:
            p = Path(d) / "aim.json"
            mapping.save(grid(), (640, 480), "abc", p)
            m = mapping.load(p, expect_config="abc")
            self.assertEqual(len(m.points), 9)
            with self.assertRaises(AimMapError):
                mapping.load(p, expect_config="different")


LIMITS = [(-56.5, 84.7), (-29.6, 59.9), (-135, 135), (-68.5, 57.1)]


class FilterTests(unittest.TestCase):
    def test_still_hand_jitter_is_smoothed(self):
        f = OneEuro()
        rng = random.Random(2)
        out = [f((300 + rng.gauss(0, 3), 200 + rng.gauss(0, 3)), i / 30) for i in range(300)]
        tail = out[100:]
        spread = max(math.hypot(a[0] - 300, a[1] - 200) for a in tail)
        self.assertLess(spread, 3.0)

    def test_follows_a_move(self):
        f = OneEuro()
        for i in range(30):
            f((100, 100), i / 30)
        for i in range(30, 120):
            x = f((400, 100), i / 30)
        self.assertAlmostEqual(x[0], 400, delta=2)


class FollowerTests(unittest.TestCase):
    def make(self):
        return Follower(AimMap(grid()), LIMITS, deadband_px=10, lost_grace_s=0.4)

    def test_nothing_sent_before_a_hand(self):
        fo = self.make()
        self.assertEqual(fo.update(None, 0.0).kind, "none")

    def test_target_then_deadband_keeps_it_still(self):
        fo = self.make()
        d = fo.update((300, 200), 0.0)
        self.assertEqual(d.kind, "target")
        first = d.targets
        rng = random.Random(3)
        for i in range(1, 90):
            d = fo.update((300 + rng.gauss(0, 2), 200 + rng.gauss(0, 2)), i / 30)
            self.assertEqual(d.targets, first)  # jitter never changes the target

    def test_lost_grace_then_single_hold(self):
        fo = self.make()
        fo.update((300, 200), 0.0)
        self.assertEqual(fo.update(None, 0.2).kind, "target")   # brief dropout: keep going
        self.assertEqual(fo.update(None, 0.5).kind, "hold")     # gone: brake and hold once
        self.assertEqual(fo.update(None, 0.6).kind, "none")
        d = fo.update((310, 210), 0.7)
        self.assertEqual(d.kind, "target")

    def test_out_of_reach_clamps_and_stays(self):
        fo = self.make()
        d1 = fo.update((620, 470), 0.0)
        self.assertTrue(d1.clamped)
        self.assertEqual(d1.state, "OUT OF REACH")
        d2 = fo.update((630, 475), 0.5)
        self.assertEqual(d1.targets, d2.targets)

    def test_targets_inside_joint_limits(self):
        far = lambda u, v: [500.0, -500.0, 0.0, 0.0]
        fo = Follower(AimMap(grid(f=far)), LIMITS)
        d = fo.update((300, 200), 0.0)
        self.assertEqual(d.targets[0], 84.7)
        self.assertEqual(d.targets[1], -29.6)

    def test_pause(self):
        fo = self.make()
        fo.update((300, 200), 0.0)
        self.assertEqual(fo.pause(True).kind, "hold")
        self.assertEqual(fo.update((300, 200), 0.1).kind, "none")
        fo.pause(False)
        self.assertEqual(fo.update((300, 200), 0.2).kind, "target")


if __name__ == "__main__":
    unittest.main()
