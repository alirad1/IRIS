"""Image point -> joint angles, from hand-made calibration points.

Each calibration point says "when the arm is at these joint angles it aims at
this pixel". The map triangulates the points (Delaunay) and interpolates the
joint angles linearly inside each triangle, so it is exact at every point and
never overshoots between them. A pixel outside the calibrated area is moved to
the nearest point on its edge first: out of reach means "closest reachable aim".

Only needs numpy and OpenCV, so it can run on the Pi later too.
"""
from __future__ import annotations

import json
import time
from dataclasses import dataclass
from pathlib import Path
from typing import List, Optional, Sequence, Tuple

import cv2
import numpy as np

from iris.config import REPO_ROOT

DEFAULT_MAP_PATH = REPO_ROOT / "config" / "aim_map.json"
MIN_AREA_PX2 = 400.0  # points must span a real area, not a line


class AimMapError(ValueError):
    pass


@dataclass
class AimPoint:
    px: Tuple[float, float]
    joints: List[float]  # [j0, pair, j3, j4]


class AimMap:
    def __init__(self, points: Sequence[AimPoint]) -> None:
        if len(points) < 3:
            raise AimMapError("need at least 3 calibration points")
        self.points = list(points)
        self.P = np.array([p.px for p in points], dtype=np.float64)
        self.J = np.array([p.joints for p in points], dtype=np.float64)
        if self.J.shape[1] != 4:
            raise AimMapError("each point needs 4 joint values")
        hull_idx = cv2.convexHull(self.P.astype(np.float32), returnPoints=False).flatten()
        self.hull = [int(i) for i in hull_idx]
        if abs(cv2.contourArea(self.P[self.hull].astype(np.float32))) < MIN_AREA_PX2:
            raise AimMapError("calibration points are (nearly) on one line; spread them out")
        self.tris = self._triangulate()

    def _triangulate(self) -> List[Tuple[int, int, int]]:
        lo = self.P.min(axis=0) - 10
        hi = self.P.max(axis=0) + 10
        sub = cv2.Subdiv2D((int(lo[0]), int(lo[1]), int(hi[0] - lo[0]) + 1, int(hi[1] - lo[1]) + 1))
        for x, y in self.P:
            sub.insert((float(x), float(y)))
        index = {(round(x, 3), round(y, 3)): i for i, (x, y) in enumerate(self.P)}
        tris = []
        for t in sub.getTriangleList():
            vs = [(round(float(t[k]), 3), round(float(t[k + 1]), 3)) for k in (0, 2, 4)]
            if all(v in index for v in vs):
                tri = tuple(index[v] for v in vs)
                if self._area(tri) > 1e-6:
                    tris.append(tri)
        if not tris:
            raise AimMapError("could not triangulate the calibration points")
        return tris

    def _area(self, tri) -> float:
        a, b, c = self.P[list(tri)]
        return abs((b[0] - a[0]) * (c[1] - a[1]) - (c[0] - a[0]) * (b[1] - a[1])) / 2.0

    @staticmethod
    def _bary(p, a, b, c):
        v0, v1, v2 = b - a, c - a, p - a
        d = v0[0] * v1[1] - v1[0] * v0[1]
        l1 = (v2[0] * v1[1] - v1[0] * v2[1]) / d
        l2 = (v0[0] * v2[1] - v2[0] * v0[1]) / d
        return 1.0 - l1 - l2, l1, l2

    def _closest_on_hull(self, p) -> Tuple[np.ndarray, int, int, float]:
        best = None
        n = len(self.hull)
        for k in range(n):
            ia, ib = self.hull[k], self.hull[(k + 1) % n]
            a, b = self.P[ia], self.P[ib]
            ab = b - a
            t = float(np.clip(np.dot(p - a, ab) / max(np.dot(ab, ab), 1e-12), 0.0, 1.0))
            q = a + t * ab
            d = float(np.hypot(*(p - q)))
            if best is None or d < best[0]:
                best = (d, q, ia, ib, t)
        _, q, ia, ib, t = best
        return q, ia, ib, t

    def inside(self, px) -> bool:
        p = np.asarray(px, dtype=np.float64)
        return cv2.pointPolygonTest(self.P[self.hull].astype(np.float32),
                                    (float(p[0]), float(p[1])), False) >= 0

    def joints_for(self, px) -> Tuple[List[float], Tuple[float, float], bool]:
        """Returns (joints, aim pixel actually used, clamped?)."""
        p = np.asarray(px, dtype=np.float64)
        if self.inside(p):
            for tri in self.tris:
                a, b, c = self.P[list(tri)]
                l = self._bary(p, a, b, c)
                if min(l) >= -1e-9:
                    j = l[0] * self.J[tri[0]] + l[1] * self.J[tri[1]] + l[2] * self.J[tri[2]]
                    return j.tolist(), (float(p[0]), float(p[1])), False
        # Outside (or on a numerical crack): nearest point on the edge of the area.
        q, ia, ib, t = self._closest_on_hull(p)
        j = (1.0 - t) * self.J[ia] + t * self.J[ib]
        return j.tolist(), (float(q[0]), float(q[1])), not self.inside(p)


def save(points: Sequence[AimPoint], image_size, config_id: str,
         path: Path = DEFAULT_MAP_PATH) -> None:
    data = {
        "image_size": list(image_size),
        "config": config_id,
        "saved": time.strftime("%Y-%m-%d %H:%M:%S"),
        "points": [{"px": [round(p.px[0], 1), round(p.px[1], 1)],
                    "joints": [round(x, 2) for x in p.joints]} for p in points],
    }
    tmp = Path(str(path) + ".tmp")
    tmp.write_text(json.dumps(data, indent=2) + "\n", encoding="utf-8")
    tmp.replace(path)


def load_points(path: Path = DEFAULT_MAP_PATH) -> Tuple[List[AimPoint], dict]:
    data = json.loads(Path(path).read_text(encoding="utf-8"))
    pts = [AimPoint((float(p["px"][0]), float(p["px"][1])), [float(x) for x in p["joints"]])
           for p in data["points"]]
    return pts, data


def load(path: Path = DEFAULT_MAP_PATH, expect_config: Optional[str] = None) -> AimMap:
    pts, data = load_points(path)
    if expect_config is not None and data.get("config") != expect_config:
        raise AimMapError("the aim map was made with different servo settings "
                          "(channels/trims/invert changed). Redo the aim calibration.")
    return AimMap(pts)
