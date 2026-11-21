"""PC: aim calibration. Jog the arm, then click where it is aiming.

    .venv\\Scripts\\python -m iris.tools.aim_calibrate

Needs the Pi running the camera streamer and the motor service in normal mode
(bash ~/IRIS/scripts/run_pi.sh). Saves config/aim_map.json.

Keys (in the camera window):
  1 2 3 4  pick joint (j0, pair, j3, j4)     a / d   jog it - / +
  w / s    bigger / smaller step              space   stop where it is
  h        go home                            click   record: "the arm aims HERE"
  u        undo last point                    c       clear all points (press twice)
  S        save (Shift+S)                     q       quit (arm holds where it is)
"""
from __future__ import annotations

import argparse
import sys
import time

import cv2
import numpy as np

from iris import config as config_mod
from iris import pihost
from iris.aim import mapping
from iris.aim.mapping import AimMap, AimMapError, AimPoint
from iris.config import JOINT_NAMES
from iris.tools.motor_client import connect_checked, stationary
from iris.vision.sources import OpenCVSource, TcpJpegSource

STEPS = (1.0, 2.0, 5.0, 10.0)
PANEL_W = 300
WIN = "IRIS aim calibration"
WHITE, GREY, GREEN, YELLOW, RED, CYAN = ((235, 235, 235), (150, 150, 150), (80, 220, 80),
                                         (0, 220, 255), (60, 60, 230), (230, 200, 60))


def draw_points(img, points, hull=True):
    if len(points) >= 3 and hull:
        try:
            m = AimMap(points)
            poly = m.P[m.hull].astype(np.int32)
            cv2.polylines(img, [poly], True, CYAN, 1, cv2.LINE_AA)
            for tri in m.tris:
                cv2.polylines(img, [m.P[list(tri)].astype(np.int32)], True, (90, 80, 40), 1)
        except AimMapError:
            pass
    for i, p in enumerate(points):
        x, y = int(p.px[0]), int(p.px[1])
        cv2.circle(img, (x, y), 6, YELLOW, 2, cv2.LINE_AA)
        cv2.putText(img, str(i + 1), (x + 8, y - 6), cv2.FONT_HERSHEY_SIMPLEX, 0.5, YELLOW, 1,
                    cv2.LINE_AA)


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description="IRIS aim calibration")
    ap.add_argument("--host", help="Pi address (default: ~/.ssh/config iris-pi)")
    ap.add_argument("--motor-host", help="motor service address (default: --host)")
    ap.add_argument("--source", help="local webcam/video instead of the Pi camera (testing)")
    ap.add_argument("--map", default=str(mapping.DEFAULT_MAP_PATH))
    args = ap.parse_args(argv)

    cfg = config_mod.load()
    host = pihost.resolve(args.host)
    motor = connect_checked(args.motor_host or host, cfg.motor.udp_port)
    st = motor.last
    goal = list(st["pos"])
    limits = [(j.min, j.max) for j in cfg.joints]

    points = []
    try:
        points, _ = mapping.load_points(args.map)
        msg = f"loaded {len(points)} points from {args.map}"
    except (OSError, ValueError, KeyError):
        msg = "no saved points yet"
    saved_n = len(points)

    src = OpenCVSource(args.source) if args.source else TcpJpegSource(host, cfg.camera.tcp_port)
    clicks = []
    cv2.namedWindow(WIN)
    cv2.setMouseCallback(WIN, lambda ev, x, y, *_: clicks.append((x, y))
                         if ev == cv2.EVENT_LBUTTONDOWN else None)

    sel, step_i, clear_armed, quit_armed = 1, 2, False, False
    try:
        while True:
            frame = src.read()
            h, w = frame.shape[:2]
            reply = motor.target(goal)
            if reply is not None:
                st = reply
            moving = not stationary(st, goal)

            for (x, y) in clicks:
                if x >= w:
                    continue
                if moving or reply is None:
                    msg = "wait until the arm has stopped, then click"
                elif any(np.hypot(x - p.px[0], y - p.px[1]) < 12 for p in points):
                    msg = "too close to an existing point"
                else:
                    points.append(AimPoint((float(x), float(y)), list(st["pos"])))
                    msg = f"point {len(points)} at ({x}, {y}) <- {[round(v, 1) for v in st['pos']]}"
            clicks.clear()

            k = cv2.waitKey(1) & 0xFF
            if k != 255:
                if k != ord("c"):
                    clear_armed = False
                if k != ord("q"):
                    quit_armed = False
            if k in (ord("1"), ord("2"), ord("3"), ord("4")):
                sel = k - ord("1")
                msg = f"picked {JOINT_NAMES[sel]}"
            elif k in (ord("a"), ord("d")):
                lo, hi = limits[sel]
                g = goal[sel] + (STEPS[step_i] if k == ord("d") else -STEPS[step_i])
                goal[sel] = min(max(g, lo), hi)
                msg = f"{JOINT_NAMES[sel]} -> {goal[sel]:+.1f}" + (" (at limit)" if g != goal[sel] else "")
            elif k == ord("w"):
                step_i = min(step_i + 1, len(STEPS) - 1)
                msg = f"step {STEPS[step_i]} deg"
            elif k == ord("s"):
                step_i = max(step_i - 1, 0)
                msg = f"step {STEPS[step_i]} deg"
            elif k == ord(" "):
                motor.hold()
                time.sleep(0.3)
                s2 = motor.status()
                if s2:
                    st = s2
                    goal = list(st["pos"])
                msg = "stopped"
            elif k == ord("h"):
                goal = [j.home for j in cfg.joints]
                msg = "going home"
            elif k == ord("u") and points:
                points.pop()
                msg = f"removed last point ({len(points)} left)"
            elif k == ord("c"):
                if clear_armed:
                    points, clear_armed = [], False
                    msg = "all points cleared"
                else:
                    clear_armed = True
                    msg = "press c again to clear ALL points"
            elif k == ord("S"):
                try:
                    AimMap(points)
                    mapping.save(points, (w, h), config_mod.fingerprint(), args.map)
                    saved_n = len(points)
                    msg = f"saved {len(points)} points to {args.map}"
                except AimMapError as e:
                    msg = f"not saved: {e}"
            elif k == ord("q"):
                if len(points) != saved_n and not quit_armed:
                    quit_armed = True
                    msg = "unsaved points! q again to quit anyway, S to save"
                else:
                    break

            # draw
            view = frame.copy()
            for i in (1, 2):
                cv2.line(view, (w * i // 3, 0), (w * i // 3, h), (70, 70, 70), 1)
                cv2.line(view, (0, h * i // 3), (w, h * i // 3), (70, 70, 70), 1)
            draw_points(view, points)
            panel = np.zeros((h, PANEL_W, 3), np.uint8)
            y = 22

            def put(text, color=WHITE, scale=0.5):
                nonlocal y
                cv2.putText(panel, text, (10, y), cv2.FONT_HERSHEY_SIMPLEX, scale, color, 1,
                            cv2.LINE_AA)
                y += 20

            put("AIM CALIBRATION", CYAN, 0.6)
            put("MOVING" if moving else "still - click where it aims",
                YELLOW if moving else GREEN)
            if reply is None:
                put(motor.error or "no reply", RED)
            y += 4
            for i, n in enumerate(JOINT_NAMES):
                put(f"{'>' if i == sel else ' '} {n:5s} {st['pos'][i]:+7.1f}  -> {goal[i]:+7.1f}",
                    WHITE if i == sel else GREY)
            put(f"step {STEPS[step_i]} deg   points {len(points)}"
                + ("" if len(points) == saved_n else " *"))
            y += 6
            for line in ("1-4 joint   a/d jog   w/s step", "space stop   h home",
                         "click = arm aims here", "u undo   c clear (x2)",
                         "S save   q quit", "", "Aim for ~9 points spread",
                         "over the area you want lit."):
                put(line, GREY, 0.45)
            y = h - 30
            for chunk in (msg[i:i + 38] for i in range(0, min(len(msg), 76), 38)):
                put(chunk, YELLOW, 0.45)
            cv2.imshow(WIN, np.hstack([view, panel]))
    except ConnectionError as e:
        print(f"camera stream ended: {e}")
    finally:
        motor.hold()
        motor.close()
        src.close()
        cv2.destroyAllWindows()
    print(f"{len(points)} points, {saved_n} saved. The arm holds where it is.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
