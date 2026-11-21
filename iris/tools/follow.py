"""PC: autonomous mode. The arm follows the hand, smoothly.

    .venv\\Scripts\\python -m iris.tools.follow              # moves the arm
    .venv\\Scripts\\python -m iris.tools.follow --preview    # shows what it would do, sends nothing

Needs the Pi running the camera streamer and the motor service in normal mode
(bash ~/IRIS/scripts/run_pi.sh), and config/aim_map.json from aim_calibrate.

Keys: space = pause/resume (pause holds the arm), q = quit (arm holds).
Hand lost: the arm brakes and holds. Hand outside the calibrated area: the arm
aims at the nearest calibrated spot and stays there.
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
from iris.aim.follow_logic import Follower
from iris.config import JOINT_NAMES
from iris.tools.aim_calibrate import draw_points
from iris.tools.motor_client import connect_checked
from iris.vision.hand import HandTracker
from iris.vision.sources import OpenCVSource, TcpJpegSource

WIN = "IRIS follow (space pause, q quit)"
COLORS = {"FOLLOW": (80, 220, 80), "OUT OF REACH": (0, 165, 255), "LOST": (60, 60, 230),
          "PAUSED": (200, 200, 200)}


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description="IRIS autonomous follow")
    ap.add_argument("--host", help="Pi address (default: ~/.ssh/config iris-pi)")
    ap.add_argument("--motor-host", help="motor service address (default: --host)")
    ap.add_argument("--source", help="local webcam/video instead of the Pi camera (testing)")
    ap.add_argument("--map", default=str(mapping.DEFAULT_MAP_PATH))
    ap.add_argument("--preview", action="store_true", help="never send motor commands")
    ap.add_argument("--deadband", type=float, default=10.0, help="pixels (default 10)")
    ap.add_argument("--frames", type=int, default=0, help="stop after N frames (testing)")
    ap.add_argument("--no-window", action="store_true")
    args = ap.parse_args(argv)

    cfg = config_mod.load()
    host = pihost.resolve(args.host)
    aim = mapping.load(args.map, expect_config=config_mod.fingerprint())
    limits = [(j.min, j.max) for j in cfg.joints]
    follower = Follower(aim, limits, deadband_px=args.deadband)

    motor = None if args.preview else connect_checked(args.motor_host or host, cfg.motor.udp_port)
    src = OpenCVSource(args.source) if args.source else TcpJpegSource(host, cfg.camera.tcp_port)
    tracker = HandTracker()
    t0 = time.monotonic()
    n = 0
    counts = {}
    paused = False
    print(("PREVIEW: no motor commands. " if args.preview else "") +
          f"Following with {len(aim.points)} calibration points.", flush=True)
    try:
        while True:
            frame = src.read()
            now = time.monotonic()
            hand = tracker.detect(frame, int((now - t0) * 1000))
            d = follower.update(hand.point_px if hand else None, now)
            counts[d.state] = counts.get(d.state, 0) + 1

            sent = ""
            if motor is not None:
                if d.kind == "target":
                    r = motor.target(d.targets)
                    sent = "" if r and r.get("ok") else (motor.error or "no reply")
                elif d.kind == "hold":
                    motor.hold()

            n += 1
            if not args.no_window:
                view = frame.copy()
                draw_points(view, aim.points)
                if hand is not None:
                    hx, hy = int(hand.point_px[0]), int(hand.point_px[1])
                    cv2.circle(view, (hx, hy), 4, (0, 220, 255), -1)
                if d.aim_px is not None and d.state != "LOST":
                    ax, ay = int(d.aim_px[0]), int(d.aim_px[1])
                    col = COLORS.get(d.state, (255, 255, 255))
                    cv2.drawMarker(view, (ax, ay), col, cv2.MARKER_CROSS, 28, 2)
                    cv2.circle(view, (ax, ay), 14, col, 2)
                col = COLORS.get(d.state, (255, 255, 255))
                cv2.rectangle(view, (0, 0), (view.shape[1], 26), (0, 0, 0), -1)
                label = d.state + ("  [PREVIEW]" if args.preview else "")
                cv2.putText(view, label, (8, 18), cv2.FONT_HERSHEY_SIMPLEX, 0.55, col, 1, cv2.LINE_AA)
                if d.targets is not None:
                    tx = "  ".join(f"{nm}{v:+.0f}" for nm, v in zip(JOINT_NAMES, d.targets))
                    cv2.putText(view, tx, (8, view.shape[0] - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.45,
                                (230, 230, 230), 1, cv2.LINE_AA)
                if sent:
                    cv2.putText(view, sent, (250, 18), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (60, 60, 230),
                                1, cv2.LINE_AA)
                cv2.imshow(WIN, view)
                k = cv2.waitKey(1) & 0xFF
                if k == ord("q"):
                    break
                if k == ord(" "):
                    paused = not paused
                    pd = follower.pause(paused)
                    if motor is not None and pd.kind == "hold":
                        motor.hold()
            if args.frames and n >= args.frames:
                break
    except ConnectionError as e:
        print(f"camera stream ended: {e}")
    finally:
        if motor is not None:
            motor.hold()
            motor.close()
        src.close()
        tracker.close()
        cv2.destroyAllWindows()
    el = time.monotonic() - t0
    print(f"{n} frames in {el:.1f} s ({n / max(el, 1e-6):.1f} fps); states: {counts}. "
          "The arm holds where it is.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
