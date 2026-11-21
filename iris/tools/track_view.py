"""PC: show the Pi camera stream with the tracked hand point drawn.

    .venv\\Scripts\\python -m iris.tools.track_view
    .venv\\Scripts\\python -m iris.tools.track_view --source 0          # local webcam
    .venv\\Scripts\\python -m iris.tools.track_view --frames 150 --no-window --snapshot out.jpg

q or Esc quits. Never sends motor commands.
"""
from __future__ import annotations

import argparse
import sys
import time
from collections import deque

import cv2

from iris import config as config_mod
from iris import pihost
from iris.vision.hand import HandTracker
from iris.vision.sources import OpenCVSource, TcpJpegSource

GREEN = (80, 220, 80)
RED = (60, 60, 230)
YELLOW = (0, 220, 255)


def draw(frame, hand, fps: float, hit_rate: float) -> None:
    h, w = frame.shape[:2]
    if hand is not None:
        for x, y in hand.landmarks_px:
            cv2.circle(frame, (int(x), int(y)), 2, YELLOW, -1)
        px, py = int(hand.point_px[0]), int(hand.point_px[1])
        cv2.circle(frame, (px, py), 12, GREEN, 2)
        cv2.drawMarker(frame, (px, py), GREEN, cv2.MARKER_CROSS, 30, 2)
        label = f"hand ({px}, {py})  {hand.handedness} {hand.score:.2f}"
        color = GREEN
    else:
        label = "NO HAND"
        color = RED
    cv2.rectangle(frame, (0, 0), (w, 26), (0, 0, 0), -1)
    cv2.putText(frame, label, (8, 18), cv2.FONT_HERSHEY_SIMPLEX, 0.55, color, 1, cv2.LINE_AA)
    stats = f"{fps:4.1f} fps  det {hit_rate * 100:3.0f}%"
    cv2.putText(frame, stats, (w - 175, 18), cv2.FONT_HERSHEY_SIMPLEX, 0.55,
                (230, 230, 230), 1, cv2.LINE_AA)


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description="IRIS hand tracking viewer")
    ap.add_argument("--host", help="Pi address (default: ~/.ssh/config iris-pi)")
    ap.add_argument("--port", type=int)
    ap.add_argument("--source", help="use a local webcam index or video file instead of the Pi")
    ap.add_argument("--frames", type=int, default=0, help="stop after N frames (0 = run until q)")
    ap.add_argument("--snapshot", help="save the last annotated frame with a hand here")
    ap.add_argument("--no-window", action="store_true")
    args = ap.parse_args(argv)

    cfg = config_mod.load()
    if args.source is not None:
        src = OpenCVSource(args.source)
    else:
        host = pihost.resolve(args.host)
        port = args.port or cfg.camera.tcp_port
        print(f"connecting to {host}:{port}")
        src = TcpJpegSource(host, port)

    tracker = HandTracker()
    times = deque(maxlen=30)
    hits = deque(maxlen=60)
    n = n_hand = 0
    saved = None
    t0 = time.monotonic()
    try:
        while True:
            frame = src.read()
            now = time.monotonic()
            hand = tracker.detect(frame, int((now - t0) * 1000))
            times.append(now)
            hits.append(hand is not None)
            n += 1
            n_hand += hand is not None
            fps = (len(times) - 1) / (times[-1] - times[0]) if len(times) > 1 else 0.0
            draw(frame, hand, fps, sum(hits) / len(hits))
            if hand is not None and args.snapshot:
                saved = frame.copy()
            if not args.no_window:
                cv2.imshow("IRIS hand tracking (q to quit)", frame)
                if cv2.waitKey(1) & 0xFF in (ord("q"), 27):
                    break
            if args.frames and n >= args.frames:
                break
    except ConnectionError as e:
        print(f"stream ended: {e}")
    finally:
        src.close()
        tracker.close()
        cv2.destroyAllWindows()

    elapsed = time.monotonic() - t0
    print(f"{n} frames in {elapsed:.1f} s ({n / max(elapsed, 1e-6):.1f} fps), "
          f"hand found in {n_hand} ({100.0 * n_hand / max(n, 1):.0f}%)")
    if args.snapshot:
        if saved is not None:
            cv2.imwrite(args.snapshot, saved)
            print(f"saved {args.snapshot}")
        else:
            print("no frame with a hand to save")
    return 0


if __name__ == "__main__":
    sys.exit(main())
