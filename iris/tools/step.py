"""Calibration helper: one small, explicit move of one joint, then exit.

    python -m iris.tools.step j0 +3        # one relative step (|delta| <= 3)
    python -m iris.tools.step j0 --to 0    # walk to an absolute angle in <= 3 deg steps

Talks to a running motor service (normally started with --calibrate). Each
step waits until the joint has stopped. The service holds afterwards.
"""
from __future__ import annotations

import argparse
import socket
import sys
import time

from iris import config as config_mod
from iris import pihost, protocol
from iris.config import JOINT_NAMES
from iris.tools.jog import Abort, move, request, servo_text

MAX_STEP = 3.0
PAUSE_S = 0.3


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description="One small calibration move")
    ap.add_argument("joint", choices=JOINT_NAMES)
    ap.add_argument("delta", nargs="?", type=float, help="relative step, at most 3 deg")
    ap.add_argument("--to", type=float, help="walk to this absolute angle in <= 3 deg steps")
    ap.add_argument("--host")
    ap.add_argument("--port", type=int)
    args = ap.parse_args(argv)
    if (args.delta is None) == (args.to is None):
        print("give either a delta or --to")
        return 2
    if args.delta is not None and not 0 < abs(args.delta) <= MAX_STEP:
        print(f"delta must be non-zero and at most {MAX_STEP} deg")
        return 2

    cfg = config_mod.load()
    addr = (pihost.resolve(args.host), args.port or cfg.motor.udp_port)
    i = JOINT_NAMES.index(args.joint)
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        st = request(sock, addr, protocol.encode_status())
        if any(abs(p - t) > 1e-3 for p, t in zip(st["pos"], st["target"])):
            raise Abort(f"arm is still moving (pos {st['pos']}, target {st['target']}); not sending")
        start = st["pos"][i]
        goal_abs = start + args.delta if args.delta is not None else args.to
        lo, hi = st["limits"][i]
        if not lo <= goal_abs <= hi:
            raise Abort(f"{goal_abs:+.2f} outside service limits [{lo}, {hi}]")
        print(f"mode={st['mode']}  {args.joint}: {start:+.2f} -> {goal_abs:+.2f}")
        while abs(st["pos"][i] - goal_abs) > 1e-3:
            here = st["pos"][i]
            nxt = here + max(-MAX_STEP, min(MAX_STEP, goal_abs - here))
            goal = list(st["pos"])
            goal[i] = nxt
            print(f"  step {here:+.2f} -> {nxt:+.2f}  ({servo_text(cfg, args.joint, nxt)})", flush=True)
            st = move(sock, addr, cfg, goal, st)
            if abs(nxt - goal_abs) > 1e-3:
                time.sleep(PAUSE_S)
        print(f"{args.joint} at {st['pos'][i]:+.2f}. pos={st['pos']}")
        return 0
    except Abort as e:
        print(f"ABORT: {e}. Stopped sending; the service holds.")
        return 1
    finally:
        sock.close()


if __name__ == "__main__":
    sys.exit(main())
