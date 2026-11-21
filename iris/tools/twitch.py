"""PC: channel check. Move ONE logical joint home -> home+delta -> home, slowly.

    .venv\\Scripts\\python -m iris.tools.twitch pair
    .venv\\Scripts\\python -m iris.tools.twitch j0 --delta -3

Goes through the motor service like everything else, so the pair moves as
one joint (CH1 = +pair, CH2 = -pair). |delta| is capped at 3 degrees. On any
problem it stops sending; the service then brakes and holds within 0.5 s.
It never retries.
"""
from __future__ import annotations

import argparse
import socket
import sys
import time

from iris import config as config_mod
from iris import pihost, protocol
from iris.config import JOINT_NAMES

MAX_DELTA = 3.0
RATE_HZ = 20.0
REACH_TOL = 0.05
MOVE_TIMEOUT_S = 10.0
REPLY_TIMEOUT_S = 0.5


class Abort(Exception):
    pass


def request(sock, addr, data: bytes) -> dict:
    sock.sendto(data, addr)
    deadline = time.monotonic() + REPLY_TIMEOUT_S
    while True:
        remaining = deadline - time.monotonic()
        if remaining <= 0:
            raise Abort("no reply from motor service")
        sock.settimeout(remaining)
        try:
            reply, _ = sock.recvfrom(65536)
        except socket.timeout:
            raise Abort("no reply from motor service")
        except ConnectionResetError:
            raise Abort("motor service not running (port closed)")
        return protocol.decode_reply(reply)


def stream_to(sock, addr, targets, idx: int, hold_s: float = 0.0) -> dict:
    """Send `targets` at RATE_HZ until joint idx reaches it, then for hold_s more."""
    t_start = time.monotonic()
    reached_at = None
    last = None
    while True:
        st = request(sock, addr, protocol.encode_target(targets))
        if not st.get("ok"):
            raise Abort(f"service rejected target: {st.get('error')}")
        last = st
        now = time.monotonic()
        if reached_at is None and abs(st["pos"][idx] - targets[idx]) <= REACH_TOL:
            reached_at = now
            print(f"  reached {targets[idx]:+.2f}", flush=True)
        if reached_at is not None and now - reached_at >= hold_s:
            return last
        if now - t_start > MOVE_TIMEOUT_S:
            raise Abort(f"did not reach target in {MOVE_TIMEOUT_S} s (pos {st['pos']})")
        time.sleep(1.0 / RATE_HZ)


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description="Twitch one logical joint and return")
    ap.add_argument("joint", choices=JOINT_NAMES)
    ap.add_argument("--delta", type=float, default=3.0)
    ap.add_argument("--pause", type=float, default=1.0, help="seconds to hold at home+delta")
    ap.add_argument("--host")
    ap.add_argument("--port", type=int)
    args = ap.parse_args(argv)

    if not 0 < abs(args.delta) <= MAX_DELTA:
        print(f"--delta must be non-zero and at most {MAX_DELTA} deg")
        return 2
    cfg = config_mod.load()
    addr = (pihost.resolve(args.host), args.port or cfg.motor.udp_port)
    idx = JOINT_NAMES.index(args.joint)

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        st = request(sock, addr, protocol.encode_status())
        print(f"service mode={st['mode']} pos={st['pos']}")
        base = list(st["pos"])
        home = cfg.joint(args.joint).home
        if abs(base[idx] - home) > REACH_TOL:
            raise Abort(f"{args.joint} is at {base[idx]}, not home {home}; not twitching")
        lo, hi = st["limits"][idx]
        goal = list(base)
        goal[idx] = base[idx] + args.delta
        if not lo <= goal[idx] <= hi:
            raise Abort(f"{goal[idx]} outside limits [{lo}, {hi}]")
        print(f"{args.joint}: {base[idx]:+.2f} -> {goal[idx]:+.2f} -> {base[idx]:+.2f}")
        stream_to(sock, addr, goal, idx, hold_s=args.pause)
        st = stream_to(sock, addr, base, idx, hold_s=0.3)
        print(f"done, pos={st['pos']}. Service holds here.")
        return 0
    except Abort as e:
        print(f"ABORT: {e}. Stopped sending; service will brake and hold.")
        return 1
    finally:
        sock.close()


if __name__ == "__main__":
    sys.exit(main())
