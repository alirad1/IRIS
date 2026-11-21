"""Type joint moves yourself. Talks to a running motor service.

On the Pi (in a second shell, while the motor service runs):
    cd ~/IRIS && ~/iris-servo-venv/bin/python -m iris.tools.jog
On the PC:
    .venv\\Scripts\\python -m iris.tools.jog

Commands (angles are logical degrees, 0 = home/centre, absolute):
    pair 10      move the pair to +10   (CH1 = 135+10, CH2 = 135-10)
    j0 -5        move j0 to -5
    home         move every joint back to home
    status       show positions and limits
    help, quit

Nothing moves until you press Enter on a move command. Moves go through the
service, so its limits (-10..+10 until calibrated), its speed limit and the pair
rule all apply. Ctrl-C during a move stops sending; the service brakes and holds.
"""
from __future__ import annotations

import argparse
import socket
import sys
import time

from iris import config as config_mod
from iris import pihost, protocol
from iris.config import JOINT_NAMES, SERVO_CENTER_DEG
from iris.motor.core import servo_signs

RATE_HZ = 20.0
REACH_TOL = 0.002  # the service lands exactly on the target, so "reached" means stopped
REPLY_TIMEOUT_S = 0.5
MOVE_TIMEOUT_S = 60.0


class Abort(Exception):
    pass


def request(sock, addr, data: bytes) -> dict:
    sock.sendto(data, addr)
    sock.settimeout(REPLY_TIMEOUT_S)
    try:
        reply, _ = sock.recvfrom(65536)
    except socket.timeout:
        raise Abort("no reply from motor service (is it running?)")
    except ConnectionResetError:
        raise Abort("motor service not running (port closed)")
    return protocol.decode_reply(reply)


def servo_text(cfg, name: str, x: float) -> str:
    j = cfg.joint(name)
    parts = [f"CH{s.channel}={SERVO_CENTER_DEG + s.trim + sign * x:.1f}"
             for s, sign in zip(j.servos, servo_signs(name))]
    return ", ".join(parts)


def move(sock, addr, cfg, goal: list, st: dict) -> dict:
    """Stream the goal until every joint reaches it."""
    t0 = time.monotonic()
    last_print = 0.0
    while True:
        st = request(sock, addr, protocol.encode_target(goal))
        if not st.get("ok"):
            raise Abort(f"service rejected the move: {st.get('error')}")
        now = time.monotonic()
        if now - last_print >= 0.5:
            print("   pos " + "  ".join(f"{n}={x:+6.2f}" for n, x in zip(JOINT_NAMES, st["pos"])),
                  flush=True)
            last_print = now
        if all(abs(p - g) <= REACH_TOL for p, g in zip(st["pos"], goal)):
            print(f"   reached in {now - t0:.1f} s. Holding.")
            return st
        if now - t0 > MOVE_TIMEOUT_S:
            raise Abort("move took too long")
        time.sleep(1.0 / RATE_HZ)


def show_status(st: dict) -> None:
    print(f"   mode={st['mode']}  {'HOLDING' if st['holding'] else 'MOVING'}")
    for n, p, (lo, hi) in zip(JOINT_NAMES, st["pos"], st["limits"]):
        print(f"   {n:5s} {p:+7.2f}   limits [{lo:+.1f}, {hi:+.1f}]")


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description="IRIS manual jog")
    ap.add_argument("--host", help="motor service address (default: this machine if "
                                   "there is no iris-pi entry in ~/.ssh/config)")
    ap.add_argument("--port", type=int)
    args = ap.parse_args(argv)

    cfg = config_mod.load()
    host = args.host or pihost.from_ssh_config() or "127.0.0.1"
    addr = (host, args.port or cfg.motor.udp_port)
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    try:
        st = request(sock, addr, protocol.encode_status())
    except Abort as e:
        print(f"{e}. Start it first:  ~/iris-servo-venv/bin/python -m iris.motor.service --max-speed 5")
        return 1
    print(f"Connected to motor service at {addr[0]}:{addr[1]}")
    show_status(st)
    print("Type e.g. 'pair 10', 'j0 -5', 'home', 'status', 'quit'.")

    while True:
        try:
            line = input("jog> ").replace("﻿", "").strip()
        except (EOFError, KeyboardInterrupt):
            print()
            break
        if not line:
            continue
        words = line.split()
        cmd = words[0].lower()
        try:
            if cmd in ("quit", "exit", "q"):
                break
            if cmd in ("help", "?"):
                print(__doc__.split("Commands")[1].split("Nothing moves")[0])
                continue
            st = request(sock, addr, protocol.encode_status())
            if cmd == "status":
                show_status(st)
                continue
            goal = list(st["pos"])
            if cmd == "home":
                goal = [j.home for j in cfg.joints]
                far = max(abs(g - p) for g, p in zip(goal, st["pos"]))
                if st.get("calibrate") and far > cfg.calibration.max_step_deg:
                    print("   calibration mode: home is more than "
                          f"{cfg.calibration.max_step_deg} deg away; step back in small moves.")
                    continue
                print("   moving all joints to home")
            elif cmd in JOINT_NAMES and len(words) == 2:
                try:
                    x = float(words[1])
                except ValueError:
                    print("   angle must be a number, e.g. 'pair 10'")
                    continue
                i = JOINT_NAMES.index(cmd)
                lo, hi = st["limits"][i]
                if not lo <= x <= hi:
                    print(f"   {x} is outside {cmd} limits [{lo}, {hi}]. Not sent.")
                    continue
                step = cfg.calibration.max_step_deg
                if st.get("calibrate") and abs(x - st["pos"][i]) > step:
                    print(f"   calibration mode: max {step} deg per move "
                          f"({cmd} is at {st['pos'][i]:+.2f}). Not sent.")
                    continue
                goal[i] = x
                print(f"   {cmd}: {st['pos'][i]:+.2f} -> {x:+.2f}   ({servo_text(cfg, cmd, x)})")
            else:
                print("   unknown command. Try 'pair 10', 'j0 -5', 'home', 'status', 'quit'.")
                continue
            st = move(sock, addr, cfg, goal, st)
        except KeyboardInterrupt:
            print("\n   stopped sending; the service brakes and holds where it is.")
        except Abort as e:
            print(f"   ABORT: {e}. Stopped sending; the service holds.")
    sock.close()
    print("Jog closed. The motor service keeps holding the arm until you stop it.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
