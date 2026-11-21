"""Calibrate joint limits and home by hand, from the Pi shell. One command:

    cd ~/IRIS && ~/iris-servo-venv/bin/python -m iris.tools.calibrate

It starts the motor service in calibration mode (max 3 deg per step, 5 deg/s),
lets you jog with single keys, mark each joint's ends and home, and saves to
config/iris.json. On quit it stops the service and the servos go limp.

Saved limits keep a 3 deg margin inside the ends you mark.
"""
from __future__ import annotations

import argparse
import copy
import json
import os
import shutil
import signal
import socket
import subprocess
import sys
import time
from typing import List, Optional

from iris import config as config_mod
from iris import protocol
from iris.config import JOINT_NAMES, REPO_ROOT, ConfigError
from iris.tools.jog import Abort, move, request, servo_text

MARGIN_DEG = 3.0
MAX_STEP = 3.0
STEP_SIZES = (0.5, 1.0, 2.0, 3.0)
SERVICE_LOG = "/tmp/iris_motor_calibrate.log"

HELP = """\
  keys   1 2 3 4   pick joint (j0, pair, j3, j4)
         d or ->   move picked joint +step      a or <-   move it -step
         w or up   bigger step                  s or down smaller step
         [         mark LOW end here            ]         mark HIGH end here
         z         set HOME (zero) here         h         walk all joints to home
         S         save to config               q         quit (servos go limp)"""


class Calibrator:
    """Keyboard-independent logic, so it can be tested without a terminal."""

    def __init__(self, cfg, raw: dict, sock, addr, save_path: str) -> None:
        self.cfg, self.raw, self.sock, self.addr = cfg, raw, sock, addr
        self.save_path = save_path
        self.sel = 0
        self.step_i = 2  # 2 deg
        self.home: List[float] = [j.home for j in cfg.joints]
        self.low: List[Optional[float]] = [None] * 4
        self.high: List[Optional[float]] = [None] * 4
        self.dirty = False
        self.msg = "All joints are at home. Pick a joint (1-4) and jog it with a / d."
        self.st = request(sock, addr, protocol.encode_status())

    @property
    def step(self) -> float:
        return STEP_SIZES[self.step_i]

    def _go(self, goal: List[float]) -> None:
        """Walk to goal with every joint moving at most MAX_STEP per command."""
        lims = self.st["limits"]
        for i, (g, (lo, hi)) in enumerate(zip(goal, lims)):
            if not lo <= g <= hi:
                raise Abort(f"{JOINT_NAMES[i]} {g:+.1f} is past the servo range [{lo}, {hi}]")
        while any(abs(p - g) > 1e-3 for p, g in zip(self.st["pos"], goal)):
            nxt = [p + max(-MAX_STEP, min(MAX_STEP, g - p)) for p, g in zip(self.st["pos"], goal)]
            self.st = move(self.sock, self.addr, self.cfg, nxt, self.st, verbose=False)

    def refresh(self) -> None:
        self.st = request(self.sock, self.addr, protocol.encode_status())

    def key(self, k: str) -> bool:
        """Handle one key. Returns False to quit."""
        name = JOINT_NAMES[self.sel]
        pos = self.st["pos"][self.sel]
        try:
            if k in "1234" and len(k) == 1:
                self.sel = int(k) - 1
                self.msg = f"picked {JOINT_NAMES[self.sel]}"
            elif k in ("d", "RIGHT", "a", "LEFT"):
                sign = 1 if k in ("d", "RIGHT") else -1
                goal = list(self.st["pos"])
                goal[self.sel] = round(pos + sign * self.step, 3)
                self._go(goal)
                self.msg = f"{name} -> {goal[self.sel]:+.1f}  ({servo_text(self.cfg, name, goal[self.sel])})"
            elif k in ("w", "UP"):
                self.step_i = min(self.step_i + 1, len(STEP_SIZES) - 1)
                self.msg = f"step {self.step} deg"
            elif k in ("s", "DOWN"):
                self.step_i = max(self.step_i - 1, 0)
                self.msg = f"step {self.step} deg"
            elif k == "[":
                self.low[self.sel] = pos
                self.dirty = True
                self.msg = f"{name} LOW end marked at {pos:+.1f} (saved min will be {pos + MARGIN_DEG:+.1f})"
            elif k == "]":
                self.high[self.sel] = pos
                self.dirty = True
                self.msg = f"{name} HIGH end marked at {pos:+.1f} (saved max will be {pos - MARGIN_DEG:+.1f})"
            elif k == "z":
                self.home[self.sel] = pos
                self.dirty = True
                self.msg = f"{name} HOME set to {pos:+.1f}"
            elif k == "h":
                self._go(list(self.home))
                self.msg = "all joints at home"
            elif k == "S":
                self.msg = self.save()
            elif k == "q":
                return False
            else:
                self.msg = "unknown key"
        except Abort as e:
            self.refresh()
            self.msg = f"NOT MOVED FURTHER: {e}"
        return True

    def proposed(self) -> dict:
        new = copy.deepcopy(self.raw)
        for i, name in enumerate(JOINT_NAMES):
            j = new["joints"][name]
            if self.low[i] is not None and self.high[i] is not None:
                j["min"] = round(self.low[i] + MARGIN_DEG, 1)
                j["max"] = round(self.high[i] - MARGIN_DEG, 1)
            j["home"] = round(self.home[i], 1)
        return new

    def save(self) -> str:
        for i, name in enumerate(JOINT_NAMES):
            if (self.low[i] is None) != (self.high[i] is None):
                return f"not saved: {name} has only one end marked. Mark both [ and ], or neither."
        new = self.proposed()
        try:
            config_mod.from_dict(new)
        except ConfigError as e:
            return f"not saved: {e}"
        if os.path.exists(self.save_path):
            shutil.copyfile(self.save_path, self.save_path + ".bak")
        tmp = self.save_path + ".tmp"
        with open(tmp, "w", encoding="utf-8") as f:
            json.dump(new, f, indent=2)
            f.write("\n")
        os.replace(tmp, self.save_path)
        self.raw = new
        self.dirty = False
        return f"saved to {self.save_path} (old version in .bak)"

    def screen(self) -> str:
        new = self.proposed()
        rows = ["IRIS calibration   (calibration mode: max 3 deg per step, 5 deg/s)", "",
                "     joint   angle    home   low end  high end   ->  saved min .. max   servo"]
        for i, name in enumerate(JOINT_NAMES):
            j = new["joints"][name]
            f = lambda v: "   --  " if v is None else f"{v:+7.1f}"
            mark = ">" if i == self.sel else " "
            rows.append(f"  {mark}  {name:5s} {self.st['pos'][i]:+7.1f} {self.home[i]:+7.1f} "
                        f"{f(self.low[i])}  {f(self.high[i])}      {j['min']:+7.1f} .. {j['max']:+7.1f}"
                        f"   {servo_text(self.cfg, name, self.st['pos'][i])}")
        rows += ["", f"  step {self.step} deg" + ("    * unsaved changes (S to save)" if self.dirty else ""),
                 "", HELP, "", f"  {self.msg}"]
        return "\n".join(rows)


# ---- terminal + service plumbing (Pi) -------------------------------------

def read_key(fd) -> str:
    import select
    c = os.read(fd, 1)
    if c == b"\x1b":
        r, _, _ = select.select([fd], [], [], 0.05)
        if r:
            seq = os.read(fd, 2)
            return {b"[A": "UP", b"[B": "DOWN", b"[C": "RIGHT", b"[D": "LEFT"}.get(seq, "")
        return ""
    return c.decode(errors="ignore")


def service_running(addr) -> bool:
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        request(s, addr, protocol.encode_status())
        return True
    except Abort:
        return False
    finally:
        s.close()


def start_service(dry_run: bool, addr) -> subprocess.Popen:
    cmd = [sys.executable, "-m", "iris.motor.service", "--calibrate", "--quiet"]
    if dry_run:
        cmd.append("--dry-run")
    log = open(SERVICE_LOG if os.name != "nt" else os.path.join(os.environ.get("TEMP", "."),
                                                                  "iris_motor_calibrate.log"), "w")
    proc = subprocess.Popen(cmd, cwd=str(REPO_ROOT), stdout=log, stderr=subprocess.STDOUT,
                            start_new_session=(os.name != "nt"))
    deadline = time.monotonic() + 15
    while time.monotonic() < deadline:
        if proc.poll() is not None:
            raise SystemExit(f"motor service exited early; see {log.name}")
        if service_running(addr):
            return proc
        time.sleep(0.3)
    stop_service(proc)
    raise SystemExit(f"motor service did not come up; see {log.name}")


def stop_service(proc: subprocess.Popen) -> None:
    if proc.poll() is None:
        proc.send_signal(signal.SIGINT if os.name != "nt" else signal.SIGTERM)
        try:
            proc.wait(5)
        except subprocess.TimeoutExpired:
            proc.terminate()
            proc.wait(5)


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description="IRIS joint calibration")
    ap.add_argument("--dry-run", action="store_true", help="fake PCA9685, nothing moves")
    ap.add_argument("--save-to", default=str(config_mod.DEFAULT_CONFIG_PATH))
    ap.add_argument("--keys", help=argparse.SUPPRESS)  # scripted keys for testing, comma separated
    args = ap.parse_args(argv)

    cfg = config_mod.load()
    raw = config_mod.load_raw()
    addr = ("127.0.0.1", cfg.motor.udp_port)

    if service_running(addr):
        print("A motor service is already running. Stop it first with:\n"
              "  pkill -INT -f iris.motor.service")
        return 1

    print("=" * 70)
    print("IRIS calibration")
    print("When you continue, ALL servos snap to home:")
    print("  " + ", ".join(f"{j.name}={j.home:+.1f}" for j in cfg.joints))
    print("Put the arm near that pose by hand first.")
    print("=" * 70)
    if args.keys is None:
        try:
            if input("Type 'go' and Enter to start (anything else cancels): ").strip().lower() != "go":
                print("cancelled, nothing moved")
                return 0
        except (EOFError, KeyboardInterrupt):
            print("\ncancelled, nothing moved")
            return 0

    print("starting motor service (3 s countdown, then servos snap to home)...", flush=True)
    proc = start_service(args.dry_run, addr)
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        cal = Calibrator(cfg, raw, sock, addr, args.save_to)
        if args.keys is not None:
            for k in args.keys.split(","):
                if not cal.key(k):
                    break
                print(f"[{k}] {cal.msg}")
            print(cal.screen())
            return 0
        run_terminal(cal)
        return 0
    finally:
        sock.close()
        stop_service(proc)
        print("Motor service stopped. Servos are limp.")


def run_terminal(cal: Calibrator) -> None:
    import termios
    import tty

    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setcbreak(fd)
        while True:
            sys.stdout.write("\x1b[2J\x1b[H" + cal.screen() + "\n")
            sys.stdout.flush()
            k = read_key(fd)
            if k == "q":
                note = "  UNSAVED CHANGES will be lost. " if cal.dirty else "  "
                sys.stdout.write(f"\n{note}Quit? Servos go limp, hold the arm. y = quit, "
                                 "S = save and quit, other = stay\n")
                sys.stdout.flush()
                k2 = read_key(fd)
                if k2 == "S":
                    cal.msg = cal.save()
                    if not cal.dirty:
                        print(cal.msg)
                        return
                elif k2 == "y":
                    return
                continue
            cal.key(k)
            termios.tcflush(fd, termios.TCIFLUSH)  # drop keys typed during the move
    except KeyboardInterrupt:
        pass
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)


if __name__ == "__main__":
    sys.exit(main())
