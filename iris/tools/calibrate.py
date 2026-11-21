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
from iris.tools.jog import Abort, Stopped, request, servo_text

MARGIN_DEG = 3.0
# The service rejects targets more than 3 deg from the position in calibration
# mode, so bigger steps are driven as a target that slides at most this far ahead.
LEAD_DEG = 2.9
STEP_SIZES = (0.5, 1.0, 2.0, 3.0, 5.0, 10.0)
RATE_HZ = 20.0
MOVE_TIMEOUT_S = 120.0
SERVICE_LOG = "/tmp/iris_motor_calibrate.log"

HELP = """\
  keys   1 2 3 4   pick joint (j0, pair, j3, j4)
         d or ->   move picked joint +step      a or <-   move it -step
         w or up   bigger step                  s or down smaller step
         [         mark one end here            ]         mark the other end here
         z         set HOME (zero) here         h         walk all joints to home
         S         save to config               q         quit (servos go limp)
         ANY KEY while moving = stop right there"""


class Calibrator:
    """Keyboard-independent logic, so it can be tested without a terminal."""

    def __init__(self, cfg, raw: dict, sock, addr, save_path: str) -> None:
        self.cfg, self.raw, self.sock, self.addr = cfg, raw, sock, addr
        self.save_path = save_path
        self.sel = 0
        self.step_i = STEP_SIZES.index(5.0)
        self.should_stop = None  # set by the terminal UI: True if a key was pressed
        self.session_path = os.path.join(os.path.dirname(os.path.abspath(save_path)),
                                         "calibration_session.json")
        self.home: List[float] = [j.home for j in cfg.joints]
        self.low: List[Optional[float]] = [None] * 4
        self.high: List[Optional[float]] = [None] * 4
        self.dirty = False
        self.msg = "All joints are at home. Pick a joint (1-4) and jog it with a / d."
        self._load_session()
        self.st = request(sock, addr, protocol.encode_status())

    # Marked ends are kept in a session file so a crash or restart loses nothing.
    def _load_session(self) -> None:
        try:
            with open(self.session_path, "r", encoding="utf-8") as f:
                s = json.load(f)
        except (OSError, ValueError):
            return
        loaded = []
        for i, j in enumerate(self.cfg.joints):
            e = s.get(j.name, {})
            flip = -1.0 if bool(e.get("invert", False)) != j.invert else 1.0
            for key, arr in (("end1", self.low), ("end2", self.high), ("home", self.home)):
                v = e.get(key)
                if isinstance(v, (int, float)) and not isinstance(v, bool):
                    if key == "home" and abs(flip * float(v) - j.home) > 1e-6:
                        self.dirty = True  # a home set last time but not saved
                    arr[i] = flip * float(v)
                    if key != "home":
                        loaded.append(j.name)
        if loaded:
            self.msg = (f"Loaded marked ends from last session for {', '.join(sorted(set(loaded)))}. "
                        "Pick a joint (1-4) and jog it with a / d.")

    def _save_session(self) -> None:
        s = {j.name: {"invert": j.invert, "end1": self.low[i], "end2": self.high[i],
                      "home": self.home[i]}
             for i, j in enumerate(self.cfg.joints)}
        tmp = self.session_path + ".tmp"
        with open(tmp, "w", encoding="utf-8") as f:
            json.dump(s, f, indent=2)
        os.replace(tmp, self.session_path)

    def ends(self, i: int):
        """(low, high) of joint i if both ends are marked (either order), else None."""
        a, b = self.low[i], self.high[i]
        if a is None or b is None:
            return None
        return min(a, b), max(a, b)

    @property
    def step(self) -> float:
        return STEP_SIZES[self.step_i]

    def _go(self, goal: List[float]) -> None:
        """Move smoothly to goal. The target sent never leads the position by
        more than LEAD_DEG, so the service's 3-deg calibration rule holds."""
        lims = self.st["limits"]
        for i, (g, (lo, hi)) in enumerate(zip(goal, lims)):
            if not lo <= g <= hi:
                raise Abort(f"{JOINT_NAMES[i]} {g:+.1f} is past the servo range [{lo}, {hi}]")
        t0 = time.monotonic()
        while any(abs(p - g) > 1e-3 for p, g in zip(self.st["pos"], goal)):
            if self.should_stop is not None and self.should_stop():
                self.st = request(self.sock, self.addr, protocol.encode_hold())
                raise Stopped("stopped by key press")
            carrot = [p + max(-LEAD_DEG, min(LEAD_DEG, g - p))
                      for p, g in zip(self.st["pos"], goal)]
            self.st = request(self.sock, self.addr, protocol.encode_target(carrot))
            if not self.st.get("ok"):
                raise Abort(f"service rejected the move: {self.st.get('error')}")
            if time.monotonic() - t0 > MOVE_TIMEOUT_S:
                raise Abort("move took too long")
            time.sleep(1.0 / RATE_HZ)

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
            elif k in ("[", "]"):
                (self.low if k == "[" else self.high)[self.sel] = pos
                self._save_session()
                self.dirty = True
                e = self.ends(self.sel)
                more = (f"saved range will be {e[0] + MARGIN_DEG:+.1f} .. {e[1] - MARGIN_DEG:+.1f}"
                        if e else "now mark the other end")
                self.msg = f"{name} end marked at {pos:+.1f} ({more})"
            elif k == "z":
                self.home[self.sel] = pos
                self._save_session()
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
        except Stopped:
            time.sleep(0.3)  # let it finish braking before showing the angle
            self.refresh()
            self.msg = f"STOPPED at {self.st['pos'][self.sel]:+.1f}"
        except Abort as e:
            self.refresh()
            self.msg = f"NOT MOVED FURTHER: {e}"
        return True

    def proposed(self) -> dict:
        new = copy.deepcopy(self.raw)
        for i, name in enumerate(JOINT_NAMES):
            j = new["joints"][name]
            e = self.ends(i)
            if e is not None:
                j["min"] = round(e[0] + MARGIN_DEG, 1)
                j["max"] = round(e[1] - MARGIN_DEG, 1)
            j["home"] = round(self.home[i], 1)
        return new

    def save(self) -> str:
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
        todo = [n for i, n in enumerate(JOINT_NAMES) if self.ends(i) is None]
        left = f" Still to mark (kept old limits): {', '.join(todo)}." if todo else ""
        return f"saved to {self.save_path} (old version in .bak).{left}"

    def screen(self) -> str:
        new = self.proposed()
        rows = ["IRIS calibration   (calibration mode: max 3 deg per step, 5 deg/s)", "",
                "     joint   angle    home   end [    end ]      ->  saved min .. max   servo"]
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


def service_status(addr) -> Optional[dict]:
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        return request(s, addr, protocol.encode_status())
    except Abort:
        return None
    finally:
        s.close()


def service_running(addr) -> bool:
    return service_status(addr) is not None


def stop_running_service(addr) -> None:
    """Stop a service we did not start ourselves (after reconnecting)."""
    try:
        subprocess.run(["pkill", "-INT", "-f", "iris.motor.service"], check=False)
    except FileNotFoundError:  # not on the Pi (e.g. Windows test run)
        print("could not stop the motor service automatically; stop it by hand")
        return
    deadline = time.monotonic() + 5
    while time.monotonic() < deadline and service_running(addr):
        time.sleep(0.3)


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

    existing = service_status(addr)
    if existing is not None:
        same = (existing.get("config") == config_mod.fingerprint()
                and "calibrate" in existing.get("mode", "")
                and ("dry-run" in existing.get("mode", "")) == args.dry_run)
        if not same:
            print("A motor service is already running, started with different settings\n"
                  "(or not in calibration mode), so this tool will not take it over.\n"
                  "Support the arm, then stop it (the arm goes limp):\n"
                  "  pkill -INT -f iris.motor.service\n"
                  "and run this again.")
            return 1
        print("Reconnecting to the motor service that is already running in calibration mode.")
        print("Nothing snaps; the arm stays where it is.", flush=True)
        return run_session(cfg, raw, addr, args, proc=None)

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
    return run_session(cfg, raw, addr, args, proc)


def run_session(cfg, raw, addr, args, proc) -> int:
    """proc is the service we started, or None if we reconnected to one."""
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
        if proc is not None:
            stop_service(proc)
        else:
            stop_running_service(addr)
        print("Motor service stopped. Servos are limp.")


def run_terminal(cal: Calibrator) -> None:
    import termios
    import tty

    import select

    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    cal.should_stop = lambda: bool(select.select([fd], [], [], 0)[0])
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
