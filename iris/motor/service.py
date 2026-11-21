"""IRIS motor service. The ONLY program that touches the PCA9685.

Run on the Pi from ~/IRIS:
    ~/iris-servo-venv/bin/python -m iris.motor.service --dry-run
    ~/iris-servo-venv/bin/python -m iris.motor.service            # real servos
    ~/iris-servo-venv/bin/python -m iris.motor.service --calibrate

Safety, enforced here whatever arrives over the network:
  * clamp to config limits (calibration mode: physical range, 3 deg steps, 5 deg/s)
  * speed and acceleration limits
  * no valid target for timeout_s (0.5 s): brake and hold
  * on exit: all PWM outputs off, chip released
"""
from __future__ import annotations

import argparse
import dataclasses
import select
import signal
import socket
import sys
import time
from typing import Callable, Optional

from iris import config as config_mod
from iris import protocol
from iris.config import JOINT_NAMES, Config
from iris.motor.core import CommandError, MotionController, ServoBus
from iris.motor.hw import FakePCA9685, RealPCA9685

STARTUP_COUNTDOWN_S = 3


class MotorService:
    """Network-free core of the service, so tests can drive it with a fake clock."""

    def __init__(self, cfg: Config, driver, calibrate: bool = False,
                 mode: str = "normal") -> None:
        self.cfg = cfg
        self.driver = driver
        self.mode = mode
        self.controller = MotionController(cfg, calibrate=calibrate)
        self.bus = ServoBus(cfg, driver)
        self.started = False

    def start(self, t: float) -> None:
        pos = self.controller.reset_to_home(t)
        self.bus.apply(pos)
        self.started = True

    def tick(self, t: float) -> None:
        if not self.started:
            raise RuntimeError("start() first")
        self.bus.apply(self.controller.step(t))

    def handle(self, data: bytes, t: float) -> bytes:
        seq = None
        try:
            cmd, targets, seq = protocol.parse_command(data)
            if cmd == "target":
                self.controller.set_target(targets, t)
            reply = {"ok": True}
        except CommandError as e:
            reply = {"ok": False, "error": str(e)}
        reply.update(self.controller.state())
        reply["mode"] = self.mode
        reply["joints"] = list(JOINT_NAMES)
        if seq is not None:
            reply["seq"] = seq
        return protocol.encode_reply(reply)

    def stop(self) -> None:
        try:
            self.bus.release()
        finally:
            self.driver.deinit()


def serve(service: MotorService, sock: socket.socket, period: float,
          should_stop: Callable[[], bool], now: Callable[[], float] = time.monotonic,
          log_every_s: float = 1.0, verbose: bool = True) -> None:
    service.start(now())
    next_tick = now() + period
    next_log = now()
    while not should_stop():
        wait = max(0.0, next_tick - now())
        readable, _, _ = select.select([sock], [], [], wait)
        if readable:
            for _ in range(64):  # drain, but never starve the motion loop
                try:
                    data, addr = sock.recvfrom(protocol.MAX_DATAGRAM + 1)
                except (BlockingIOError, InterruptedError):
                    break
                except ConnectionResetError:  # Windows ICMP noise; harmless
                    continue
                sock.sendto(service.handle(data, now()), addr)
        t = now()
        if t >= next_tick:
            was_holding = service.controller.holding
            service.tick(t)
            if verbose and service.controller.holding and not was_holding:
                c = service.controller
                print(f"no command for {c.timeout_s} s: braking to hold at "
                      f"{[round(x, 3) for x in c.target]} (pos {[round(x, 3) for x in c.pos]}, "
                      f"vel {[round(v, 2) for v in c.vel]})", flush=True)
            next_tick += period
            if next_tick < t:
                next_tick = t + period
        if verbose and t >= next_log:
            st = service.controller.state()
            print(f"pos={st['pos']} target={st['target']} "
                  f"{'HOLD' if st['holding'] else 'MOVE'}", flush=True)
            next_log = t + log_every_s


def main(argv: Optional[list] = None) -> int:
    ap = argparse.ArgumentParser(description="IRIS motor service")
    ap.add_argument("--config", default=str(config_mod.DEFAULT_CONFIG_PATH))
    ap.add_argument("--dry-run", action="store_true",
                    help="fake PCA9685: no I2C, nothing moves")
    ap.add_argument("--calibrate", action="store_true",
                    help="calibration mode for the jog tool (physical range, 3 deg steps, slow)")
    ap.add_argument("--max-speed", type=float, metavar="DPS",
                    help="lower the speed limit for this run (can never raise it)")
    ap.add_argument("--quiet", action="store_true")
    args = ap.parse_args(argv)

    cfg = config_mod.load(args.config)
    if args.max_speed is not None:
        if not 0 < args.max_speed <= cfg.motor.max_speed_dps:
            print(f"--max-speed must be > 0 and <= config {cfg.motor.max_speed_dps}")
            return 2
        cfg = dataclasses.replace(
            cfg, motor=dataclasses.replace(cfg.motor, max_speed_dps=args.max_speed))
    mode = ("dry-run " if args.dry_run else "") + ("calibrate" if args.calibrate else "normal")
    mode = mode.strip()

    stop = {"flag": False}

    def on_signal(signum, frame):
        stop["flag"] = True
    signal.signal(signal.SIGINT, on_signal)
    if hasattr(signal, "SIGTERM"):
        signal.signal(signal.SIGTERM, on_signal)

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((cfg.motor.udp_bind, cfg.motor.udp_port))
    sock.setblocking(False)

    home = {j.name: j.home for j in cfg.joints}
    print(f"IRIS motor service, mode={mode}, UDP {cfg.motor.udp_bind}:{cfg.motor.udp_port}")
    print(f"limits: " + ", ".join(f"{j.name}=[{j.min}, {j.max}]" for j in cfg.joints))
    print(f"speed {cfg.motor.max_speed_dps if not args.calibrate else min(cfg.motor.max_speed_dps, cfg.calibration.max_speed_dps)} deg/s, "
          f"timeout {cfg.motor.timeout_s} s")

    if args.dry_run:
        driver = FakePCA9685(cfg.pca, cfg.channels)
    else:
        print("!" * 70)
        print(f"WARNING: servos will SNAP to home {home} when PWM starts.")
        print("Place the arm near home now. Ctrl-C to abort.")
        print("!" * 70, flush=True)
        for i in range(STARTUP_COUNTDOWN_S, 0, -1):
            if stop["flag"]:
                sock.close()
                print("aborted before PWM start")
                return 1
            print(f"starting PWM in {i}...", flush=True)
            time.sleep(1)
        if stop["flag"]:
            sock.close()
            print("aborted before PWM start")
            return 1
        driver = RealPCA9685(cfg.pca, cfg.channels)

    service = MotorService(cfg, driver, calibrate=args.calibrate, mode=mode)
    try:
        serve(service, sock, 1.0 / cfg.motor.loop_hz, lambda: stop["flag"],
              verbose=not args.quiet)
    finally:
        try:
            service.stop()
        finally:
            sock.close()
        print("PWM released (servos are limp). Bye.", flush=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())
