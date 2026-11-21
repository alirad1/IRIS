"""Small UDP client for the motor service, for GUI loops on the PC.

Never blocks for long (default 0.2 s), matches replies to requests by seq, and
returns None instead of raising when the service doesn't answer.
"""
from __future__ import annotations

import socket
import time
from typing import Optional, Sequence

from iris import protocol


class MotorClient:
    def __init__(self, host: str, port: int, timeout_s: float = 0.2) -> None:
        self.addr = (host, port)
        self.timeout_s = timeout_s
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.seq = 0
        self.last: Optional[dict] = None
        self.error: Optional[str] = None

    def _drain(self) -> None:
        self.sock.setblocking(False)
        try:
            while True:
                self.sock.recvfrom(65536)
        except (BlockingIOError, ConnectionResetError, OSError):
            pass

    def _req(self, data_fn) -> Optional[dict]:
        self._drain()
        self.seq = (self.seq + 1) % 1_000_000
        try:
            self.sock.sendto(data_fn(self.seq), self.addr)
        except OSError as e:
            self.error = f"send failed: {e}"
            return None
        deadline = time.monotonic() + self.timeout_s
        while True:
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                self.error = "motor service not answering"
                return None
            self.sock.settimeout(remaining)
            try:
                raw, _ = self.sock.recvfrom(65536)
            except socket.timeout:
                self.error = "motor service not answering"
                return None
            except (ConnectionResetError, OSError):
                self.error = "motor service not running"
                return None
            try:
                reply = protocol.decode_reply(raw)
            except ValueError:
                continue
            if reply.get("seq") == self.seq:
                self.last, self.error = reply, (None if reply.get("ok") else reply.get("error"))
                return reply

    def status(self) -> Optional[dict]:
        return self._req(lambda s: protocol.encode_status(seq=s))

    def target(self, targets: Sequence[float]) -> Optional[dict]:
        return self._req(lambda s: protocol.encode_target(targets, seq=s))

    def hold(self) -> Optional[dict]:
        return self._req(lambda s: protocol.encode_hold(seq=s))

    def close(self) -> None:
        self.sock.close()


def connect_checked(host: str, port: int) -> "MotorClient":
    """Connect and make sure the running service uses the same servo settings as
    this PC (same channels/trims/invert) and normal mode. Exits otherwise."""
    from iris import config as config_mod

    c = MotorClient(host, port, timeout_s=0.5)
    st = c.status()
    if st is None:
        raise SystemExit(f"Motor service at {host}:{port} is not answering. Start it on the Pi:\n"
                         "  bash ~/IRIS/scripts/run_pi.sh")
    if "calibrate" in st.get("mode", ""):
        raise SystemExit("The motor service is in calibration mode. Stop it on the Pi and start "
                         "the normal one:\n  bash ~/IRIS/scripts/run_pi.sh")
    mine = config_mod.fingerprint()
    if not str(st.get("config", "")).startswith(mine):
        raise SystemExit("The motor service on the Pi uses different servo settings than this PC "
                         "(config/iris.json differs). Deploy first, then restart the service.")
    c.timeout_s = 0.2
    return c


def stationary(st: dict, goal: Sequence[float], tol: float = 0.05) -> bool:
    return all(abs(p - g) <= tol for p, g in zip(st["pos"], goal))
