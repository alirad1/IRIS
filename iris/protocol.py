"""Wire formats.

Motor commands (UDP, JSON, one object per datagram):
    {"cmd": "target", "targets": [j0, pair, j3, j4]}
    {"cmd": "status"}
Both may carry an optional integer "seq" that is echoed back. Any other key,
command, length or value is rejected. There is no message that names a
channel.

Camera frames (TCP): 4-byte big-endian length, then that many bytes of JPEG.
"""
from __future__ import annotations

import json
import socket
import struct
from typing import Any, Dict, List, Optional, Sequence, Tuple

from iris.motor.core import CommandError, validate_targets

MAX_DATAGRAM = 1024
MAX_FRAME_BYTES = 5 * 1024 * 1024


class ProtocolError(CommandError):
    pass


def encode_target(targets: Sequence[float], seq: Optional[int] = None) -> bytes:
    msg: Dict[str, Any] = {"cmd": "target", "targets": [float(x) for x in targets]}
    if seq is not None:
        msg["seq"] = seq
    return json.dumps(msg).encode("utf-8")


def encode_status(seq: Optional[int] = None) -> bytes:
    msg: Dict[str, Any] = {"cmd": "status"}
    if seq is not None:
        msg["seq"] = seq
    return json.dumps(msg).encode("utf-8")


def parse_command(data: bytes) -> Tuple[str, Optional[List[float]], Optional[int]]:
    """Returns (cmd, targets or None, seq or None). Raises ProtocolError."""
    if len(data) > MAX_DATAGRAM:
        raise ProtocolError("datagram too large")
    try:
        msg = json.loads(data.decode("utf-8"))
    except (UnicodeDecodeError, ValueError):
        raise ProtocolError("not valid JSON")
    if not isinstance(msg, dict):
        raise ProtocolError("message must be a JSON object")
    seq = msg.get("seq")
    if seq is not None and (isinstance(seq, bool) or not isinstance(seq, int)):
        raise ProtocolError("seq must be an integer")
    cmd = msg.get("cmd")
    if cmd == "status":
        allowed = {"cmd", "seq"}
    elif cmd == "target":
        allowed = {"cmd", "targets", "seq"}
    else:
        raise ProtocolError(f"unknown cmd {cmd!r}")
    extra = set(msg) - allowed
    if extra:
        raise ProtocolError(f"unknown keys {sorted(extra)}")
    if cmd == "status":
        return cmd, None, seq
    if "targets" not in msg:
        raise ProtocolError("target command needs 'targets'")
    try:
        targets = validate_targets(msg["targets"])
    except CommandError as e:
        raise ProtocolError(str(e))
    return cmd, targets, seq


def encode_reply(reply: Dict[str, Any]) -> bytes:
    return json.dumps(reply, separators=(",", ":")).encode("utf-8")


def decode_reply(data: bytes) -> Dict[str, Any]:
    return json.loads(data.decode("utf-8"))


# ---- camera frames -------------------------------------------------------

def send_frame(sock: socket.socket, jpeg: bytes) -> None:
    sock.sendall(struct.pack(">I", len(jpeg)) + jpeg)


def _recv_exact(sock: socket.socket, n: int) -> bytes:
    buf = bytearray()
    while len(buf) < n:
        chunk = sock.recv(n - len(buf))
        if not chunk:
            raise ConnectionError("stream closed")
        buf.extend(chunk)
    return bytes(buf)


def recv_frame(sock: socket.socket) -> bytes:
    (n,) = struct.unpack(">I", _recv_exact(sock, 4))
    if n == 0 or n > MAX_FRAME_BYTES:
        raise ConnectionError(f"bad frame length {n}")
    return _recv_exact(sock, n)
