from __future__ import annotations

import socket
import struct
import threading
import unittest
from pathlib import Path
import tempfile

from iris import pihost, protocol
from iris.protocol import ProtocolError


class CommandParsing(unittest.TestCase):
    def test_round_trip(self):
        cmd, t, seq = protocol.parse_command(protocol.encode_target([1, -2.5, 3, 4], seq=7))
        self.assertEqual((cmd, t, seq), ("target", [1.0, -2.5, 3.0, 4.0], 7))
        self.assertEqual(protocol.parse_command(protocol.encode_status()), ("status", None, None))

    def test_rejects(self):
        for data in (b'{"cmd":"status","targets":[0,0,0,0]}', b'{"cmd":"target","targets":[0,0,0,0],"seq":1.5}',
                     b'{"cmd":"target","targets":[0,0,0,0],"seq":true}', b'"status"', b'{}'):
            with self.assertRaises(ProtocolError, msg=data):
                protocol.parse_command(data)


class Frames(unittest.TestCase):
    def test_length_prefix_round_trip(self):
        a, b = socket.socketpair()
        try:
            payloads = [b"\xff\xd8" + bytes(range(256)) * 40, b"x"]
            t = threading.Thread(target=lambda: [protocol.send_frame(a, p) for p in payloads])
            t.start()
            got = [protocol.recv_frame(b), protocol.recv_frame(b)]
            t.join()
            self.assertEqual(got, payloads)
        finally:
            a.close()
            b.close()

    def test_header_is_big_endian(self):
        a, b = socket.socketpair()
        try:
            protocol.send_frame(a, b"abc")
            self.assertEqual(b.recv(7), struct.pack(">I", 3) + b"abc")
        finally:
            a.close()
            b.close()

    def test_bad_length_and_eof(self):
        a, b = socket.socketpair()
        try:
            a.sendall(struct.pack(">I", 0))
            with self.assertRaises(ConnectionError):
                protocol.recv_frame(b)
            a.close()
            with self.assertRaises(ConnectionError):
                protocol.recv_frame(b)
        finally:
            b.close()


class PiHost(unittest.TestCase):
    def test_ssh_config_parse(self):
        with tempfile.TemporaryDirectory() as d:
            p = Path(d) / "config"
            p.write_text("Host other\n  HostName 1.1.1.1\n\nHost iris-pi\n"
                         "  HostName 192.168.1.50\n  User prestonnguyen\n", encoding="utf-8")
            self.assertEqual(pihost.from_ssh_config(path=p), "192.168.1.50")
            self.assertIsNone(pihost.from_ssh_config("nope", path=p))


if __name__ == "__main__":
    unittest.main()
