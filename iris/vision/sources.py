"""Frame sources. Vision code only sees BGR numpy frames, never where they came from.

Each source has read() -> BGR frame (raises ConnectionError when the source is
gone) and close().
"""
from __future__ import annotations

import socket

import cv2
import numpy as np

from iris import protocol


class TcpJpegSource:
    """Frames from iris.camera.streamer over TCP."""

    def __init__(self, host: str, port: int, timeout_s: float = 5.0) -> None:
        self.host, self.port = host, port
        self._sock = socket.create_connection((host, port), timeout=timeout_s)
        self._sock.settimeout(timeout_s)

    def read(self) -> np.ndarray:
        try:
            jpeg = protocol.recv_frame(self._sock)
        except socket.timeout:
            raise ConnectionError("no frame within timeout")
        frame = cv2.imdecode(np.frombuffer(jpeg, np.uint8), cv2.IMREAD_COLOR)
        if frame is None:
            raise ConnectionError("could not decode JPEG")
        return frame

    def close(self) -> None:
        self._sock.close()


class OpenCVSource:
    """A local webcam index or a video file, for testing without the Pi."""

    def __init__(self, spec) -> None:
        self._cap = cv2.VideoCapture(int(spec) if str(spec).isdigit() else spec)
        if not self._cap.isOpened():
            raise ConnectionError(f"cannot open {spec}")

    def read(self) -> np.ndarray:
        ok, frame = self._cap.read()
        if not ok:
            raise ConnectionError("end of stream")
        return frame

    def close(self) -> None:
        self._cap.release()


# Later, to run vision on the Pi: a PicameraSource whose read() returns
# Picamera2.capture_array("main"), with no other change to vision code.
