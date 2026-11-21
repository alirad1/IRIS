"""Pi camera streamer: Picamera2 -> JPEG -> TCP (4-byte BE length + JPEG).

Run on the Pi from ~/IRIS:
    ~/iris-servo-venv/bin/python -m iris.camera.streamer
    ~/iris-servo-venv/bin/python -m iris.camera.streamer --snapshot /tmp/iris.jpg

One client at a time. When a client disconnects, it waits for the next one.
Never touches the PCA9685.
"""
from __future__ import annotations

import argparse
import socket
import sys
import time

from iris import config as config_mod
from iris import protocol


def open_camera(width: int, height: int, framerate: float):
    from picamera2 import Picamera2

    cam = Picamera2()
    # "RGB888" in Picamera2 gives pixels in B,G,R order, which is what OpenCV wants.
    cam_cfg = cam.create_video_configuration(
        main={"size": (width, height), "format": "RGB888"},
        controls={"FrameRate": float(framerate)},
    )
    cam.configure(cam_cfg)
    cam.start()
    time.sleep(0.5)  # let auto exposure settle
    return cam


def encode(cv2, frame, quality: int) -> bytes:
    ok, buf = cv2.imencode(".jpg", frame, [int(cv2.IMWRITE_JPEG_QUALITY), quality])
    if not ok:
        raise RuntimeError("JPEG encode failed")
    return buf.tobytes()


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description="IRIS camera streamer")
    ap.add_argument("--config", default=str(config_mod.DEFAULT_CONFIG_PATH))
    ap.add_argument("--snapshot", metavar="PATH", help="save one frame as JPEG and exit")
    args = ap.parse_args(argv)
    cc = config_mod.load(args.config).camera

    import cv2

    cam = open_camera(cc.width, cc.height, cc.framerate)
    try:
        if args.snapshot:
            frame = cam.capture_array("main")
            with open(args.snapshot, "wb") as f:
                f.write(encode(cv2, frame, 90))
            print(f"saved {frame.shape[1]}x{frame.shape[0]} frame to {args.snapshot}")
            return 0

        srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        srv.bind(("0.0.0.0", cc.tcp_port))
        srv.listen(1)
        print(f"camera {cc.width}x{cc.height} @ {cc.framerate} fps, TCP port {cc.tcp_port}",
              flush=True)
        try:
            while True:
                conn, addr = srv.accept()
                conn.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
                print(f"client {addr[0]} connected", flush=True)
                n, t0 = 0, time.monotonic()
                try:
                    while True:
                        frame = cam.capture_array("main")
                        protocol.send_frame(conn, encode(cv2, frame, cc.jpeg_quality))
                        n += 1
                        if n % 150 == 0:
                            print(f"{n / (time.monotonic() - t0):.1f} fps", flush=True)
                except (BrokenPipeError, ConnectionResetError, ConnectionAbortedError):
                    print(f"client {addr[0]} disconnected", flush=True)
                finally:
                    conn.close()
        except KeyboardInterrupt:
            pass
        finally:
            srv.close()
    finally:
        cam.stop()
        cam.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
