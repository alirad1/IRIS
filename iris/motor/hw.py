"""PCA9685 drivers. Only iris.motor.core.ServoBus calls write_all().

Both drivers require every configured channel in every write, so a partial
write (e.g. CH1 without CH2) is impossible.
"""
from __future__ import annotations

from typing import Dict, Iterable, List, Optional

from iris.config import SERVO_RANGE_DEG, PcaCfg


def _check_full_write(channels: tuple, angles: Dict[int, float]) -> None:
    if set(angles) != set(channels):
        raise AssertionError(
            f"write must cover exactly channels {sorted(channels)}, got {sorted(angles)}")


class FakePCA9685:
    """Records writes, touches no hardware. Used by tests and --dry-run."""

    def __init__(self, pca: Optional[PcaCfg], channels: Iterable[int]) -> None:
        self.channels = tuple(channels)
        self.angles: Dict[int, Optional[float]] = {ch: None for ch in self.channels}
        self.history: List[Dict[int, Optional[float]]] = []
        self.released = False
        self.deinited = False

    def write_all(self, angles: Dict[int, float]) -> None:
        _check_full_write(self.channels, angles)
        if self.deinited:
            raise RuntimeError("write after deinit")
        self.angles.update(angles)
        self.history.append(dict(angles))
        self.released = False

    def release_all(self) -> None:
        self.angles = {ch: None for ch in self.channels}
        self.history.append(dict(self.angles))
        self.released = True

    def deinit(self) -> None:
        self.deinited = True


class RealPCA9685:
    """Adafruit PCA9685 + Servo(actuation_range=270, 500-2500 us).

    Creating this resets the chip with all outputs off. No servo is driven
    until the first write_all().
    """

    def __init__(self, pca: PcaCfg, channels: Iterable[int]) -> None:
        import board  # noqa: imported here so dry-run/tests never load I2C
        from adafruit_motor import servo
        from adafruit_pca9685 import PCA9685

        self.channels = tuple(channels)
        self._i2c = board.I2C()
        self._pca = PCA9685(self._i2c, address=pca.i2c_address)
        self._pca.frequency = pca.frequency_hz
        self._servos = {
            ch: servo.Servo(self._pca.channels[ch],
                            actuation_range=pca.actuation_range_deg,
                            min_pulse=pca.min_pulse_us,
                            max_pulse=pca.max_pulse_us)
            for ch in self.channels
        }

    def write_all(self, angles: Dict[int, float]) -> None:
        _check_full_write(self.channels, angles)
        for ch in self.channels:
            a = angles[ch]
            if not 0.0 <= a <= SERVO_RANGE_DEG:
                raise AssertionError(f"channel {ch} angle {a} outside 0..270")
            self._servos[ch].angle = a

    def release_all(self) -> None:
        for ch in range(16):
            self._pca.channels[ch].duty_cycle = 0

    def deinit(self) -> None:
        try:
            self._pca.deinit()
        finally:
            try:
                self._i2c.deinit()
            except Exception:
                pass
