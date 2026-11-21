"""Load and validate config/iris.json. Shared by the Pi and the PC.

The config can set limits, home, channels and trims. It can NOT set servo
direction signs: the pair rule (CH1 = +pair, CH2 = -pair) lives in code,
see iris.motor.core.servo_signs.
"""
from __future__ import annotations

import json
import math
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Dict, Tuple

JOINT_NAMES: Tuple[str, ...] = ("j0", "pair", "j3", "j4")
SERVO_CENTER_DEG = 135.0
SERVO_RANGE_DEG = 270.0

# Hard caps. Config values outside these are rejected, not clamped.
MAX_TRIM_DEG = 30.0
MAX_SPEED_CAP_DPS = 30.0
MAX_ACCEL_CAP_DPS2 = 200.0
MAX_TIMEOUT_S = 0.5
CAL_MAX_SPEED_CAP_DPS = 5.0
CAL_MAX_STEP_CAP_DEG = 3.0

REPO_ROOT = Path(__file__).resolve().parent.parent
DEFAULT_CONFIG_PATH = REPO_ROOT / "config" / "iris.json"


class ConfigError(ValueError):
    pass


@dataclass(frozen=True)
class ServoCfg:
    channel: int
    trim: float


@dataclass(frozen=True)
class JointCfg:
    name: str
    min: float
    max: float
    home: float
    servos: Tuple[ServoCfg, ...]


@dataclass(frozen=True)
class PcaCfg:
    i2c_address: int
    frequency_hz: int
    min_pulse_us: int
    max_pulse_us: int
    actuation_range_deg: float


@dataclass(frozen=True)
class MotorCfg:
    udp_bind: str
    udp_port: int
    loop_hz: float
    timeout_s: float
    max_speed_dps: float
    max_accel_dps2: float


@dataclass(frozen=True)
class CalibrationCfg:
    max_speed_dps: float
    max_step_deg: float


@dataclass(frozen=True)
class CameraCfg:
    tcp_port: int
    width: int
    height: int
    framerate: float
    jpeg_quality: int


@dataclass(frozen=True)
class Config:
    pca: PcaCfg
    motor: MotorCfg
    calibration: CalibrationCfg
    camera: CameraCfg
    joints: Tuple[JointCfg, ...]  # always in JOINT_NAMES order

    def joint(self, name: str) -> JointCfg:
        return self.joints[JOINT_NAMES.index(name)]

    @property
    def channels(self) -> Tuple[int, ...]:
        return tuple(s.channel for j in self.joints for s in j.servos)


def _section(d: Any, keys: Tuple[str, ...], where: str) -> Dict[str, Any]:
    if not isinstance(d, dict):
        raise ConfigError(f"{where}: expected an object")
    missing = [k for k in keys if k not in d]
    extra = [k for k in d if k not in keys]
    if missing:
        raise ConfigError(f"{where}: missing {missing}")
    if extra:
        raise ConfigError(f"{where}: unknown keys {extra}")
    return d


def _num(v: Any, where: str, lo: float = -math.inf, hi: float = math.inf) -> float:
    if isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(v):
        raise ConfigError(f"{where}: expected a finite number, got {v!r}")
    if not lo <= v <= hi:
        raise ConfigError(f"{where}: {v} not in [{lo}, {hi}]")
    return float(v)


def _int(v: Any, where: str, lo: int, hi: int) -> int:
    if isinstance(v, bool) or not isinstance(v, int):
        raise ConfigError(f"{where}: expected an integer, got {v!r}")
    if not lo <= v <= hi:
        raise ConfigError(f"{where}: {v} not in [{lo}, {hi}]")
    return v


def _positive(v: Any, where: str, hi: float) -> float:
    x = _num(v, where, 0.0, hi)
    if x <= 0:
        raise ConfigError(f"{where}: must be > 0")
    return x


def _joint(name: str, d: Any) -> JointCfg:
    where = f"joints.{name}"
    d = _section(d, ("min", "max", "home", "servos"), where)
    lo = _num(d["min"], f"{where}.min", -SERVO_CENTER_DEG, SERVO_CENTER_DEG)
    hi = _num(d["max"], f"{where}.max", -SERVO_CENTER_DEG, SERVO_CENTER_DEG)
    home = _num(d["home"], f"{where}.home")
    if not lo < hi:
        raise ConfigError(f"{where}: min must be < max")
    if not lo <= home <= hi:
        raise ConfigError(f"{where}: home {home} outside [{lo}, {hi}]")
    servos = d["servos"]
    want = 2 if name == "pair" else 1
    if not isinstance(servos, list) or len(servos) != want:
        raise ConfigError(f"{where}.servos: expected exactly {want} servo(s)")
    parsed = []
    for i, s in enumerate(servos):
        sw = f"{where}.servos[{i}]"
        # Only channel and trim. A 'sign' key is rejected on purpose.
        s = _section(s, ("channel", "trim"), sw)
        parsed.append(ServoCfg(
            channel=_int(s["channel"], f"{sw}.channel", 0, 15),
            trim=_num(s["trim"], f"{sw}.trim", -MAX_TRIM_DEG, MAX_TRIM_DEG),
        ))
    return JointCfg(name, lo, hi, home, tuple(parsed))


def from_dict(d: Any) -> Config:
    d = _section(d, ("pca9685", "motor", "calibration", "camera", "joints"), "config")

    p = _section(d["pca9685"], ("i2c_address", "frequency_hz", "min_pulse_us",
                                "max_pulse_us", "actuation_range_deg"), "pca9685")
    pca = PcaCfg(
        i2c_address=_int(p["i2c_address"], "pca9685.i2c_address", 0x40, 0x7F),
        frequency_hz=_int(p["frequency_hz"], "pca9685.frequency_hz", 50, 50),
        min_pulse_us=_int(p["min_pulse_us"], "pca9685.min_pulse_us", 500, 500),
        max_pulse_us=_int(p["max_pulse_us"], "pca9685.max_pulse_us", 2500, 2500),
        actuation_range_deg=_num(p["actuation_range_deg"], "pca9685.actuation_range_deg",
                                 SERVO_RANGE_DEG, SERVO_RANGE_DEG),
    )

    m = _section(d["motor"], ("udp_bind", "udp_port", "loop_hz", "timeout_s",
                              "max_speed_dps", "max_accel_dps2"), "motor")
    if not isinstance(m["udp_bind"], str):
        raise ConfigError("motor.udp_bind: expected a string")
    motor = MotorCfg(
        udp_bind=m["udp_bind"],
        udp_port=_int(m["udp_port"], "motor.udp_port", 1024, 65535),
        loop_hz=_num(m["loop_hz"], "motor.loop_hz", 10, 200),
        timeout_s=_positive(m["timeout_s"], "motor.timeout_s", MAX_TIMEOUT_S),
        max_speed_dps=_positive(m["max_speed_dps"], "motor.max_speed_dps", MAX_SPEED_CAP_DPS),
        max_accel_dps2=_positive(m["max_accel_dps2"], "motor.max_accel_dps2", MAX_ACCEL_CAP_DPS2),
    )

    c = _section(d["calibration"], ("max_speed_dps", "max_step_deg"), "calibration")
    calibration = CalibrationCfg(
        max_speed_dps=_positive(c["max_speed_dps"], "calibration.max_speed_dps",
                                CAL_MAX_SPEED_CAP_DPS),
        max_step_deg=_positive(c["max_step_deg"], "calibration.max_step_deg",
                               CAL_MAX_STEP_CAP_DEG),
    )

    cam = _section(d["camera"], ("tcp_port", "width", "height", "framerate",
                                 "jpeg_quality"), "camera")
    camera = CameraCfg(
        tcp_port=_int(cam["tcp_port"], "camera.tcp_port", 1024, 65535),
        width=_int(cam["width"], "camera.width", 64, 4096),
        height=_int(cam["height"], "camera.height", 64, 4096),
        framerate=_positive(cam["framerate"], "camera.framerate", 120),
        jpeg_quality=_int(cam["jpeg_quality"], "camera.jpeg_quality", 10, 100),
    )
    if motor.udp_port == camera.tcp_port:
        raise ConfigError("motor.udp_port and camera.tcp_port must differ")

    jd = _section(d["joints"], JOINT_NAMES, "joints")
    joints = tuple(_joint(n, jd[n]) for n in JOINT_NAMES)

    channels = [s.channel for j in joints for s in j.servos]
    if len(set(channels)) != len(channels):
        raise ConfigError(f"joints: channels must all be different, got {channels}")

    cfg = Config(pca, motor, calibration, camera, joints)

    # Every servo must stay inside its physical 0..270 range across [min, max].
    # Imported here to avoid a cycle; core owns the sign rule.
    from iris.motor.core import physical_envelope
    for j in joints:
        lo, hi = physical_envelope(j)
        if j.min < lo or j.max > hi:
            raise ConfigError(
                f"joints.{j.name}: [{j.min}, {j.max}] goes past the servo range "
                f"with these trims (allowed [{lo}, {hi}])")
    return cfg


def load(path: Path | str = DEFAULT_CONFIG_PATH) -> Config:
    with open(path, "r", encoding="utf-8") as f:
        return from_dict(json.load(f))


def load_raw(path: Path | str = DEFAULT_CONFIG_PATH) -> Dict[str, Any]:
    with open(path, "r", encoding="utf-8") as f:
        return json.load(f)
