# IRIS

Robot arm that aims a surgical light at the surgeon's hand. A fixed camera (Pi, CSI) watches the table; the PC tracks the hand with MediaPipe; the Pi drives 5 servos through a PCA9685.

```
PC                                         Pi 5
iris.tools.track_view  <-- TCP 5006 ----   iris.camera.streamer   (Picamera2, 640x480 JPEG)
iris.tools.twitch      --- UDP 5005 --->   iris.motor.service     (only thing that touches the PCA9685)
```

## Safety model

- **Pair rule.** CH1 and CH2 are one logical joint, `pair`: CH1 = 135 + trim + pair, CH2 = 135 + trim − pair. The signs are in code (`iris/motor/core.py`) and can't be set in config. Every PWM write covers all 5 channels, and no API, CLI or network message can name a channel.
- The motor service accepts only `[j0, pair, j3, j4]` and enforces everything itself:
  - It clamps to the config limits.
  - It limits speed (10 deg/s) and acceleration (40 deg/s²), with no overshoot.
  - If no valid target arrives for 0.5 s, it brakes and holds.
  - On exit (Ctrl-C or SIGTERM), it turns all PWM off.
- **On startup the servos snap to `home`.** Hobby servos have no position feedback. The service prints a warning and counts down 3 s, so put the arm near home first.
- **When the service exits, the servos go limp.** Support the arm.
- `--calibrate` is only for calibrating:
  - Limits widen to the servo's physical range.
  - The service rejects any target more than 3° from the current position.
  - Speed is capped at 5 deg/s.
  - It can only be turned on from the command line.

## Config

`config/iris.json` is shared by the PC and the Pi. Each joint has `min`, `max`, `home` (logical degrees, 0 = servo centre 135) and `servos: [{channel, trim}]`. Until calibration, every joint is limited to −10..+10. The values are validated on load, and hard caps in `iris/config.py` stop the speed, timeout and calibration step from being loosened.

| Logical joint | Channel(s) | Axis |
|---|---|---|
| `j0` (top) | CH0 | |
| `pair` | CH1 (+), CH2 (−) | |
| `j3` | CH3 | b1 (out from the bottom disk) |
| `j4` (bottom) | CH4 | b3 = b1 × b2 (along the j4 motor; b2 is up) |

Channel mapping verified on hardware 2026-11-21. Positive directions and real limits come in Phase 2.

## PC setup (Windows, once)

```powershell
py -3.13 -m venv .venv
.venv\Scripts\python -m pip install -r requirements-pc.txt
mkdir models
curl.exe -L -o models\hand_landmarker.task https://storage.googleapis.com/mediapipe-models/hand_landmarker/hand_landmarker/float16/latest/hand_landmarker.task
```

The SSH key and alias are set up once. Afterwards `ssh iris-pi` works with no password. When the Pi's IP changes, run `hostname -I` on the Pi and edit the `HostName` line under `Host iris-pi` in `%USERPROFILE%\.ssh\config`. The PC tools read the address from there too.

## Tests (no hardware)

```powershell
.venv\Scripts\python -m unittest -v
```

On the Pi: `cd ~/IRIS && ~/iris-servo-venv/bin/python -m unittest -v`

## Deploy to the Pi

```powershell
powershell -ExecutionPolicy Bypass -File scripts\deploy.ps1
```

This copies `iris/`, `config/`, `tests/` and the README into `~/IRIS` on the Pi. Add `-WithModel` to copy the MediaPipe model as well. Uses the existing venv `~/iris-servo-venv`; nothing gets installed.

## Run

All Pi commands run from `~/IRIS`, in an SSH session (`ssh iris-pi`).

Camera streamer (Pi):
```bash
cd ~/IRIS && ~/iris-servo-venv/bin/python -m iris.camera.streamer
```

Hand tracking view (PC). The window shows the stream, the hand landmarks and a green cross on the palm centre. Press `q` to quit.
```powershell
.venv\Scripts\python -m iris.tools.track_view
```

Motor service, dry run (Pi). This uses a fake PCA9685: no I2C, nothing moves.
```bash
cd ~/IRIS && ~/iris-servo-venv/bin/python -m iris.motor.service --dry-run
```

Motor service, real servos (Pi). **Place the arm near home first; it snaps to home.**
```bash
cd ~/IRIS && ~/iris-servo-venv/bin/python -m iris.motor.service --max-speed 5
```

Manual jog (Pi shell or PC, while the motor service runs). You type each move and it happens when you press Enter. Angles are absolute logical degrees (0 = home), within the service limits:
```bash
cd ~/IRIS && ~/iris-servo-venv/bin/python -m iris.tools.jog
jog> pair 10      # CH1 -> 145, CH2 -> 125
jog> home
jog> quit
```
In `--calibrate` mode, jog refuses any single move bigger than 3°.

Channel check (PC, while the motor service runs). This moves one logical joint +3° and back:
```powershell
.venv\Scripts\python -m iris.tools.twitch pair
.venv\Scripts\python -m iris.tools.twitch j0
```

Stop the service with Ctrl-C. The servos go limp.

## Layout

```
config/iris.json          the one config file
iris/config.py            load + validate
iris/protocol.py          UDP command format, TCP frame format
iris/motor/core.py        pair rule, clamping, speed/accel limit, timeout (pure, tested)
iris/motor/hw.py          real and fake PCA9685
iris/motor/service.py     UDP motor service
iris/camera/streamer.py   Picamera2 -> TCP
iris/vision/sources.py    frame sources (vision code doesn't know where frames come from)
iris/vision/hand.py       MediaPipe hand landmarker -> hand point
iris/tools/               PC tools: track_view, twitch
scripts/deploy.ps1        copy to the Pi
tests/                    unittest, fake hardware only
```
