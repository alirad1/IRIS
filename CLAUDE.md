# IRIS: autonomous surgical light arm

Fresh project. No old code is used. The goal is a robot arm that aims a (future) surgical light at the surgeon's hand, tracked by a fixed camera. Motion must be smooth and slow: no jitter, oscillation, overshoot, or sudden moves.

## Hardware

- Raspberry Pi 5. User: `prestonnguyen`. IP: ask me (I get it with `hostname -I` on the Pi; it changes sometimes). Code goes in the GitHub repo https://github.com/alirad1/IRIS.
- Existing Pi venv with Picamera2, OpenCV, I2C and Adafruit libs already installed: `/home/prestonnguyen/iris-servo-venv`. Use it. Do not reinstall or create a new venv on the Pi.
- Camera: Picamera2 on CSI, 640x480, fixed at the top of the stand. It does NOT move with the arm.
- PCA9685 at I2C 0x40, 50 Hz. Five 270 degree servos, 500-2500 us pulses, Adafruit `Servo` class with `actuation_range=270` (135 = center). Servos run on a separate 6V supply.
- No light is attached yet. Just 5 servos and the camera.

## Channels (top of arm, farthest from ground, down to the base)

Verified on hardware 2026-11-21 (Phase 1 channel check, jogged ±10 by hand).

| Logical joint | PCA9685 channel(s) | Rule |
|---|---|---|
| `j0` (top) | CH0 | independent |
| `pair` | CH1 and CH2 | CH1 = +pair, CH2 = -pair (servos face opposite ways, already verified on hardware) |
| `j3` | CH3 | independent |
| `j4` (bottom) | CH4 | independent |

Axes of the two bottom joints, in a frame centred on the disk at the bottom of the arm: b1 points out from the disk, b2 points up, b3 = b1 x b2 (along the j4 motor). `j3` rotates about b1, `j4` rotates about b3. Positive directions are not recorded yet (Phase 2).

The pair rule is the most important rule in this project. CH1 and CH2 are one joint. No function, CLI, config, or network message may ever command CH1 or CH2 separately. If they get different commands, the arm breaks.

## Architecture

- **Pi:** one motor service that is the ONLY thing that touches the PCA9685. It accepts 4 logical targets `[j0, pair, j3, j4]` over UDP and enforces safety itself no matter what it receives: clamp to config limits, limit speed (deg/s), hold position if no command for 0.5 s, release PWM cleanly on exit. It also has a calibration mode for the jog tool. Plus a camera streamer (TCP, 4-byte big-endian length + JPEG).
- **PC (this machine):** MediaPipe hand tracking (`models/hand_landmarker.task`, download the official one) and control logic. Keep vision code separate from the frame source so it can run on the Pi later.
- Config is one file (YAML or JSON) with, per logical joint: `min`, `max`, `home`, and per servo a `trim` offset. Until I calibrate, every joint's limits are -10 to +10.
- Deploy to the Pi over SSH (rsync or scp) into `~/IRIS`. Keep it simple: one clean repo, a few files, a README with the exact run commands.

Hobby servos have no position feedback, and they snap to the commanded angle when PWM starts. On startup the service should command `home` only, and print a warning to place the arm near home first.

## Safety rules for you (Claude)

- Never run anything that moves a servo without me saying I'm watching and ready, in that same message.
- Never raise limits except by writing values I confirmed out loud.
- Moves during calibration: max 2-3 degrees per step, slow speed.
- If anything fails on the Pi, stop and tell me. Don't retry motor commands in a loop.

## Phases

### Phase 1: setup + build everything that doesn't need me
1. Setup: link this folder to the GitHub repo, set up a PC Python venv, and set up SSH key login to the Pi. If a step needs me (typing the Pi password once, getting the IP), give me the exact command to run.
2. Repo skeleton, config, README, Pi motor service, camera streamer, PC tracker, deploy script.
3. Unit tests for the pair rule, clamping, rate limiting, and the timeout. These run with a fake PCA9685, no hardware.
4. On the Pi over SSH, verify without moving anything: `i2cdetect` shows 0x40, the camera captures a frame, and the motor service starts in a dry-run mode.
5. Run the camera stream + hand tracking on the PC with the hand point drawn.
6. Channel check (needs me watching): twitch each one by 3 degrees from home and back, one at a time. Do `pair` first (CH1 and CH2 together using the pair rule), then CH0, CH3, CH4. After each one, ask me what moved, then update the channel table if I say it's different.

**STOP (Gate 1).** Tell me what to look at so I can confirm hand tracking works.

### Phase 2: calibrate each joint with me
Order: j0, j3, j4, then pair last. For each joint:
1. I put the arm near home and say ready.
2. You move that joint in 2-3 degree steps in one direction. I say "keep going" or "stop".
3. At "stop", record that angle minus a 3 degree margin. Then do the other direction.
4. I tell you which way is positive (e.g. "positive tilts the lamp down") and where home is. Write it into the config and commit.

**STOP (Gate 2)** after every joint has confirmed limits.

### Phase 3: aim calibration + autonomous mode
1. Build an aim calibration tool for me to run: camera view on the PC, keyboard jog of joints (within limits), and I click the spot on the table the arm is aimed at. About 9 spots in a grid. Save the map.
2. Map the hand's image position to joint targets by interpolating that map. If the hand is out of reach, clamp to the closest reachable aim and stay there.
3. Autonomous mode: follow the hand smoothly (one-euro or EMA filter, deadband, speed limit). Hand lost: hold position.

**STOP (Gate 3).** Tell me exactly how to run it so I can test.
