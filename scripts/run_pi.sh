#!/usr/bin/env bash
# Start IRIS on the Pi (run in the Pi Connect shell):
#   bash ~/IRIS/scripts/run_pi.sh                  # normal motor service, 10 deg/s
#   bash ~/IRIS/scripts/run_pi.sh --max-speed 5    # slower
# Starts the camera streamer if it isn't running, then the motor service.
# Ctrl+C stops the motor service (servos go limp). If the browser shell drops,
# the service keeps holding the arm; run this again to see its log, or stop it
# with: pkill -INT -f iris.motor.service
cd ~/IRIS || exit 1
PY=~/iris-servo-venv/bin/python
LOG=/tmp/iris_motor.log

if pgrep -f "iris.camera.streamer" > /dev/null; then
  echo "camera streamer: already running"
else
  setsid -f "$PY" -m iris.camera.streamer > /tmp/iris_cam.log 2>&1 < /dev/null
  echo "camera streamer: started (log /tmp/iris_cam.log)"
fi

if pgrep -f "iris.motor.service" > /dev/null; then
  echo "motor service: already running. Showing its log (Ctrl+C stops it)."
else
  echo "motor service: starting. ALL SERVOS SNAP TO HOME after a 3 s countdown."
  echo "Place the arm near home now (Ctrl+C within 3 s aborts)."
  setsid -f "$PY" -m iris.motor.service "$@" > "$LOG" 2>&1 < /dev/null
  sleep 0.5
fi

trap 'echo; echo "stopping motor service (servos go limp)"; pkill -INT -f "iris.motor.service"; sleep 1; exit 0' INT
tail -n +1 -f "$LOG"
