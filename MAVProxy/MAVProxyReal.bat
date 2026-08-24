cd ..
python setup.py build install --user
REM Using ArduPilot-specific MAVLink message set.
set MAVLINK_DIALECT=ardupilotmega

REM Using MAVLink v2.0 encoding.
set MAVLINK20=1

@echo off
REM ==========================================================
REM REAL HARDWARE MODE
REM Pixhawk <-> PC via telemetry radio
REM Pixhawk <-> Companion computer via UART (on drone)
REM Mission Planner runs on this PC
REM ==========================================================

REM --- MAVLink settings (recommended for ArduPilot) ---
set MAVLINK_DIALECT=ardupilotmega
set MAVLINK20=1

REM --- Launch MAVProxy on control PC ---
REM Replace COM6 and baudrate with your telemetry radio values

python .\MAVProxy\mavproxy.py --master=COM7,57600 --out=udp:127.0.0.1:14550 --aircraft=splashy --console --cmd "set moddebug 3"

pause