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
REM Replace COM5 and baudrate with your telemetry radio values

python .\MAVProxy\mavproxy.py --master=COM5,57600 --out=udp:127.0.0.1:14560 --aircraft=splashy --console --cmd "set moddebug 3"

pause
