
cd ..
python setup.py build install --user
REM Using ArduPilot-specific MAVLink message set.
set MAVLINK_DIALECT=ardupilotmega
REM Using MAVLink v2.0 encoding.
set MAVLINK20=1

REM ================= FULL DUPLEX PORT MAP ======================
REM Receive from Simulator:   --master=udp:127.0.0.1:14550
REM PC to Pi:                 --out=udpout:10.226.217.65:14551
REM PC from Pi:               --master=udpin:0.0.0.0:14555
REM to mission planner        --out=udp:127.0.0.1:14560
REM =============================================================

python .\MAVProxy\mavproxy.py --master=udp:127.0.0.1:14550 --out=udpout:10.226.217.65:14551 --master=udpin:0.0.0.0:14555 --out=udp:127.0.0.1:14560 --aircraft=splashy --console --cmd "set moddebug 3"

pause

REM ======================================================================
REM ======================================================================
REM ======================================================================
REM python .\MAVProxy\mavproxy.py --master=udp:127.0.0.1:14550 --out=udpout:192.168.1.193:14551 --master=udpin:0.0.0.0:14555 --out=udp:127.0.0.1:14560 --aircraft=splashy --console --cmd "set moddebug 3"
REM =======================================================================
REM python .\MAVProxy\mavproxy.py --master=udp:127.0.0.1:14550 --out=udpout:10.120.239.65:14551 --master=udpin:0.0.0.0:14555 --out=udp:127.0.0.1:14560 --aircraft=splashy --console --cmd "set moddebug 3"
REM =======================================================================
REM python .\MAVProxy\mavproxy.py --master=udp:127.0.0.1:14550 --out=udpout:192.168.1.196:14551 --master=udpin:0.0.0.0:14555 --out=udp:127.0.0.1:14560 --aircraft=splashy --console --cmd "set moddebug 3"
