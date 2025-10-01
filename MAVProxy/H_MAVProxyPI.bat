cd ..\
python setup.py build install --user

set MAVLINK_DIALECT=ardupilotmega
set MAVLINK20=1

python .\MAVProxy\mavproxy.py --master=udp:0.0.0.0:14555 --aircraft=splashy --console --cmd "set moddebug 3"

pause