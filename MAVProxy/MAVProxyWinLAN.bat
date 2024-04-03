cd ..\
python setup.py build install --user
python .\MAVProxy\mavproxy.py --master=udp:127.0.0.1:14550 --aircraft=splashy --console
pause
