cd ..\
python setup.py build install --user >err.log

python -X dev -u -v .\MAVProxy\mavproxy.py --console --aircraft=splashy --out 127.0.0.1:14550 
pause