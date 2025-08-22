# Forked Version of MavProxy

## HAUCS CODE

All mavproxy development occurs in the haucs module located in `MAVProxy/modules/mavproxy_haucs`.

## Initial Setup

Run the following commands to create a virtual environment
```
python3 -m venv mavproxy --system-site-packages
```

Make sure to initialize the virtual environment whenever running or developing the code

```
source mavpproxy/bin/activate
```

### MAVProxy Development Guide

Follow all of the MAVProxy instructions for setting up your development environment:

https://ardupilot.org/mavproxy/docs/development/index.html

### Additional Steps

Install all of the necessary python libraries for HAUCS

```
pip3 install -r requirements.txt
```

