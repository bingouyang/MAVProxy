import time
import sys
import threading
import queue
from pymavlink import mavutil

# For connection state
#HEARTBEAT_TIMEOUT = 10
#start_time = time.time()
#last_heartbeat_time = time.time()
#connected = True  # Drone starts as connected (sends initial), but will detect if ground doesn't respond

# Queue for user input
input_queue = queue.Queue()

# Function for input thread
#def input_thread():
#    while True:
#        try:
#            user_input = input()  # This blocks here, but in a separate thread
#            input_queue.put(user_input.strip().lower())
#        except EOFError:
#            pass  # Handle Ctrl+C gracefully

# Start the input thread
#threading.Thread(target=input_thread, daemon=True).start()

# Replace with your Pi's IP
#PI_IP = '10.1.10.252'  # Update this!

# Fake GPS data for the Drone (simulating drone position)
FAKE_GPS = {
    'lat': 407148000,  # Example: 40.7148 (New York)
    'lon': -740058000,  # -74.0058
    'alt': 5000,  # 5 meters
    'relative_alt': 0,
    'vx': 0, 'vy': 0, 'vz': 0,
    'hdg': 90  # Heading east
}

# Create the connection (send to Pi)
#master = mavutil.mavlink_connection(f'udpout:{PI_IP}:14550')

# Send initial heartbeat to establish connection
#master.mav.heartbeat_send(
#    mavutil.mavlink.MAV_TYPE_QUADROTOR,  # Simulate a drone
#    mavutil.mavlink.MAV_AUTOPILOT_ARDUPILOTMEGA,
#    0, 0, 0
#)

#print("Initial Heartbeat sent! Waiting for connection...")
last_heartbeat_time = time.time()  # Assume initial send starts "connected"

# Function to send GPS data
def send_gps():
    print(f"Sending GPS from Drone: lat={FAKE_GPS['lat']/1e7}, lon={FAKE_GPS['lon']/1e7}")
    master.mav.global_position_int_send(
        time_boot_ms=int((time.time() - start_time) * 1000),
        lat=FAKE_GPS['lat'],
        lon=FAKE_GPS['lon'],
        alt=FAKE_GPS['alt'],
        relative_alt=FAKE_GPS['relative_alt'],
        vx=FAKE_GPS['vx'], vy=FAKE_GPS['vy'], vz=FAKE_GPS['vz'],
        hdg=FAKE_GPS['hdg']
    )

# Function to request GPS from Ground
def request_gps_from_ground():
    print("Requesting GPS from Ground...")
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_DO_SET_PARAMETER,
        0,
        1,  # Param1: Custom "send GPS" request
        0, 0, 0, 0, 0, 0
    )

# Main loop: Similar to ground side
try:
    while True:
        msg = master.recv_match(blocking=False)
        if msg:
            msg_type = msg.get_type()
            if msg_type == 'HEARTBEAT':
                print("Received Heartbeat from Ground")
                last_heartbeat_time = time.time()
                if not connected:
                    print("Reconnected!")
                    connected = True
            elif msg_type == 'GLOBAL_POSITION_INT':
                print(f"Received GPS from Ground: lat={msg.lat/1e7}, lon={msg.lon/1e7}, alt={msg.alt/1000}m")
            elif msg_type == 'COMMAND_LONG':
                if msg.param1 == 1:
                    print("Received GPS request from Ground")
                    if connected:
                        send_gps()

        # Send heartbeat every 5 seconds (always, even when disconnected, to probe for reconnect)
        if time.time() % 5 < 0.1:
            master.mav.heartbeat_send(
                mavutil.mavlink.MAV_TYPE_QUADROTOR,
                mavutil.mavlink.MAV_AUTOPILOT_ARDUPILOTMEGA,
                0, 0, 0
            )
            if not connected:
                print("Probing for ground station with heartbeat...")  # Optional debug; remove if too noisy

        # Check for disconnection
        if connected and (time.time() - last_heartbeat_time > HEARTBEAT_TIMEOUT):
            print("Connection lost! Waiting for reconnect...")
            connected = False

        # User input check
        try:
            user_input = input_queue.get_nowait()
            if user_input == 'request_gps':
                if connected:
                    request_gps_from_ground()
                else:
                    print("Not connected—cannot request. Waiting for heartbeat...")
            elif user_input == 'send_gps':
                if connected:
                    send_gps()
                else:
                    print("Not connected—cannot send. Waiting for heartbeat...")
            else:
                print("Unknown command. Try 'request_gps' or 'send_gps'")
        except queue.Empty:
            pass

        time.sleep(0.1)
except KeyboardInterrupt:
    print("Shutting down gracefully...")
    sys.exit(0)