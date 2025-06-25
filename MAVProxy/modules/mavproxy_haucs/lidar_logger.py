from datetime import datetime
import csv
import os
from pymavlink import mavutil

HEADER = ['time', 'type', 'param0', 'param1', 'param2', 'param3', 'param4', 'param5', 'param6']
GPS_HEADER = ['time', 'GLOBAL_POSITION_INT', 'lat', 'lng', 'alt', 'hdg', 'vx', 'vy', 'vz']
DISTANCE_HEADER = ['time', 'DISTANCE_SENSOR', 'dist', 'N/A', 'N/A', 'N/A', 'N/A', 'N/A', 'N/A']
ATTITUDE_HEADER = ['time', 'ATTITUDE', 'roll', 'pitch', 'yaw', 'rollspd', 'pitchspd', 'yawspd', 'N/A']

lidar_file = ""

def write(data):
    global lidar_file
    if len(lidar_file) > 0:
        with open(lidar_file,'a',newline='') as csvfile:
            writer = csv.writer(csvfile, delimiter=',')
            writer.writerow(data)

def init():
    filePath = "lidar_data/"
    date = datetime.now().strftime('%Y-%m-%d-%H-%M-%S')

    if not os.path.exists(filePath):
        os.mkdir(filePath)

    global lidar_file
    lidar_file = filePath + "/" + date + ".csv"

    with open(lidar_file,'w',newline='') as csvfile:
      writer = csv.writer(csvfile, delimiter=',')
      writer.writerow(HEADER)
      writer.writerow(GPS_HEADER)
      writer.writerow(DISTANCE_HEADER)
      writer.writerow(ATTITUDE_HEADER)

def subscribe(vehicle):
    vehicle.master.mav.command_long_send(
        vehicle.target_system,
        vehicle.target_component,
        mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, # command
        0, # confirmation
        mavutil.mavlink.MAVLINK_MSG_ID_DISTANCE_SENSOR, # param1: message id
        100000, #param2: interval in microseconds (1 second)
        0,0,0,0,0)
    vehicle.master.mav.command_long_send(
        vehicle.target_system,
        vehicle.target_component,
        mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, # command
        0, # confirmation
        mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE, # param1: message id
        100000, #param2: interval in microseconds (1 second)
        0,0,0,0,0)
    vehicle.master.mav.command_long_send(
        vehicle.target_system,
        vehicle.target_component,
        mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, # command
        0, # confirmation
        mavutil.mavlink.MAVLINK_MSG_ID_GLOBAL_POSITION_INT, # param1: message id
        100000, #param2: interval in microseconds (1 second)
        0,0,0,0,0)