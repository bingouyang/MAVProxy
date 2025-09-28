import random, csv
import matplotlib
#from matplotlib import pyplot as plt
from scipy.optimize import curve_fit
import numpy as np
import pandas as pd
import json, logging, time, os, sys
from time import sleep
from datetime import datetime
import subprocess
import serial
from subprocess import call

import firebase_admin
from firebase_admin import credentials,db

import time
import threading
import hashlib

############### Running On Startup ###############
data_folder = "../data/"

### INIT JSON FILE
sdata = {}

sensor_file = folder + "sensor.json"
header = ['time', 'do', 'temperature', 'pressure']
pond_history = np.array(["unknown"])

################# defining functions############################
def exp_func(x, a, b, c):
    return a * np.exp(-b * x) + c
        
def find_subsequence_index(A, B):
    len_A = len(A)
    len_B = len(B)
    # Iterate through A to find if B exists as a contiguous subsequence
    for i in range(len_A - len_B + 1):
        if A[i:i+len_B].equals(B):
            return i+len_B # return the end in A.
    return -1

def get_pond_id(lat, lng):
    global pond_history
    global sdata
    df = pd.read_csv('sampling_points.csv')
    pond_ids = df.pop('pond')
    pond_gps = df.to_numpy()
    point = np.array([float(lng), float(lat)])
    point_y = np.tile(point, (pond_gps.shape[0], 1))
    distances = np.linalg.norm(pond_gps - point_y, axis=1)
    min_dist = distances.min() * 111_000
    if min_dist < 100:
        pond_id = str(pond_ids[np.argmin(distances)])
    else:
        pond_id = "unknown"
   
    if pond_id == "unknown":
        pass # do not append
    elif len(pond_history) < 2:
        pond_history = np.append(pond_history, [pond_id])
    else:
        pond_history = np.append(pond_history[1:2], [pond_id])
    sdata['pond_id'] = pond_id
    save_json()
    return pond_id
    
def db_helper():
    global sdata
    counter = 0
    ######################## Initialization ###############################
    cnt = 0
    global sdata
    init_DO = -1
    init_pressure = -1
    prev_pond_id = -1
    prev_sample_size = -1

    init_connect = -1 #set a flag to indicate first time connect to the payload
    check_size = -1 # check the dsize

    while True:   
        if msg[0].startswith("df"):#dfinish
            save_json()
            try:                
                truck_id =sdata['name'] #hard code it for now
                print(f'len of do: {len(do)}')
                avg_do_perc = 100*do[do > 0].mean()
                print(f'pond_id:{pond_id} lat:{lat} lng:{lng} avg_do_perc:{avg_do_perc} init_do:{init_DO} init_pressure:{init_pressure}')
                data = {'do': avg_do_perc, 'init_do': init_DO, 'init_pressure': init_pressure,
                        'lat': lat, 'lng': lng, 'pid': pond_id, 'pressure': [float(p.mean())], 'sid': truck_id,
                        'temp': [float(t.mean())],
                        'batt_v': batt_v, 'type': 'truck'}
                db.reference('LH_Farm/pond_' + pond_id + '/' + message_time + '/').set(data)
                fails['internet'] = 0
            except:
                logger.warning("uploading data to firebase failed")
                print("uploading data to firebase failed")
                fails['internet'] += 1          
            ######## reset the values and counters    #################################        
            cnt = 0
            check_size = 1 # reset the check size flag to resume checking for next pond
            prev_sample_size = -1
            msg = 'sample reset' 
            ble_uart_write(msg)
            sleep(5)
                
        time.sleep(1)

