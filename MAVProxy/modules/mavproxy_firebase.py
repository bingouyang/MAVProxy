#!/usr/bin/env python
from MAVProxy.modules.lib import mp_module
from MAVProxy.modules.lib import mp_util
from MAVProxy.modules.lib import mp_settings
import firebase_admin
from firebase_admin import db
from firebase_admin import credentials
import time
import json
import os
from pymavlink import mavutil

#### COMMAND PROMPTS ####
# mavproxy.py --master=/dev/cu.usbserial-B001793K  --aircraft=splashy
# mavproxy.py --master=/dev/cu.usbserial-B001793K  --aircraft=splashy --out 172.20.10.5:14550

LANDED_STATE = {0:'unknown',
                1:'landed',
                2:'in air',
                3:'taking off',
                4:'landing'}

#### FIREBASE FUNCTIONS ####
def login(key_dict):
    """
    Start a Firebase Instance

    return: an app instance
    """
    data = json.loads(key_dict)
    cred = credentials.Certificate(data)
    return firebase_admin.initialize_app(cred, {'databaseURL': 'https://haucs-monitoring-default-rtdb.firebaseio.com'})

def logout(app):
    """
    Logout of a Firebase Instance
    """
    firebase_admin.delete_app(app)

def restart_firebase(app, key_dict):
    firebase_admin.delete_app(app)
    time.sleep(10)
    new_app = login(key_dict)
    return new_app

class firebase(mp_module.MPModule):
    def __init__(self, mpstate):
        """Initialise module"""
        super(firebase, self).__init__(mpstate, "firebase", "")
        self.status_callcount = 0
        self.logged_in = False
        self.firebase_update = time.time()
        self.timers = {"NAMED_VALUE_FLOAT":time.time(),
                       "EXTENDED_SYS_STATE":time.time(),
                       "BATTERY_STATUS":time.time(),
                       "GLOBAL_POSITION_INT":time.time()}
        self.drone_variables = {}
        self.example_settings = mp_settings.MPSettings(
            [ ('verbose', bool, False),])
        
        self.add_command('firebase', self.cmd_firebase, "firebase module", ['status','set (LOGSETTING)'])

        self.drone_variables = {}

    def usage(self):
        '''show help on command line options'''
        return "Usage: example <status|set>"

    def extended_sys_subscribe(self):
        self.master.mav.command_long_send(
            self.target_system,
            self.target_component,
            mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, # command
            0, # confirmation
            mavutil.mavlink.MAVLINK_MSG_ID_EXTENDED_SYS_STATE, # param1: message id
            1000000, #param2: interval in microseconds
            0,
            0,
            0,
            0,
            0
        )

    def cmd_firebase(self, args):
        '''control behaviour of the module'''
        if len(args) == 0:
            print(self.usage())
        elif args[0] == "status":
            print(self.status())
        elif args[0] == "set":
            self.example_settings.command(args[1:])
        elif args[0] == "sub":
            print("subscribing ...")
            self.extended_sys_subscribe()
        elif args[0] == "login":
            self.extended_sys_subscribe()
            with open('fb_key.json', 'r') as file:
                fb_key = file.read()
            try:
                global fb_app
                fb_app = login(fb_key)
                self.logged_in = True
                print("logged in to firebase")
            except:
                print("failed login")
        elif args[0] == "logout":
            logout(fb_app)
            print("logged out")
            self.logged_in = False
        else:
            print(self.usage())

    def status(self):
        '''returns information about module'''
        self.status_callcount += 1
        return("logged in: " + 'yes' if self.logged_in else 'no')

    def idle_task(self):
        '''called rapidly by mavproxy'''
        if (time.time() - self.firebase_update) > 2:
            self.firebase_update = time.time()
            if self.logged_in:
                #upload variables
                db.reference('LH_Farm/drone/data/').set(self.drone_variables)
                #upload timers
                for i in self.timers:
                    period = time.time() - self.timers[i]
                    db.reference('LH_Farm/drone/time/' + i ).set(round(period, 2))
                

        # now = time.time()
        # if now-self.last_bored > self.boredom_interval:
        #     self.last_bored = now
        #     message = "testing"
        #     self.say("%s: %s" % (self.name,message))
            # See if whatever we're connected to would like to play:
            # self.master.mav.statustext_send(mavutil.mavlink.MAV_SEVERITY_NOTICE,
            #                                 message)


    def mavlink_packet(self, m):
        '''handle mavlink packets'''
        if self.logged_in:
            if m.get_type() == 'NAMED_VALUE_FLOAT':
               self.timers[m.get_type()] = time.time()
               self.drone_variables[m.name] = round(m.value,2)
            if m.get_type() == 'EXTENDED_SYS_STATE':
                self.timers[m.get_type()] = time.time()
                self.drone_variables['flight_status'] = LANDED_STATE[m.landed_state]
            if m.get_type() == 'GLOBAL_POSITION_INT':
                self.timers[m.get_type()] = time.time()
                self.drone_variables['lat'] = m.lat/1e7
                self.drone_variables['lon'] = m.lon/1e7
                self.drone_variables['alt'] = m.alt/1000
                self.drone_variables['hdg'] = m.hdg/100
            if m.get_type() == 'BATTERY_STATUS':
                self.timers[m.get_type()] = time.time()
                self.drone_variables['voltage'] = m.voltages[0]/1000
                self.drone_variables['currrent'] = m.current_battery/100

def init(mpstate):
    '''initialise module'''
    return firebase(mpstate)