#!/usr/bin/env python
from MAVProxy.modules.lib import mp_module, mp_util, mp_settings
import firebase_admin
from firebase_admin import db, credentials
import time
import json
import os
from pymavlink import mavutil
from shapely.geometry import Point
from shapely.geometry.polygon import Polygon
import math
from MAVProxy.modules.mavproxy_haucs import path_planner
import webbrowser

#### COMMAND PROMPTS ####
# mavproxy.py --master=/dev/cu.usbserial-B001793K  --aircraft=splashy
# mavproxy.py --master=/dev/cu.usbserial-B001793K  --aircraft=splashy --out 172.20.10.5:14550
# mavproxy.py --master=udp:127.0.0.1:14550 --aircraft=splashy

LANDED_STATE = {0:'unknown',
                1:'landed',
                2:'in air',
                3:'taking off',
                4:'landing'}

FLIGHT_MODE = {0:'Stabilize', 1:'Acro', 2:'AltHold', 3:'Auto', 4:'Guided', 5:'Loiter', 6:'RTL', 7:'Circle',
               9:'Land', 11:'Drift', 13:'Sport', 14:'Flip', 15:'AutoTune', 16:'PosHold', 17:'Brake'}

ID_LOOKUP = {'003A003C 30325113 37363931':'SPLASHY_1',
             '003F003F 30325115 33383839':'SPLASHY_2'}

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

def get_pond_table():
    with open("ponds.json") as file:
        data = json.load(file)

    ponds = {}
    for i in data['features']:
        id = i['properties']['number']
        coords = i['geometry']['coordinates'][0]
        ponds[id] = Polygon(coords)
    
    return ponds

class haucs(mp_module.MPModule):
    def __init__(self, mpstate):
        """Initialise module"""
        super(haucs, self).__init__(mpstate, "haucs", "")
        self.status_callcount = 0
        self.drone_id = "UNKNOWN"
        self.logged_in = False
        self.firebase_update = time.time()
        self.payload_update = time.time()
        self.timers = {"NAMED_VALUE_FLOAT":time.time(),
                       "EXTENDED_SYS_STATE":time.time(),
                       "BATTERY_STATUS":time.time(),
                       "GLOBAL_POSITION_INT":time.time()}
        self.drone_variables = {"p_pres":0,
                                "on_water":False}
        self.on_water = False
        self.pond_table = get_pond_table()
        self.pond_data = {"do":[],
                          "pressure":[],
                          "temp":[]}
        self.initial_data = {"DO":0,
                             "pressure":0}
        self.haucs_settings = mp_settings.MPSettings(
            [ ('verbose', bool, False),])
        
        self.add_command('haucs', self.cmd_haucs, "haucs module", ['status','set (LOGSETTING)'])

    def usage(self):
        '''show help on command line options'''
        return "Usage: haucs <cmd>\n\tstatus\n\tsub\n\tlogin\n\tlogout\n\tpayload_init\n\tgen_mission"

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

    def cmd_haucs(self, args):
        '''control behaviour of the module'''
        if len(args) == 0:
            print(self.usage())
        elif args[0] == "status":
            print(self.status())
        elif args[0] == "set":
            self.haucs_settings.command(args[1:])
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
        elif args[0] == "payload_init":
            self.initial_data['DO'] = self.drone_variables['p_DO']
            self.initial_data['pressure'] = self.drone_variables['p_pres']
            print(self.initial_data)
        elif args[0] == "gen_mission":
            if len(args) == 7:
                if args[6] == "test":
                    print("TESTING GEN_MISSION")
                    home = (27.535321985800824, -80.35167917904866, 0)
                    self.gen_mission(home, args[1:6])
            elif len(args) != 6:
                print("gen_mission <source> (.csv) <output> (.txt) <alt> (meters) <delay> (seconds) <land> (True/False)")
                print('example:\nhaucs gen_mission points mission 15 30 True')
            else:
                if not self.drone_variables.get('lat'):
                    print("PLANNING FAILED: no data from gps (check power, gps status, GCS messages)")
                else:
                    home = (self.drone_variables['lat'], self.drone_variables['lon'], self.drone_variables['alt'])
                    self.gen_mission(home, args[1:])
        else:
            print(self.usage())

    def status(self):
        '''returns information about module'''
        self.status_callcount += 1
        return("logged in: " + str(self.logged_in) + ", payload init: " + str(self.initial_data) + ", drone id: ", self.drone_id)

    def idle_task(self):
        '''called rapidly by mavproxy'''
        # update firebase with drone status
        if (time.time() - self.firebase_update) > 2:
            self.firebase_update = time.time()
            if self.logged_in:
                #format time
                periods = {}
                for i in self.timers:
                    period = time.time() - self.timers[i]
                    periods[i] = round(period, 2)

                data = {"data":self.drone_variables, "time":periods}
                #upload variables
                db.reference('LH_Farm/drone/' + self.drone_id + '/').set(data)
        
        # update pond data
        if (time.time() - self.payload_update) > 1:
            self.payload_update = time.time()
            self.handle_pond()


    def mavlink_packet(self, m):
        '''handle mavlink packets'''
        if m.get_type() == 'NAMED_VALUE_FLOAT':
            self.timers[m.get_type()] = time.time()
            self.drone_variables[m.name] = round(m.value,2)
        elif m.get_type() == 'EXTENDED_SYS_STATE':
            self.timers[m.get_type()] = time.time()
            self.drone_variables['flight_status'] = LANDED_STATE[m.landed_state]
        elif m.get_type() == 'GLOBAL_POSITION_INT':
            self.timers[m.get_type()] = time.time()
            self.drone_variables['lat'] = m.lat/1e7
            self.drone_variables['lon'] = m.lon/1e7
            self.drone_variables['alt'] = m.alt/1000
            self.drone_variables['hdg'] = m.hdg/100
            self.drone_variables['vel'] = round(math.sqrt(m.vx**2 + m.vy**2 + m.vz**2)/100, 2)
        elif m.get_type() == 'BATTERY_STATUS':
            self.timers[m.get_type()] = time.time()
            self.drone_variables['voltage'] = m.voltages[0]/1000
            self.drone_variables['current'] = m.current_battery/100
        elif m.get_type() == 'HEARTBEAT':
            mode = FLIGHT_MODE.get(m.custom_mode)
            if not mode:
                mode = 'unknown'
            self.drone_variables['flight_mode'] = mode
        elif m.get_type() == 'STATUSTEXT':
            self.drone_variables['msg_severity'] = m.severity
            self.drone_variables['msg'] = m.text

            #handle unique id
            msg = m.text.split(' ')
            if msg[0] == 'CubeOrangePlus':
                drone_id = " ".join(msg[1:])
                if ID_LOOKUP.get(drone_id):
                    self.drone_id = ID_LOOKUP[drone_id]

    def handle_pond(self):        
        #get pressure
        pressure = self.drone_variables['p_pres']
        #set water state
        prev_on_water = self.drone_variables['on_water']
        if pressure >1024:
            self.drone_variables['on_water'] = True
        else:
            self.drone_variables['on_water'] = False
        on_water = self.drone_variables['on_water']

        #state machine for handling pond data
        # if taking off, send data, clear data
        if prev_on_water and not on_water:
            self.send_pond_data()
            self.pond_data['pressure'] = []
            self.pond_data['do'] = []
            self.pond_data['temp'] = []
        # if sitting on pond, record data
        elif on_water:
            last_update = time.time() - self.timers['NAMED_VALUE_FLOAT']
            # if payload data is fresh
            if last_update < 5:
                self.pond_data['pressure'].append(self.drone_variables['p_pres'])
                self.pond_data['do'].append(self.drone_variables['p_DO'])
                self.pond_data['temp'].append(self.drone_variables['p_temp'])
            else:
                self.pond_data['pressure'].append(-1)
                self.pond_data['do'].append(-1)
                self.pond_data['temp'].append(-1)
        # initialize data
        else:
            if self.drone_variables.get('p_DO') and self.drone_variables.get('p_pres'):
                if (self.initial_data['pressure'] == 0):
                    self.initial_data['DO'] = self.drone_variables['p_DO']
                    self.initial_data['pressure'] = self.drone_variables['p_pres']
                    print("payload initialized")

    def send_pond_data(self):
        #get current location
        coord = [self.drone_variables['lon'], self.drone_variables['lat']]
        location = Point(coord)
        #get last do measurement
        print("last do")
        last_do = self.pond_data['do'][-1] / self.initial_data['DO'] * 100
        #get current pond
        pond_id = "unknown"
        for i in self.pond_table:
            if self.pond_table[i].contains(location):
                pond_id = str(i)
                break

        print("sampled at: ", pond_id)
        print("last do: ", last_do)
        print(self.pond_data)

        #upload data to firebase
        if self.logged_in:
            message_time = time.strftime('%Y%m%d_%H:%M:%S', time.gmtime(time.time()))
            data = {"lat":coord[1], "lng":coord[0],
                    "init_do":self.initial_data['DO'],
                    "init_pressure":self.initial_data['pressure'],
                    "drone_id":self.drone_id,
                    "type":"splashdrone",
                    **self.pond_data}
            
            db.reference('LH_Farm/pond_' + pond_id + '/' + message_time + '/').set(data)
            db.reference("LH_Farm/overview/pond_" + pond_id + '/last_do/').set(last_do)

            #update recent
            recent_data = db.reference('/LH_Farm/recent').order_by_key().limit_to_last(9).get()
            recent_data[message_time] = data
            db.reference('/LH_Farm/recent').set(recent_data)
    
            print("uploaded data")

    def gen_mission(self, home, args):
        input_file = args[0] + ".csv"
        output_file = args[1] + ".txt"
        alt =  int(args[2])
        delay = int(args[3])
        land = args[4]
        print(f"output file: {output_file}\n   altitude: {alt}\n      delay: {delay}\n       land: {land}")
        path_planner.main(input_file, output_file, home, alt, land, delay)


def init(mpstate):
    '''initialise module'''
    return haucs(mpstate)