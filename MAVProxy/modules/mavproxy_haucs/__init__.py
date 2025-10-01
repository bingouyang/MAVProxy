from MAVProxy.modules.lib import mp_module, mp_util, mp_settings
from MAVProxy.modules.mavproxy_haucs import path_planner
from MAVProxy.modules.mavproxy_haucs import lidar_logger
from MAVProxy.modules.mavproxy_haucs.waypoint_helper import *

from pymavlink import mavutil, mavwp
import firebase_admin
from firebase_admin import db, credentials, exceptions
import time
import json
import os
from shapely.geometry import Point
from shapely.geometry.polygon import Polygon
import math
import threading
import pandas as pd
import struct
import traceback, time

#### COMMAND PROMPTS ####
# mavproxy.py --master=/dev/cu.usbserial-B001793K  --aircraft=splashy
# mavproxy.py --master=/dev/cu.usbserial-B000G5WR  --aircraft=splashy
# mavproxy.py --master=/dev/cu.usbserial-B001793K  --aircraft=splashy --out 172.20.10.5:14550
# mavproxy.py --master=udp:127.0.0.1:14550 --aircraft=splashy



LANDED_STATE = {0:'unknown',
                1:'landed',
                2:'in air',
                3:'taking off',
                4:'landing'}

FLIGHT_MODE = {0:'Stabilize', 1:'Acro', 2:'AltHold', 3:'Auto', 4:'Guided', 5:'Loiter', 6:'RTL', 7:'Circle',
               9:'Land', 11:'Drift', 13:'Sport', 14:'Flip', 15:'AutoTune', 16:'PosHold', 17:'Brake'}

sensor_data_names = {
    0: "time",
    1: "DO",
    2: "temp",
    3: "pressure",
    4: "init_DO",
    5: "init_pressure",
    6: "batt_v",
}

sensor_data_values = {
    "time": [],
    "DO": [],
    "temp": [],
    "pressure": [],
    "init_DO":[],
    "init_pressure":[],
    "batt_v":[],
}
# payload and header for encoding
DATA_BYTES = 96
HDR_LEN = 8   # seq_id 32bit(4)  varbyte (variable type uint8)  base (int16)  len (uint8)
MAX_SAMPLES = (DATA_BYTES - HDR_LEN) // 1  # int8 residues
SCALE = 32    # tradeoff between accuracy (higher) vs dynamic range (lower). 
# set or read the two high bits in var_len (payload[7])
FLAG_NONE = 0
FLAG_EOF  = 1  # end of frame
FLAG_SOF  = 2  # optional start
FLAG_SOLO = 3  # optional solo

################### Logic to handle sampling ####################
# define KEYWORDS
NAV_TAKEOFF   = mavutil.mavlink.MAV_CMD_NAV_TAKEOFF
NAV_WP        = mavutil.mavlink.MAV_CMD_NAV_WAYPOINT
NAV_LAND      = mavutil.mavlink.MAV_CMD_NAV_LAND

NAV_IN_AIR    = mavutil.mavlink.MAV_LANDED_STATE_IN_AIR
NAV_ON_GROUND = mavutil.mavlink.MAV_LANDED_STATE_ON_GROUND

# landing/taking off status initialization
st = {
  'last': None, 'seen_init': False,
  'touch_t0': None, 'touch_confirmed': False,
  'final_pending': False
}

SENSORDIR = r'C:\SENSOR_DATA_ARCHIVE'
################# param for comm with PI ################################
PC_SYSID = 255
PC_COMP = 1
PI_SYSID = 200
PORT = 5770
########################################################################

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


def save_json(sdata,sensor_file):
    with open(sensor_file, 'w') as outfile:
        json.dump(sdata, outfile)

def append_json(key,value,sensor_file):
    with open(sensor_file, 'r') as file:
        data = json.load(file)
    data[key] = value
    with open(sensor_file, 'w') as file:
        json.dump(data, file, indent=4)

# -------------------- Msg Decoder --------------------
def msg_decoder(buf):
    seq_id = struct.unpack_from("!I", buf, 0)[0]
    var_byte = buf[4]
    var_base = struct.unpack_from("!h", buf, 5)[0]
    varlen_raw = buf[7]
    flags = (varlen_raw >> 6) & 0x3
    var_len = varlen_raw & 0x3F
    residues = list(struct.unpack_from("!" + "b"*var_len, buf, 8))
    is_resend = 1 if (var_byte & 0x80) else 0
    var_id = var_byte & 0x7F
    values = [var_base + r / SCALE for r in residues]
    return seq_id, is_resend, var_id, var_len, values, flags

class haucs(mp_module.MPModule):
    def __init__(self, mpstate):
        """Initialise module"""
        super(haucs, self).__init__(mpstate, "haucs", "")
        self._hooked_ids = set()
        self._hooks_attached = 0
        self.console.writeln("[haucs] loaded; will attach raw hooks as masters come up")
        self.drone_id = "SPLASHY_UNK"        
        self.logged_in = False
        self.firebase_update = time.time()
        self.firebase_thread = False
        self.payload_update = time.time()
        self.time_boot_ms = 0
        self.timers = {"NAMED_VALUE_FLOAT":time.time(),
                       "BATTERY_STATUS":time.time(),
                       "GLOBAL_POSITION_INT":time.time(),
                       "GCS_HBEAT":time.time()}
        self.drone_variables = {"p_pres":0,
                                "on_water":False,
                                "battery_time":0,
                                "flight_time":0,
                                "mission_time":0,
                                "arm_state":"disarmed",}
        self.on_water = False
        self.pond_table = get_pond_table()
        self.pond_data = {"do":[],
                          "pressure":[],
                          "temp":[]}
        self.pressure_threshold = 1024
        self.initial_data = {"DO":0, "pressure":0}
        self.cal_count = 0
        self.cal_target = 0
        #waypoints
        self.wploader_by_sysid = {}
        self.loading_waypoints = False
        self.loading_waypoint_lasttime = time.time()
        
        self.haucs_settings = mp_settings.MPSettings(
            [ ('verbose', bool, False),])
        self.add_command('haucs', self.cmd_haucs, "haucs module", ['status','set (LOGSETTING)'])

        ################################
        # initialize take/off land status
        self.state = {'last': None, 'seen_init': False}
        
        self.sampling_lat = 0.0
        self.sampling_lng = 0.0
        try:
            lidar_logger.init()
            lidar_logger.subscribe(self)
            print("[haucs] lidar logger initialized")
        except Exception as e:
            print(f"[haucs] lidar logger init failed: {e}")

    @property
    def wploader(self):
        '''per-sysid wploader'''
        if self.target_system not in self.wploader_by_sysid:
            self.wploader_by_sysid[self.target_system] = mavwp.MAVWPLoader()
            self.wploader_by_sysid[self.target_system].expected_count = 0
        return self.wploader_by_sysid[self.target_system]
    
    def usage(self):
        '''show help on command line options'''
        return "Usage: haucs <cmd>\n\tstatus\n\tsub\n\tlogin\n\tlogout\n\tdo_init\n\tgen_mission\n\tset_threshold\n\tset_id"

    def _masters(self):
        mm = getattr(self.mpstate, "mav_master", None)
        if mm is None:
            return []
        if isinstance(mm, (list, tuple)):
            return [m for m in mm if m]
        return [mm]

    def _try_attach_hooks(self):
        new = 0
        for m in self._masters():
            if hasattr(m, "message_hooks") and id(m) not in self._hooked_ids:
                m.message_hooks.append(self._raw_hook)  # signature: (master, msg)
                self._hooked_ids.add(id(m))
                new += 1
        if new:
            self._hooks_attached += new
            self.console.writeln(f"[haucs] raw hook attached on {new} master(s) (total {self._hooks_attached})")

    # pymavlink calls hooks with (master, msg)
    def _raw_hook(self, master, msg):
        try:
            self.mavlink_packet(msg)  # forward every message into your canonical handler
        except Exception:
            err = traceback.format_exc(limit=3).strip().replace("\n", " | ")
            self.console.writeln(f"[haucs] mavlink_packet error: {err}")

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
            with open('../fb_key.json', 'r') as file:
                self.fb_key = file.read()
            try:
                self.fb_app = login(self.fb_key)
                self.logged_in = True
                print("logged in to firebase")
            except:
                print("failed login")
        elif args[0] == "logout":
            logout(self.fb_app)
            print("logged out")
            self.logged_in = False
        elif args[0] == "do_init":
            if self.drone_variables.get('p_DO') == None:
                print("initialization failed: DO sensor not connected")
            else:
                if len(args) < 2:
                    self.cal_target = 30
                else:
                    self.cal_target = int(args[1])
                self.initial_data['DO'] = 0
                self.cal_count = 0

                print(f"STARTING {self.cal_target}sec DO CALIBRATION ...")
                
        elif args[0] == "set_id":
            if len(args) != 2:
                print("set drone id\nexample: haucs set_id SPLASHY_1")
            else:
                self.drone_id = args[1]
        elif args[0] == "gen_mission":
            if len(args) != 6:
                print("gen_mission <source> (.csv) <alt> (meters) <delay> (seconds) <land> (True/False) <dive> (0 < x < alt)")
                print('example:\nhaucs gen_mission points 15 30 True 5')
            else:
                if not self.drone_variables.get('lat'):
                    print("!!!TEST MODE!!!: no data from gps (check power, gps status, GCS messages)")
                    test_home = (27.535321985800824, -80.35167917904866, 0) # lab
                    # test_home = (37.706386197905516, -89.45029871125445, 0) # logan hollow
                    # test_home = (37.70852528763561,  -89.45354741670316, 0)
                    self.gen_mission(test_home, 'true', args[1:])
                else:
                    home = (self.drone_variables['lat'], self.drone_variables['lon'], self.drone_variables['alt'])
                    self.gen_mission(home, 'false', args[1:])
        else:
            print(self.usage())

    def status(self):
        '''returns information about module'''
        output =  f"\n        logged in: {self.logged_in}"
        output += f"\n     payload init: {self.initial_data}"
        output += f"\n  pressure thrhld: {self.pressure_threshold}"
        output += f"\n         drone id: {self.drone_id}"
        for var in self.drone_variables:
            output += '\n{0: >17}: '.format(var) + str(self.drone_variables[var])
        return output

    def idle_task(self):
        '''called rapidly by mavproxy'''
        # ensure hooks are attached early
        if self._hooks_attached == 0:
            self._try_attach_hooks()
            
        # update firebase with drone status
        update_time = 1
        if (time.time() - self.firebase_update) > update_time:
            self.firebase_update = time.time()
            #handle flight time
            if self.drone_variables['arm_state'] == 'armed':
                #only update is drone is flying
                if self.drone_variables['current'] > 2:
                    self.drone_variables['flight_time'] += update_time
                    self.drone_variables['battery_time'] += update_time
                self.drone_variables['mission_time'] += update_time
            #handle database update
            if self.logged_in:
                #format time
                self.drone_variables['timers'] = {}
                for i in self.timers:
                    period = time.time() - self.timers[i]
                    self.drone_variables['timers'][i] = round(period, 2)

                #upload variables
                if not self.firebase_thread:
                    self.firebase_thread = True
                    db_thread = threading.Thread(target=self.idle_firebase_update, args=(self.drone_variables.copy(),))
                    db_thread.start()
        
        # update pond data
        if (time.time() - self.payload_update) > 1:
            self.payload_update = time.time()
            #self.handle_pond()
            #self.handle_DO_cal(1)

    def mavlink_packet(self, m):
        '''handle mavlink packets'''
        try:
            #self.console.writeln(f"msg type: {m.get_type()}")
            if m.get_type() == 'NAMED_VALUE_FLOAT':
                self.timers[m.get_type()] = time.time()
                self.drone_variables[m.name] = round(m.value,2)
                
            elif m.get_type() == 'GLOBAL_POSITION_INT':
                self.timers[m.get_type()] = time.time()
                self.drone_variables['lat'] = m.lat/1e7
                self.drone_variables['lon'] = m.lon/1e7
                self.drone_variables['alt'] = m.alt/1000
                self.drone_variables['hdg'] = m.hdg/100
                self.drone_variables['vel'] = round(math.sqrt(m.vx**2 + m.vy**2 + m.vz**2)/100, 2)
                lidar_logger.write([m.time_boot_ms, m.get_type(), m.lat, m.lon, m.alt, m.hdg, m.vx, m.vy, m.vz])
                
            elif m.get_type() == 'BATTERY_STATUS':
                self.timers[m.get_type()] = time.time()
                self.drone_variables['voltage'] = m.voltages[0]/1000
                self.drone_variables['current'] = m.current_battery/100
                self.drone_variables['mah_consumed'] = m.current_consumed
                self.drone_variables['battery_remaining'] = m.battery_remaining
                self.drone_variables['time_remaining'] = m.time_remaining
                
            elif m.get_type() == 'HEARTBEAT':
                self.handle_heartbeat(m)
                
            elif m.get_type() == 'STATUSTEXT':
                self.drone_variables['msg_severity'] = m.severity
                self.drone_variables['msg'] = m.text
                #handle unique id
                msg = m.text.split(' ')
                if msg[0] == 'CubeOrangePlus':
                    drone_id = " ".join(msg[1:])
                    if ID_LOOKUP.get(drone_id):
                        self.drone_id = ID_LOOKUP[drone_id]
                
                ### determine final landing is next ##################
                detect_mission_complete(m.text, st)
                
            elif m.get_type() == 'SYSTEM_TIME':
                if m.time_boot_ms < self.time_boot_ms:
                    print("REBOOT DETECTED")
                    self.drone_variables['battery_time'] = 0
                self.time_boot_ms = m.time_boot_ms
                
            elif m.get_type() in ["WAYPOINT_REQUEST", "MISSION_REQUEST"]:
                process_waypoint_request(self, m, self.master)
                
            elif m.get_type() == 'DISTANCE_SENSOR':
                lidar_logger.write([m.time_boot_ms, m.get_type(), m.current_distance, 0, 0, 0, 0, 0, 0])
                
            elif m.get_type() == 'ATTITUDE':
                lidar_logger.write([m.time_boot_ms, m.get_type(), m.roll, m.pitch, m.yaw, m.rollspeed, m.pitchspeed, m.yawspeed, 0])
                
            # use the EXTENDED_SYS_STATE to trigger sampling state machine
            elif m.get_type() == "EXTENDED_SYS_STATE":
                evt = handle_extsys_with_final(m.landed_state, st)
                if evt == "INIT_TAKEOFF":
                    gcs_broadcast(self, "[GCS] INIT_TAKEOFF")
                elif evt == "TAKEOFF":
                    gcs_broadcast(self, "[GCS] TAKEOFF")
                elif evt == "TOUCHDOWN_INTERMEDIATE":
                    gcs_broadcast(self, "[GCS] Touchdown (sampling)")
                    self.sampling_lat = self.drone_variables['lat']
                    self.sampling_lng = self.drone_variables['lon']

                    self.pond_id=get_pond_id(self.sampling_lat, self.sampling_lng)
                    print("Locked GPS:", self.sampling_lat, self.sampling_lng)
                elif evt == "TOUCHDOWN_FINAL":
                    gcs_broadcast(self, "[GCS] Touchdown (FINAL)")
            
            # handles data packets
            elif m.get_type() == "DATA96":
                self.console.writeln(f"msg type: {m.get_type()}")
                #data = bytes(bytearray(getattr(m, "data", b"")))
                #ln = getattr(m, "len", 0) or 0
                #payload = data[:ln]
                #if len(payload) >= 20:
                #    seq, unix_ts, tempC, press_kPa, DO_mgL = struct.unpack("<IIfff", payload[:20])
                #    # store or display
                #    self.console.writeln(
                #        f"[haucs] D96 seq={seq} T={tempC:.2f}C P={press_kPa:.2f}kPa DO={DO_mgL:.2f}"
                #    )

                self.proc_sensordata(m)
        except Exception as e:
                # never let one bad packet kill the whole dispatcher
                self.console.writeln(f"[haucs] mavlink handler error: {type(e).__name__}: {e}")
        
    def proc_sensordata(self, m):
        payload = bytes(m.data)[:m.len]      
        seq_id, is_resend, var_id, var_len, values, flags = msg_decoder(payload)
        name = sensor_data_names.get(var_id)
        if name is not None:              
            sensor_data_values[name].extend(values)
        else:
            self.console.writeln(f"[haucs] unknown var_id: {var_id}")
            return               
        self.console.writeln(f"PC got var {var_id} seq {seq_id} resend {is_resend} len {var_len} flags {flags}")
        if flags == FLAG_SOF:
            print("PC start of frame")
        elif flags == FLAG_EOF or flags == FLAG_SOLO:
            print("PC end of frame")
            message_time = time.strftime('%Y%m%d_%H:%M:%S', time.gmtime(time.time()))
            os.makedirs(SENSORDIR, exist_ok=True)
            sensor_file= os.path.join(SENSORDIR, f'{message_time}.json')
            #send_pond_data
            try:                
                drone_id =self.drone_id #hard code it for now
                pond_id  =self.pond_id
                do_array = sensor_data_values['DO']
                temp_array = sensor_data_values['temp']
                pres_array = sensor_data_values['pressure']
                ####################################################################
                # being lazy... pading init_DO, init_pressure, batt_v to same length as data,
                # here just grab one value.
                init_DO_list = sensor_data_values['init_DO']
                init_pressure_list = sensor_data_values['init_pressure']
                batt_v_list = sensor_data_values['batt_v']
                init_DO       = init_DO_list[0] if init_DO_list else 0.0
                init_pressure = init_pressure_list[0] if init_pressure_list else 0.0
                batt_v        = batt_v_list[0] if batt_v_list else 0.0
                lat = getattr(self, "sampling_lat", None)   ### CHANGED (safer access)
                lng = getattr(self, "sampling_lng", None)   ### CHANGED (fix: you had sampling_lng)

                data = {'do': do_array, 'init_do': init_DO, 'init_pressure': init_pressure,
                        'lat': lat, 'lng': lng, 'pid': str(pond_id), 'pressure': pres_array, 'sid': drone_id,
                        'temp': temp_array,
                        'batt_v': batt_v, 'type': 'winch'}
                
                save_json(data,sensor_file) # save the data
                # clear the current back of sensor data
                for k in sensor_data_values:
                    sensor_data_values[k] = []
                    
                db.reference(f"LH_Farm/pond_{pond_id}/{message_time}/").set(data)
                append_json('upload',1,sensor_file) # update the upload status
            except Exception as e:
                #logger.warning("uploading data to firebase failed")
                self.fb_app = restart_firebase(self.fb_app, self.fb_key)
                self.console.writeln(f"uploading data to firebase failed: {e}")
                append_json('upload',0,sensor_file) # update the upload status - to prepare for retry
    
    def handle_heartbeat(self, m):
        #flight mode
        mode = FLIGHT_MODE.get(m.custom_mode)
        if not mode:
            mode = 'unknown'
        self.drone_variables['flight_mode'] = mode
        #arm status
        if m.base_mode & 0b1000_0000:
            #handle just armed
            if self.drone_variables['arm_state'] == "disarmed":
                self.drone_variables['flight_time'] = 0   
                self.drone_variables['mission_time'] = 0   
            self.drone_variables['arm_state'] = "armed"
        else:
            self.drone_variables['arm_state'] = "disarmed"

    def idle_firebase_update(self, data):     
        try:
            db.reference('LH_Farm/drone/' + self.drone_id + '/data').set(data)
        except:
            print("FAILED FIREBASE UPDATE: no internet likely cause")
        self.firebase_thread = False

    def gen_mission(self, home, testing, args):
        mission_args = {"home": home}
        if args[0][-4:] != ".csv":
            mission_args['source'] = args[0] + ".csv"
            mission_args['output'] = args[0] + ".txt"
        else:
            mission_args['output'] = args[0:-4] + ".txt"

        if testing == 'true':
            mission_args['output'] = "test_" + mission_args['output']
        
        mission_args['alt'] =  int(args[1])
        mission_args['delay'] = int(args[2])
        mission_args['land'] = args[3].lower()

        if (int(args[4]) > mission_args['alt']) or (int(args[4]) <= 0):
            mission_args['dive'] = mission_args['alt']
        else:
            mission_args['dive'] = int(args[4])

        if mission_args['alt'] < 5:
            print(f"ALTITUDE ERROR: {mission_args['alt']}m is too low, set alt >= 5m")
        else:
            for i in mission_args:
                print('{0: >10}: '.format(i) + str(mission_args[i]))
            #get sorted coords
            sorted_coords = path_planner.main(self, mission_args)
            if testing == 'false':
                #load to drone
                load_waypoints(self, mission_args['output'])
                #load to website
                sorted_coords = [(home[0], home[1])] + sorted_coords + [(home[0], home[1])]
                db.reference('LH_Farm/drone/' + self.drone_id + '/mission/').set(sorted_coords)
                print('LH_Farm/drone/' + self.drone_id + '/mission/')
                print("uploading points", sorted_coords)

    def extended_sys_subscribe(self):
        self.master.mav.command_long_send(
            self.target_system,
            self.target_component,
            mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, # command
            0, # confirmation
            mavutil.mavlink.MAVLINK_MSG_ID_EXTENDED_SYS_STATE, # param1: message id
            1000000, #param2: interval in microseconds
            0,0,0,0,0)
    
    def get_init_DO(self):
        try:
            df = pd.read_csv('do_calibration.csv')
        except:
            df = pd.DataFrame({'time':[time.time()], 'value':[1]})
            df.to_csv('do_calibration.csv', index=False)

        return float(df['value'].iloc[-1])
    
    def handle_DO_cal(self, t):
        if self.cal_count < self.cal_target:
            if self.drone_variables['p_DO'] != 0:
                self.cal_count += t
                self.initial_data['DO'] += (self.drone_variables['p_DO'] - self.initial_data['DO']) / min(self.cal_count, 60)
                avg_data = round(self.initial_data['DO'], 1)
                new_data = round(self.drone_variables['p_DO'], 1)
                time_left = round(self.cal_target - self.cal_count)
                print(f"DO CALIBRATING ... sensor {new_data}mV ... average {avg_data}mV ... {time_left} second(s) left")  
            else:
                print("got a 0 ... trying again")
      
            if (self.cal_count >= self.cal_target):
                print(f"DO Calibration FINISHED: set to {round(self.initial_data['DO'], 2)}mV")
                try:
                    df = pd.read_csv('do_calibration.csv')
                except:
                    df = pd.DataFrame({'time':[time.time() - 1e7], 'value':[1]})
                df = pd.concat([df, pd.DataFrame([[time.time(), round(self.initial_data['DO'], 2)]], columns=df.columns)], axis=0, ignore_index=True)
                df.to_csv('do_calibration.csv', index=False)

    def rng_subscribe(self):
        self.master.mav.command_long_send(
            self.target_system,
            self.target_component,
            mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, # command
            0, # confirmation
            mavutil.mavlink.MAVLINK_MSG_ID_DISTANCE_SENSOR, # param1: message id
            100000, #param2: interval in microseconds (1 second)
            0,0,0,0,0)

def init(mpstate):
    '''initialise module'''
    return haucs(mpstate)

