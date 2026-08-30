from MAVProxy.modules.lib import mp_module, mp_util, mp_settings
from MAVProxy.modules.mavproxy_haucs import path_planner
from MAVProxy.modules.mavproxy_haucs import lidar_logger
from MAVProxy.modules.mavproxy_haucs.waypoint_helper import *
from MAVProxy.modules.mavproxy_haucs.sampling_helper import *

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
import logging          # 083026


# 083026: GCS-side log file. The module had no file logging at all - every
# console.writeln() went to the MAVProxy window and was lost when it closed,
# so there was no record of pond resolution, DATA96 gating, or upload results
# after a session. This tees every writeln to a file without touching any of
# the ~30 call sites: the haucs class shadows MPModule's console property with
# a wrapper that logs, then forwards to the real console.
class _TeeConsole(object):
    def __init__(self, real, log):
        self._real = real
        self._log = log

    def writeln(self, text, *a, **kw):
        try:
            self._log.info(str(text))
        except Exception:
            pass                      # logging must never break the console
        return self._real.writeln(text, *a, **kw)

    def __getattr__(self, name):
        # everything else (write, set_status, ...) passes straight through
        return getattr(self._real, name)

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

# payload and header for encoding
# 081326: these local copies shadowed the values imported from sampling_helper
# and were already stale (HDR_LEN 8, flat SCALE 32 vs the encoder's SCALE_MAP).
# They fed only the dead msg_decoder in the triple-quoted block below, but
# leaving a wrong HDR_LEN next to a wire-format change is asking for trouble.
# The live values come from sampling_helper: DATA_BYTES, HDR_LEN, SCALE,
# SCALE_MAP, max_samples().
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
PI_SYSID = 1
PORT = 5770
########################################################################

ID_LOOKUP = {'003A003C 30325113 37363931':'SPLASHY_1',
             '003F003F 30325115 33383839':'SPLASHY_2'}

#### FIREBASE FUNCTIONS ####
def login(key_dict):
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
'''
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
'''
class haucs(mp_module.MPModule):
    def __init__(self, mpstate):
        """Initialise module"""
        super(haucs, self).__init__(mpstate, "haucs", "")

        # 083026: set this up first - the writeln below already goes through it.
        # An explicit FileHandler rather than logging.basicConfig(): MAVProxy
        # configures the root logger during startup, and basicConfig() is a
        # silent no-op once the root logger has handlers. propagate=False keeps
        # these lines out of MAVProxy's own log.
        self._hlog = logging.getLogger("haucs")
        self._hlog.setLevel(logging.INFO)
        self._hlog.propagate = False
        try:
            # prefer MAVProxy's per-flight directory, so haucs.log sits next to
            # flight.tlog and rotates with it
            _logdir = getattr(getattr(mpstate, "status", None), "logdir", None)
            if not _logdir:
                _logdir = "logs"
            os.makedirs(_logdir, exist_ok=True)
            self._hlog_path = os.path.join(_logdir, "haucs.log")
            if not self._hlog.handlers:
                _h = logging.FileHandler(self._hlog_path, encoding="utf-8")
                _h.setFormatter(logging.Formatter("%(asctime)s %(message)s"))
                self._hlog.addHandler(_h)
        except Exception as e:
            self._hlog_path = None
            print("[haucs] file logging disabled: %s" % e)
        self._tee = _TeeConsole(self.mpstate.console, self._hlog)
        self._hlog.info("=== haucs module loaded, log at %s ===" % self._hlog_path)
        # 083026: the decoding contract this GCS is running. Must match the
        # line the Pi logs at startup; a mismatch decodes to plausible wrong
        # numbers with no error on either side.
        try:
            self._hlog.info("wire contract: %s" % contract_id())
        except Exception as e:
            self._hlog.info("wire contract: unavailable (%s)" % e)

        self._hooked_ids = set()
        self._hooks_attached = 0
        self.console.writeln("[haucs] loaded; will attach raw hooks as masters come up")
        self.console.writeln("[haucs] log file: %s" % self._hlog_path)
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
                                "arm_state":"disarmed",
                                "current":0,
                                }
        self.on_water = False
        self.pond_table = get_pond_table()
        self.pond_data = {"do":[],
                          "pressure":[],
                          "temp":[],
                          "pond_id":"wukn",
                          "seq":0,
                          }
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
        self.add_command('haucs', self.cmd_haucs, "haucs module", ['status','set (LOGSETTING)','gen_mission_winch','gen_mission'])
        
        # montioring servo events
        self.locked_fix = None                         # (lat, lon) when edge detected
        self.gps_fix = {"lat": None, "lon": None}      # latest fix from GLOBAL_POSITION_INT

        # --- debugging counters: shows in the MAVProxy console via idle_task() below ---
        self._dbg_hb_count = 0
        self._dbg_servo_msg_count = 0
        self._dbg_servo_last_pwm = None
        self._dbg_servo_last_t = None
        self._dbg_servo_last_src = None
        self._dbg_data96_seen = 0
        self._dbg_data96_dropped = 0
        self._dbg_data96_processed = 0
        self._dbg_last_status_print = time.time()
        self._dbg_status_period = 5.0  # seconds between status lines

        self.servo_mon = {
            # Must match TRIGGER_RC_CH in the Pi's main script (currently RC8). This was
            # set to 9, which never matches the real trigger channel -- servo_mon["state"]
            # never flips to 1, so DATA96 packets were being silently dropped at the gate
            # in mavlink_packet(), and the GPS lock below never fired either.
            "chan": 8,          # <== set your winch/servo channel here (1..16)
            "on_th": 1800,      # rising edge when pwm crosses >= on_th (matches Pi's PWM_RELEASE)
            "off_th": 1200,     # (hysteresis lower band, matches Pi's PWM_RETRACT)
            "state": 0          # 0=OFF, 1=ON (debounced)
        }
        # How long (seconds) to keep accepting DATA96 after the falling edge.
        # The Pi doesn't send the payload instantly at the falling edge -- it
        # still has to BLE-FETCH from the sensor and transmit the frame, which
        # can take several seconds. Without this grace window, real DATA96
        # packets show up after servo_mon["state"] is already back to 0 and
        # get silently dropped.
        self._data96_grace_sec = 20.0
        self._data96_armed_until = 0.0  # monotonic deadline; 0 = not armed

        ################################
        # initialize take/off land status
        self.state = {'last': None, 'seen_init': False}
        
        self.sampling_lat = 0.0
        self.sampling_lng = 0.0
        # ----- per-frame aggregator (single-upload-per-frame) -----
        self._var_id_frame_end = 127   # must match Pi side
        self._last_uploaded_seq = None # idempotency guard
        # 081326: chunks, not flat lists. A frame is placed by its chunk_idx so
        # a lost DATA96 packet leaves an explicit gap instead of shifting every
        # later sample. _sensor_chunks[name] = {chunk_idx: [values]}
        self._sensor_chunks = {
            "DO": {}, "temp": {}, "pressure": {},
            "init_DO": {}, "init_pressure": {}, "batt_v": {}
        }
        self._sensor_data_values = {
            "DO": [], "temp": [], "pressure": [],
            "init_DO": [], "init_pressure": [], "batt_v": []
        }
        # keep time separately, also chunk-keyed
        self._time_chunks = {}
        self._time_buf = []
        self._frame_seq = None
        self._frame_done = set()

        try:
            lidar_logger.init()
            lidar_logger.subscribe(self)
            print("[haucs] lidar logger initialized")
        except Exception as e:
            print(f"[haucs] lidar logger init failed: {e}")

    # 083026: shadows MPModule.console so every writeln in this module is also
    # written to haucs.log. Falls back to the real console if __init__ has not
    # reached the setup yet, so nothing can break at import time.
    @property
    def console(self):
        tee = self.__dict__.get("_tee")
        return tee if tee is not None else self.mpstate.console

    @property
    def wploader(self):
        '''per-sysid wploader'''
        if self.target_system not in self.wploader_by_sysid:
            self.wploader_by_sysid[self.target_system] = mavwp.MAVWPLoader()
            self.wploader_by_sysid[self.target_system].expected_count = 0
        return self.wploader_by_sysid[self.target_system]
    
    def get_pond_id(self):
        #get current location
        coord = [self.sampling_lng, self.sampling_lat]
        location = Point(coord)
        #get last do measurement
        #get current pond
        pond_id = "wukn"
        for i in self.pond_table:
            if self.pond_table[i].contains(location):
                pond_id = str(i)
                break
        return pond_id

    def usage(self):
        '''show help on command line options'''
        return "Usage: haucs <cmd>\n\tstatus\n\tsub\n\tlogin\n\tlogout\n\tdo_init\n\tgen_mission\n\tset_threshold\n\tset_id\n\twinch release | fetch | clear"

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

    # ---- 081426: trigger the winch without an RC transmitter -------------
    #
    # The Pi decides what to do from trigger channel 8, and which message it
    # watches depends on flight mode (main_rc8_uart_parm.py):
    #   AUTO      -> SERVO_OUTPUT_RAW.servo8_raw, driven by mission DO_SET_SERVO
    #   any other -> RC_CHANNELS.chan8_raw, driven by the manual radio
    # Thresholds are PWM_RELEASE 1800 and PWM_RETRACT 1200.
    #
    # So the trigger has to match the mode. In AUTO we issue DO_SET_SERVO,
    # matching what the mission itself does. Otherwise we send an RC override
    # on channel 8, which is what the radio would have produced.
    TRIGGER_CH   = 8
    PWM_HIGH     = 1975
    PWM_LOW      = 1025

    def _winch_trigger(self, pwm, what):
        mode = self.drone_variables.get('flight_mode', 'unknown')
        auto = (str(mode).lower() == 'auto')

        if auto:
            # Mission-style servo command. Same path the waypoint sequence uses.
            self.master.mav.command_long_send(
                self.target_system, self.target_component,
                mavutil.mavlink.MAV_CMD_DO_SET_SERVO, 0,
                self.TRIGGER_CH, pwm, 0, 0, 0, 0, 0)
            how = "DO_SET_SERVO ch%d=%d" % (self.TRIGGER_CH, pwm)
        else:
            # RC override. Channels we are not driving are sent as 0, which
            # RELEASES any override on them rather than pinning them - never
            # send a fixed value to a channel you do not intend to control.
            chans = [0] * 8
            chans[self.TRIGGER_CH - 1] = pwm
            self.master.mav.rc_channels_override_send(
                self.target_system, self.target_component, *chans)
            how = "RC override ch%d=%d" % (self.TRIGGER_CH, pwm)

        print("[haucs] %s: %s (mode %s)" % (what, how, mode))
        # 082426: sev=4 is what gets this onto Mission Planner's high priority
        # line at all - INFO never appears there. repeat=4 repaints it so the
        # ~0.2s single paint becomes readable. This is also the easiest message
        # to fire on demand ("haucs winch release"), so it doubles as the test
        # case for the whole alert path.
        self.gcs_status("%s via %s" % (what, "servo" if auto else "rc"),
                        force=True, sev=4, repeat=4)
        if not auto:
            print("[haucs] override stays until 'haucs winch clear' or the "
                  "next trigger. The radio cannot move ch%d meanwhile."
                  % self.TRIGGER_CH)

    def _cycle_seconds(self):
        """
        081426: how long the Pi's winch cycle will take, from SCR_USER1/2/3.
        Returns (release, pause, retract_cap, total) or None if the params
        have not been fetched yet.
        """
        try:
            rel = float(self.get_mav_param("SCR_USER1", 0) or 0)
            pau = float(self.get_mav_param("SCR_USER2", 0) or 0)
            ret = float(self.get_mav_param("SCR_USER3", 0) or 0)
        except Exception:
            return None
        if rel <= 0 and pau <= 0 and ret <= 0:
            return None
        return rel, pau, ret, rel + pau + ret

    def cmd_winch(self, args):
        """haucs winch release | fetch | clear"""
        if not args:
            print("usage: haucs winch release | fetch | clear")
            print("  release  ch8 HIGH. Starts the Pi's whole timed cycle:")
            print("           release, bottom pause, then retract. The Pi runs")
            print("           it on its own timers; nothing here drives it.")
            print("  fetch    ch8 LOW. Only pulls samples off the sensor and")
            print("           uploads. Does NOT retract. Send it after the")
            print("           cycle above has finished, or you get a part cast.")
            print("  clear    drop any RC override on ch8")
            return

        if args[0] == "release":
            self._winch_trigger(self.PWM_HIGH, "winch RELEASE")
            c = self._cycle_seconds()
            if c:
                print("[haucs] Pi cycle: release %.0fs + pause %.0fs + retract "
                      "up to %.0fs = up to %.0fs. Wait for 'CAST COMPLETE' or "
                      "at least that long before 'haucs winch fetch'."
                      % c)
            else:
                print("[haucs] SCR_USER1/2/3 not read yet, so cycle length is "
                      "unknown. Wait for the Pi's retract to finish before "
                      "'haucs winch fetch'.")
        elif args[0] == "fetch":
            self._winch_trigger(self.PWM_LOW, "data FETCH and upload")
        elif args[0] == "clear":
            self.master.mav.rc_channels_override_send(
                self.target_system, self.target_component, *([0] * 8))
            print("[haucs] RC overrides released, radio has ch%d back"
                  % self.TRIGGER_CH)
        else:
            print("usage: haucs winch release | fetch | clear")

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
                
        elif args[0] == "winch":
            self.cmd_winch(args[1:])
        elif args[0] == "set_id":
            if len(args) != 2:
                print("set drone id\nexample: haucs set_id SPLASHY_1")
            else:
                self.drone_id = args[1]
        elif args[0] == "gen_mission_winch":
            # NEW WINCH FORMAT: NAV_LAND + DO_SET_SERVO servo sequence
            # *** DIFFERENT FROM gen_mission — read prompts carefully ***
            # Usage: haucs gen_mission_winch <source> <alt> <upload_alt> <stab_sec> <soak_sec>
            # Example: haucs gen_mission_winch points 15 10 3 60
            if len(args) != 6:
                print("gen_mission_winch usage:")
                print("  haucs gen_mission_winch <source> <alt> <upload_alt> <stab_sec> <soak_sec>")
                print("  source     : CSV file with pond lat/lon (no .csv extension needed)")
                print("  alt        : cruise altitude AGL in metres (e.g. 15)")
                print("  upload_alt : min altitude before data upload trigger (e.g. 10)")
                print("  stab_sec   : stabilization delay after landing in seconds (e.g. 3)")
                print("  soak_sec   : soak time on water in seconds (e.g. 60)")
                print("")
                print("  Example: haucs gen_mission_winch points 15 10 3 60")
                print("")
                print("  *** NEW FORMAT — servo sequence has changed from gen_mission ***")
                print("  *** ch8=1500 neutral, ch8=1900 deploy, ch8=1200 upload      ***")
                print("  *** Set SCR_USER1/2/3 in Mission Planner for winch timing   ***")
                print("  *** soak_sec must be >= SCR_USER1 + SCR_USER2 + SCR_USER3   ***")
                print("  *** Verify in Mission Planner before uploading to drone     ***")
            else:
                source = args[1] if args[1].endswith(".csv") else args[1] + ".csv"
                output = args[1].replace(".csv", "") + "_winch.waypoints"

                if not self.drone_variables.get("lat"):
                    print("!!!TEST MODE!!!: no GPS fix -- using lab coordinates for home")
                    home = (27.535321985800824, -80.35167917904866, 0)
                else:
                    home = (self.drone_variables["lat"],
                            self.drone_variables["lon"],
                            self.drone_variables["alt"])

                mission_args = {
                    "home"      : home,
                    "source"    : source,
                    "output"    : output,
                    "alt"       : int(args[2]),
                    "upload_alt": int(args[3]),
                    "stab_sec"  : float(args[4]),
                    "soak_sec"  : float(args[5]),
                    "winch"     : True,
                    # legacy keys needed by path_planner.main() internals
                    "delay"     : float(args[5]),
                    "dive"      : int(args[2]),
                    "land"      : "true",
                }

                for k, v in mission_args.items():
                    if k != "home":
                        print("  {:>12}: {}".format(k, v))

                sorted_coords = path_planner.main(self, mission_args)
                if sorted_coords and self.drone_variables.get("lat"):
                    sorted_coords_full = (
                        [[home[0], home[1]]]
                        + [[c[0], c[1]] for c in sorted_coords]
                        + [[home[0], home[1]]]
                    )
                    try:
                        db.reference("LH_Farm/drone/" + self.drone_id + "/mission/").set(sorted_coords_full)
                        print("Mission uploaded to Firebase: LH_Farm/drone/" + self.drone_id + "/mission/")
                    except Exception as e:
                        print("Firebase upload failed: {}".format(e))

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

        # --- periodic debug status, so you can see in the MAVProxy console where the
        # chain is (or isn't) working without waiting for an actual sampling event ---
        if (time.time() - self._dbg_last_status_print) > self._dbg_status_period:
            self._dbg_last_status_print = time.time()
            servo_age = (
                "%.1fs ago" % (time.time() - self._dbg_servo_last_t)
                if self._dbg_servo_last_t else "never"
            )
#             self.console.writeln(
#                 "[haucs][status] hooks=%d hb=%d | trigger(%s): count=%d last_pwm=%s (%s), chan=%d state=%s, mode=%s | "
#                 "DATA96: seen=%d processed=%d dropped(armed=0)=%d | gps=(%s,%s)"
#                 % (
#                     self._hooks_attached,
#                     self._dbg_hb_count,
#                     self._dbg_servo_last_src or "none yet",
#                     self._dbg_servo_msg_count,
#                     self._dbg_servo_last_pwm,
#                     servo_age,
#                     self.servo_mon["chan"],
#                     self.servo_mon["state"],
#                     self.drone_variables.get("flight_mode", "unknown"),
#                     self._dbg_data96_seen,
#                     self._dbg_data96_processed,
#                     self._dbg_data96_dropped,
#                     self.drone_variables.get("lat"),
#                     self.drone_variables.get("lon"),
#                 )
#             )
            
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
                self._dbg_hb_count += 1
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
                    self.console.writeln("INIT_TAKEOFF") 
                elif evt == "TAKEOFF":
                    self.console.writeln("SAMPLING_TAKEOFF") 
                
                elif evt == "TOUCHDOWN_SAMPLING":
                    self.console.writeln("Touchdown (sampling)")
                    
                elif evt == "TOUCHDOWN_FINAL":
                    self.console.writeln("Touchdown (FINAL)")

            elif m.get_type() == 'RC_CHANNELS':
                # RC8 raw pilot input -- this is the trigger source in every flight mode
                # EXCEPT AUTO, where SERVO_OUTPUT_RAW (below) is used instead. Mirrors
                # the Pi's own mv_state["auto_mode"] source switching. Without this,
                # servo_mon only ever watched SERVO_OUTPUT_RAW, which stays at 0 in
                # manual flight unless that specific output channel is separately
                # configured to pass RC8 through -- which is why testing in manual mode
                # showed last_pwm=0 forever even while flipping the real switch.
                if self.drone_variables.get("flight_mode") != "Auto":
                    ch = int(self.servo_mon["chan"])
                    pwm = getattr(m, f"chan{ch}_raw", None)
                    if pwm is not None:
                        self._handle_trigger_pwm(pwm, "RC_CHANNELS")

            elif m.get_type() == 'SERVO_OUTPUT_RAW':
                # Only authoritative in AUTO mode (mission DO_SET_SERVO commands show up
                # here). Map channel to MAVLink port + index (8 channels per port).
                if self.drone_variables.get("flight_mode") == "Auto":
                    ch = int(self.servo_mon["chan"])
                    port_needed = (ch - 1) // 8     # ch 1..8 -> 0, 9..16 -> 1
                    idx = (ch - 1) % 8 + 1          # 1..8
                    if getattr(m, "port", 0) != port_needed:
                        return
                    pwm = getattr(m, f"servo{idx}_raw", None)
                    if pwm is not None:
                        self._handle_trigger_pwm(pwm, "SERVO_OUTPUT_RAW")

            # handles data packets when the servo is engaged.
            elif m.get_type() == "DATA96":
                self._dbg_data96_seen += 1
                gate_open = (self.servo_mon["state"] == 1) or (time.time() < self._data96_armed_until)
                if gate_open:
                    self._dbg_data96_processed += 1
                    self.proc_sensordata(m)
                else:
                    self._dbg_data96_dropped += 1
                    # Log the first few and then only occasionally, so a burst of
                    # dropped packets doesn't flood the console.
                    if self._dbg_data96_dropped <= 5 or self._dbg_data96_dropped % 20 == 0:
                        self.console.writeln(
                            "[haucs] DATA96 received but DROPPED: gate closed "
                            "(state=0, grace expired, chan=%d) -- total dropped=%d"
                            % (self.servo_mon["chan"], self._dbg_data96_dropped)
                        )
                        # 082426: night operators cannot watch the MAVProxy
                        # console. Data arriving and being discarded is the
                        # failure worth seeing live, so raise it on the GCS.
                        self.gcs_status(
                            "DATA RX but GATED ch%d (x%d)"
                            % (self.servo_mon["chan"], self._dbg_data96_dropped),
                            force=True, sev=mavutil.mavlink.MAV_SEVERITY_WARNING,
                            repeat=4)   # 082426: repaint so it can be read

        except Exception as e:
                err = traceback.format_exc(limit=1)  # 1 = just the last frame
                self.console.writeln(f"[haucs] handler error: {err.strip()}")
                # never let one bad packet kill the whole dispatcher
                #self.console.writeln(f"[haucs] mavlink handler error: {type(e).__name__}: {e}")
    def _handle_trigger_pwm(self, pwm, src):
        '''Shared rising/falling-edge detection for the trigger channel, fed by
        either RC_CHANNELS (manual modes) or SERVO_OUTPUT_RAW (AUTO mode) depending on
        current flight mode -- see mavlink_packet() above.'''
        self._dbg_servo_msg_count += 1
        self._dbg_servo_last_pwm = pwm
        self._dbg_servo_last_t = time.time()
        self._dbg_servo_last_src = src

        prev = self.servo_mon["state"]
        on_th, off_th = self.servo_mon["on_th"], self.servo_mon["off_th"]

        if prev == 0 and pwm >= on_th:
            self.servo_mon["state"] = 1
            mode = self.drone_variables.get("flight_mode", "unknown")
            self.console.writeln(f"Servo Rising Edge (ready for sampling), src={src}, pwm={pwm}, flight_mode={mode}")
            # .get() instead of [...]: drone_variables has no "lat"/"lon" key until the
            # first GLOBAL_POSITION_INT arrives, so an edge before that would otherwise
            # raise KeyError here and silently abort the GPS lock for this sample.
            self.sampling_lat = self.drone_variables.get('lat')
            self.sampling_lng = self.drone_variables.get('lon')
            if self.sampling_lat is None or self.sampling_lng is None:
                self.console.writeln("WARNING: no GPS fix yet -- sampling_lat/lng will be None for this drop")
            self.pond_data['pond_id'] = self.get_pond_id()
            self.console.writeln(f"Locked GPS: {self.sampling_lat}, {self.sampling_lng} for pond: {self.pond_data['pond_id']}")
        elif prev == 1 and pwm <= off_th:
            self.servo_mon["state"] = 0
            self._data96_armed_until = time.time() + self._data96_grace_sec
            mode = self.drone_variables.get("flight_mode", "unknown")
            self.console.writeln(
                f"Servo Falling Edge (sampling done), src={src}, pwm={pwm}, flight_mode={mode} "
                f"-- DATA96 gate stays open for {self._data96_grace_sec:.0f}s more"
            )

    def _reset_for_new_seq(self, new_seq):
        self._frame_seq = new_seq
        self._frame_done.clear()
        for k in self._sensor_data_values:
            self._sensor_data_values[k] = []
        for k in self._sensor_chunks:
            self._sensor_chunks[k] = {}
        self._time_chunks = {}
        self._time_buf = []

    def proc_sensordata(self, m):
        try:
            payload = bytes(m.data)[:m.len]
            seq_id, is_resend, var_id, var_len, values, flags, chunk_idx = msg_decoder(payload)
            name = sensor_data_names.get(var_id)
            #print(f"seq_id:{seq_id}, var_id:{var_id}, var_len:{var_len}, values:{values}, name:{name}")
            if self._frame_seq is None:
                # First packet after startup or after a committed frame
                self._reset_for_new_seq(seq_id)
                # 081426: tell the operator the ground station started hearing
                # DATA96 for a new cast.
                self.gcs_status("RX cast started (seq %d)" % seq_id, force=True)
            elif seq_id < self._frame_seq:
                # Pi rebooted -- seq_id wrapped back to 0 (or near 0).
                # Discard old buffer contents from previous session.
                self.console.writeln(
                    "[haucs] seq_id went backwards (%d -> %d) -- "
                    "Pi likely rebooted, discarding old frame and resetting"
                    % (self._frame_seq, seq_id))
                self._reset_for_new_seq(seq_id)
            elif seq_id > self._frame_seq + 1000:
                # Large seq jump -- new deployment started before previous frame_end arrived.
                # Commit whatever we have then start fresh.
                self.console.writeln(
                    "[haucs] large seq_id jump (%d -> %d) -- "
                    "forcing commit of incomplete frame before starting new one"
                    % (self._frame_seq, seq_id))
                self._commit_current(self._frame_seq, "seq_jump")
                self._reset_for_new_seq(seq_id)
            # --- special streams ---
            if name == "time":
                # 081326: was self._time_buf = list(values), which discarded every
                # frame but the last. Store by chunk like everything else.
                if values:
                    self._time_chunks[chunk_idx] = list(values)
                return

            # End-of-frame control (Pi sends values=list(b"CONTROL"); we ignore content)
            if (var_id == self._var_id_frame_end) or (name == "frame_end"):
                self.console.writeln(f"[haucs] frame end received (var_id:{var_id}, name:{name}) -- committing upload")
                self._commit_current(seq_id,"frame_end")
                return
            # --- regular variables: accumulate into your existing buffers ---
            if name in self._sensor_chunks:
                if values:
                    # 081326: place, do not extend. Arrival order is not sample order
                    # once a frame is lost, and duplicates (resends) overwrite cleanly.
                    self._sensor_chunks[name][chunk_idx] = list(values)
                # optional: keep EOF tracking for debugging/logging only
                if flags in (FLAG_EOF, FLAG_SOLO):
                    self._frame_done.add(name)
                return

            # Unknown var id/name – ignore safely
            return

        except Exception as e:
            err = traceback.format_exc(limit=1)
            self.console.writeln(f"[haucs] file error: {err.strip()}")
            # try to recover FB; don't assume sensor_file exists here
            try:
                self.fb_app = restart_firebase(self.fb_app, self.fb_key)
            except Exception:
                pass
            self.console.writeln(f"uploading data to firebase failed: {e}")

    def _assemble(self, name, var_id):
        """
        081326: rebuild one variable from its received chunks, placing each at
        its true sample offset. Missing frames become None, so index i always
        means sample i across every variable. Returns (values, n_missing).
        """
        chunks = self._sensor_chunks.get(name, {}) if name != "time" else self._time_chunks
        if not chunks:
            return [], 0
        width = max_samples(var_id)
        last = max(chunks)
        out, missing = [], 0
        for c in range(last + 1):
            block = chunks.get(c)
            if block is None:
                # a full frame never arrived; only the final chunk may be short,
                # so a missing interior chunk is always exactly `width` samples
                out.extend([None] * width)
                missing += width
            else:
                out.extend(block)
        return out, missing

    def _status_emit(self, text, sev):
        """082426: the actual STATUSTEXT write, split out of gcs_status() so
        the repeat worker can reuse it without re-running the console line or
        the rate limiter."""
        payload = ("HAUCS-GCS: " + text)[:49].encode("ascii", "replace")
        outs = getattr(self.mpstate, "mav_outputs", [])
        if not outs:
            # Nothing is listening. Mission Planner attaches to a MAVProxy
            # --out link; without one these messages exist only in the
            # MAVProxy console, which is easy to miss.
            if not getattr(self, "_warned_no_out", False):
                self._warned_no_out = True
                self.console.writeln(
                    "[haucs] no --out link, so GCS status will not reach "
                    "Mission Planner (console only)")
            return
        try:
            for out in outs:
                # 081426: stamp as the vehicle, not as sysid 255. Mission
                # Planner shows STATUSTEXT for the vehicle it is tracking, so
                # a message from the default GCS sysid can land outside the
                # pane the operator is watching. Forwarding uses raw
                # r.write(msgbuf) and never touches out.mav, so changing this
                # affects only the messages generated here. Saved and
                # restored anyway.
                old_sys = out.mav.srcSystem
                old_comp = out.mav.srcComponent
                try:
                    out.mav.srcSystem = self.target_system or 1
                    # 082426: component 1 was tried so these would land on the
                    # Mission Planner HUD. It does reach the HUD, but only as a
                    # sub-100ms flash at every severity, so the HUD is not a
                    # usable operator channel. Reverted to the honest identity;
                    # the Messages tab is where operators read these.
                    out.mav.srcComponent = \
                        mavutil.mavlink.MAV_COMP_ID_ONBOARD_COMPUTER
                    out.mav.statustext_send(
                        sev if sev is not None
                        else mavutil.mavlink.MAV_SEVERITY_INFO, payload)
                finally:
                    out.mav.srcSystem = old_sys
                    out.mav.srcComponent = old_comp
        except Exception as e:
            self.console.writeln("[haucs] statustext to GCS failed: %s" % e)

    def _status_repeat(self, text, sev, count, gap):
        """082426: worker for gcs_status(repeat=N). Console line already
        written by the caller, so emit to the links only."""
        for i in range(count):
            try:
                self._status_emit("%s (%d)" % (text, count - i), sev)
            except Exception as e:
                self.console.writeln("[haucs] repeat status failed: %s" % e)
                return
            if i < count - 1:
                time.sleep(gap)

    def gcs_status(self, text, force=False, sev=None, repeat=1, gap=1.0):
        """
        081426: push a short INFO STATUSTEXT to every connected GCS output.

        self.master.mav sends to the VEHICLE, which is not what we want.
        Mission Planner sits on mpstate.mav_outputs (the --out links), so
        write there. Also mirrored to the MAVProxy console so the message is
        not lost when no output link is configured.

        Severity INFO (6) is below every mission-critical severity (0-4), so
        these can never displace a real warning.

        082426: sev >= WARNING (4) reaches Mission Planner's high priority
        line, but MP repaints it once and moves on - it does not hold it. The
        reason ArduPilot's prearm text appears to persist is that the FC keeps
        resending it. repeat=N does the same thing: N repaints, gap seconds
        apart, so an alert is on screen long enough to be read. A counter
        suffix is appended when repeating in case MP collapses identical
        consecutive text. Sending happens on a daemon thread so the caller,
        which may be inside packet handling, is never blocked.
        """
        self.console.writeln("[haucs] " + text)
        now = time.time()
        if not force and (now - getattr(self, "_status_last", 0.0)) < 1.0:
            return
        self._status_last = now

        # 082426: repaint N times off-thread so the caller is not blocked.
        if repeat > 1:
            threading.Thread(
                target=self._status_repeat,
                args=(text, sev, int(repeat), float(gap)),
                daemon=True).start()
            return
        self._status_emit(text, sev)

    def _commit_current(self, end_seq, reason):
        self.console.writeln(f"[haucs] commit_current: end_seq={end_seq}, last_uploaded_seq={self._last_uploaded_seq}, reason={reason}")
        if self._last_uploaded_seq == end_seq:
            self.console.writeln(f"[haucs] skip upload: seq {end_seq} already uploaded")
            return
        else:
            # assemble payload
            message_time_file = time.strftime('%Y%m%d_%H%M%S', time.gmtime(time.time()))
            message_time = time.strftime('%Y%m%d_%H:%M:%S', time.gmtime(time.time()))

            os.makedirs(SENSORDIR, exist_ok=True)
            sensor_file = os.path.join(SENSORDIR, f'{message_time_file}.json')
            seq = self.pond_data.get('seq', 0)
            drone_id = self.drone_id
            pond_id  = self.pond_data.get('pond_id', 'wukn')

            # 081326: assemble from chunks so every array is index-aligned.
            do_array,   miss_do   = self._assemble('DO', VAR_MAP['DO'])
            temp_array, miss_temp = self._assemble('temp', VAR_MAP['temp'])
            pres_array, miss_pres = self._assemble('pressure', VAR_MAP['pressure'])
            time_array, miss_time = self._assemble('time', VAR_MAP['time'])

            init_DO_list, _       = self._assemble('init_DO', VAR_MAP['init_DO'])
            init_pressure_list, _ = self._assemble('init_pressure', VAR_MAP['init_pressure'])
            batt_v_list, _        = self._assemble('batt_v', VAR_MAP['batt_v'])

            # keep the flat buffers populated for anything else reading them
            self._sensor_data_values['DO']       = do_array
            self._sensor_data_values['temp']     = temp_array
            self._sensor_data_values['pressure'] = pres_array
            self._time_buf = time_array

            n_expect = max(len(do_array), len(temp_array), len(pres_array), len(time_array))
            n_missing = miss_do + miss_temp + miss_pres + miss_time
            complete  = (n_missing == 0 and
                         len(do_array) == len(temp_array) == len(pres_array) == n_expect)
            if not complete:
                self.console.writeln(
                    "[haucs] INCOMPLETE cast: %d of %d sample slots missing "
                    "(do %d, temp %d, pressure %d, time %d). Uploading with nulls "
                    "in the gaps and complete=False."
                    % (n_missing, n_expect * 4, miss_do, miss_temp, miss_pres, miss_time))
                # 082426: the console line above is invisible to a night
                # operator, and lost frames are worth knowing about at once.
                self.gcs_status("CAST GAPS %d/%d slots missing"
                                % (n_missing, n_expect * 4), force=True,
                                sev=mavutil.mavlink.MAV_SEVERITY_WARNING,
                                repeat=4)   # 082426

            # 081326: a lost first chunk would put None at index 0
            def _first(lst, dflt=0.0):
                for v in lst:
                    if v is not None:
                        return v
                return dflt
            init_DO       = _first(init_DO_list)
            init_pressure = _first(init_pressure_list)
            batt_v        = _first(batt_v_list)

            lat = getattr(self, "sampling_lat", None)
            lng = getattr(self, "sampling_lng", None)

            data = {
                'seq': int(seq),
                'sid': drone_id,
                'pid': str(pond_id),
                'lat': lat, 'lng': lng,
                'type': 'winch',
                'do': do_array, 'temp': temp_array, 'pressure': pres_array,
                'init_do': init_DO, 'init_pressure': init_pressure,
                'batt_v': batt_v,
                # 081326: so a consumer can tell a whole cast from a gappy one
                'complete': bool(complete),
                'n_missing': int(n_missing),
            }
            try:
                save_json(data, sensor_file)
                db.reference(f"LH_Farm/pond_{pond_id}/{message_time}/").set(data)
                self.console.writeln(
                    "[haucs] DB UPLOAD OK -> LH_Farm/pond_%s/%s | seq=%s samples=%d lat=%s lng=%s"
                    % (pond_id, message_time, seq, len(do_array), lat, lng)
                )
                # 081426: the confirmation the operator is waiting for.
                self.gcs_status(
                    "DB OK %s %s %ds%s" % (pond_id, message_time[-8:],
                                           len(do_array),
                                           "" if complete else " GAPS"),
                    force=True)
                self.pond_data['seq'] = self.pond_data.get('seq', 0) + 1
                append_json('upload', 1, sensor_file)
            except Exception as e:
                err = traceback.format_exc(limit=1)
                self.console.writeln(f"[haucs] DB UPLOAD FAILED -> LH_Farm/pond_{pond_id}/{message_time}: {err.strip()}")
                self.gcs_status("DB UPLOAD FAILED pond %s" % pond_id,
                                force=True,
                                sev=mavutil.mavlink.MAV_SEVERITY_WARNING,
                                repeat=4)   # 082426
            finally:
                # Finalize the frame regardless of success/failure
                self._last_uploaded_seq = end_seq                 # ignore duplicate end markers for this seq
                self.pond_data["seq"] = self.pond_data.get("seq", 0) + 1
                self._reset_for_new_seq(None)     # clear buffers & ready for next frame
                print(f"updating seq:{seq}, self._last_uploaded_seq:{self._last_uploaded_seq}")
                print(f"[haucs] committed seq={end_seq} reason={reason}")

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
            #print(f"idle_update:drone:{self.drone_id},{data}")
        except Exception as e:
            err = traceback.format_exc(limit=1)
            print(f"[haucs] db idle upload error: {err.strip()}")
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

