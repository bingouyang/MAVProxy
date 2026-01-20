import struct
import time
from pymavlink import mavutil

# define KEYWORDS
NAV_TAKEOFF   = mavutil.mavlink.MAV_CMD_NAV_TAKEOFF
NAV_WP        = mavutil.mavlink.MAV_CMD_NAV_WAYPOINT
NAV_LAND      = mavutil.mavlink.MAV_CMD_NAV_LAND

NAV_IN_AIR    = mavutil.mavlink.MAV_LANDED_STATE_IN_AIR
NAV_ON_GROUND = mavutil.mavlink.MAV_LANDED_STATE_ON_GROUND

# -------------------- Mission utils --------------------
def detect_mission_complete(statustext, st):
    """
    If a STATUSTEXT contains 'mission complete',
    mark that the *next* touchdown is FINAL.
    st: dict you keep, e.g. {'final_pending': False}
    """
    if statustext and "mission complete" in statustext.lower():
        st['final_pending'] = True

def handle_extsys_with_final(landed_state, st):
    """
    Use only EXTENDED_SYS_STATE for events; 'mission complete' just sets a flag.
    st: dict you persist, initialize once:
        {
          'last': None,
          'seen_init': False,
          'touch_t0': None,
          'touch_confirmed': False,
          'final_pending': False
        }
    Returns one of: 'INIT_TAKEOFF', 'TAKEOFF',
                    'TOUCHDOWN_SAMPLING', 'TOUCHDOWN_FINAL', or None
    """
    now  = time.time()
    last = st.get('last')
    evt  = None

    # Takeoff edge
    if landed_state == NAV_IN_AIR and last != NAV_IN_AIR:
        evt = "INIT_TAKEOFF" if not st.get('seen_init', False) else "TAKEOFF"
        st['seen_init'] = True
        st['touch_t0'] = None
        st['touch_confirmed'] = False

    if landed_state == NAV_ON_GROUND:
        if last != NAV_ON_GROUND:
            st['touch_t0'] = now
            st['touch_confirmed'] = False
        else:
            if not st['touch_confirmed'] and st.get('touch_t0') and (now - st['touch_t0']) >= 2.0: # pause for 2 seconds
                st['touch_confirmed'] = True
                if st.get('final_pending', False):
                    evt = "TOUCHDOWN_FINAL"
                    st['final_pending'] = False  # consume the flag
                else:
                    evt = "TOUCHDOWN_SAMPLING" 
    st['last'] = landed_state
    return evt

SEND_ORDER = ["time","do", "temp", "press"] 
VAR_MAP = {"time": 0, "do": 1, "temp": 2, "press": 3}
# -------------------- Msg Decoder --------------------
def parse_inner(buf):
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