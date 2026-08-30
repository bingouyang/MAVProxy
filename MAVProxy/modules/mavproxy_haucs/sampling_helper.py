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

SEND_ORDER = ["time", "DO", "temp", "pressure", "init_DO", "init_pressure", "batt_v"]
VAR_MAP    = {"time": 0, "DO": 1, "temp": 2, "pressure": 3, "init_DO": 4, "init_pressure": 5, "batt_v": 6}

# ---- Residue coding, MUST match the other side exactly -------------------
#
# Each frame carries: var_base (int16) + N residues, where
#     residue = round((value - var_base) * SCALE_MAP[var_id])
# and var_base = round(mean of the chunk).
#
# Two things bound the scale:
#   1. the residue must fit its integer width
#   2. var_base is rounded to an integer, which alone costs up to 0.5
#
# Worst-case |value - var_base| measured over 8 real casts (BP2, Aug 2026):
#     DO        0.41 ratio      temp   2.07 C
#     pressure  64.1 hPa        batt   0.22 V
# Scales below keep at least 2x margin on those figures.
#
# Pressure is the exception: 64 hPa of spread against an int8 limit of 127
# caps it at scale 1, i.e. 1 hPa = 0.40 in of depth. That is adequate at a
# fast ascent but becomes coarser than the sample spacing once the winch is
# slowed to resolve near-bottom structure. It therefore uses int16 residues,
# which costs one extra frame per cast and lifts the safe scale to ~255.

SCALE = 32          # fallback for any var_id not listed

SCALE_MAP = {
    0: 1,      # time           sample index, integer
    1: 100,    # DO             0.01 ratio, matches the sensor's own 2 dp
    2: 8,      # temp           0.125 C, +/-15.9 C from the chunk mean.
               # 083026: was 32 (+/-3.97 C), sized on the 2.07 C worst case
               # from the BP2 casts. A stratified pond can span 10-15 C in one
               # cast, which clips at 32 - and build_frames only reports that
               # with print(), so it would not appear in the log. 0.125 C is
               # still finer than the probe's thermistor resolves.
    3: 100,    # pressure       0.01 hPa = 0.004 in   (int16, see WIDTH_MAP)
    4: 32,     # init_DO        constant per cast, base captures it
    5: 100,    # init_pressure  0.01 hPa; scale 1 was discarding the decimals
    6: 100,    # batt_v         0.01 V
}

# Residue width in bytes. Anything not listed is 1 (int8).
WIDTH_MAP = {
    3: 2,      # pressure needs int16
}

def residue_width(var_id):
    return WIDTH_MAP.get(int(var_id), 1)

def residue_fmt(var_id):
    return "h" if residue_width(var_id) == 2 else "b"

def residue_limits(var_id):
    return (-32768, 32767) if residue_width(var_id) == 2 else (-128, 127)

# 081326: header carries chunk_idx now. HDR_LEN and the unpack format below
# MUST match encoder_helper.py exactly - they are the wire contract.
DATA_BYTES = 96
HDR_LEN    = 9      # seq_id(4) varbyte(1) base(int16,2) len(1) chunk_idx(1)

def max_samples(var_id):
    return (DATA_BYTES - HDR_LEN) // residue_width(var_id)

# ---- Wire contract version ------------------------------------------------
# 083026: the encoder and decoder each look SCALE_MAP up locally; nothing about
# the coding is carried in the frame. A one-sided deploy therefore decodes to
# plausible wrong numbers with no error anywhere. This makes the contract
# explicit so both sides log it and a mismatch is visible after the fact.
#
# BUMP WIRE_VERSION whenever any of these change:
#   SCALE_MAP, WIDTH_MAP, SEND_ORDER, HDR_LEN / HDR_FMT, DATA_BYTES
# and deploy encoder_helper.py (Pi) and sampling_helper.py (GCS) together.
#
#   1  083026  9-byte header with chunk_idx; temp scale 8, pressure int16/100
#
# contract_id() also hashes the actual values, so a forgotten bump still shows
# as a differing hash even when the version numbers agree.
WIRE_VERSION = 1


def contract_id():
    import hashlib, json
    c = {"SCALE": SCALE, "SCALE_MAP": SCALE_MAP, "WIDTH_MAP": WIDTH_MAP,
         "HDR_LEN": HDR_LEN, "DATA_BYTES": DATA_BYTES, "SEND_ORDER": SEND_ORDER}
    h = hashlib.sha256(json.dumps(c, sort_keys=True).encode()).hexdigest()[:8]
    return "v%d/%s" % (WIRE_VERSION, h)

# -------------------- Msg Decoder --------------------
def msg_decoder(buf):
    seq_id = struct.unpack_from("!I", buf, 0)[0]
    var_byte = buf[4]
    var_base = struct.unpack_from("!h", buf, 5)[0]
    varlen_raw = buf[7]
    chunk_idx = buf[8]     # 081326: which block of samples this frame holds
    flags = 0              # encoder never sets flag bits; kept for caller compatibility
    var_len = varlen_raw & 0xFF
    is_resend = 1 if (var_byte & 0x80) else 0
    var_id = var_byte & 0x7F
    # Residue width is per-variable; pressure is int16. Must match the encoder.
    fmt = residue_fmt(var_id)
    residues = list(struct.unpack_from("!" + fmt*var_len, buf, HDR_LEN))
    scale = SCALE_MAP.get(var_id, SCALE)
    values = [var_base + r / scale for r in residues]
    # 081326: chunk_idx appended LAST so existing 6-tuple unpacking still works.
    return seq_id, is_resend, var_id, var_len, values, flags, chunk_idx
