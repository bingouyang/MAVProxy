"""
log_parser.py -- offline recovery of winch casts.

Why this exists
---------------
The Pi sends each cast to the ground over DATA96 frames on the shared
115200 UART / telemetry link. Frames get dropped, and a cast can fail to
reach Firebase entirely (link down, retract timeout, wrong pond, lost
FRAME_END). The Pi, however, logs the complete column set BEFORE it
transmits, and separately caches it to cache/sample_*.csv. Both are
lossless. This module reads either one, shows you what is in it, and
uploads only the casts you pick.

Nothing here runs automatically and nothing uploads without an explicit
confirm step.

Two ways to use it
------------------
1. Inside MAVProxy, if you wire the command in (see WIRING below):
       haucs parselog <file> [utc_offset_hours]
       haucs parselog show <n>
       haucs parselog csv <n> <outfile>
       haucs parselog upload <n>        (asks for confirmation)

2. Standalone, no changes to __init__.py at all:
       python -m MAVProxy.modules.mavproxy_haucs.log_parser <file>
       python -m ... log_parser <file> --show 2
       python -m ... log_parser <file> --csv 2 cast2.csv
       python -m ... log_parser <file> --upload 2 --pond BP1 --confirm

WIRING (optional, add to cmd_haucs in __init__.py when you want it)
-------------------------------------------------------------------
    elif args[0] == "parselog":
        from MAVProxy.modules.mavproxy_haucs import log_parser
        log_parser.cmd_parselog(self, args[1:])

That is the only change to __init__.py, and the module works without it.
"""

import ast
import csv
import json
import os
import re
import sys
from datetime import datetime, timedelta

# Column names as the Pi writes them. Must match encoder_helper.SEND_ORDER.
VAR_ORDER = ["time", "DO", "temp", "pressure",
             "init_DO", "init_pressure", "batt_v"]
SAMPLE_VARS = ["DO", "temp", "pressure"]

# Matches HaucsModule.drone_id in __init__.py, and every winch record already
# in Firebase. bt_helper logs the BLE sensor name at DEBUG only, so an INFO
# level log usually has no name to detect; this is the fallback that keeps a
# recovered record identical to a live one.
DEFAULT_SID = "SPLASHY_UNK"

_TS = r"(\d{4}-\d{2}-\d{2} \d{2}:\d{2}:\d{2}),(\d{3})"
RE_COLS = re.compile(_TS + r".*Uploading fetched BLE data: cols:(\{.*\})\s*$")
RE_RELEASE = re.compile(_TS + r".*winch release.*lat=(-?[\d.]+) lon=(-?[\d.]+)")
RE_DSIZE = re.compile(_TS + r".*finish sampling.*sample_size:\['dsize', '(\d+)'\]")
RE_NOSAMP = re.compile(_TS + r".*BLE fetch returned no samples")
# Only present if the Pi ran at DEBUG. Used when available.
RE_SENSOR = re.compile(r"found sensor with UART service (\S+?)[,\s]")


def _ts(datepart, msec):
    return datetime.strptime(datepart, "%Y-%m-%d %H:%M:%S") + \
        timedelta(milliseconds=int(msec))


# ---------------------------------------------------------------- parsing

def parse_log(path):
    """
    Read a Pi companion log and return a list of cast dicts.

    Each cast:
        local_time    datetime, Pi clock, when the payload was assembled
        cols          dict of column name -> list, exactly as the Pi had it
        n             sample count
        releases      list of (datetime, lat, lon) since the previous cast
        dsize         sample count the sensor reported, for cross-check
        source        the file it came from
        warnings      list of strings
    """
    casts = []
    releases = []
    pending_dsize = None
    sid = None

    with open(path, "r", errors="ignore") as fh:
        for line in fh:
            m = RE_SENSOR.search(line)
            if m:
                sid = m.group(1).strip()
                continue

            m = RE_RELEASE.search(line)
            if m:
                releases.append((_ts(m.group(1), m.group(2)),
                                 float(m.group(3)), float(m.group(4))))
                continue

            m = RE_DSIZE.search(line)
            if m:
                pending_dsize = int(m.group(3))
                continue

            m = RE_NOSAMP.search(line)
            if m:
                releases = []
                pending_dsize = None
                continue

            m = RE_COLS.search(line)
            if not m:
                continue

            try:
                cols = ast.literal_eval(m.group(3))
            except Exception as e:
                print("skip: could not parse cols dict at %s: %s"
                      % (m.group(1), e))
                continue

            cast = {
                "local_time": _ts(m.group(1), m.group(2)),
                "cols": cols,
                "n": len(cols.get("DO", [])),
                "releases": list(releases),
                "dsize": pending_dsize,
                "source": os.path.basename(path),
                "sid": sid,
                "warnings": [],
            }
            _validate(cast)
            casts.append(cast)
            releases = []
            pending_dsize = None

    return casts


def parse_cache_csv(path):
    """
    Read one cache/sample_*.csv written by the Pi. Same shape as parse_log,
    one cast per file. This is the better source when you can pull the
    cache directory, because it does not depend on log retention.
    """
    cols = {}
    with open(path, "r", newline="") as fh:
        for row in csv.DictReader(fh):
            for k, v in row.items():
                if v is None or v == "":
                    continue
                try:
                    cols.setdefault(k, []).append(float(v))
                except ValueError:
                    pass

    base = os.path.basename(path)
    m = re.search(r"(\d{8})_(\d{6})", base)
    local = datetime.strptime(m.group(1) + m.group(2), "%Y%m%d%H%M%S") \
        if m else datetime.fromtimestamp(os.path.getmtime(path))

    lat = cols.get("lat", [None])
    lon = cols.get("lon", [None])
    cast = {
        "local_time": local,
        "cols": cols,
        "n": len(cols.get("DO", [])),
        "releases": [(local, lat[0], lon[0])] if lat[0] is not None else [],
        "dsize": None,
        "source": base,
        "sid": None,
        "warnings": [],
    }
    _validate(cast)
    return cast


def parse_source(path):
    """Accept a log file, a single cache CSV, or a directory of cache CSVs."""
    if os.path.isdir(path):
        out = []
        for name in sorted(os.listdir(path)):
            if name.startswith("sample_") and name.endswith(".csv"):
                out.append(parse_cache_csv(os.path.join(path, name)))
        return out
    if path.lower().endswith(".csv"):
        return [parse_cache_csv(path)]
    return parse_log(path)


def _validate(cast):
    cols, w = cast["cols"], cast["warnings"]
    lens = {v: len(cols.get(v, [])) for v in SAMPLE_VARS}
    if len(set(lens.values())) != 1:
        w.append("column lengths disagree: %s" %
                 ", ".join("%s=%d" % (k, v) for k, v in lens.items()))
    if cast["dsize"] is not None and cast["dsize"] != cast["n"]:
        w.append("sensor reported %d samples, payload has %d"
                 % (cast["dsize"], cast["n"]))
    if cast["n"] == 0:
        w.append("no samples")
    if len(cast["releases"]) > 1:
        w.append("%d releases fed this one fetch, so it spans more than one "
                 "dip and one location" % len(cast["releases"]))
    if not cast["releases"]:
        w.append("no release event found, position unknown")
    for name in VAR_ORDER:
        if name not in cols:
            w.append("missing column %s" % name)


# ------------------------------------------------------------- presenting

def utc_key(cast, utc_offset_hours):
    """
    Firebase node name: YYYYMMDD_HH:MM:SS in UTC.

    The log records the Pi's LOCAL clock; Firebase keys are UTC. There is
    deliberately no default offset. Passing None returns a LOCAL_ prefixed
    string that is obviously not a Firebase key, and upload is refused, so
    forgetting the offset can never quietly write a record at the wrong time.
    """
    if utc_offset_hours is None:
        return "LOCAL_" + cast["local_time"].strftime("%Y%m%d_%H:%M:%S")
    return (cast["local_time"] +
            timedelta(hours=utc_offset_hours)).strftime("%Y%m%d_%H:%M:%S")


def offset_banner(utc_offset_hours):
    if utc_offset_hours is None:
        return ("NO UTC OFFSET SET. The log is the Pi's local clock, Firebase "
                "keys are UTC.\n"
                "Keys below are LOCAL_ placeholders and upload is blocked.\n"
                "Re-run with the offset, e.g. 4 if the Pi is on Eastern, "
                "5 if Central.")
    return ("Pi clock %+d h = UTC. Check this against a record you know is "
            "good before uploading." % utc_offset_hours)


def _rng(vals):
    v = [x for x in vals if x is not None]
    return "%.2f to %.2f" % (min(v), max(v)) if v else "n/a"


def summarize(casts, utc_offset_hours, out=print):
    if not casts:
        out("no casts found")
        return
    out("")
    for ln in offset_banner(utc_offset_hours).split("\n"):
        out(ln)
    out("")
    out("  #  pi local time        cast_id (UTC key)     n    lat        lon"
        "         flags")
    for i, c in enumerate(casts):
        lat = lon = None
        if c["releases"]:
            _, lat, lon = c["releases"][0]
        out("  %-2d %s  %s  %4d  %-10s %-11s %s"
            % (i,
               c["local_time"].strftime("%Y-%m-%d %H:%M:%S"),
               utc_key(c, utc_offset_hours),
               c["n"],
               ("%.6f" % lat) if lat is not None else "unknown",
               ("%.6f" % lon) if lon is not None else "unknown",
               ("WARN(%d)" % len(c["warnings"])) if c["warnings"] else "ok"))
    out("")
    out("'haucs parselog show <n>' for detail, 'upload <n>' to send one.")


def detail(cast, utc_offset_hours, out=print):
    cols = cast["cols"]
    out("")
    out("source        %s" % cast["source"])
    out("pi local time %s" % cast["local_time"].strftime("%Y-%m-%d %H:%M:%S"))
    out("firebase key  %s  (UTC)" % utc_key(cast, utc_offset_hours))
    out("samples       %d" % cast["n"])
    for v in SAMPLE_VARS:
        out("  %-13s n=%-4d range %s" % (v, len(cols.get(v, [])),
                                         _rng(cols.get(v, []))))
    for v in ("init_DO", "init_pressure", "batt_v"):
        vals = cols.get(v, [])
        out("  %-13s %s" % (v, ("%.2f" % vals[0]) if vals else "missing"))
    out("releases:")
    for t, la, lo in cast["releases"]:
        out("  %s  lat %.7f lon %.7f" % (t.strftime("%H:%M:%S"), la, lo))
    if cast["warnings"]:
        out("warnings:")
        for w in cast["warnings"]:
            out("  - %s" % w)
    else:
        out("no warnings")


def export_csv(cast, path):
    cols = cast["cols"]
    names = [v for v in VAR_ORDER if v in cols]
    extra = [v for v in ("lat", "lon") if v in cols]
    n = cast["n"]
    with open(path, "w", newline="") as fh:
        wr = csv.writer(fh)
        wr.writerow(names + extra)
        for i in range(n):
            row = []
            for v in names + extra:
                col = cols.get(v, [])
                row.append(col[i] if i < len(col) else "")
            wr.writerow(row)
    return path


CSV_HEAD = ["cast_id", "cast_n", "pi_local", "utc_key",
            "release_lat", "release_lon", "sample"]


def cast_id(cast, utc_offset_hours):
    """
    Stable identifier for a cast: the Firebase node name it would be stored
    under. Changes if you change the UTC offset, which is the point - the id
    you filter on is the id it lands under.
    """
    return utc_key(cast, utc_offset_hours)


def write_all_csv(casts, source_path, utc_offset_hours):
    """
    Write every extracted cast to ONE csv beside the input, with a cast_id
    column so they can be told apart. Uploading stays per-cast and opt-in;
    this is just so the data is on disk and inspectable.

    Returns the path written, or None if there was nothing to write.
    """
    if not casts:
        return None

    base = os.path.abspath(source_path)
    stem = os.path.basename(base)
    if os.path.isdir(base):
        out_dir, stem = base, os.path.basename(base.rstrip(os.sep))
    else:
        out_dir = os.path.dirname(base)
        stem = os.path.splitext(stem)[0]
    out_path = os.path.join(out_dir, stem + "_casts.csv")

    present = []
    for name in VAR_ORDER + ["lat", "lon"]:
        if any(name in c["cols"] for c in casts) and name not in present:
            present.append(name)

    with open(out_path, "w", newline="") as fh:
        wr = csv.writer(fh)
        wr.writerow(CSV_HEAD + present)
        for idx, c in enumerate(casts):
            cid = cast_id(c, utc_offset_hours)
            rlat = rlon = ""
            if c["releases"]:
                _, rlat, rlon = c["releases"][0]
            head = [cid, idx,
                    c["local_time"].strftime("%Y-%m-%d %H:%M:%S"),
                    utc_key(c, utc_offset_hours), rlat, rlon]
            for i in range(c["n"]):
                row = list(head) + [i]
                for name in present:
                    col = c["cols"].get(name, [])
                    row.append(col[i] if i < len(col) else "")
                wr.writerow(row)
    return out_path


# --------------------------------------------------------------- upload

def build_record(cast, drone_id=None, pond_id=None, seq=0,
                 normalize_init_do=True):
    """
    Build the record exactly as _commit_current does, so a recovered cast is
    indistinguishable from a live one.

    normalize_init_do: the sensor already divides DO by its air calibration
    before it leaves the probe, so the DO column is a saturation ratio while
    init_DO is the raw ADC count. firebase_worker.py writes 1 for the truck
    for this reason. Keep True unless you have changed that convention. The
    raw count is preserved as init_do_raw either way.
    """
    cols = cast["cols"]

    def first(name, dflt=0.0):
        vals = cols.get(name, [])
        for v in vals:
            if v is not None:
                return v
        return dflt

    lat = lon = None
    if cast["releases"]:
        _, lat, lon = cast["releases"][0]

    # explicit argument wins, then a name detected in the log, then default
    sid = drone_id or cast.get("sid") or DEFAULT_SID

    raw_init_do = first("init_DO")
    return {
        "seq": int(seq),
        "sid": sid,
        "pid": str(pond_id),
        "lat": lat, "lng": lon,
        "type": "winch",
        "do": list(cols.get("DO", [])),
        "temp": list(cols.get("temp", [])),
        "pressure": list(cols.get("pressure", [])),
        "init_do": 1 if normalize_init_do else raw_init_do,
        "init_do_raw": raw_init_do,
        "init_pressure": first("init_pressure"),
        "batt_v": first("batt_v"),
        "recovered_from": cast["source"],
    }


def pond_for(cast, pond_table):
    """Resolve pond id from the release position, same rule as get_pond_id."""
    if pond_table is None or not cast["releases"]:
        return None
    try:
        from shapely.geometry import Point
    except ImportError:
        return None
    _, lat, lon = cast["releases"][0]
    pt = Point([lon, lat])
    for pid, poly in pond_table.items():
        if poly.contains(pt):
            return str(pid)
    return "wukn"


def upload_record(record, key, confirmed, out=print):
    """
    Write to LH_Farm/pond_<pid>/<key>/. Refuses unless confirmed is True, and
    refuses to overwrite an existing node.
    """
    path = "LH_Farm/pond_%s/%s/" % (record["pid"], key)

    if not confirmed:
        out("DRY RUN, nothing written.")
        out("  would write %d samples to %s" % (len(record["do"]), path))
        out("  pass the confirm flag to actually upload")
        return False

    # imported here, not at the top, so a dry run works on a machine that
    # has no firebase_admin installed
    from firebase_admin import db
    ref = db.reference(path)
    if ref.get() is not None:
        out("REFUSED: %s already exists. Delete it first if you mean to "
            "replace it." % path)
        return False

    ref.set(record)
    out("UPLOADED %d samples to %s" % (len(record["do"]), path))
    return True


# --------------------------------------------- MAVProxy command interface

_STATE = {"casts": [], "offset": None, "path": None}


def cmd_parselog(module, args):
    """Handler for 'haucs parselog ...'. module is the HaucsModule instance."""
    out = module.console.writeln if hasattr(module, "console") else print

    if not args:
        out("usage: haucs parselog <file|dir> [utc_offset_hours]")
        out("       haucs parselog show <n>")
        out("       haucs parselog csv <n> <outfile>")
        out("       haucs parselog upload <n> [confirm]")
        return

    sub = args[0]

    if sub in ("show", "csv", "upload"):
        if not _STATE["casts"]:
            out("nothing parsed yet, run: haucs parselog <file>")
            return
        try:
            idx = int(args[1])
            cast = _STATE["casts"][idx]
        except (IndexError, ValueError):
            out("give a cast number from the list")
            return

        if sub == "show":
            detail(cast, _STATE["offset"], out)
            return

        if sub == "csv":
            if len(args) < 3:
                out("usage: haucs parselog csv <n> <outfile>")
                return
            out("wrote %s" % export_csv(cast, args[2]))
            return

        if _STATE["offset"] is None:
            out("refusing to upload: no UTC offset set. Re-run "
                "'haucs parselog <file> <offset>' first.")
            return
        pond = pond_for(cast, getattr(module, "pond_table", None))
        if pond is None:
            out("could not resolve pond from position; no pond table or no "
                "release event in the log")
            return
        rec = build_record(cast, getattr(module, "drone_id", None), pond)
        key = utc_key(cast, _STATE["offset"])
        confirmed = len(args) > 2 and args[2] == "confirm"
        if cast["warnings"] and confirmed:
            out("this cast has warnings, run 'show %d' first" % idx)
            for w in cast["warnings"]:
                out("  - %s" % w)
            return
        upload_record(rec, key, confirmed, out)
        return

    path = sub
    if not os.path.exists(path):
        out("no such file: %s" % path)
        return
    _STATE["offset"] = int(args[1]) if len(args) > 1 else None
    _STATE["path"] = path
    _STATE["casts"] = parse_source(path)
    summarize(_STATE["casts"], _STATE["offset"], out)
    written = write_all_csv(_STATE["casts"], path, _STATE["offset"])
    if written:
        out("all %d casts written to %s" % (len(_STATE["casts"]), written))


# ------------------------------------------------------------ standalone

def main(argv=None):
    argv = list(sys.argv[1:] if argv is None else argv)
    if not argv:
        print(__doc__)
        return 1

    path = argv[0]
    opts = argv[1:]

    def opt(name, default=None):
        return opts[opts.index(name) + 1] if name in opts else default

    raw_off = opt("--utc-offset")
    offset = int(raw_off) if raw_off is not None else None
    casts = parse_source(path)

    # always keep everything extracted on disk, whatever the subcommand is
    written = write_all_csv(casts, path, offset)
    if written:
        print("all %d casts written to %s" % (len(casts), written))

    if "--show" in opts:
        detail(casts[int(opt("--show"))], offset)
        return 0
    if "--csv" in opts:
        i = opts.index("--csv")
        print("wrote %s" % export_csv(casts[int(opts[i + 1])], opts[i + 2]))
        return 0
    if "--upload" in opts:
        cast = casts[int(opt("--upload"))]
        if offset is None:
            print("refusing to upload: pass --utc-offset. The log is local "
                  "time, Firebase keys are UTC.")
            return 1
        pond = opt("--pond")
        if pond is None:
            print("give --pond <id>; standalone mode has no pond table")
            return 1
        rec = build_record(cast, opt("--sid"), pond)
        if cast["warnings"]:
            print("warnings on this cast:")
            for w in cast["warnings"]:
                print("  - %s" % w)
        if "--confirm" in opts:
            import firebase_admin
            from firebase_admin import credentials
            key = opt("--key", "fb_key.json")
            if not firebase_admin._apps:
                firebase_admin.initialize_app(
                    credentials.Certificate(key),
                    {"databaseURL": "https://haucs-monitoring-default-rtdb."
                                    "firebaseio.com"})
        upload_record(rec, utc_key(cast, offset), "--confirm" in opts)
        return 0

    summarize(casts, offset)
    return 0


if __name__ == "__main__":
    sys.exit(main())
