# MAVProxy/modules/mavproxy_linktap.py
from MAVProxy.modules.lib import mp_module
import time, traceback

class LinkTap(mp_module.MPModule):
    def __init__(self, mpstate):
        super().__init__(mpstate, "linktap")
        self.console.writeln("[linktap] loaded; attaching hooks as masters come up")
        self._hooked_ids = set()
        self.attached = 0

        # simple counters (optional)
        self.tot = self.hb = self.cs = self.d64 = self.d96 = self.other = 0
        self._last_summary = time.time()

        # expose a tiny command
        self.add_command("linktap", self.cmd_info, "show totals")

    # ---- hook wiring ---------------------------------------------------------
    def _masters(self):
        mm = getattr(self.mpstate, "mav_master", None)
        if mm is None: return []
        if isinstance(mm, (list, tuple)): return [m for m in mm if m]
        return [mm]

    def _try_attach(self):
        new = 0
        for m in self._masters():
            if hasattr(m, "message_hooks") and id(m) not in self._hooked_ids:
                m.message_hooks.append(self._raw_hook)  # signature: (master, msg)
                self._hooked_ids.add(id(m)); new += 1
        if new:
            self.attached += new
            self.console.writeln(f"[linktap] raw hook attached on {new} master(s) (total {self.attached})")

    def idle_task(self):
        if self.attached == 0:
            self._try_attach()
        # once/sec summary so you know we're alive
        now = time.time()
        if now - self._last_summary >= 1.0:
            self._last_summary = now
            #self.console.writeln(
            #    f"[linktap] 1s totals: HB={self.hb} SYS={self.cs} D64={self.d64} D96={self.d96} other={self.other}"
            #)

    # ---- raw ingress -> forward to mavlink_packet() --------------------------
    def _raw_hook(self, master, msg):
        # 1) lightweight type counting (optional)
        t = msg.get_type(); self.tot += 1
        if   t == "HEARTBEAT": self.hb += 1
        elif t == "SYS_STATUS": self.cs += 1
        elif t == "DATA64": self.d64 += 1
        elif t == "DATA96": self.d96 += 1
        else: self.other += 1

        # 2) forward to the canonical handler that you will extend
        try:
            self.mavlink_packet(msg)
        except Exception:
            # never let an exception kill dispatch; surface it once
            err = traceback.format_exc(limit=2).strip().replace("\n", " | ")
            self.console.writeln(f"[linktap] mavlink_packet error: {err}")

    # ---- your canonical handler (now guaranteed to be called) ----------------
    def mavlink_packet(self, m):
        """Put all your future logic here; called for EVERY message via the raw hook."""
        t = m.get_type()

        # example: always show DATA96 and SYS_STATUS immediately
        if t == "DATA96":
            try:
                first8 = bytes(bytearray(getattr(m, "data", b"")))[:8].hex()
                ln = getattr(m, "len", None)
                self.console.writeln(f"[linktap] DATA96 len={ln} first8={first8}")
            except Exception:
                self.console.writeln("[linktap] DATA96")
            return

        if t == "SYS_STATUS":
            try:
                v = f"{m.voltage_battery/1000:.2f}V"
                i = f"{m.current_battery/100:.2f}A" if m.current_battery != -1 else "n/a"
                soc = f"{m.battery_remaining}%"
                load = f"{m.load/10:.1f}%"
                #self.console.writeln(f"[linktap] SYS_STATUS v={v} i={i} soc={soc} load={load}")
            except Exception:
                self.console.writeln("[linktap] SYS_STATUS")
            return

        # (optional) show first heartbeat to prove flow
        if t == "HEARTBEAT" and self.hb == 1:
            test = 1
            #self.console.writeln("[linktap] HEARTBEAT (first)")

    # ---- command -------------------------------------------------------------
    def cmd_info(self, args):
        test = 2
        #self.console.writeln(
        #    f"[linktap] totals: HB={self.hb} SYS={self.cs} D64={self.d64} D96={self.d96} other={self.other}"
        #)

def init(mpstate):
    return LinkTap(mpstate)
