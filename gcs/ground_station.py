import tkinter as tk
from tkinter import ttk, messagebox
import threading
import time
import os
from datetime import datetime

# Try importing pymavlink
try:
    from pymavlink import mavutil
    MAVLINK_AVAILABLE = True
except ImportError:
    MAVLINK_AVAILABLE = False

# ══════════════════════════════════════════════════════════════════
# CONFIGURATION — change these to match your setup
# ══════════════════════════════════════════════════════════════════
MAVLINK_HOST = "127.0.0.1"
MAVLINK_PORT = 14551
SAVE_FILE    = "victim_location.txt"
# ══════════════════════════════════════════════════════════════════


class GCSApp:
    def __init__(self, root):
        self.root = root
        self.root.title("Ground Control Station — Rescue Coordinator")
        self.root.geometry("900x680")
        self.root.configure(bg="#0a0f1e")
        self.root.resizable(False, False)

        # State
        self.connected        = False
        self.detected         = False
        self.master           = None
        self.monitor_thread   = None
        self.telemetry_thread = None

        self.lat = tk.StringVar(value="----.-------")
        self.lon = tk.StringVar(value="----.-------")
        self.alt = tk.StringVar(value="-- m")
        self.mode_var    = tk.StringVar(value="UNKNOWN")
        self.battery_var = tk.StringVar(value="-- V")
        self.altitude_var= tk.StringVar(value="-- m")
        self.sats_var    = tk.StringVar(value="--")
        self.armed_var   = tk.StringVar(value="NO")
        self.conn_var    = tk.StringVar(value="DISCONNECTED")
        self.alert_title = tk.StringVar(value="WAITING FOR HUMAN DETECTION")
        self.alert_sub   = tk.StringVar(
            value="Connect to MAVLink Mirror to begin monitoring..."
        )

        self.captured_lat = None
        self.captured_lon = None
        self.captured_alt = None

        # Track current mission item seq
        self._current_seq   = -1
        self._mission_items = {}

        # Buffer latest GPS so we can capture immediately on mode change
        self._latest_lat = None
        self._latest_lon = None
        self._latest_alt = None

        # Track previous flight mode to detect AUTO → LOITER transition
        self._prev_mode = None

        self._build_ui()

    # ── UI BUILD ──────────────────────────────────────────────────
    def _build_ui(self):
        PAD = 10

        # ── Header ───────────────────────────────────────────────
        hdr = tk.Frame(self.root, bg="#0d1528",
                       highlightbackground="#1e3a5f",
                       highlightthickness=1)
        hdr.pack(fill="x", padx=PAD, pady=(PAD, 0))
        tk.Label(hdr,
                 text="GROUND CONTROL STATION — RESCUE COORDINATOR",
                 bg="#0d1528", fg="#4fc3f7",
                 font=("Segoe UI", 13, "bold")).pack(pady=(8, 2))
        tk.Label(hdr,
                 text="AERIAL SEARCH AND RESCUE DRONE — VICTIM LOCATION SYSTEM",
                 bg="#0d1528", fg="#607d8b",
                 font=("Segoe UI", 8)).pack(pady=(0, 8))

        # ── Alert Banner ─────────────────────────────────────────
        self.alert_frame = tk.Frame(self.root, bg="#0d1528",
                                    highlightbackground="#1e3a5f",
                                    highlightthickness=1)
        self.alert_frame.pack(fill="x", padx=PAD, pady=(8, 0))

        self.alert_icon_lbl = tk.Label(self.alert_frame, text="📡",
                                       bg="#0d1528", font=("Segoe UI", 22))
        self.alert_icon_lbl.pack(side="left", padx=12, pady=8)

        alert_txt = tk.Frame(self.alert_frame, bg="#0d1528")
        alert_txt.pack(side="left", pady=8)
        self.alert_title_lbl = tk.Label(alert_txt,
                                        textvariable=self.alert_title,
                                        bg="#0d1528", fg="#4fc3f7",
                                        font=("Segoe UI", 11, "bold"))
        self.alert_title_lbl.pack(anchor="w")
        self.alert_sub_lbl = tk.Label(alert_txt,
                                      textvariable=self.alert_sub,
                                      bg="#0d1528", fg="#607d8b",
                                      font=("Segoe UI", 9))
        self.alert_sub_lbl.pack(anchor="w")

        # ── Main Grid ────────────────────────────────────────────
        grid = tk.Frame(self.root, bg="#0a0f1e")
        grid.pack(fill="both", expand=True, padx=PAD, pady=8)

        left = tk.Frame(grid, bg="#0a0f1e")
        left.pack(side="left", fill="both", expand=True, padx=(0, 6))

        right = tk.Frame(grid, bg="#0a0f1e")
        right.pack(side="left", fill="both", expand=True)

        self._build_status_card(left)
        self._build_log_card(left)
        self._build_coords_card(right)
        self._build_buttons()

    def _card(self, parent, title):
        frame = tk.Frame(parent, bg="#0d1528",
                         highlightbackground="#1e3a5f",
                         highlightthickness=1)
        frame.pack(fill="both", expand=True, pady=(0, 8))
        tk.Label(frame, text=title.upper(),
                 bg="#0d1528", fg="#607d8b",
                 font=("Segoe UI", 8, "bold")).pack(
            anchor="w", padx=12, pady=(8, 0))
        sep = tk.Frame(frame, bg="#1e3a5f", height=1)
        sep.pack(fill="x", padx=12, pady=(4, 8))
        return frame

    def _status_row(self, parent, label, var, color="#ffffff"):
        row = tk.Frame(parent, bg="#0d1528")
        row.pack(fill="x", padx=12, pady=2)
        tk.Label(row, text=label, bg="#0d1528", fg="#90a4ae",
                 font=("Segoe UI", 10), width=16, anchor="w").pack(side="left")
        lbl = tk.Label(row, textvariable=var, bg="#0d1528", fg=color,
                       font=("Segoe UI", 10, "bold"), anchor="w")
        lbl.pack(side="left")
        return lbl

    def _build_status_card(self, parent):
        card = self._card(parent, "Flight Status")
        self.conn_lbl = self._status_row(
            card, "Connection",     self.conn_var,    "#f44336")
        self.mode_lbl = self._status_row(
            card, "Flight Mode",   self.mode_var,    "#4fc3f7")
        self._status_row(card, "Battery",       self.battery_var, "#ffc107")
        self._status_row(card, "Altitude",      self.altitude_var,"#ffffff")
        self._status_row(card, "GPS Satellites",self.sats_var,    "#4caf50")
        self._status_row(card, "Armed",         self.armed_var,   "#f44336")

    def _build_log_card(self, parent):
        card = self._card(parent, "System Log")
        self.log_text = tk.Text(card, bg="#060c1a", fg="#90a4ae",
                                font=("Consolas", 9),
                                height=10, bd=0,
                                highlightthickness=0,
                                wrap="word", state="disabled")
        self.log_text.pack(fill="both", expand=True, padx=12, pady=(0, 10))

        self.log_text.tag_config("green",  foreground="#4caf50")
        self.log_text.tag_config("blue",   foreground="#4fc3f7")
        self.log_text.tag_config("amber",  foreground="#ffc107")
        self.log_text.tag_config("red",    foreground="#f44336")
        self.log_text.tag_config("time",   foreground="#607d8b")

        self._log("System initialized", "blue")
        self._log("Waiting for connection to MAVLink Mirror...")

    def _gps_box(self, parent, label_text, var):
        box = tk.Frame(parent, bg="#060c1a",
                       highlightbackground="#1e3a5f",
                       highlightthickness=1)
        box.pack(fill="x", padx=12, pady=4)
        tk.Label(box, text=label_text, bg="#060c1a", fg="#607d8b",
                 font=("Segoe UI", 8)).pack(anchor="w", padx=8, pady=(6, 0))
        lbl = tk.Label(box, textvariable=var,
                       bg="#060c1a", fg="#263238",
                       font=("Consolas", 20, "bold"))
        lbl.pack(anchor="w", padx=8, pady=(0, 6))
        return lbl

    def _build_coords_card(self, parent):
        card = self._card(parent, "Victim Location — Captured Coordinates")

        self.lat_lbl = self._gps_box(card, "LATITUDE",       self.lat)
        self.lon_lbl = self._gps_box(card, "LONGITUDE",      self.lon)
        self.alt_lbl = self._gps_box(card, "ALTITUDE (AGL)", self.alt)

        ts_frame = tk.Frame(card, bg="#0d1528")
        ts_frame.pack(fill="x", padx=12, pady=(4, 0))
        tk.Label(ts_frame, text="CAPTURED AT", bg="#0d1528",
                 fg="#607d8b", font=("Segoe UI", 8)).pack(side="left")
        self.time_var = tk.StringVar(value="--")
        tk.Label(ts_frame, textvariable=self.time_var,
                 bg="#0d1528", fg="#ffffff",
                 font=("Segoe UI", 8, "bold")).pack(side="left", padx=6)

        # Trigger badge — shows what caused detection
        trig_frame = tk.Frame(card, bg="#0d1528")
        trig_frame.pack(fill="x", padx=12, pady=(2, 0))
        tk.Label(trig_frame, text="TRIGGER", bg="#0d1528",
                 fg="#607d8b", font=("Segoe UI", 8)).pack(side="left")
        self.seq_var = tk.StringVar(value="--")
        tk.Label(trig_frame, textvariable=self.seq_var,
                 bg="#0d1528", fg="#ffc107",
                 font=("Segoe UI", 8, "bold")).pack(side="left", padx=6)

        self.maps_var = tk.StringVar(value="")
        self.maps_lbl = tk.Label(card, textvariable=self.maps_var,
                                 bg="#0d1528", fg="#4fc3f7",
                                 font=("Segoe UI", 8, "underline"),
                                 cursor="hand2")
        self.maps_lbl.pack(padx=12, pady=(4, 0), anchor="w")
        self.maps_lbl.bind("<Button-1>", self._open_maps)

    def _build_buttons(self):
        btn_frame = tk.Frame(self.root, bg="#0a0f1e")
        btn_frame.pack(fill="x", padx=10, pady=(0, 10))

        def btn(text, cmd, color, tc="#ffffff"):
            b = tk.Button(btn_frame, text=text, command=cmd,
                          bg=color, fg=tc,
                          font=("Segoe UI", 9, "bold"),
                          relief="flat", padx=16, pady=10,
                          cursor="hand2",
                          activebackground=color,
                          activeforeground=tc)
            b.pack(side="left", expand=True, fill="x", padx=4)
            return b

        self.btn_connect  = btn("CONNECT",            self.connect,     "#1565c0")
        self.btn_simulate = btn("SIMULATE DETECTION", self.simulate,    "#1b5e20")
        self.btn_save     = btn("SAVE COORDS",        self.save_coords, "#4a148c")
        self.btn_reset    = btn("RESET",              self.reset,       "#37474f")

        self.btn_save.config(state="disabled")

    # ── LOGGING ───────────────────────────────────────────────────
    def _log(self, msg, color=""):
        def _do():
            self.log_text.config(state="normal")
            ts = datetime.now().strftime("%H:%M:%S")
            self.log_text.insert("end", f"[{ts}] ", "time")
            self.log_text.insert("end", msg + "\n", color if color else "")
            self.log_text.see("end")
            self.log_text.config(state="disabled")
        self.root.after(0, _do)

    # ── CONNECT ───────────────────────────────────────────────────
    def connect(self):
        if self.connected:
            return
        if not MAVLINK_AVAILABLE:
            messagebox.showerror(
                "pymavlink not found",
                "Install pymavlink:\n\npip install pymavlink"
            )
            return

        self.conn_var.set("CONNECTING...")
        self.conn_lbl.config(fg="#ffc107")
        self._log(f"Connecting to MAVLink Mirror {MAVLINK_HOST}:{MAVLINK_PORT}...", "amber")
        self.btn_connect.config(state="disabled")

        t = threading.Thread(target=self._connect_worker, daemon=True)
        t.start()

    def _connect_worker(self):
        try:
            self.master = mavutil.mavlink_connection(
                f"udp:{MAVLINK_HOST}:{MAVLINK_PORT}"
            )
            self.master.wait_heartbeat(timeout=10)
            self.connected = True

            def _ok():
                self.conn_var.set("CONNECTED")
                self.conn_lbl.config(fg="#4caf50")
                self._log("MAVLink Mirror connection established", "green")
                self._log("Watching for AUTO → LOITER mode change...", "blue")
                self._start_monitor()

            self.root.after(0, _ok)

        except Exception as e:
            def _fail():
                self.conn_var.set("FAILED")
                self.conn_lbl.config(fg="#f44336")
                self._log(f"Connection failed: {e}", "red")
                self._log(
                    f"Make sure MAVLink Mirror is enabled in Mission Planner "
                    f"and outputting to port {MAVLINK_PORT}", "amber")
                self.btn_connect.config(state="normal")
            self.root.after(0, _fail)

    # ── MONITOR THREAD ────────────────────────────────────────────
    def _start_monitor(self):
        self.monitor_thread = threading.Thread(
            target=self._monitor_worker, daemon=True)
        self.monitor_thread.start()

    def _monitor_worker(self):
        """
        Listens to the MAVLink stream.

        Detection logic:
        ────────────────
        PRIMARY — HEARTBEAT mode change: AUTO → LOITER
            When the flight mode transitions from AUTO to LOITER,
            victim detection is triggered immediately using the
            latest buffered GPS position.

        GPS is continuously buffered from GLOBAL_POSITION_INT so
        coordinates at the exact moment of mode change are captured
        without any delay.
        """
        while self.connected:
            try:
                msg = self.master.recv_match(blocking=True, timeout=2)
                if msg is None:
                    continue

                mtype = msg.get_type()

                # ── Heartbeat — mode + armed + AUTO→LOITER detection ──
                if mtype == "HEARTBEAT":
                    mode  = mavutil.mode_string_v10(msg)
                    armed = bool(
                        msg.base_mode &
                        mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED
                    )

                    def _hb(m=mode, a=armed):
                        self.mode_var.set(m)
                        self.armed_var.set("YES" if a else "NO")

                        # ── AUTO → LOITER transition check ────────────
                        # Normalise to upper-case for safe comparison
                        prev = (self._prev_mode or "").upper()
                        curr = m.upper()

                        if (prev == "AUTO"
                                and "LOITER" in curr
                                and not self.detected):
                            self._log(
                                f"Mode changed: {self._prev_mode} → {m}",
                                "amber"
                            )
                            self._on_mode_loiter(m)

                        # Update previous mode AFTER the comparison
                        self._prev_mode = m

                    self.root.after(0, _hb)

                # ── GPS position — buffer continuously ────────────────
                elif mtype == "GLOBAL_POSITION_INT":
                    lat = msg.lat / 1e7
                    lon = msg.lon / 1e7
                    alt = msg.relative_alt / 1000

                    # Always buffer latest GPS
                    self._latest_lat = lat
                    self._latest_lon = lon
                    self._latest_alt = alt

                    def _gps(al=alt):
                        self.altitude_var.set(f"{al:.1f} m")
                        # Fallback: if detection fired but GPS not yet captured,
                        # grab it on the very next GPS tick
                        if self.detected and self.captured_lat is None:
                            self._capture_coords(
                                self._latest_lat,
                                self._latest_lon,
                                self._latest_alt
                            )

                    self.root.after(0, _gps)

                # ── SYS_STATUS — battery ──────────────────────────────
                elif mtype == "SYS_STATUS":
                    v = msg.voltage_battery / 1000.0

                    def _bat(vv=v):
                        self.battery_var.set(f"{vv:.2f} V")

                    self.root.after(0, _bat)

                # ── GPS_RAW_INT — satellite count ─────────────────────
                elif mtype == "GPS_RAW_INT":
                    sats = msg.satellites_visible

                    def _sat(s=sats):
                        self.sats_var.set(str(s))

                    self.root.after(0, _sat)

                # ── Cache mission items on download ───────────────────
                elif mtype == "MISSION_ITEM":
                    self._mission_items[msg.seq] = msg.command

                elif mtype == "MISSION_ITEM_INT":
                    self._mission_items[msg.seq] = msg.command

                # ── MISSION_CURRENT — track seq (informational only) ──
                elif mtype == "MISSION_CURRENT":
                    self._current_seq = msg.seq

            except Exception:
                pass

    # ── AUTO → LOITER DETECTED ────────────────────────────────────
    def _on_mode_loiter(self, new_mode):
        """
        Triggered when flight mode transitions from AUTO to LOITER.
        Captures the GPS coordinates buffered at that exact instant.
        """
        if self.detected:
            return   # guard against double-fire

        self.detected = True
        self._log("LOITER mode detected — human found by drone!", "amber")
        self._log("Capturing GPS coordinates at mode-change instant...", "amber")

        # Update alert banner
        self.alert_frame.config(highlightbackground="#4caf50")
        self.alert_icon_lbl.config(text="🎯")
        self.alert_title.set("HUMAN DETECTED — VICTIM LOCATION CAPTURED")
        self.alert_sub.set(
            f"AUTO → {new_mode} transition detected. "
            "Share coordinates with ground bot team immediately."
        )
        self.alert_title_lbl.config(fg="#4caf50")

        # Capture immediately using the GPS buffered at this exact moment
        if (self._latest_lat is not None
                and self._latest_lon is not None
                and self._latest_alt is not None):
            self._capture_coords(
                self._latest_lat,
                self._latest_lon,
                self._latest_alt,
                trigger=f"AUTO → {new_mode}"
            )
        # else: _monitor_worker's GPS tick will capture on next GLOBAL_POSITION_INT

    def _capture_coords(self, lat, lon, alt, trigger=None):
        if self.captured_lat is not None:
            return   # already captured, don't overwrite

        self.captured_lat = lat
        self.captured_lon = lon
        self.captured_alt = alt

        ts = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        self.time_var.set(ts)

        if trigger:
            self.seq_var.set(trigger)
        else:
            self.seq_var.set("AUTO → LOITER")

        self.lat.set(f"{lat:.7f}")
        self.lon.set(f"{lon:.7f}")
        self.alt.set(f"{alt:.1f} m")

        for lbl in [self.lat_lbl, self.lon_lbl, self.alt_lbl]:
            lbl.config(fg="#4caf50")

        maps_url = f"https://maps.google.com/?q={lat:.7f},{lon:.7f}"
        self.maps_var.set(f"Open in Google Maps: {maps_url}")
        self._maps_url = maps_url

        self.btn_save.config(state="normal")

        self._log(
            f"GPS captured — Lat: {lat:.7f}  Lon: {lon:.7f}  Alt: {alt:.1f}m",
            "green"
        )
        self._log("Click SAVE COORDS to export victim_location.txt", "green")

    def _open_maps(self, event=None):
        if hasattr(self, "_maps_url"):
            import webbrowser
            webbrowser.open(self._maps_url)

    # ── SIMULATE (testing without real drone) ────────────────────
    def simulate(self):
        if self.detected:
            return

        self._log("--- SIMULATION MODE ---", "amber")
        self._log("Camera frame analysis: object of interest detected.", "amber")

        def _step2():
            self._log("Monitoring telemetry stream...", "blue")
            self.conn_var.set("CONNECTED (SIM)")
            self.connected = True
            self.mode_var.set("AUTO")
            self.battery_var.set("14.6 V")
            self.altitude_var.set("28.0 m")
            self.sats_var.set("14")
            self.armed_var.set("YES")
            # Seed previous mode as AUTO so transition check works
            self._prev_mode = "AUTO"

        def _step3():
            import random
            # Buffer a realistic GPS position (simulating what drone would report)
            self._latest_lat = 10.048765 + random.uniform(-0.0005, 0.0005)
            self._latest_lon = 76.321890 + random.uniform(-0.0005, 0.0005)
            self._latest_alt = 28.4

            # Simulate the mode switching to LOITER — triggers detection
            self._log("Simulating AUTO → LOITER mode change...", "amber")
            self._on_mode_loiter("LOITER")
            self.mode_var.set("LOITER")

        self.root.after(0,    _step2)
        self.root.after(800,  _step3)

    # ── SAVE ──────────────────────────────────────────────────────
    def save_coords(self):
        if self.captured_lat is None:
            return

        ts      = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        url     = getattr(self, "_maps_url", "N/A")
        trigger = self.seq_var.get()

        content = (
            f"VICTIM LOCATION — AERIAL SEARCH AND RESCUE DRONE\n"
            f"{'='*50}\n"
            f"Trigger    : {trigger}\n"
            f"Latitude   : {self.captured_lat:.7f}\n"
            f"Longitude  : {self.captured_lon:.7f}\n"
            f"Altitude   : {self.captured_alt:.1f} m (AGL)\n"
            f"Captured at: {ts}\n"
            f"Google Maps: {url}\n"
            f"{'='*50}\n"
            f"Send these coordinates to the ground bot team.\n"
        )

        with open(SAVE_FILE, "w") as f:
            f.write(content)

        self._log(f"Saved to {os.path.abspath(SAVE_FILE)}", "green")
        messagebox.showinfo(
            "Saved",
            f"Victim coordinates saved to:\n{os.path.abspath(SAVE_FILE)}"
        )

    # ── RESET ─────────────────────────────────────────────────────
    def reset(self):
        self.connected    = False
        self.detected     = False
        self.captured_lat = None
        self.captured_lon = None
        self.captured_alt = None
        self._current_seq   = -1
        self._mission_items = {}
        self._latest_lat    = None
        self._latest_lon    = None
        self._latest_alt    = None
        self._prev_mode     = None   # reset mode tracking

        if self.master:
            try:
                self.master.close()
            except Exception:
                pass
            self.master = None

        self.lat.set("----.-------")
        self.lon.set("----.-------")
        self.alt.set("-- m")
        self.mode_var.set("UNKNOWN")
        self.battery_var.set("-- V")
        self.altitude_var.set("-- m")
        self.sats_var.set("--")
        self.armed_var.set("NO")
        self.conn_var.set("DISCONNECTED")
        self.time_var.set("--")
        self.seq_var.set("--")
        self.maps_var.set("")
        self.alert_title.set("WAITING FOR HUMAN DETECTION")
        self.alert_sub.set("Connect to MAVLink Mirror to begin monitoring...")
        self.alert_frame.config(highlightbackground="#1e3a5f")
        self.alert_icon_lbl.config(text="📡")
        self.alert_title_lbl.config(fg="#4fc3f7")
        self.conn_lbl.config(fg="#f44336")
        self.btn_connect.config(state="normal")
        self.btn_save.config(state="disabled")

        self.log_text.config(state="normal")
        self.log_text.delete("1.0", "end")
        self.log_text.config(state="disabled")
        self._log("System reset", "blue")
        self._log("Waiting for connection...")


# ── ENTRY POINT ───────────────────────────────────────────────────
if __name__ == "__main__":
    root = tk.Tk()
    app  = GCSApp(root)
    root.mainloop()
