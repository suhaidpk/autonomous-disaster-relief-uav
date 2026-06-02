"""
mission.py  —  Raspberry Pi Human Detection + Drone Mode Controller
════════════════════════════════════════════════════════════════════
Hardware : Raspberry Pi (any model with CSI camera port)
Camera   : Raspberry Pi Camera Module v3
Drone FC : Pixhawk connected via UART

Behaviour
─────────
1. Drone flies an autonomous AUTO mission uploaded via Mission Planner.
2. This script runs on the Raspberry Pi simultaneously.
3. Pi camera continuously captures frames.
4. A lightweight MobileNet SSD (COCO) detects "person" class in real time.
5. On first confirmed human detection:
      a. Save the current mission sequence number (so we can resume).
      b. Command the drone: SET_MODE → LOITER
      c. Wait 5 seconds (hover over the victim).
      d. Command the drone: SET_MODE → AUTO
      e. Jump back to the saved waypoint via MAV_CMD_DO_JUMP (or
         MAV_CMD_DO_SET_MISSION_CURRENT) so the mission continues
         from exactly where it paused.
6. Detection is locked for COOLDOWN_SECONDS after each event so the
   drone isn't interrupted repeatedly by the same person.

Connection
──────────
Default : /dev/serial0  (GPIO UART pins 8/10 on Pi)
Override : set MAVLINK_CONNECTION env-var, e.g.
              MAVLINK_CONNECTION=/dev/ttyUSB0 python3 mission.py
           or for testing over UDP from a PC:
              MAVLINK_CONNECTION=udp:0.0.0.0:14551 python3 mission.py

Model files (download once — see bottom of this file)
──────────
  MobileNetSSD_deploy.prototxt
  MobileNetSSD_deploy.caffemodel

Dependencies
────────────
  pip3 install pymavlink opencv-python-headless numpy
  (picamera2 is pre-installed on Raspberry Pi OS Bullseye+)
"""

import os
import sys
import time
import threading
import logging
from datetime import datetime

import cv2
import numpy as np

# ── pymavlink ─────────────────────────────────────────────────────
try:
    from pymavlink import mavutil
except ImportError:
    sys.exit("[ERROR] pymavlink not found.  Run: pip3 install pymavlink")

# ── picamera2 (preferred on Pi OS Bullseye / Bookworm) ────────────
try:
    from picamera2 import Picamera2
    PICAMERA2 = True
except ImportError:
    PICAMERA2 = False   # fall back to OpenCV VideoCapture (USB cam / legacy)

# ══════════════════════════════════════════════════════════════════
# CONFIGURATION
# ══════════════════════════════════════════════════════════════════
MAVLINK_CONNECTION  = os.getenv("MAVLINK_CONNECTION", "/dev/serial0")
MAVLINK_BAUD        = 57600          # match Mission Planner telemetry baud

# Detection thresholds
CONFIDENCE_THRESH   = 0.55           # MobileNet confidence to count as human
CONFIRM_FRAMES      = 3              # consecutive frames needed before acting
COOLDOWN_SECONDS    = 30             # seconds before another detection is honoured

# Loiter duration
LOITER_SECONDS      = 5

# MobileNet SSD model paths (place next to this script)
PROTOTXT_PATH       = os.path.join(os.path.dirname(__file__),
                                   "MobileNetSSD_deploy.prototxt")
MODEL_PATH          = os.path.join(os.path.dirname(__file__),
                                   "MobileNetSSD_deploy.caffemodel")

# Camera resolution
CAM_WIDTH, CAM_HEIGHT = 640, 480

# Frame display (set False on headless Pi to save CPU)
SHOW_PREVIEW        = False

# Log file
LOG_FILE            = "mission_detection.log"
# ══════════════════════════════════════════════════════════════════

# COCO class labels for MobileNet SSD
CLASSES = [
    "background", "aeroplane", "bicycle", "bird", "boat",
    "bottle", "bus", "car", "cat", "chair", "cow", "diningtable",
    "dog", "horse", "motorbike", "person", "pottedplant",
    "sheep", "sofa", "train", "tvmonitor"
]
PERSON_CLASS_ID = CLASSES.index("person")   # == 15

# ArduPilot custom mode numbers
COPTER_MODE_AUTO   = 3
COPTER_MODE_LOITER = 5

# ── Logging setup ─────────────────────────────────────────────────
logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s  [%(levelname)s]  %(message)s",
    handlers=[
        logging.StreamHandler(sys.stdout),
        logging.FileHandler(LOG_FILE),
    ]
)
log = logging.getLogger("mission")


# ══════════════════════════════════════════════════════════════════
# MAVLink helper
# ══════════════════════════════════════════════════════════════════
class DroneController:
    """Thread-safe wrapper around a MAVLink connection."""

    def __init__(self, connection_str: str, baud: int):
        self.connection_str = connection_str
        self.baud           = baud
        self.master          = None
        self._lock           = threading.Lock()

        # Live telemetry (updated by background reader)
        self.current_mode    = "UNKNOWN"
        self.current_seq     = 0
        self.lat             = None   # degrees
        self.lon             = None   # degrees
        self.alt             = None   # metres AGL
        self._reader_thread  = None

    # ── Connect ───────────────────────────────────────────────────
    def connect(self):
        log.info(f"Connecting to FC: {self.connection_str}  baud={self.baud}")
        self.master = mavutil.mavlink_connection(
            self.connection_str, baud=self.baud
        )
        log.info("Waiting for heartbeat...")
        self.master.wait_heartbeat(timeout=30)
        log.info(
            f"Heartbeat received — sysid={self.master.target_system} "
            f"compid={self.master.target_component}"
        )
        self._start_reader()

    # ── Background telemetry reader ───────────────────────────────
    def _start_reader(self):
        self._reader_thread = threading.Thread(
            target=self._reader_worker, daemon=True, name="mav-reader"
        )
        self._reader_thread.start()

    def _reader_worker(self):
        while True:
            try:
                msg = self.master.recv_match(blocking=True, timeout=2)
                if msg is None:
                    continue
                mtype = msg.get_type()

                if mtype == "HEARTBEAT":
                    mode = mavutil.mode_string_v10(msg)
                    with self._lock:
                        self.current_mode = mode

                elif mtype == "MISSION_CURRENT":
                    with self._lock:
                        self.current_seq = msg.seq

                elif mtype == "GLOBAL_POSITION_INT":
                    with self._lock:
                        self.lat = msg.lat / 1e7
                        self.lon = msg.lon / 1e7
                        self.alt = msg.relative_alt / 1000.0

            except Exception as exc:
                log.debug(f"Reader exception: {exc}")

    # ── Mode helpers ──────────────────────────────────────────────
    def get_mode(self) -> str:
        with self._lock:
            return self.current_mode

    def get_seq(self) -> int:
        with self._lock:
            return self.current_seq

    def get_position(self):
        with self._lock:
            return self.lat, self.lon, self.alt

    def set_mode(self, mode_id: int, mode_name: str, retries: int = 5):
        """
        Send SET_MODE and wait for the heartbeat to confirm the change.
        Retries up to `retries` times with a 1-second gap.
        """
        log.info(f"Requesting mode → {mode_name} (id={mode_id})")
        for attempt in range(1, retries + 1):
            with self._lock:
                self.master.set_mode(mode_id)
            time.sleep(1.0)
            current = self.get_mode()
            if mode_name.upper() in current.upper():
                log.info(f"Mode confirmed: {current}")
                return True
            log.warning(
                f"  Attempt {attempt}/{retries}: mode is still '{current}'"
            )
        log.error(f"Failed to set mode to {mode_name} after {retries} retries")
        return False

    def resume_mission_from(self, seq: int):
        """
        Tell the FC to jump to waypoint `seq` and carry on the mission.
        Uses MAV_CMD_DO_SET_MISSION_CURRENT (command 224) which is
        supported by ArduPilot ≥ 4.0.
        """
        log.info(f"Resuming mission from waypoint seq={seq}")
        with self._lock:
            self.master.mav.command_long_send(
                self.master.target_system,
                self.master.target_component,
                mavutil.mavlink.MAV_CMD_DO_SET_MISSION_CURRENT,
                0,          # confirmation
                seq,        # param1 — target sequence
                0, 0, 0, 0, 0, 0
            )


# ══════════════════════════════════════════════════════════════════
# Camera helper
# ══════════════════════════════════════════════════════════════════
class Camera:
    def __init__(self, width: int, height: int):
        self.width  = width
        self.height = height
        self._cam   = None

    def open(self):
        if PICAMERA2:
            self._cam = Picamera2()
            cfg = self._cam.create_preview_configuration(
                main={"size": (self.width, self.height), "format": "RGB888"}
            )
            self._cam.configure(cfg)
            self._cam.start()
            log.info("picamera2 opened")
        else:
            self._cam = cv2.VideoCapture(0)
            self._cam.set(cv2.CAP_PROP_FRAME_WIDTH,  self.width)
            self._cam.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
            if not self._cam.isOpened():
                raise RuntimeError("Cannot open camera via OpenCV")
            log.info("OpenCV VideoCapture opened (fallback mode)")

    def read(self) -> np.ndarray:
        """Returns a BGR numpy frame."""
        if PICAMERA2:
            # picamera2 gives RGB; convert to BGR for OpenCV
            frame_rgb = self._cam.capture_array()
            return cv2.cvtColor(frame_rgb, cv2.COLOR_RGB2BGR)
        else:
            ret, frame = self._cam.read()
            if not ret:
                raise RuntimeError("Camera frame read failed")
            return frame

    def release(self):
        if self._cam:
            if PICAMERA2:
                self._cam.stop()
            else:
                self._cam.release()
            self._cam = None


# ══════════════════════════════════════════════════════════════════
# Detector
# ══════════════════════════════════════════════════════════════════
class HumanDetector:
    def __init__(self, prototxt: str, model: str, confidence: float):
        if not os.path.isfile(prototxt):
            raise FileNotFoundError(
                f"Prototxt not found: {prototxt}\n"
                "See download instructions at the bottom of mission.py"
            )
        if not os.path.isfile(model):
            raise FileNotFoundError(
                f"Caffemodel not found: {model}\n"
                "See download instructions at the bottom of mission.py"
            )
        log.info("Loading MobileNet SSD model...")
        self.net        = cv2.dnn.readNetFromCaffe(prototxt, model)
        self.confidence = confidence
        log.info("Model loaded ✓")

    def detect(self, frame: np.ndarray) -> list:
        """
        Run inference on `frame`.
        Returns a list of dicts:
          { 'confidence': float, 'box': (x1, y1, x2, y2) }
        for each detected person.
        """
        h, w = frame.shape[:2]
        blob = cv2.dnn.blobFromImage(
            cv2.resize(frame, (300, 300)),
            scalefactor=0.007843,
            size=(300, 300),
            mean=127.5
        )
        self.net.setInput(blob)
        detections = self.net.forward()

        people = []
        for i in range(detections.shape[2]):
            conf  = float(detections[0, 0, i, 2])
            label = int(detections[0, 0, i, 1])
            if label == PERSON_CLASS_ID and conf >= self.confidence:
                box = detections[0, 0, i, 3:7] * np.array([w, h, w, h])
                x1, y1, x2, y2 = box.astype(int)
                people.append({"confidence": conf, "box": (x1, y1, x2, y2)})

        return people

    @staticmethod
    def annotate(frame: np.ndarray, detections: list) -> np.ndarray:
        """Draw bounding boxes on frame (for preview / logging)."""
        out = frame.copy()
        for d in detections:
            x1, y1, x2, y2 = d["box"]
            cv2.rectangle(out, (x1, y1), (x2, y2), (0, 255, 0), 2)
            cv2.putText(
                out,
                f"HUMAN {d['confidence']:.2f}",
                (x1, max(y1 - 6, 0)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.55, (0, 255, 0), 2
            )
        return out


# ══════════════════════════════════════════════════════════════════
# Detection → Loiter → Resume state machine
# ══════════════════════════════════════════════════════════════════
class MissionController:
    def __init__(self, drone: DroneController):
        self.drone            = drone
        self._confirm_count   = 0
        self._last_event_time = 0.0   # epoch seconds of last loiter event
        self._handling        = False  # True while loiter sequence runs

    def process_frame(self, detections: list):
        """
        Call once per camera frame with the list of detected people.
        Manages confirmation window and cooldown, then fires the
        loiter sequence in a background thread so the camera loop
        is never blocked.
        """
        now = time.time()

        # Guard: ignore if already handling an event or in cooldown
        if self._handling:
            return
        if (now - self._last_event_time) < COOLDOWN_SECONDS:
            remaining = COOLDOWN_SECONDS - (now - self._last_event_time)
            # (silent cooldown — no log spam)
            return

        if detections:
            self._confirm_count += 1
            best_conf = max(d["confidence"] for d in detections)
            log.info(
                f"Human candidate frame {self._confirm_count}/{CONFIRM_FRAMES} "
                f"— best confidence {best_conf:.2f}"
            )
            if self._confirm_count >= CONFIRM_FRAMES:
                self._confirm_count   = 0
                self._last_event_time = now
                self._handling        = True
                lat, lon, alt         = self.drone.get_position()
                seq                   = self.drone.get_seq()
                log.info(
                    f"✅ HUMAN CONFIRMED at seq={seq}  "
                    f"lat={lat:.7f}  lon={lon:.7f}  alt={alt:.1f}m"
                )
                # Save snapshot timestamp for GCS app / logging
                self._save_event(lat, lon, alt, seq)
                # Run loiter sequence off the main thread
                t = threading.Thread(
                    target=self._loiter_and_resume,
                    args=(seq,),
                    daemon=True,
                    name="loiter-seq"
                )
                t.start()
        else:
            # No detection this frame — reset confirmation window
            if self._confirm_count > 0:
                log.debug("Confirmation window reset (no person in frame)")
            self._confirm_count = 0

    # ── Core loiter sequence ──────────────────────────────────────
    def _loiter_and_resume(self, saved_seq: int):
        """
        1. Switch AUTO → LOITER
        2. Hover for LOITER_SECONDS
        3. Switch LOITER → AUTO
        4. Jump back to saved_seq so mission continues from there
        """
        try:
            log.info("▶ Step 1/4 — Switching to LOITER mode")
            ok = self.drone.set_mode(COPTER_MODE_LOITER, "LOITER")
            if not ok:
                log.error("Could not enter LOITER — aborting loiter sequence")
                return

            log.info(f"▶ Step 2/4 — Hovering for {LOITER_SECONDS} seconds...")
            time.sleep(LOITER_SECONDS)

            log.info("▶ Step 3/4 — Returning to AUTO mode")
            ok = self.drone.set_mode(COPTER_MODE_AUTO, "AUTO")
            if not ok:
                log.error("Could not re-enter AUTO — pilot intervention required")
                return

            log.info(f"▶ Step 4/4 — Resuming mission from waypoint seq={saved_seq}")
            self.drone.resume_mission_from(saved_seq)
            log.info("✅ Mission resumed successfully")

        except Exception as exc:
            log.exception(f"Loiter sequence error: {exc}")
        finally:
            self._handling = False

    # ── Persist detection event ───────────────────────────────────
    @staticmethod
    def _save_event(lat, lon, alt, seq):
        ts  = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        url = f"https://maps.google.com/?q={lat:.7f},{lon:.7f}" if lat else "N/A"
        content = (
            f"VICTIM DETECTION EVENT\n"
            f"{'='*50}\n"
            f"Timestamp  : {ts}\n"
            f"Trigger    : AUTO → LOITER (human detected by camera)\n"
            f"Mission seq: {seq}\n"
            f"Latitude   : {lat:.7f}\n"
            f"Longitude  : {lon:.7f}\n"
            f"Altitude   : {alt:.1f} m (AGL)\n"
            f"Google Maps: {url}\n"
            f"{'='*50}\n"
        )
        fname = f"victim_location.txt"
        with open(fname, "w") as f:
            f.write(content)
        log.info(f"Event saved → {os.path.abspath(fname)}")


# ══════════════════════════════════════════════════════════════════
# Main loop
# ══════════════════════════════════════════════════════════════════
def main():
    log.info("═" * 60)
    log.info(" mission.py  —  Raspberry Pi Human Detection Controller")
    log.info("═" * 60)

    # ── Init drone ────────────────────────────────────────────────
    drone = DroneController(MAVLINK_CONNECTION, MAVLINK_BAUD)
    drone.connect()

    # ── Wait until drone is in AUTO before we start watching ──────
    log.info("Waiting for drone to be in AUTO mode...")
    while True:
        mode = drone.get_mode()
        if "AUTO" in mode.upper():
            log.info(f"Drone is in AUTO mode ({mode}) — starting detection")
            break
        log.info(f"  Current mode: {mode} — waiting...")
        time.sleep(2)

    # ── Init detector ─────────────────────────────────────────────
    detector = HumanDetector(PROTOTXT_PATH, MODEL_PATH, CONFIDENCE_THRESH)

    # ── Init camera ───────────────────────────────────────────────
    camera = Camera(CAM_WIDTH, CAM_HEIGHT)
    camera.open()

    # ── Init mission controller ───────────────────────────────────
    mission_ctrl = MissionController(drone)

    log.info("Detection loop running — press Ctrl+C to quit")
    frame_count = 0

    try:
        while True:
            # Grab frame
            frame = camera.read()
            frame_count += 1

            # Run detection every frame (Pi 4 handles ~10–15 fps at 300×300)
            detections = detector.detect(frame)

            # Feed to state machine
            mission_ctrl.process_frame(detections)

            # Optional preview window (disable on headless Pi)
            if SHOW_PREVIEW:
                annotated = HumanDetector.annotate(frame, detections)
                mode_text = drone.get_mode()
                seq_text  = drone.get_seq()
                cv2.putText(
                    annotated,
                    f"Mode: {mode_text}  Seq: {seq_text}",
                    (10, 20),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.6, (0, 200, 255), 2
                )
                cv2.imshow("mission.py — Human Detection", annotated)
                if cv2.waitKey(1) & 0xFF == ord("q"):
                    break

    except KeyboardInterrupt:
        log.info("Interrupted by user — shutting down")
    finally:
        camera.release()
        if SHOW_PREVIEW:
            cv2.destroyAllWindows()
        log.info("Done.")


if __name__ == "__main__":
    main()


# ══════════════════════════════════════════════════════════════════
# HOW TO DOWNLOAD THE MODEL FILES
# ══════════════════════════════════════════════════════════════════
# Run these commands once on the Raspberry Pi (same folder as this script):
#
#   wget https://raw.githubusercontent.com/chuanqi305/MobileNet-SSD/master/deploy.prototxt \
#        -O MobileNetSSD_deploy.prototxt
#
#   wget https://drive.google.com/uc?id=0B3gersZ2cHIxRm5PMWRoTkdHdHc \
#        -O MobileNetSSD_deploy.caffemodel
#
# Or use gdown:
#   pip3 install gdown
#   gdown 0B3gersZ2cHIxRm5PMWRoTkdHdHc -O MobileNetSSD_deploy.caffemodel
#
# ══════════════════════════════════════════════════════════════════
# WIRING (Pixhawk ↔ Raspberry Pi UART)
# ══════════════════════════════════════════════════════════════════
#
#   Pixhawk TELEM2      Raspberry Pi GPIO
#   ───────────────     ─────────────────
#   TX  (pin 2)    →    RX  (GPIO 15 / pin 10)
#   RX  (pin 3)    →    TX  (GPIO 14 / pin  8)
#   GND (pin 6)    →    GND (pin 6)
#   (do NOT connect 5 V — Pi is already powered)
#
#   Enable UART in /boot/config.txt:
#       enable_uart=1
#   Disable serial console in raspi-config → Interface Options → Serial
#
# ══════════════════════════════════════════════════════════════════
# INSTALL DEPENDENCIES
# ══════════════════════════════════════════════════════════════════
#
#   pip3 install pymavlink opencv-python-headless numpy
#   # picamera2 comes pre-installed on Raspberry Pi OS Bullseye+
#
# ══════════════════════════════════════════════════════════════════
# RUNNING
# ══════════════════════════════════════════════════════════════════
#
#   python3 mission.py
#
#   # Over USB (e.g. FTDI cable):
#   MAVLINK_CONNECTION=/dev/ttyUSB0 python3 mission.py
#
#   # Simulate from PC (UDP):
#   MAVLINK_CONNECTION=udp:0.0.0.0:14551 python3 mission.py
#
# ══════════════════════════════════════════════════════════════════
