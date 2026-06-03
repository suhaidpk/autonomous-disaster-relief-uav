import threading
import time
import os
import subprocess
from picamera2 import Picamera2
from ultralytics import YOLO
import cv2

# ── try pymavlink ─────────────────────────────────────────────────
try:
    from pymavlink import mavutil
    MAVLINK_AVAILABLE = True
except ImportError:
    MAVLINK_AVAILABLE = False
    print("pymavlink not found — install with: pip3 install pymavlink")

# ══════════════════════════════════════════════════════
# CONFIG
# ══════════════════════════════════════════════════════
MAVLINK_CONNECTION = "/dev/serial0"
MAVLINK_BAUD       = 57600
LOITER_SECONDS     = 5
CONFIDENCE         = 0.30

COPTER_MODE_AUTO   = 3
COPTER_MODE_LOITER = 5
# ══════════════════════════════════════════════════════


# ── MAVLink helpers ───────────────────────────────────
master = None

def connect_drone():
    global master
    print("Connecting to flight controller...")
    master = mavutil.mavlink_connection(MAVLINK_CONNECTION, baud=MAVLINK_BAUD)
    master.wait_heartbeat()
    print("Flight controller connected")

def set_mode(mode_id, mode_name):
    for _ in range(5):
        master.set_mode(mode_id)
        time.sleep(1)
        msg = master.recv_match(type="HEARTBEAT", blocking=True, timeout=2)
        if msg:
            current = mavutil.mode_string_v10(msg)
            if mode_name.upper() in current.upper():
                print(f"Mode set to {mode_name}")
                return True
    print(f"Failed to set {mode_name}")
    return False

def get_gps():
    msg = master.recv_match(type="GLOBAL_POSITION_INT", blocking=True, timeout=3)
    if msg:
        return msg.lat / 1e7, msg.lon / 1e7, msg.relative_alt / 1000.0
    return None, None, None

def get_current_seq():
    msg = master.recv_match(type="MISSION_CURRENT", blocking=True, timeout=3)
    return msg.seq if msg else 0

def resume_mission(seq):
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_DO_SET_MISSION_CURRENT,
        0, float(seq), 0, 0, 0, 0, 0, 0
    )
    print(f"Resuming mission from waypoint {seq}")


# ── called when human is detected ────────────────────
def on_human_detected():
    print("HUMAN DETECTED — switching to LOITER")

    seq = get_current_seq()
    lat, lon, alt = get_gps()
    print(f"Location: lat={lat}  lon={lon}  alt={alt}m")

    set_mode(COPTER_MODE_LOITER, "LOITER")
    print(f"Hovering for {LOITER_SECONDS} seconds...")
    time.sleep(LOITER_SECONDS)

    set_mode(COPTER_MODE_AUTO, "AUTO")
    resume_mission(seq)

    # save coords
    with open("victim_location.txt", "w") as f:
        f.write(f"Latitude : {lat}\n")
        f.write(f"Longitude: {lon}\n")
        f.write(f"Altitude : {alt} m\n")
        f.write(f"Maps     : https://maps.google.com/?q={lat},{lon}\n")
    print("Saved victim_location.txt")


# ── detection loop ────────────────────────────────────
def run_detection():
    model = YOLO("best.pt")

    cam = Picamera2()
    cam.configure(cam.create_preview_configuration(
        main={"format": "RGB888", "size": (640, 480)}
    ))
    cam.start()
    print("Camera open — watching for humans...")

    human_handled = False

    try:
        while True:
            frame = cam.capture_array()
            results = model(frame, verbose=False)[0]

            human_found = False

            for box in results.boxes:
                confidence = float(box.conf[0])
                if confidence >= CONFIDENCE:
                    human_found = True
                    label = results.names[int(box.cls[0])]
                    x1, y1, x2, y2 = map(int, box.xyxy[0])
                    cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                    cv2.putText(
                        frame,
                        f"{label} {confidence:.0%}",
                        (x1, y1 - 8),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.6, (0, 255, 0), 2
                    )

            if human_found and not human_handled:
                human_handled = True
                # run drone response in a separate thread so camera keeps going
                threading.Thread(target=on_human_detected, daemon=True).start()
                # reset after cooldown so it can detect again
                def reset():
                    time.sleep(40)
                    nonlocal human_handled
                    human_handled = False
                threading.Thread(target=reset, daemon=True).start()

            bgr = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
            cv2.imshow("detection", bgr)

            if cv2.waitKey(1) & 0xFF == ord("q"):
                break

    except KeyboardInterrupt:
        pass

    cam.stop()
    cv2.destroyAllWindows()


# ── entry point ───────────────────────────────────────
if __name__ == "__main__":
    # start detection.py in the background
    subprocess.Popen(["python3", os.path.join(os.path.dirname(__file__), "detection.py")])
    print("detection.py started")

    if MAVLINK_AVAILABLE:
        connect_drone()
