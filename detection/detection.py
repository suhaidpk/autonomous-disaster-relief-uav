from picamera2 import Picamera2
from ultralytics import YOLO
import cv2

# load your trained weights
model = YOLO("best.pt")

# open Pi camera module 3
cam = Picamera2()
cam.configure(cam.create_preview_configuration(main={"format": "RGB888", "size": (640, 480)}))
cam.start()

print("Camera open — detecting humans. Press Ctrl+C to stop.")

try:
    while True:
        frame = cam.capture_array()

        results = model(frame, verbose=False)[0]

        human_found = False

        for box in results.boxes:
            confidence = float(box.conf[0])

            if confidence >= 0.30:
                human_found = True
                label = results.names[int(box.cls[0])]
                x1, y1, x2, y2 = map(int, box.xyxy[0])

                # draw box on frame
                cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                cv2.putText(
                    frame,
                    f"{label} {confidence:.0%}",
                    (x1, y1 - 8),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.6, (0, 255, 0), 2
                )

        if human_found:
            print("HUMAN DETECTED")

        # convert RGB to BGR for display
        bgr = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
        cv2.imshow("detection", bgr)

        if cv2.waitKey(1) & 0xFF == ord("q"):
            break

except KeyboardInterrupt:
    pass

cam.stop()
cv2.destroyAllWindows()
