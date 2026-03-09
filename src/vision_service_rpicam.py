#!/usr/bin/env python3

import subprocess
import numpy as np
import socket
import json
import time
import cv2
from ultralytics import YOLO

# =========================
# DEBUG / DISPLAY SETTINGS
# =========================
SHOW_VIDEO = True   

# =========================
# Camera configuration
# =========================
WIDTH = 640
HEIGHT = 480
YUV_FRAME_SIZE = WIDTH * HEIGHT * 3 // 2  # YUV420

# =========================
# Gating parameters
# =========================
CONF_THRESHOLD = 0.7
AREA_THRESHOLD = 8000
STABLE_FRAMES = 3
EVENT_COOLDOWN = 1.0  # seconds

# =========================
# Model & UDP
# =========================
MODEL_PATH = "/home/pi/Documents/models/best.pt"
UDP_IP = "127.0.0.1"
UDP_PORT = 5005

model = YOLO(MODEL_PATH)
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

# =========================
# rpicam command (NEW OS)
# =========================
cmd = [
    "rpicam-vid",
    "-t", "0",
    "--width", str(WIDTH),
    "--height", str(HEIGHT),
    "--codec", "yuv420",
    "-o", "-",
    "--nopreview"
]

proc = subprocess.Popen(
    cmd,
    stdout=subprocess.PIPE,
    bufsize=YUV_FRAME_SIZE
)

print("Vision service (RPiCam + YOLO + GATING + VIDEO) running...")

# =========================
# State variables
# =========================
stable_count = 0
last_event_time = 0.0

# =========================
# Main loop
# =========================
while True:
    try:
        raw = proc.stdout.read(YUV_FRAME_SIZE)
        if len(raw) != YUV_FRAME_SIZE:
            continue

        # Convert YUV420 -> RGB
        yuv = np.frombuffer(raw, dtype=np.uint8)
        yuv = yuv.reshape((HEIGHT * 3 // 2, WIDTH))
        frame = cv2.cvtColor(yuv, cv2.COLOR_YUV2RGB_I420)

        # -------------------------
        # YOLO inference
        # -------------------------
        results = model(frame, conf=CONF_THRESHOLD, device="cpu", verbose=False)

        detected = False
        best_event = None

        for r in results:
            if r.boxes is None:
                continue

            for box in r.boxes:
                cls_id = int(box.cls[0])
                conf = float(box.conf[0])
                w = float(box.xywh[0][2])
                h = float(box.xywh[0][3])
                area = w * h

                if conf >= CONF_THRESHOLD and area >= AREA_THRESHOLD:
                    detected = True
                    best_event = {
                        "object": model.names[cls_id],
                        "confidence": round(conf, 3),
                        "bbox_area": int(area),
                        "timestamp": time.time()
                    }

                    # Optional bounding box overlay
                    if SHOW_VIDEO:
                        x1, y1, x2, y2 = map(int, box.xyxy[0])
                        cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                        cv2.putText(
                            frame,
                            f"{model.names[cls_id]} {conf:.2f}",
                            (x1, y1 - 10),
                            cv2.FONT_HERSHEY_SIMPLEX,
                            0.6,
                            (0, 255, 0),
                            2
                        )
                    break

        # -------------------------
        # Stability gating
        # -------------------------
        if detected:
            stable_count += 1
        else:
            stable_count = 0

        now = time.time()
        if stable_count >= STABLE_FRAMES and best_event:
            if now - last_event_time >= EVENT_COOLDOWN:
                sock.sendto(
                    json.dumps(best_event).encode(),
                    (UDP_IP, UDP_PORT)
                )
                last_event_time = now
                stable_count = 0

        # -------------------------
        # OpenCV display (DEBUG)
        # -------------------------
        if SHOW_VIDEO:
            cv2.imshow("Vision Debug Feed", frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    except KeyboardInterrupt:
        print("\nStopping vision service...")
        break

    except Exception as e:
        print("Vision error:", e)
        time.sleep(0.1)

# =========================
# Cleanup
# =========================
proc.terminate()
sock.close()
cv2.destroyAllWindows()
