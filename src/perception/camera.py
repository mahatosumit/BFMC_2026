import sys
import time
import numpy as np
import logging
import threading
import queue
import cv2

log = logging.getLogger(__name__)

try:
    from picamera2 import Picamera2
    _CAM_AVAILABLE = True
except ImportError:
    _CAM_AVAILABLE = False
    log.warning("picamera2 not found. Using fallback mode for Camera.")

class Camera:
    def __init__(self, sim_video=None):
        self.sim_video = sim_video
        self.camera = None
        self.video_cap = None
        self._frame_queue = queue.Queue(maxsize=1)
        self._running = True

        if self.sim_video:
            self.video_cap = cv2.VideoCapture(self.sim_video)
            log.info(f"Loaded simulation video: {self.sim_video}")
            threading.Thread(target=self._video_worker, daemon=True, name="video_worker").start()
        elif _CAM_AVAILABLE:
            try:
                self.camera = Picamera2()
                cfg = self.camera.create_video_configuration(
                    main={"size": (1280, 720), "format": "XRGB8888"},
                    controls={
                        "AwbEnable":   True,
                        "AeEnable":    True,
                        "Saturation":  1.2,
                        "Sharpness":   1.2,
                    }
                )
                self.camera.configure(cfg)
                self.camera.start()
                log.info("PiCamera2 initialized.")
                threading.Thread(target=self._camera_worker, daemon=True, name="camera_worker").start()
            except Exception as e:
                log.error(f"PiCamera2 init error: {e}")
                self.camera = None
        
        # Fallback to default webcam if no sim video and no picamera
        if not self.sim_video and self.camera is None:
            self.video_cap = cv2.VideoCapture(0)
            if self.video_cap.isOpened():
                log.info("Loaded default webcam")
                threading.Thread(target=self._video_worker, daemon=True, name="video_worker").start()
            else:
                log.warning("Could not open default webcam.")

    def _push_frame(self, frame):
        if self._frame_queue.full():
            try:
                self._frame_queue.get_nowait()
            except queue.Empty:
                pass
        self._frame_queue.put(frame)

    def _camera_worker(self):
        while self._running:
            try:
                if self.camera is None:
                    time.sleep(0.1)
                    continue
                frame = self.camera.capture_array()
                if frame is not None:
                    if frame.ndim == 3 and frame.shape[2] == 4:
                        frame = cv2.cvtColor(frame, cv2.COLOR_BGRA2BGR)
                    else:
                        frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
                    self._push_frame(cv2.resize(frame, (640, 480)))
            except Exception as e:
                log.warning(f"Camera worker error: {e}")
                time.sleep(0.033)

    def _video_worker(self):
        while self._running:
            if self.video_cap is None or not self.video_cap.isOpened():
                time.sleep(0.033)
                continue
            ret, frame = self.video_cap.read()
            if not ret:
                if self.sim_video: # Loop video
                    self.video_cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
                    ret, frame = self.video_cap.read()
                else:
                    time.sleep(0.033)
                    continue
            if ret:
                self._push_frame(cv2.resize(frame, (640, 480)))
            time.sleep(0.033)

    def read_frame(self):
        try:
            return self._frame_queue.get_nowait()
        except queue.Empty:
            return None

    def stop(self):
        self._running = False
        time.sleep(0.1)
        if self.camera:
            self.camera.stop()
        if self.video_cap:
            self.video_cap.release()
