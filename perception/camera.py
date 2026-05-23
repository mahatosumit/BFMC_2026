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
            for attempt in range(3):  # Retry up to 3 times
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
                    break  # Success, exit retry loop
                except Exception as e:
                    log.warning(f"PiCamera2 init attempt {attempt+1}/3 failed: {e}")
                    self.camera = None
                    if attempt < 2:  # Don't sleep on last attempt
                        time.sleep(2)  # Wait 2 seconds before retry
        
        # Fallback to default webcam if no sim video and no picamera.
        # Use DirectShow on Windows — MSMF (default) raises MF_E_HW_MFT_FAILED_START_STREAMING
        # (-1072875772) on many USB webcams even though the device opens without error.
        if not self.sim_video and self.camera is None:
            backend = cv2.CAP_DSHOW if sys.platform == "win32" else cv2.CAP_ANY
            self.video_cap = cv2.VideoCapture(0, backend)
            if self.video_cap.isOpened():
                log.info("Loaded default webcam")
                threading.Thread(target=self._video_worker, daemon=True, name="video_worker").start()
            else:
                log.warning("Could not open default webcam. Using dummy camera.")
                # Create a dummy frame generator
                threading.Thread(target=self._dummy_worker, daemon=True, name="dummy_worker").start()

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

    def _dummy_worker(self):
        """Generate dummy frames when no camera is available."""
        import numpy as np
        while self._running:
            # Create a dummy frame with some pattern
            frame = np.zeros((480, 640, 3), dtype=np.uint8)
            cv2.putText(frame, "NO CAMERA", (200, 240), cv2.FONT_HERSHEY_SIMPLEX, 2, (255, 255, 255), 3)
            cv2.putText(frame, "Check connections", (160, 280), cv2.FONT_HERSHEY_SIMPLEX, 1, (200, 200, 200), 2)
            self._push_frame(frame)
            time.sleep(0.5)  # Update every 0.5 seconds

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
