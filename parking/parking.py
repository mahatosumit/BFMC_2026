from .parking_config import PARKING_SPEED_MULTIPLIER, NORMAL_SPEED_MULTIPLIER
from .parking_imu import ParkingDistanceTracker
from .parking_detector import ParkingDetector
from .parking_slot_manager import ParkingSlotManager
from .parking_trajectory import ParkingTrajectory
from .parking_state_machine import ParkingStateMachine


class ParkingSystem:
    """
    Vision-based parallel parking orchestrator.

    Slot detection strategy
    -----------------------
    Instead of distance-grid slot counting (which depends on an unreliable
    PWM→speed proxy), we use continuous YOLO-based gap detection:
      • Car visible on right side  → occupied, keep scanning
      • No car for SLOT_FREE_FRAMES consecutive frames → free slot found → stop

    Trajectory execution
    --------------------
    When a slot is found the system loads a pre-recorded CSV trajectory.
    If the CSV file is absent it falls back to a programmatic open-loop
    maneuver whose timings are tuned in parking_config.py.
    """

    def __init__(self,
                 onnx_model_path="models/parking_car.onnx",
                 left_csv="parking/left_parallel_parking.csv",
                 right_csv="parking/right_parallel_parking.csv",
                 debug_dashboard=True):

        self.tracker      = ParkingDistanceTracker()   # kept for display only
        self.detector     = ParkingDetector(onnx_model_path)
        self.slot_manager = ParkingSlotManager()
        self.trajectory   = ParkingTrajectory(left_csv, right_csv)
        self.state_machine = ParkingStateMachine()

        self.last_debug_data    = None
        self.last_sign_detections = []
        self.last_car_detections  = []
        self.debug_dashboard    = debug_dashboard
        self.main_live_imu      = {}
        self.parking_start_reference = 0.0

        # ── YOLO fallback when ONNX model is absent ───────────────────────
        if not getattr(self.detector, 'parking_detection_enabled', True) \
                or self.detector.session is None:
            self._try_load_yolo_fallback()

        # Override detect_cars_in_roi to use the wider 50% ROI
        self.detector.detect_cars_in_roi = self._detect_cars_wide_roi

    # ── YOLO fallback loader ──────────────────────────────────────────────
    def _try_load_yolo_fallback(self):
        import os
        try:
            from ultralytics import YOLO
            for candidate in ("best.pt", "assets/Niranjan.pt"):
                if os.path.exists(candidate):
                    _yolo = YOLO(candidate)
                    self.detector.parking_detection_enabled = True
                    print(f"[Parking] YOLO fallback loaded: {candidate}")

                    def _detect_via_yolo(frame):
                        if frame is None:
                            return []
                        results = _yolo.predict(frame, conf=0.25, verbose=False)
                        detections = []
                        for box in results[0].boxes:
                            x1, y1, x2, y2 = map(int, box.xyxy[0].tolist())
                            lbl  = _yolo.names[int(box.cls[0].item())].lower()
                            conf = float(box.conf[0].item())
                            cls  = 0
                            if any(k in lbl for k in ("parking", "park")):
                                cls = 1
                            elif lbl == "car":
                                cls = 2
                            if cls == 0:
                                continue
                            h_f, w_f = frame.shape[:2]
                            cx = (x1 + x2) / 2.0
                            cy = (y1 + y2) / 2.0
                            bw = x2 - x1
                            bh = y2 - y1
                            # normalise to 640×640 model space
                            sx, sy = 640.0 / w_f, 640.0 / h_f
                            detections.append({
                                "class_id":  cls,
                                "confidence": conf,
                                "cx": cx * sx,
                                "cy": cy * sy,
                                "w":  bw * sx,
                                "h":  bh * sy,
                                "bbox": [int(x1 * sx), int(y1 * sy),
                                         int(x2 * sx), int(y2 * sy)],
                            })
                        return detections

                    self.detector._detect = _detect_via_yolo
                    return
        except Exception as e:
            print(f"[Parking] YOLO fallback failed: {e}")

    # ── Wide-ROI car detector (bottom 50% of frame) ───────────────────────
    def _detect_cars_wide_roi(self, frame):
        if frame is None:
            return []
        h, w = frame.shape[:2]
        roi = frame[int(h * 0.50):h, 0:w]
        detections = self.detector._detect(roi)
        cars = []
        for det in detections:
            if det["class_id"] == 2:
                side = "left" if det["cx"] < 320.0 else "right"
                cars.append({**det, "side": side})
        return cars

    # ─────────────────────────────────────────────────────────────────────
    def update(self,
               frame,
               dt,
               real_imu=None,
               current_speed=0.0,
               reverse_parking_done=False,
               autonomous_mode=True,
               pedestrian_detected=False):
        """
        Call every control-loop frame.  Returns a standard dict consumed by main.py.
        """
        # ── IMU bookkeeping (for display only) ────────────────────────────
        self.main_live_imu = real_imu or {"yaw": 0.0, "pitch": 0.0, "roll": 0.0}
        current_yaw  = self.main_live_imu.get("yaw", 0.0)
        relative_yaw = (current_yaw - self.parking_start_reference + 180.0) % 360.0 - 180.0
        yaw_diff     = abs(relative_yaw)

        roi_frame = None
        if frame is not None:
            h, w = frame.shape[:2]
            roi_frame = frame[int(h * 0.50):h, 0:w]

        # ── Early exit: manual mode ───────────────────────────────────────
        if not autonomous_mode:
            self.tracker.reset()
            self.slot_manager.reset()
            self.state_machine.reset()
            self.parking_start_reference = current_yaw
            self._save_debug(frame, roi_frame, [], [], relative_yaw, yaw_diff, -1)
            return self._null_output()

        # ── Early exit: pedestrian freeze ────────────────────────────────
        if pedestrian_detected:
            st = self.state_machine.get_state()
            if st == 6:
                self.state_machine.exec_start_time += dt
            traj = self.trajectory.trajectory_points if st >= 6 else None
            self._save_debug(frame, roi_frame, [], [], relative_yaw, yaw_diff, st)
            return {
                "parking_completed":  self.state_machine.parking_completed,
                "parking_failed":     self.state_machine.parking_failed,
                "selected_slot":      self.slot_manager.selected_slot,
                "selected_side":      self.slot_manager.selected_side,
                "occupancy_status":   self.slot_manager.occupancy_map,
                "trajectory":         traj,
                "speed_multiplier":   0.0,
                "parking_mode_active": st >= 1,
                "parking_takeover":   False,
            }

        # ── Distance tracker (display / debug only) ───────────────────────
        self.tracker.update(yaw_diff, dt, current_speed)

        state_before = self.state_machine.get_state()

        # ── Vision: run detection based on state ──────────────────────────
        sign_detections = []
        car_detections  = []
        sign_detected   = False
        vision_slot_found = False

        if state_before == 0:
            # Look for the parking sign
            sign_detected, sign_detections = self.detector.detect_parking_sign(frame)
            if sign_detected:
                self.last_sign_detections = sign_detections

        elif state_before == 2:
            # Actively scanning for a free slot via YOLO car detection
            car_detections = self.detector.detect_cars_in_roi(frame)
            self.last_car_detections = car_detections
            vision_slot_found = self.slot_manager.update_by_vision(car_detections)

        # ── State machine transition ──────────────────────────────────────
        self.state_machine.transition(
            sign_detected=sign_detected,
            vision_slot_found=vision_slot_found,
            reverse_parking_done=reverse_parking_done,
        )
        state_after = self.state_machine.get_state()

        # ── Post-transition hooks ─────────────────────────────────────────
        if state_before == 0 and state_after == 1:
            # Arm: record IMU reference and reset slot manager
            self.parking_start_reference = current_yaw
            self.tracker.reset()
            self.slot_manager.reset()

        elif state_before == 4 and state_after == 5:
            # Load the correct trajectory
            side = self.slot_manager.selected_side or "right"
            if side == "left":
                self.trajectory.load_left_trajectory()
            else:
                self.trajectory.load_right_trajectory()

        # ── Output formatting ─────────────────────────────────────────────
        # States 0-3: normal lane-follow at reduced speed
        # State 4:    full stop (speed_multiplier=0) while loading trajectory
        # State 5+:   trajectory playback (parking_takeover=True)
        if state_after <= 3:
            speed_mult    = NORMAL_SPEED_MULTIPLIER if state_after == 0 else PARKING_SPEED_MULTIPLIER
            takeover      = False
            traj_points   = None
        elif state_after == 4:
            speed_mult    = 0.0
            takeover      = False
            traj_points   = None
        else:
            speed_mult    = PARKING_SPEED_MULTIPLIER
            traj_points   = self.trajectory.trajectory_points if state_after >= 6 else None
            takeover      = (state_after == 6) and (traj_points is not None)

        self._save_debug(frame, roi_frame, sign_detections, car_detections,
                         relative_yaw, yaw_diff, state_after)

        return {
            "parking_completed":  self.state_machine.parking_completed,
            "parking_failed":     self.state_machine.parking_failed,
            "selected_slot":      self.slot_manager.selected_slot,
            "selected_side":      self.slot_manager.selected_side,
            "occupancy_status":   self.slot_manager.occupancy_map,
            "trajectory":         traj_points,
            "speed_multiplier":   speed_mult,
            "parking_mode_active": state_after >= 1,
            "parking_takeover":   takeover,
        }

    # ── Debug data helpers ────────────────────────────────────────────────
    def _null_output(self):
        return {
            "parking_completed":  False,
            "parking_failed":     False,
            "selected_slot":      None,
            "selected_side":      None,
            "occupancy_status":   {},
            "trajectory":         None,
            "speed_multiplier":   1.0,
            "parking_mode_active": False,
            "parking_takeover":   False,
        }

    def _save_debug(self, frame, roi_frame, sign_dets, car_dets,
                    relative_yaw, yaw_diff, state):
        self.last_debug_data = {
            "full_frame":       frame,
            "roi_frame":        roi_frame,
            "sign_detections":  sign_dets,
            "car_detections":   car_dets,
            "main_live_imu":    self.main_live_imu,
            "parking_reset_imu": {
                "reset_yaw":    self.parking_start_reference,
                "current_yaw":  relative_yaw,
                "yaw_difference": yaw_diff,
                "distance_cm":  self.tracker.distance_cm,
                "current_slot": self.slot_manager.current_slot,
            },
            "state":            state,
            "distance_cm":      self.tracker.distance_cm,
            "current_slot":     self.slot_manager.current_slot,
            "selected_slot":    self.slot_manager.selected_slot,
            "selected_side":    self.slot_manager.selected_side,
            "occupancy_map":    self.slot_manager.occupancy_map,
            "speed_multiplier": 0.0,
            "parking_completed": self.state_machine.parking_completed,
            "parking_failed":   self.state_machine.parking_failed,
            "trajectory":       self.trajectory.trajectory_points,
            "target_stop_distance": self.state_machine.target_stop_distance_cm,
        }

    # ── Dashboard rendering ───────────────────────────────────────────────
    def render_dashboard(self, frame):
        try:
            if not hasattr(self, 'dashboard'):
                from .parking_dashboard import ParkingDashboard
                self.dashboard = ParkingDashboard()

            data = self.last_debug_data
            if data is None:
                data = {
                    "state": -1, "distance_cm": 0.0, "current_slot": 0,
                    "selected_slot": None, "selected_side": None,
                    "occupancy_map": {}, "speed_multiplier": 1.0,
                    "parking_completed": False, "parking_failed": False,
                    "trajectory": None, "roi_frame": None, "full_frame": frame,
                    "sign_detections": [], "car_detections": [],
                    "target_stop_distance": 0.0,
                    "main_live_imu": {"yaw": 0.0, "pitch": 0.0, "roll": 0.0},
                    "parking_reset_imu": {
                        "reset_yaw": 0.0, "current_yaw": 0.0,
                        "yaw_difference": 0.0, "distance_cm": 0.0, "current_slot": 0,
                    },
                    "model_loaded": getattr(self.detector, 'parking_detection_enabled', False),
                }
            else:
                data = data.copy()
                data["full_frame"]   = frame
                data["model_loaded"] = getattr(self.detector, 'parking_detection_enabled', False)

            self.dashboard.update(data)
        except Exception as e:
            print(f"[Parking Dashboard Error] {e}")
