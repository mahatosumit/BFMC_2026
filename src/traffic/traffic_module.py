"""
traffic_module.py — BFMC Traffic Decision Engine
=================================================
Handles all required traffic signs and rules:

  Traffic Lights : Red (stop) / Yellow (slow) / Green (go)
  Signs          : Stop, Parking, Crosswalk, Priority, Highway-Entry/Exit,
                   One-Way, Roundabout, No-Entry, Speed-Limit
  General rules  : Pedestrian-at-crosswalk stop, pedestrian-on-road stop,
                   obstacle overtake gated by dashed/continuous line
  Zone tracking  : CITY / HIGHWAY with appropriate speed multipliers
  Parking FSM    : SEEK -> ENTER -> WAIT -> EXIT -> DONE
"""

import threading
import queue
import time
import math
import numpy as np
import cv2
from dataclasses import dataclass, field
from typing import List

try:
    from ultralytics import YOLO
    _YOLO_AVAILABLE = True
except ImportError:
    _YOLO_AVAILABLE = False
    print("WARNING: ultralytics (YOLO) not found.")


# ══════════════════════════════════════════════════════════════════════════════
# Data contract
# ══════════════════════════════════════════════════════════════════════════════

@dataclass
class TrafficResult:
    """
    Single output produced by TrafficDecisionEngine.process().

    state           : primary driving command for the control layer.
    reason          : human-readable string for the dashboard.
    speed_multiplier: applied to base_speed in Controller.compute().
    zone_mode       : "CITY" | "HIGHWAY"   — governs speed floors.
    parking_state   : "NONE"|"SEEK"|"ENTER"|"WAIT"|"EXIT"|"DONE"
    steer_bias      : additional steer angle (deg) requested by parking FSM.
    pedestrian_blocking : True when pedestrian is holding us at a crosswalk.
    light_status    : dashboard string for the traffic-light colour.
    active_labels   : all YOLO class names seen this frame.
    yolo_debug_frame: BGR frame with detection overlays.
    sign_approach_m : estimated approach distance to nearest sign (for dash)
    """
    state: str              # SYS_GO | SYS_STOP | SYS_SLOW | SYS_APPROACH | SYS_LANE_CHANGE_LEFT | SYS_LIMIT
    reason: str
    speed_multiplier: float
    zone_mode: str = "CITY"
    parking_state: str = "NONE"
    steer_bias: float = 0.0
    pedestrian_blocking: bool = False
    light_status: str = "NONE"
    active_labels: List[str] = field(default_factory=list)
    yolo_debug_frame: np.ndarray = None
    sign_approach_m: float = 99.0   # estimated distance to nearest active sign


# ══════════════════════════════════════════════════════════════════════════════
# Threaded YOLO detector (unchanged from original, minus imports)
# ══════════════════════════════════════════════════════════════════════════════

class ThreadedYOLODetector:
    def __init__(self, model_path="best.pt"):
        self.frame_queue   = queue.Queue(maxsize=1)
        self.result_queue  = queue.Queue(maxsize=1)
        self.running       = True
        self.active_detections = []
        self.yolo_ok       = False
        self.model_path_used = None
        self.model         = None

        if _YOLO_AVAILABLE:
            resolved = self._resolve_model_path(model_path)
            if resolved:
                try:
                    import torch
                    _orig = torch.load
                    def _patched(*args, **kwargs):
                        kwargs['weights_only'] = False
                        return _orig(*args, **kwargs)
                    torch.load = _patched
                    try:
                        self.model = YOLO(resolved)
                        self.yolo_ok = True
                        self.model_path_used = resolved
                        print(f"[YOLO] Loaded: {resolved}")
                    finally:
                        torch.load = _orig
                except Exception as e:
                    print(f"[YOLO] Load failed: {e}")
            else:
                print(f"[YOLO] '{model_path}' not found — detection disabled.")

        self.worker = threading.Thread(target=self._run, daemon=True)
        self.worker.start()

    @staticmethod
    def _resolve_model_path(model_path):
        import os
        candidates = [
            model_path,
            os.path.join(os.path.dirname(os.path.abspath(__file__)), model_path),
            os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", model_path),
            os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..", model_path),
        ]
        for c in candidates:
            if os.path.isfile(c):
                return os.path.abspath(c)
        return None

    def _run(self):
        while self.running:
            try:
                frame = self.frame_queue.get(timeout=0.1)
                if self.model is None:
                    continue
                results = self.model.predict(source=frame, conf=0.25, verbose=False)
                detections = []
                for box in results[0].boxes:
                    x1, y1, x2, y2 = map(int, box.xyxy[0].tolist())
                    label = self.model.names[int(box.cls[0].item())]
                    conf  = box.conf[0].item()
                    detections.append({"label": label, "confidence": conf,
                                       "bbox": (x1, y1, x2, y2)})
                # Flush existing results to push the newest one immediately
                while not self.result_queue.empty():
                    try:
                        self.result_queue.get_nowait()
                    except queue.Empty:
                        break
                self.result_queue.put(detections)
            except queue.Empty:
                pass
            except Exception as e:
                print(f"YOLO Thread Error: {e}")

    def update_frame(self, frame):
        # Drop stale frames to process ONLY the newest frame
        while not self.frame_queue.empty():
            try:
                self.frame_queue.get_nowait()
            except queue.Empty:
                break
        self.frame_queue.put(frame.copy())

    def get_detections(self):
        if not self.result_queue.empty():
            self.active_detections = self.result_queue.get()
        return self.active_detections

    def stop(self):
        self.running = False
        self.worker.join()


# ══════════════════════════════════════════════════════════════════════════════
# Traffic-light state machine  (added YELLOW)
# ══════════════════════════════════════════════════════════════════════════════

class TrafficLightStateMachine:
    """
    States: NO_LIGHT | LIGHT_DETECTED_FAR | LIGHT_RED_STOPPING |
            LIGHT_RED_STOPPED | LIGHT_YELLOW_SLOW | LIGHT_GREEN_GO
    """
    def __init__(self):
        self.state = "NO_LIGHT"
        self.last_seen_red = 0.0

    def update(self, is_red, is_yellow, is_green, dist_cat):
        now = time.time()

        if is_red:
            self.last_seen_red = now
            if dist_cat == "HALT":
                self.state = "LIGHT_RED_STOPPED"
            elif dist_cat == "APPROACH":
                self.state = "LIGHT_RED_STOPPING"
            elif self.state == "NO_LIGHT":
                self.state = "LIGHT_DETECTED_FAR"

        elif is_yellow:
            # Yellow: slow down and prepare to stop or proceed with caution
            self.state = "LIGHT_YELLOW_SLOW"

        elif is_green:
            if self.state in ["LIGHT_RED_STOPPED", "LIGHT_RED_STOPPING"]:
                if now - self.last_seen_red > 1.0:   # 1 s delay before moving
                    self.state = "LIGHT_GREEN_GO"
            else:
                self.state = "LIGHT_GREEN_GO"

        else:
            # Light lost - DO NOTHING if we are stopped, wait permanently for Green!
            if self.state not in ["LIGHT_RED_STOPPED", "LIGHT_RED_STOPPING"]:
                self.state = "NO_LIGHT"

        return self.state


# ══════════════════════════════════════════════════════════════════════════════
# Collision predictor (unchanged)
# ══════════════════════════════════════════════════════════════════════════════

class CollisionPredictor:
    def __init__(self):
        self.history = {}

    def update_and_predict(self, detections, dt):
        critical = []
        current  = {}
        for det in detections:
            lbl = det["label"]
            if lbl not in ["car", "pedestrian", "closed-road-stand",
                           "roadblock", "obstacle"]:
                continue
            x1, y1, x2, y2 = det["bbox"]
            h  = y2 - y1
            cx = (x1 + x2) / 2

            matched = None
            best_dist = 50   # pixel proximity threshold
            for tid, data in self.history.items():
                if data["label"] == lbl:
                    dist = abs(data["cx"] - cx)
                    if dist < best_dist:   # BUG-05: pick CLOSEST, not first match
                        best_dist = dist
                        matched   = tid
            if not matched:
                self._next_id = getattr(self, '_next_id', 0) + 1
                matched = f"{lbl}_{self._next_id}"

            current[matched] = {"label": lbl, "cx": cx, "h": h}
            if matched in self.history:
                last_h = self.history[matched]["h"]
                growth = (h - last_h) / max(dt, 0.01)
                if growth > 5.0 and h > 40:
                    ttc = h / growth if growth > 0 else 999
                    if ttc < 3.0:
                        critical.append(det)

        self.history = current
        return critical


# ══════════════════════════════════════════════════════════════════════════════
# Pedestrian-at-crosswalk monitor
# ══════════════════════════════════════════════════════════════════════════════

class PedestrianCrosswalkMonitor:
    """
    Tracks when the car is near a crosswalk AND a pedestrian is present.
    Rule: stop and WAIT until the pedestrian has cleared the road.
    """
    CROSSWALK_MEMORY_S = 6.0    # hold crosswalk-detected state for 6 s

    def __init__(self):
        self._crosswalk_ts  = 0.0   # time last crosswalk sign was seen
        self._blocking      = False
        self._clear_ts      = 0.0   # time pedestrian last disappeared

    def update(self, dets, h, w, now):
        """
        Returns (blocking: bool, reason: str).
        blocking=True means SYS_STOP should be committed.
        """
        # Refresh crosswalk memory
        for d in dets:
            lbl = d["label"].lower()
            if any(k in lbl for k in ("crosswalk", "pedestrian_crossing",
                                      "crosswalk-sign")):
                self._crosswalk_ts = now

        near_crosswalk = (now - self._crosswalk_ts) < self.CROSSWALK_MEMORY_S

        if not near_crosswalk:
            self._blocking = False
            return False, ""

        # Check for pedestrian in the lower-centre road area
        ped_on_road = False
        for d in dets:
            if d["label"].lower() not in ("pedestrian", "person"):
                continue
            x1, y1, x2, y2 = d["bbox"]
            # Pedestrian is considered "on road" when their feet are in the
            # lower 60 % of the frame and their body straddles the road centre
            if y2 > h * 0.40 and x1 < w * 0.75 and x2 > w * 0.25:
                ped_on_road = True
                break

        if ped_on_road:
            self._blocking  = True
            self._clear_ts  = 0.0
        elif self._blocking:
            # Wait 1.5 s after pedestrian clears before releasing
            if self._clear_ts == 0.0:
                self._clear_ts = now
            if now - self._clear_ts > 1.5:
                self._blocking = False

        return self._blocking, "PEDESTRIAN AT CROSSWALK" if self._blocking else ""


# ══════════════════════════════════════════════════════════════════════════════
# Parking state machine
# ══════════════════════════════════════════════════════════════════════════════

class ParkingStateMachine:
    """
    Full parking maneuver FSM:
      NONE -> TRIGGERED -> SEEK -> ENTER -> WAIT -> EXIT -> DONE -> NONE

    Returns (state_str, speed_mult, steer_bias_deg) each frame.
    speed_mult < 0 is not used (no reverse); EXIT re-uses slow forward.
    steer_bias_deg is added to the controller's steering output.
    """
    # Time limits for each phase (seconds)
    SEEK_TIMEOUT   = 6.0    # if no clear spot found, park anyway after 6 s
    ENTER_DURATION = 2.0    # drive forward into spot
    WAIT_DURATION  = 3.0    # mandatory stop in spot (competition requirement)
    EXIT_DURATION  = 2.5    # drive forward out of spot

    # Steer bias angles (degrees)
    ENTER_STEER =  22.0     # right steer to angle into spot
    EXIT_STEER  = -18.0     # left steer to pull out of spot

    def __init__(self):
        self.state  = "NONE"
        self._ts    = 0.0

    def trigger(self, now):
        """Call when a parking sign is detected."""
        if self.state == "NONE":
            self.state = "TRIGGERED"
            self._ts   = now

    def reset(self):
        self.state = "NONE"
        self._ts   = 0.0

    def update(self, dets, frame, now):
        """
        Returns (parking_state: str, speed_mult: float, steer_bias_deg: float).
        """
        if self.state == "NONE" or self.state == "DONE":
            return "NONE", 1.0, 0.0

        if self.state == "TRIGGERED":
            # Brief delay to slow down before seeking
            if now - self._ts > 0.3:
                self.state = "SEEK"
                self._ts   = now
            return "SEEK", 0.35, 0.0

        if self.state == "SEEK":
            right_clear = self._right_lane_clear(dets, frame)
            if right_clear or (now - self._ts > self.SEEK_TIMEOUT):
                self.state = "ENTER"
                self._ts   = now
            return "SEEK", 0.30, 0.0

        if self.state == "ENTER":
            if now - self._ts > self.ENTER_DURATION:
                self.state = "WAIT"
                self._ts   = now
            return "ENTER", 0.20, self.ENTER_STEER

        if self.state == "WAIT":
            if now - self._ts > self.WAIT_DURATION:
                self.state = "EXIT"
                self._ts   = now
            return "WAIT", 0.0, 0.0

        if self.state == "EXIT":
            if now - self._ts > self.EXIT_DURATION:
                self.state = "DONE"
                self._ts   = now
            return "EXIT", 0.28, self.EXIT_STEER

        # DONE: hand back control
        return "NONE", 1.0, 0.0

    @staticmethod
    def _right_lane_clear(dets, frame):
        """
        Returns True when the right side of the frame is clear of obstacles
        (indicating an available parking spot).
        """
        if frame is None:
            return True
        h, w = frame.shape[:2]
        right_x = int(w * 0.55)
        for d in dets:
            lbl = d["label"].lower()
            if lbl in ("car", "obstacle", "roadblock", "closed-road-stand"):
                x1, y1, x2, y2 = d["bbox"]
                if x1 > right_x and y2 > h * 0.35:
                    return False
        return True


# ══════════════════════════════════════════════════════════════════════════════
# Main decision engine
# ══════════════════════════════════════════════════════════════════════════════

class TrafficDecisionEngine:
    """
    Processes one frame per call and returns a TrafficResult.

    Sign / rule coverage
    --------------------
    Traffic Light : Red (stop) / Yellow (slow) / Green (go)
    Stop sign     : 3-second halt then cooldown
    Crosswalk     : slow; stop if pedestrian detected at crossing
    Priority      : skip intersection stop (right-of-way)
    Highway Entry : switch zone_mode -> HIGHWAY
    Highway Exit  : switch zone_mode -> CITY
    No-Entry      : commit SYS_STOP; planner should reroute
    One-Way       : informational log; routing handled by A* graph
    Roundabout    : slow on entry; routing handled by A* graph (CCW)
    Speed-limit   : reduce speed_multiplier to match sign value
    Parking       : trigger ParkingStateMachine
    Obstacle/car  : lane-change LEFT if on dashed segment (caller provides
                    line_type); just follow on continuous segment
    Pedestrian    : TTC-based emergency stop (CollisionPredictor)
                    + crosswalk-specific wait (PedestrianCrosswalkMonitor)
    """

    def __init__(self, threaded_detector):
        self.threaded_detector  = threaded_detector
        self.state              = "SYS_GO"
        self.reason             = "CLEAR"

        # Stop-sign FSM
        self.stop_timer  = 0.0   # time stop sign was first triggered
        self.stop_cd     = 0.0   # cooldown — don't re-trigger for 5 s

        # Priority: skip next stop-sign check
        self._priority_until = 0.0

        self.last_t  = time.time()

        self.tl_fsm     = TrafficLightStateMachine()
        self.col_pred   = CollisionPredictor()
        self.ped_xwalk  = PedestrianCrosswalkMonitor()
        self.parking_fsm = ParkingStateMachine()

        # Zone tracking
        self._zone_mode = "CITY"   # "CITY" | "HIGHWAY"

    # ── Internal helpers ─────────────────────────────────────────────────────

    def _is_glowing(self, frame, x1, y1, x2, y2):
        """
        Analyse the HSV colour mass inside a traffic-light bounding box.
        Returns ("RED"|"YELLOW"|"GREEN"|"NONE", pixel_mass).
        """
        fh, fw = frame.shape[:2]
        x1, y1 = max(0, x1), max(0, y1)
        x2, y2 = min(fw, x2), min(fh, y2)
        if y2 - y1 < 15 or x2 - x1 < 10:
            return "NONE", 0
        crop = frame[y1:y2, x1:x2]
        hsv  = cv2.cvtColor(crop, cv2.COLOR_BGR2HSV)

        # Red (wraps around hue wheel)
        mr1 = cv2.inRange(hsv, np.array([0,  50, 150]), np.array([10, 255, 255]))
        mr2 = cv2.inRange(hsv, np.array([170, 50, 150]), np.array([180,255,255]))
        # Yellow
        my  = cv2.inRange(hsv, np.array([15, 100, 150]), np.array([35, 255, 255]))
        # Green
        mg  = cv2.inRange(hsv, np.array([50,  50, 150]), np.array([90, 255, 255]))

        red    = cv2.countNonZero(cv2.bitwise_or(mr1, mr2))
        yellow = cv2.countNonZero(my)
        green  = cv2.countNonZero(mg)
        MIN_MASS = 30

        # Highest mass wins, with a minimum threshold
        best = max(red, yellow, green)
        if best < MIN_MASS:
            return "NONE", best
        if red >= yellow and red >= green:
            return "RED", red
        if yellow >= green:
            return "YELLOW", yellow
        return "GREEN", green

    def _dist_cat(self, box_h):
        """Classify distance from bounding-box height.
        FAR      : box_h < 30  px  — sign detected at long range, pre-decelerate
        APPROACH : 30–70 px      — sign coming up, sign-specific slow/stop fires
        HALT     : > 70 px       — right at sign, full action
        """
        if box_h < 30:
            return "FAR"
        elif box_h < 70:
            return "APPROACH"
        return "HALT"

    def _approx_dist_m(self, box_h):
        """Very rough distance estimate from bbox height (calibrate on track).
        Assumes a sign of real height ~0.30 m and focal length ~400 px.
        dist = (real_h * focal) / box_h  => 0.30*400 / box_h = 120 / box_h
        """
        return 120.0 / max(box_h, 1)

    # ── Main process call ─────────────────────────────────────────────────────

    def process(self, frame, line_type="UNKNOWN"):
        """
        Parameters
        ----------
        frame      : BGR camera frame (640×480).
        line_type  : "DASHED" | "CONTINUOUS" | "UNKNOWN"
                     Injected from map_planner so we know whether
                     overtaking is permitted.

        Returns
        -------
        TrafficResult
        """
        fh, fw = frame.shape[:2]
        dbg    = frame.copy()
        now    = time.time()
        dt     = max(now - self.last_t, 0.001)
        self.last_t = now

        self.threaded_detector.update_frame(frame)
        dets = self.threaded_detector.get_detections()

        # ── Priority queue helpers ────────────────────────────────────────────
        pri     = 99
        p_state = "SYS_GO"
        p_res   = "CLEAR PATH"

        def commit(pr, st, rs):
            nonlocal pri, p_state, p_res
            if pr < pri:
                pri, p_state, p_res = pr, st, rs

        # ── Collision predictor (TTC) ─────────────────────────────────────────
        crit = self.col_pred.update_and_predict(dets, dt)
        if crit:
            commit(1, "SYS_STOP", "COLLISION IMMINENT")

        # ── Stop-sign FSM: release after 5 s ─────────────────────────────────
        if self.stop_timer > 0.0:
            if now - self.stop_timer >= 5.0:
                self.stop_timer = 0.0
                self.stop_cd    = now + 15.0

        # ── Pedestrian at crosswalk ───────────────────────────────────────────
        ped_blocking, ped_reason = self.ped_xwalk.update(dets, fh, fw, now)
        if ped_blocking:
            commit(1, "SYS_STOP", ped_reason)

        light_st = "NONE"
        act_lbl  = []
        _nearest_sign_dist_m = 99.0   # for TrafficResult.sign_approach_m

        # ── Per-detection logic ───────────────────────────────────────────────
        for d in dets:
            lbl       = d["label"]
            lbl_lower = lbl.lower()
            x1, y1, x2, y2 = d["bbox"]
            box_h = y2 - y1
            dist_cat = self._dist_cat(box_h)

            # Estimate approach distance for dashboard
            approx_m = self._approx_dist_m(box_h)

            # Draw detection on debug frame
            cv2.rectangle(dbg, (x1, y1), (x2, y2), (0, 255, 0), 2)
            cv2.putText(dbg, f"{lbl} ~{approx_m:.1f}m", (x1, y1 - 5),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 0), 1)
            act_lbl.append(lbl)

            # ── Traffic light ─────────────────────────────────────────────────
            if "traffic" in lbl_lower and "light" in lbl_lower:
                clr, mass = self._is_glowing(frame, x1, y1, x2, y2)
                dist = self._dist_cat(box_h)
                fsm_st = self.tl_fsm.update(
                    clr == "RED", clr == "YELLOW", clr == "GREEN", dist
                )
                if fsm_st in ("LIGHT_RED_STOPPING", "LIGHT_RED_STOPPED"):
                    light_st = f"[RED] {dist}"
                    commit(1, "SYS_STOP", "RED LIGHT")
                elif fsm_st == "LIGHT_YELLOW_SLOW":
                    light_st = "[YELLOW] CAUTION"
                    commit(3, "SYS_SLOW", "YELLOW LIGHT")
                elif fsm_st == "LIGHT_GREEN_GO":
                    light_st = "[GREEN] GO"
                continue   # done with this detection

            # ── APPROACH PRE-DECELERATION ───────────────────────────────────────
            # Any sign detected at FAR range triggers gentle deceleration
            # (0.85×) so the car has time to slow before the sign action zone.
            # This is separate from sign-specific logic below.
            is_sign = not ("car" in lbl_lower or "pedestrian" in lbl_lower
                           or "person" in lbl_lower or "obstacle" in lbl_lower
                           or "roadblock" in lbl_lower)
            if is_sign and dist_cat == "FAR":
                commit(8, "SYS_APPROACH", f"APPROACHING {lbl}")
                _nearest_sign_dist_m = min(_nearest_sign_dist_m, approx_m)
                continue   # no further sign logic until APPROACH/HALT range

            # Track distance for dashboard
            if is_sign:
                _nearest_sign_dist_m = min(_nearest_sign_dist_m, approx_m)

            # Skip tiny detections for all non-light signs
            if box_h < 30:
                continue

            # ── Stop sign ─────────────────────────────────────────────────────
            if any(k in lbl_lower for k in ("stop-sign", "stop_sign",)) or lbl_lower == "stop":
                if approx_m < 2.5:
                    if now > self.stop_cd and now > self._priority_until:
                        if self.stop_timer == 0.0:
                            self.stop_timer = now
                        commit(2, "SYS_STOP", "STOP SIGN (5 s)")
                else:
                    commit(4, "SYS_SLOW", "STOP SIGN AHEAD")

            # ── Crosswalk sign ────────────────────────────────────────────────
            elif any(k in lbl_lower for k in ("crosswalk", "pedestrian_crossing",
                                               "zebra")):
                commit(4, "SYS_SLOW", "CROSSWALK AHEAD")

            # ── Priority / right-of-way ───────────────────────────────────────
            elif any(k in lbl_lower for k in ("priority", "right-of-way",
                                               "priority-road", "give-way")):
                # Right-of-way: no stop required at next intersection
                self._priority_until = now + 8.0
                # No commit needed — car continues at SYS_GO

            # ── Highway entry ─────────────────────────────────────────────────
            elif any(k in lbl_lower for k in ("highway-entry", "highway_entry",
                                               "highway_start", "motorway-entry")):
                self._zone_mode = "HIGHWAY"
                commit(5, "SYS_GO", "HIGHWAY MODE ON")

            # ── Highway exit ──────────────────────────────────────────────────
            elif any(k in lbl_lower for k in ("highway-exit", "highway_exit",
                                               "highway_end", "motorway-exit")):
                self._zone_mode = "CITY"
                commit(5, "SYS_GO", "HIGHWAY MODE OFF")

            # ── No-entry ─────────────────────────────────────────────────────
            elif any(k in lbl_lower for k in ("no-entry", "no_entry",
                                               "no entry", "do-not-enter")):
                commit(1, "SYS_STOP", "NO-ENTRY SIGN — REROUTE")

            # ── One-way (informational — routing handled by A* graph) ─────────
            elif any(k in lbl_lower for k in ("one-way", "oneway", "one_way")):
                pass   # A* graph edge directions already enforce one-way

            # ── Roundabout ────────────────────────────────────────────────────
            elif any(k in lbl_lower for k in ("roundabout",)):
                commit(4, "SYS_SLOW", "ROUNDABOUT ENTRY")

            # ── Speed-limit signs (BUG-09: exact class name match) ────────────
            elif lbl_lower in {
                "speed_30", "speed_50", "speed_80",
                "speed-limit-30", "speed-limit-50", "speed-limit-80",
                "speedlimit_30", "speedlimit_50", "speedlimit_80",
                "speed-limit", "speedlimit", "speed_limit",
            } or (
                lbl_lower.startswith(("speed_", "speed-limit-", "speedlimit_"))
                and any(ch.isdigit() for ch in lbl_lower)
            ):
                commit(4, "SYS_LIMIT", "SPEED LIMIT ZONE")


            # ── Parking sign ──────────────────────────────────────────────────
            elif any(k in lbl_lower for k in ("parking", "park-sign", "park_sign",
                                               "car-park")):
                self.parking_fsm.trigger(now)

            # ── Pedestrian on Road (Universal Fallback) ───────────────────────
            elif lbl_lower in ("pedestrian", "person"):
                # Stop if pedestrian's bounding box is wide and low enough to be in the lane
                in_path = (x1 < fw * 0.75 and x2 > fw * 0.25 and y2 > fh * 0.40)
                if in_path:
                    commit(1, "SYS_STOP", "PEDESTRIAN ON ROAD")

            # ── Static obstacles / other cars ────────────────────────────────
            elif lbl_lower in ("car", "closed-road-stand", "roadblock", "obstacle"):
                in_path = (x1 < fw * 0.80 and x2 > fw * 0.20 and y2 > fh * 0.60)
                if in_path:
                    if line_type == "DASHED":
                        # Dashed line → overtake allowed
                        commit(3, "SYS_LANE_CHANGE_LEFT", "OVERTAKING OBSTACLE")
                    elif line_type == "CONTINUOUS":
                        # Continuous line → must follow, not overtake
                        commit(3, "SYS_SLOW", "TAILING OBSTACLE (CONT LINE)")
                    else:
                        # Unknown: default to lane-change (conservative)
                        commit(3, "SYS_LANE_CHANGE_LEFT", "EVADING OBSTACLE")

        # ── Apply active stop-sign hold ───────────────────────────────────────
        if self.stop_timer > 0.0 and now - self.stop_timer < 5.0:
            commit(2, "SYS_STOP", "STOP SIGN (holding)")

        # ── Override: active red light beats everything else ──────────────────
        if "RED" in light_st:
            commit(1, "SYS_STOP", "RED LIGHT")

        # ── Parking FSM ───────────────────────────────────────────────────────
        park_state, park_speed_mult, park_steer = self.parking_fsm.update(
            dets, frame, now
        )
        if park_state not in ("NONE", "DONE"):
            if park_state == "WAIT":
                commit(1, "SYS_STOP", "PARKING — IN SPOT")
            else:
                commit(2, "SYS_SLOW", f"PARKING — {park_state}")

        self.state  = p_state
        self.reason = p_res

        # ── Speed multiplier ──────────────────────────────────────────────────
        mult = 1.0
        if self.state == "SYS_STOP":
            mult = 0.0
        elif self.state in ("SYS_SLOW", "SYS_LANE_CHANGE_LEFT"):
            mult = 0.55
        elif self.state == "SYS_APPROACH":
            mult = 0.85   # gentle pre-decel on sign approach
        elif self.state == "SYS_LIMIT":
            mult = 0.75

        # Parking FSM overrides multiplier
        if park_state not in ("NONE", "DONE"):
            mult = park_speed_mult

        return TrafficResult(
            state            = self.state,
            reason           = self.reason,
            speed_multiplier = mult,
            zone_mode        = self._zone_mode,
            parking_state    = park_state,
            steer_bias       = park_steer,
            pedestrian_blocking = ped_blocking,
            light_status     = light_st,
            active_labels    = act_lbl,
            yolo_debug_frame = dbg,
            sign_approach_m  = _nearest_sign_dist_m,
        )