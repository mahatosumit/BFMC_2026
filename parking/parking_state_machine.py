import time
from .parking_config import (
    SIGN_CONFIRM_FRAMES, SCAN_TIMEOUT_S,
    ALIGN_COAST_S, REVERSE_TIMEOUT
)

# ── State labels (for logging / dashboard) ───────────────────
STATE_NAMES = {
    0: "IDLE",
    1: "SIGN_DETECTED",
    2: "SCANNING",
    3: "SLOT_FOUND_COASTING",
    4: "STOPPING",
    5: "LOAD_TRAJECTORY",
    6: "EXECUTING",
    7: "COMPLETE",
    8: "FAILED",
}

class ParkingStateMachine:
    def __init__(self):
        self._reset_state()

    def _reset_state(self):
        self.state = 0
        self.sign_detection_count  = 0
        self.scan_start_time       = 0.0
        self.align_start_time      = 0.0
        self.exec_start_time       = 0.0
        self.trajectory_started    = False  # guards 1-frame race in state 6
        self.parking_completed     = False
        self.parking_failed        = False
        # kept for dashboard compatibility
        self.target_stop_distance_cm       = 0.0
        self.reverse_parking_start_time    = 0.0

    def get_state(self):
        return self.state

    def get_state_name(self):
        return STATE_NAMES.get(self.state, "UNKNOWN")

    def reset(self):
        self._reset_state()

    # ─────────────────────────────────────────────────────────
    def transition(self,
                   sign_detected=False,
                   vision_slot_found=False,
                   reverse_parking_done=False):
        """
        Single-responsibility FSM.  Each state handles exactly one condition.

        Parameters
        ----------
        sign_detected       : bool  – parking sign visible in this frame
        vision_slot_found   : bool  – SlotManager confirmed free gap
        reverse_parking_done: bool  – main.py signals playback finished
        """

        # ── State 0: IDLE ────────────────────────────────────
        if self.state == 0:
            if sign_detected:
                self.sign_detection_count += 1
                if self.sign_detection_count >= SIGN_CONFIRM_FRAMES:
                    print("[Parking] Sign confirmed → starting scan")
                    self.sign_detection_count = 0
                    self.state = 1
            else:
                self.sign_detection_count = 0

        # ── State 1: SIGN_DETECTED → arm scanner ─────────────
        elif self.state == 1:
            self.scan_start_time = time.time()
            print("[Parking] Scan armed")
            self.state = 2

        # ── State 2: SCANNING ────────────────────────────────
        elif self.state == 2:
            if vision_slot_found:
                self.align_start_time = time.time()
                print(f"[Parking] Coasting {ALIGN_COAST_S:.2f}s to align")
                self.state = 3
            elif time.time() - self.scan_start_time > SCAN_TIMEOUT_S:
                print("[Parking] Scan timeout — no free slot")
                self.parking_failed = True
                self.state = 8

        # ── State 3: SLOT_FOUND_COASTING ─────────────────────
        # Car coasts briefly so it is fully aligned with the empty slot
        elif self.state == 3:
            if time.time() - self.align_start_time >= ALIGN_COAST_S:
                self.state = 4

        # ── State 4: STOPPING ────────────────────────────────
        # speed_multiplier=0.0 for one frame while we load the trajectory
        elif self.state == 4:
            self.state = 5

        # ── State 5: LOAD_TRAJECTORY ─────────────────────────
        # parking.py loads the CSV here (post-transition hook)
        elif self.state == 5:
            self.exec_start_time = time.time()
            self.reverse_parking_start_time = time.time()  # dashboard compat
            self.trajectory_started = False
            print("[Parking] Trajectory loaded — executing maneuver")
            self.state = 6

        # ── State 6: EXECUTING ───────────────────────────────
        elif self.state == 6:
            if not self.trajectory_started:
                # Skip the first frame so main.py has set is_playing_back=True
                self.trajectory_started = True
            elif reverse_parking_done:
                print("[Parking] Maneuver complete")
                self.state = 7
            elif time.time() - self.exec_start_time > REVERSE_TIMEOUT:
                print("[Parking] Execution timeout")
                self.parking_failed = True
                self.state = 8

        # ── State 7: COMPLETE ────────────────────────────────
        elif self.state == 7:
            self.parking_completed = True

        # ── State 8: FAILED ──────────────────────────────────
        # stays here; main.py can reset by toggling auto mode
