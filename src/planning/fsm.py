from enum import Enum
import time


class State(Enum):
    LANE_FOLLOW = 1
    CAUTION = 2
    BLIND = 3
    OBSTACLE = 4
    STOP = 5


class FSM:
    def __init__(self):
        # Start conservatively
        self.state = State.CAUTION
        self.state_enter_time = time.time()

        # ================= TUNING =================

        # Lane confidence thresholds (with hysteresis)
        self.LANE_GOOD_ENTER = 0.85
        self.LANE_GOOD_EXIT  = 0.65
        self.LANE_LOST       = 0.15

        # Minimum stability times (seconds)
        self.LANE_STABLE_TIME = 0.5
        self.OBSTACLE_CLEAR_TIME = 1.0

        # Sign thresholds (optional in Phase-2)
        self.SIGN_FAR_THRESH = 0.05
        self.SIGN_CLOSE_THRESH = 0.35
        self.STOP_WAIT_TIME = 3.0
        self.SIGN_COOLDOWN = 5.0

        # ================= MEMORY =================
        self.sign_memory = 0.0
        self.ignore_sign_until = 0.0
        self.stop_reason = None

        self.last_lane_good_time = None
        self.last_obstacle_clear_time = None

    # ------------------------------------------------
    def transition(self, new_state: State):
        if new_state != self.state:
            print(f"[FSM] {self.state.name} -> {new_state.name}")
            self.state = new_state
            self.state_enter_time = time.time()

    # ------------------------------------------------
    def update(self, signals: dict) -> State:
        now = time.time()
        elapsed = now - self.state_enter_time

        # ================= INPUT SIGNALS =================
        lane_conf = signals.get("lane_confidence", 0.0)
        obstacle = signals.get("obstacle", False)
        raw_sign = signals.get("sign_intensity", 0.0)

        # ================= SIGN FILTER (OPTIONAL) =================
        if raw_sign > self.sign_memory:
            decay = 0.3
        else:
            decay = 0.8

        self.sign_memory = (
            self.sign_memory * decay +
            raw_sign * (1.0 - decay)
        )

        if now < self.ignore_sign_until:
            effective_sign = 0.0
        else:
            effective_sign = self.sign_memory

        # ================= LANE STABILITY TRACKING =================
        if lane_conf >= self.LANE_GOOD_ENTER:
            if self.last_lane_good_time is None:
                self.last_lane_good_time = now
        else:
            self.last_lane_good_time = None

        # ================= OBSTACLE CLEAR TRACKING =================
        if not obstacle:
            if self.last_obstacle_clear_time is None:
                self.last_obstacle_clear_time = now
        else:
            self.last_obstacle_clear_time = None

        # ================= FSM LOGIC =================

        # ---------- LANE FOLLOW ----------
        if self.state == State.LANE_FOLLOW:
            if obstacle:
                self.transition(State.OBSTACLE)
            elif effective_sign > self.SIGN_CLOSE_THRESH:
                self.stop_reason = "SIGN"
                self.transition(State.STOP)
            elif lane_conf < self.LANE_GOOD_EXIT:
                self.transition(State.CAUTION)

        # ---------- CAUTION ----------
        elif self.state == State.CAUTION:
            if obstacle:
                self.transition(State.OBSTACLE)
            elif effective_sign > self.SIGN_CLOSE_THRESH:
                self.stop_reason = "SIGN"
                self.transition(State.STOP)
            elif lane_conf < self.LANE_LOST:
                self.transition(State.BLIND)
            elif (
                self.last_lane_good_time is not None and
                (now - self.last_lane_good_time) >= self.LANE_STABLE_TIME
            ):
                self.transition(State.LANE_FOLLOW)

        # ---------- BLIND ----------
        elif self.state == State.BLIND:
            # No STOP here; STM32 watchdog handles safety
            if lane_conf >= self.LANE_GOOD_EXIT:
                self.transition(State.CAUTION)

        # ---------- OBSTACLE ----------
        elif self.state == State.OBSTACLE:
            if (
                not obstacle and
                self.last_obstacle_clear_time is not None and
                (now - self.last_obstacle_clear_time) >= self.OBSTACLE_CLEAR_TIME
            ):
                self.transition(State.CAUTION)

        # ---------- STOP (SIGN ONLY) ----------
        elif self.state == State.STOP:
            if self.stop_reason == "SIGN" and elapsed >= self.STOP_WAIT_TIME:
                print("[FSM] Stop complete, resuming")
                self.ignore_sign_until = now + self.SIGN_COOLDOWN
                self.sign_memory = 0.0
                self.transition(State.CAUTION)

        return self.state
