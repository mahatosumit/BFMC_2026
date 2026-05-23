"""
behavior_controller.py — BFMC Priority-Based Reactive Controller
================================================================
Translates Visual Detection Inputs into Motor / Steering Commands.

Input Surface
-------------
  perc_res   : PerceptionResult  (from perception.py)
  t_res      : TrafficResult     (from traffic_module.py)

Output
------
  BehaviorOutput dataclass  →  speed_pwm, steer_deg, state_str, reason

Priority Hierarchy  (lower number = higher priority)
------------------------------------------------------
  0  EMERGENCY — pedestrian detected on road
  1  MANDATORY — RED light · STOP sign (3 s non-blocking halt)
  2  LEGAL     — No-Entry · Bus-Lane virtual wall
  3  MISSION   — Roundabout CCW · Parking FSM · Highway mode
  4  NORMAL    — default right-lane city driving

Sign Approach Logic
-------------------
  When any sign is detected with sign_approach_m > APPROACH_THRESH_M,
  speed is smoothly reduced BEFORE the sign action zone so the car
  decelerates gracefully rather than braking at the last moment.
"""

import time
import math
import logging
from dataclasses import dataclass, field
from typing import Optional

log = logging.getLogger(__name__)


# ══════════════════════════════════════════════════════════════════════════════
# Output contract
# ══════════════════════════════════════════════════════════════════════════════

@dataclass
class BehaviorOutput:
    """Single-frame output of BehaviorController.compute()."""
    speed_pwm   : float        # 0 = stopped, positive = forward
    steer_deg   : float        # negative = left, positive = right
    priority    : int          # which priority level fired (0-4)
    state       : str          # human-readable state label
    reason      : str          # why this state was chosen
    zone_mode   : str = "CITY" # "CITY" | "HIGHWAY"
    maneuver    : str = "NONE" # "NONE" | "OVERTAKE" | "PARKING" | "ROUNDABOUT"


# ══════════════════════════════════════════════════════════════════════════════
# Overtake state machine
# ══════════════════════════════════════════════════════════════════════════════

class OvertakeStateMachine:
    """
    Dashed-line obstacle overtake sequence:
      IDLE → CHANGE_LEFT → PASS → CHANGE_RIGHT → IDLE

    Timing is open-loop (duration-based). Steer biases are additive
    on top of the lane-following steering from perception.
    """
    CHANGE_DURATION = 1.5   # seconds for each lane-change segment
    PASS_DURATION   = 2.0   # seconds to pass the obstacle
    STEER_BIAS_DEG  = 12.0  # extra steer angle during lane-change phases
    SPEED_MULT      = 0.70  # slow slightly during maneuver

    def __init__(self):
        self.state  = "IDLE"
        self._ts    = 0.0

    @property
    def active(self):
        return self.state != "IDLE"

    def trigger(self, now: float):
        if self.state == "IDLE":
            self.state = "CHANGE_LEFT"
            self._ts   = now
            log.info("OVERTAKE: starting lane-change left")

    def update(self, now: float, base_steer: float, base_speed: float):
        """Returns (steer_deg, speed_pwm, maneuver_label)."""
        if self.state == "IDLE":
            return base_steer, base_speed, "NONE"

        elapsed = now - self._ts

        if self.state == "CHANGE_LEFT":
            if elapsed > self.CHANGE_DURATION:
                self.state = "PASS"
                self._ts   = now
            return base_steer - self.STEER_BIAS_DEG, base_speed * self.SPEED_MULT, "OVERTAKE"

        if self.state == "PASS":
            if elapsed > self.PASS_DURATION:
                self.state = "CHANGE_RIGHT"
                self._ts   = now
            return base_steer, base_speed * self.SPEED_MULT, "OVERTAKE"

        if self.state == "CHANGE_RIGHT":
            if elapsed > self.CHANGE_DURATION:
                self.state = "IDLE"
                log.info("OVERTAKE: complete, back in right lane")
            return base_steer + self.STEER_BIAS_DEG, base_speed * self.SPEED_MULT, "OVERTAKE"

        return base_steer, base_speed, "NONE"


# ══════════════════════════════════════════════════════════════════════════════
# Parking state machine  (full parallel-parking sequence)
# ══════════════════════════════════════════════════════════════════════════════

class ParkingSequenceFSM:
    """
    Full parallel-parking sequence:
      IDLE → SEEK → ENTER → WAIT → EXIT → DONE → IDLE

    All timings use time.time() — no time.sleep() used anywhere.
    """
    SEEK_TIMEOUT   = 6.0     # give up seeking after 6 s
    SLOW_SPEED     = 0.30    # speed multiplier while seeking
    ENTER_DURATION = 2.0
    ENTER_STEER    = 22.0    # right steer into spot
    WAIT_DURATION  = 3.0     # mandatory stop in spot
    EXIT_DURATION  = 2.5
    EXIT_STEER     = -18.0   # left steer to pull out

    def __init__(self):
        self.state = "IDLE"
        self._ts   = 0.0

    @property
    def active(self):
        return self.state not in ("IDLE", "DONE")

    def trigger(self, now: float):
        if self.state == "IDLE":
            self.state = "SEEK"
            self._ts   = now
            log.info("PARKING: seek started")

    def reset(self):
        self.state = "IDLE"

    def update(self, now: float, base_speed: float, spot_clear: bool = True):
        """
        Returns (speed_pwm_mult, steer_bias_deg, state_str).
        Caller multiplies their base speed by speed_pwm_mult.
        """
        if self.state in ("IDLE", "DONE"):
            return 1.0, 0.0, "NONE"

        elapsed = now - self._ts

        if self.state == "SEEK":
            if spot_clear or elapsed > self.SEEK_TIMEOUT:
                self.state = "ENTER"
                self._ts   = now
                log.info("PARKING: entering spot")
            return self.SLOW_SPEED, 0.0, "SEEK"

        if self.state == "ENTER":
            if elapsed > self.ENTER_DURATION:
                self.state = "WAIT"
                self._ts   = now
                log.info("PARKING: in spot — waiting %.1fs", self.WAIT_DURATION)
            return 0.20, self.ENTER_STEER, "ENTER"

        if self.state == "WAIT":
            if elapsed > self.WAIT_DURATION:
                self.state = "EXIT"
                self._ts   = now
                log.info("PARKING: exiting spot")
            return 0.0, 0.0, "WAIT"

        if self.state == "EXIT":
            if elapsed > self.EXIT_DURATION:
                self.state = "DONE"
                log.info("PARKING: done, handing back to lane follow")
            return 0.28, self.EXIT_STEER, "EXIT"

        return 1.0, 0.0, "IDLE"


# ══════════════════════════════════════════════════════════════════════════════
# Main BehaviorController
# ══════════════════════════════════════════════════════════════════════════════

class BehaviorController:
    """
    Priority-Based Reactive Controller.

    Usage (in main._pilot_loop):

        # Instantiate once
        self.behavior = BehaviorController()

        # Each loop tick
        out = self.behavior.compute(perc_res, t_res, dt,
                                    base_steer=ctrl.steer_angle_deg)
        self.hw.set_speed(out.speed_pwm)
        self.hw.set_steering(out.steer_deg)
    """

    # ── Priority constants ───────────────────────────────────────────────────
    PRI_EMERGENCY = 0
    PRI_MANDATORY = 1
    PRI_LEGAL     = 2
    PRI_MISSION   = 3
    PRI_NORMAL    = 4

    # ── Speed constants (PWM units, tunable) ────────────────────────────────
    CITY_SPEED_PWM     = 22.0   # ~20 cm/s at nominal calibration
    HIGHWAY_SPEED_PWM  = 26.4   # ~40 cm/s
    APPROACH_SPEED_PWM = 16.0   # sign-approach decel floor
    SLOW_SPEED_PWM     = 14.0   # near crosswalk / roundabout entry
    MIN_SPEED_PWM      = 18.0   # absolute floor (prevents stall)

    # ── Sign approach deceleration ───────────────────────────────────────────
    # Car begins decelerating when a sign is closer than this distance.
    APPROACH_DECEL_M   = 2.5    # metres — start slowing
    APPROACH_FULL_M    = 0.8    # metres — reach APPROACH_SPEED_PWM

    # ── Mandatory STOP duration ──────────────────────────────────────────────
    STOP_SIGN_HOLD_S   = 5.0
    STOP_SIGN_COOLDOWN = 15.0    # don't re-trigger for 15 s after release

    # ── Bus-lane virtual wall ────────────────────────────────────────────────
    BUS_LANE_STEER_CORRECTION = -8.0   # deg correction to stay out of bus lane

    def __init__(self):
        self._zone_mode        : str   = "CITY"
        self._stop_timer       : float = 0.0    # time stop sign was first seen
        self._stop_cooldown    : float = 0.0    # time stop sign cooldown expires
        self._priority_until   : float = 0.0    # priority-road right-of-way expiry
        self._roundabout_active: bool  = False
        self._no_entry_active  : bool  = False
        self.overtake_fsm   = OvertakeStateMachine()
        self.parking_fsm    = ParkingSequenceFSM()
        self._last_state    = "NORMAL"

    # ── Public API ───────────────────────────────────────────────────────────

    def compute(self,
                perc_res,
                t_res,
                dt: float,
                base_speed: float = 22.0,
                base_steer: float = 0.0) -> BehaviorOutput:
        """
        Evaluate all priority layers and return the highest-priority command.

        Parameters
        ----------
        perc_res   : PerceptionResult from perception.py
        t_res      : TrafficResult from traffic_module.py
        dt         : elapsed seconds since last call
        base_steer : steering angle (deg) already computed by StanleyController
        """
        now = time.time()

        # Update zone from traffic module (highway detection)
        self._update_zone(t_res, now)

        # Compute base speed for this zone
        zone_speed = (base_speed * (self.HIGHWAY_SPEED_PWM / self.CITY_SPEED_PWM)
                      if self._zone_mode == "HIGHWAY"
                      else base_speed)

        # ── Apply sign-approach deceleration BEFORE priority checks ──────────
        approach_mult = self._sign_approach_mult(t_res)
        zone_speed   *= approach_mult

        # ── Priority 0: EMERGENCY (pedestrian on road) ───────────────────────
        em_out = self._check_emergency(t_res, base_steer)
        if em_out:
            return em_out

        # ── Priority 1: MANDATORY (red light / STOP sign) ───────────────────
        mand_out = self._check_mandatory(t_res, now, base_steer)
        if mand_out:
            return mand_out

        # ── Priority 2: LEGAL (no-entry, bus lane) ───────────────────────────
        legal_out = self._check_legal(t_res, base_steer)
        if legal_out:
            return legal_out

        # ── Priority 3: MISSION (parking, roundabout, highway) ──────────────
        mission_out = self._check_mission(t_res, perc_res, now,
                                          zone_speed, base_steer)
        if mission_out:
            return mission_out

        # ── Priority 4: NORMAL lane-following ────────────────────────────────
        return self._normal_drive(t_res, perc_res, now,
                                  zone_speed, base_steer)

    # ── Priority 0: Emergency ────────────────────────────────────────────────

    def _check_emergency(self, t_res, base_steer: float) -> Optional[BehaviorOutput]:
        """Pedestrian on road or collision → immediate stop regardless of anything else."""
        is_emergency = t_res.pedestrian_blocking or "PEDESTRIAN" in t_res.reason.upper() or "COLLISION" in t_res.reason.upper()
        
        if not is_emergency:
            return None
            
        reason_str = t_res.reason if t_res.reason else "EMERGENCY HAZARD"
        if t_res.pedestrian_blocking and not ("PEDESTRIAN" in reason_str.upper() or "COLLISION" in reason_str.upper()):
            reason_str = "PEDESTRIAN AT CROSSWALK"
            
        return BehaviorOutput(
            speed_pwm = 0.0,
            steer_deg = base_steer,
            priority  = self.PRI_EMERGENCY,
            state     = "EMERGENCY_STOP",
            reason    = reason_str,
        )

    # ── Priority 1: Mandatory ────────────────────────────────────────────────

    def _check_mandatory(self, t_res, now: float,
                         base_steer: float) -> Optional[BehaviorOutput]:
        """
        RED light or STOP sign → 3-second non-blocking halt.
        Uses time.time() comparisons; no time.sleep().
        """
        is_red_light = (t_res.light_status is not None and
                        "RED" in t_res.light_status)
        is_stop_sign = (t_res.state == "SYS_STOP" and
                        "STOP SIGN" in t_res.reason)

        # STOP sign FSM: start timer on first detection
        if is_stop_sign and now > self._stop_cooldown:
            if now > self._priority_until:  # priority-road overrides stop
                if self._stop_timer == 0.0:
                    self._stop_timer = now
                    log.info("MANDATORY: STOP sign triggered — 3 s halt")

        # Hold for 3 seconds then release with cooldown
        if self._stop_timer > 0.0:
            held = now - self._stop_timer
            if held < self.STOP_SIGN_HOLD_S:
                return BehaviorOutput(
                    speed_pwm = 0.0,
                    steer_deg = base_steer,
                    priority  = self.PRI_MANDATORY,
                    state     = "STOP_SIGN_HOLD",
                    reason    = f"STOP SIGN — {held:.1f}/{self.STOP_SIGN_HOLD_S:.0f}s",
                )
            else:
                self._stop_timer   = 0.0
                self._stop_cooldown = now + self.STOP_SIGN_COOLDOWN
                log.info("MANDATORY: STOP sign released")

        # Red light (no timer — stays stopped while light is RED)
        if is_red_light:
            return BehaviorOutput(
                speed_pwm = 0.0,
                steer_deg = base_steer,
                priority  = self.PRI_MANDATORY,
                state     = "RED_LIGHT_STOP",
                reason    = t_res.light_status,
            )

        return None

    # ── Priority 2: Legal ────────────────────────────────────────────────────

    def _check_legal(self, t_res, base_steer: float) -> Optional[BehaviorOutput]:
        """
        No-Entry: refuse to proceed.
        Bus Lane: apply virtual-wall steer correction to stay out.
        """
        if "NO-ENTRY" in t_res.reason.upper() or "NO_ENTRY" in t_res.reason.upper():
            self._no_entry_active = True
            log.warning("LEGAL: No-Entry sign — refusing path")
            return BehaviorOutput(
                speed_pwm = 0.0,
                steer_deg = base_steer,
                priority  = self.PRI_LEGAL,
                state     = "NO_ENTRY",
                reason    = "NO-ENTRY SIGN — PATH REFUSED",
            )
        else:
            self._no_entry_active = False

        # Bus lane: apply steer correction only — don't stop
        if "BUS" in " ".join(t_res.active_labels).upper():
            corrected_steer = base_steer + self.BUS_LANE_STEER_CORRECTION
            return BehaviorOutput(
                speed_pwm = base_speed * 0.80,
                steer_deg = corrected_steer,
                priority  = self.PRI_LEGAL,
                state     = "BUS_LANE_AVOID",
                reason    = "BUS LANE — virtual wall active",
            )

        return None

    # ── Priority 3: Mission ──────────────────────────────────────────────────

    def _check_mission(self, t_res, perc_res, now: float,
                       base_speed: float,
                       base_steer: float) -> Optional[BehaviorOutput]:
        """
        Handles roundabout CCW navigation, parking FSM, and overtake.
        """
        active_lower = " ".join(t_res.active_labels).lower()

        # ── Roundabout ───────────────────────────────────────────────────
        if "roundabout" in active_lower or self._roundabout_active:
            self._roundabout_active = True
            # CCW: bias steer left and slow down at entry
            ccw_steer = base_steer - 8.0
            out = BehaviorOutput(
                speed_pwm = base_speed * 0.65,
                steer_deg = ccw_steer,
                priority  = self.PRI_MISSION,
                state     = "ROUNDABOUT_CCW",
                reason    = "ROUNDABOUT — CCW navigation",
                maneuver  = "ROUNDABOUT",
            )
            # Exit roundabout when no sign seen for > 4 s (use stale detection)
            if "roundabout" not in active_lower:
                self._roundabout_active = False
            return out

        # ── Parking ──────────────────────────────────────────────────────
        parking_sign = any(k in active_lower
                           for k in ("parking", "park-sign", "park_sign"))
        if parking_sign and not self.parking_fsm.active:
            self.parking_fsm.trigger(now)

        if self.parking_fsm.active:
            # Determine if right side of road is clear (simple heuristic)
            spot_clear = True  # TrafficResult.parking_state drives this in t_res
            if t_res.parking_state in ("SEEK",):
                spot_clear = (t_res.parking_state != "SEEK")
            speed_mult, steer_bias, park_label = self.parking_fsm.update(
                now, base_speed, spot_clear=spot_clear
            )
            if park_label == "DONE":
                self.parking_fsm.reset()
                return None
            return BehaviorOutput(
                speed_pwm = base_speed * speed_mult,
                steer_deg = base_steer + steer_bias,
                priority  = self.PRI_MISSION,
                state     = f"PARKING_{park_label}",
                reason    = f"PARKING — phase: {park_label}",
                maneuver  = "PARKING",
            )

        # ── Overtake (dashed line + obstacle) ───────────────────────────
        if (t_res.state == "SYS_LANE_CHANGE_LEFT" and
                not self.overtake_fsm.active):
            # Confirmed dashed line + obstacle in path
            line_type = getattr(perc_res, 'lane_type', 'DASHED')
            if line_type != "CONTINUOUS":
                self.overtake_fsm.trigger(now)

        if self.overtake_fsm.active:
            steer, speed, label = self.overtake_fsm.update(
                now, base_steer, base_speed
            )
            return BehaviorOutput(
                speed_pwm = speed,
                steer_deg = steer,
                priority  = self.PRI_MISSION,
                state     = "OVERTAKE",
                reason    = "DASHED LINE — overtaking obstacle",
                maneuver  = "OVERTAKE",
            )

        return None

    # ── Priority 4: Normal ───────────────────────────────────────────────────

    def _normal_drive(self, t_res, perc_res, now: float,
                      base_speed: float,
                      base_steer: float) -> BehaviorOutput:
        """
        Default right-lane driving with dynamic speed and lane rules.
        """
        speed   = base_speed
        steer   = base_steer
        reason  = "NORMAL DRIVE"
        state   = "RL_DRIVE"

        # Continuous line + obstacle in path → TAILING MODE
        if (t_res.state == "SYS_SLOW" and
                "TAILING" in t_res.reason.upper()):
            speed  *= 0.55
            state   = "TAILING"
            reason  = "CONTINUOUS LINE — tailing obstacle"

        # Crosswalk: slow down
        elif "CROSSWALK" in t_res.reason.upper():
            speed = min(speed, base_speed * 0.6)
            state  = "CROSSWALK_SLOW"
            reason = "CROSSWALK AHEAD — slowing"

        # Parking sign (before triggered): slow to scan
        elif any(k in " ".join(t_res.active_labels).lower()
                 for k in ("parking", "park-sign")):
            speed = min(speed, base_speed * 0.6)
            state  = "PARKING_SCAN"
            reason = "PARKING SIGN — scanning for spot"

        # Yellow light: slow
        elif "YELLOW" in (t_res.light_status or ""):
            speed *= 0.65
            state  = "YELLOW_SLOW"
            reason = "YELLOW LIGHT — prepare to stop"

        # Speed-limit zone
        elif t_res.state == "SYS_LIMIT":
            speed *= 0.75
            state  = "SPEED_LIMIT"
            reason = "SPEED LIMIT ZONE"

        # Enforce minimum speed floor (prevent stall from compounded reductions)
        if 0 < speed < self.MIN_SPEED_PWM:
            speed = self.MIN_SPEED_PWM

        return BehaviorOutput(
            speed_pwm = speed,
            steer_deg = steer,
            priority  = self.PRI_NORMAL,
            state     = state,
            reason    = reason,
            zone_mode = self._zone_mode,
        )

    # ── Helpers ──────────────────────────────────────────────────────────────

    def _update_zone(self, t_res, now: float):
        """
        Update zone mode from TrafficResult.
        Highway Entry sign → HIGHWAY.
        Highway Exit sign  → CITY.
        Must be within 3.0m to trigger to avoid random detections.
        """
        active_lower = " ".join(t_res.active_labels).lower()
        dist = getattr(t_res, 'sign_approach_m', 99.0)
        
        if dist < 3.0:
            if any(k in active_lower for k in ("highway-entry", "highway_entry",
                                                "highway_start")):
                if self._zone_mode != "HIGHWAY":
                    log.info("ZONE: → HIGHWAY (speed limit raised)")
                self._zone_mode = "HIGHWAY"
            elif any(k in active_lower for k in ("highway-exit", "highway_exit",
                                                  "highway_end")):
                if self._zone_mode != "CITY":
                    log.info("ZONE: → CITY")
                self._zone_mode = "CITY"

    def _sign_approach_mult(self, t_res) -> float:
        """
        Smooth approach deceleration multiplier based on estimated distance
        to nearest detected sign.

        At APPROACH_DECEL_M metres away → start ramping from 1.0
        At APPROACH_FULL_M metres away  → floor at APPROACH_SPEED_PWM / base

        Returns a multiplier in [0.60, 1.0].
        """
        dist = getattr(t_res, 'sign_approach_m', 99.0)

        if dist >= self.APPROACH_DECEL_M:
            return 1.0  # too far — no decel yet

        if dist <= self.APPROACH_FULL_M:
            return 0.60  # right at sign — 60% speed floor

        # Linear ramp between APPROACH_DECEL_M and APPROACH_FULL_M
        ratio = ((dist - self.APPROACH_FULL_M) /
                 (self.APPROACH_DECEL_M - self.APPROACH_FULL_M))
        return 0.60 + 0.40 * ratio   # interpolates 0.60 → 1.0

    def set_priority_road(self, duration_s: float = 8.0):
        """Called externally when a priority/right-of-way sign is detected."""
        self._priority_until = time.time() + duration_s
        log.info("LEGAL: priority road active for %.1f s", duration_s)

    @property
    def zone_mode(self) -> str:
        return self._zone_mode
