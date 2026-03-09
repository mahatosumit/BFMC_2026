import math
import numpy as np
from dataclasses import dataclass

@dataclass
class ControlOutput:
    steer_angle_deg: float
    speed_pwm:       float
    target_x:        float
    anchor:          str
    lookahead_px:    float
    steer_ff_deg:    float = 0.0
    steer_react_deg: float = 0.0

class StanleyController:
    def __init__(self, k: float = 1.5, ks: float = 0.5, wheelbase_m: float = 0.23):
        self.k  = k
        self.ks = ks
        self.L  = wheelbase_m

    def compute(self, target_x_px: float, heading_rad: float,
                velocity_ms: float, lane_width_px: float,
                map_curvature: float = 0.0):
        ppm  = max(lane_width_px, 50) / 0.35
        # FATAL BUG FIX: Target - Center ensures that Target on Right (>320) yields Positive error.
        # Positive error yields Positive steering angle (Right turn). 
        # This aligns with the DividerGuard math which adds positive values to steer Right.
        ce_m = (target_x_px - 320.0) / ppm

        k_eff = self.k * min(1.0, velocity_ms / 0.25) if velocity_ms > 0 else self.k
        reactive_rad = heading_rad + math.atan2(k_eff * ce_m, velocity_ms + self.ks)
        feed_forward_rad = math.atan(self.L * map_curvature)

        total_deg    = math.degrees(reactive_rad + feed_forward_rad)
        reactive_deg = math.degrees(reactive_rad)
        ff_deg       = math.degrees(feed_forward_rad)
        return total_deg, reactive_deg, ff_deg

class DividerGuard:
    DIVIDER_SAFE_PX = 130
    EDGE_SAFE_PX    = 100
    GAIN            = 0.35
    MAX_CORR        = 25.0
    DEADBAND_PX     =  2

    def apply(self, steer_angle, left_fit, right_fit, y_eval=440, car_x=320):
        correction, speed_scale, triggered = 0.0, 1.0, False
        div_corr = edge_corr = 0.0

        if left_fit is not None:
            div_x = float(np.polyval(left_fit, y_eval))
            gap   = car_x - div_x
            if gap < self.DIVIDER_SAFE_PX - self.DEADBAND_PX:
                err      = float(self.DIVIDER_SAFE_PX - gap)
                div_corr = min((self.GAIN * 3.0) * err, self.MAX_CORR)
                speed_scale = min(speed_scale, max(0.2, 1.0 - err / 60.0))
                triggered   = True

        if right_fit is not None:
            edge_x = float(np.polyval(right_fit, y_eval))
            gap    = edge_x - car_x
            if gap < self.EDGE_SAFE_PX - self.DEADBAND_PX:
                err       = float(self.EDGE_SAFE_PX - gap)
                edge_corr = min(self.GAIN * err, self.MAX_CORR * 0.4)
                speed_scale = min(speed_scale, max(0.5, 1.0 - err / 100.0))
                triggered   = True

        if div_corr > 0 and edge_corr > 0:
            correction = max(div_corr - edge_corr, self.DEADBAND_PX * self.GAIN)
        else:
            correction = div_corr - edge_corr

        return steer_angle + correction, speed_scale, triggered

class Controller:
    MAX_STEER      = 30.0
    MAX_STEER_RATE = 60.0
    BRAKING_DISTANCE_M = 1.8
    MIN_CURVE_SPEED_F  = 0.45

    def __init__(self):
        self.prev_steer = 0.0
        self.guard      = DividerGuard()
        self.stanley    = StanleyController(k=3.5, ks=0.05, wheelbase_m=0.23)

    def compute(self, perc_res,
                nav_state:      str   = "NORMAL",
                velocity_ms:    float = 0.0,
                dt:             float = 0.033,
                base_speed:     float = 50.0,
                traffic_mult:   float = 1.0,
                map_curvature:  float = 0.0,
                upcoming_curve: str   = "STRAIGHT",
                curve_dist_m:   float = 99.0) -> ControlOutput:

        raw_steer, react_steer_deg, ff_steer_deg = self.stanley.compute(
            perc_res.target_x, perc_res.heading_rad,
            velocity_ms, perc_res.lane_width_px,
            map_curvature=map_curvature)

        rate_delta  = max(-self.MAX_STEER_RATE,
                  min(self.MAX_STEER_RATE, raw_steer - self.prev_steer))
        steer_angle = self.prev_steer + rate_delta

        # Disable mechanical steering lag for maximum aggressiveness
        alpha = 0.0
        steer_angle = alpha * self.prev_steer + (1 - alpha) * steer_angle
        self.prev_steer = steer_angle

        steer_guarded, guard_spd_mult, _ = self.guard.apply(
            steer_angle, perc_res.sl, perc_res.sr, y_eval=perc_res.y_eval)
        steer_angle = max(-self.MAX_STEER, min(self.MAX_STEER, steer_guarded))

        speed           = float(base_speed)
        min_curve_speed = base_speed * self.MIN_CURVE_SPEED_F

        if nav_state == "ROUNDABOUT":
            speed = min(speed, base_speed * 0.50)

        if upcoming_curve != "STRAIGHT" and curve_dist_m < self.BRAKING_DISTANCE_M:
            decel_factor = max(0.0, curve_dist_m / self.BRAKING_DISTANCE_M)
            braked_speed = min_curve_speed + (base_speed - min_curve_speed) * decel_factor
            speed = min(speed, braked_speed)
        elif abs(steer_angle) < 5:
            speed = min(speed * 1.15, base_speed * 1.20)

        if "DEAD_RECKONING" in perc_res.anchor:
            try:
                dr_conf = float(perc_res.anchor.split("_")[2])
            except Exception:
                dr_conf = 0.5
            speed *= (0.4 + 0.4 * dr_conf)

        if perc_res.anchor == "DIVIDER_FOLLOW":
            speed *= 0.75

        final_speed = speed * traffic_mult * guard_spd_mult

        MINIMUM_DRIVE_PWM = 18.0
        if nav_state not in ("SYS_STOP", "STOPPED") and final_speed > 0:
            final_speed = max(final_speed, MINIMUM_DRIVE_PWM)

        return ControlOutput(
            steer_angle_deg = steer_angle,
            speed_pwm       = final_speed,
            target_x        = perc_res.target_x,
            anchor          = perc_res.anchor,
            lookahead_px    = 0.0,
            steer_ff_deg    = ff_steer_deg,
            steer_react_deg = react_steer_deg,
        )
