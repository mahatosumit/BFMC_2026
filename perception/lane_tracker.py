import numpy as np
import cv2

class DeadReckoningNavigator:
    def __init__(self):
        self.last_valid_target    = 320.0
        self.last_valid_curvature = 0.0
        self._lost_time_s         = 0.0
        self.yaw_at_loss          = 0.0
        self.is_lost              = False

    def reset_lost_timer(self, current_yaw: float):
        self._lost_time_s = 0.0
        self.yaw_at_loss  = current_yaw
        self.is_lost      = False

    def accumulate(self, dt: float, current_yaw: float):
        if not self.is_lost:
            self.yaw_at_loss = current_yaw
            self.is_lost = True
        self._lost_time_s += dt

    def predict_target(self, last_speed, last_steering, current_yaw):
        t = max(0.0, self._lost_time_s)
        delta_yaw_deg = current_yaw - self.yaw_at_loss

        if abs(self.last_valid_curvature) > 0.0015 or abs(last_steering) > 5.0:
            # Curved road - use the last known target and steering to navigate the bend
            # We hold the steering through the curve using the previous optimal track positioning
            predicted_target = self.last_valid_target
            confidence = max(0.0, 1.0 - t / 3.0) # Decay over 3s on curve
        else:
            # Straight road - force target to centre to go straight
            # If IMU indicates drift, counteract it heavily by pushing the target in the OPPOSITE direction!
            # Example: If delta_yaw is +5 (Right), target becomes 320 - (5*20) = 220 (Left), forcing Stanley to steer Left!
            predicted_target = 320.0 - (delta_yaw_deg * 20.0) 
            confidence = max(0.0, 1.0 - t / 5.0) # Decay over 5s on straight

        predicted_target = float(np.clip(predicted_target, 150, 490))
        return predicted_target, confidence

class HybridLaneTracker:
    NWINDOWS         = 9
    SW_MARGIN        = 60
    MINPIX           = 50
    POLY_MARGIN_BASE = 60
    POLY_MARGIN_CURV = 120
    MIN_PIX_OK       = 200
    EMA_ALPHA        = 0.85
    EMA_ALPHA_TURN   = 1.0
    STALE_FIT_FRAMES = 12

    WIDE_ROAD_PX             = 420
    SINGLE_LANE_PX           = 200
    RIGHT_LANE_BIAS_PX       = 0    # Shift target 0 pixels (absolute center tracking)
    DIVIDER_FOLLOW_OFFSET_PX = 145  # Must be > DIVIDER_SAFE_PX (130) to avoid force-field oscillations

    def __init__(self, img_shape=(480, 640)):
        self.h, self.w = img_shape
        self.mode       = "SEARCH"
        self.left_fit   = None
        self.right_fit  = None
        self.sl         = None
        self.sr         = None
        self.left_conf  = 0
        self.right_conf = 0
        self.left_stale  = 0
        self.right_stale = 0
        self.estimated_lane_width = 380.0
        self.right_lost_frames = 0
        self.dead_reckoner = DeadReckoningNavigator()

    def update(self, warped_binary, map_hint: str = "STRAIGHT"):
        nz  = warped_binary.nonzero()
        nzy = np.array(nz[0])
        nzx = np.array(nz[1])

        if self.mode == "TRACKING" and (self.sl is not None or self.sr is not None):
            curv = self.get_curvature(self.h // 2)
            li, ri, dbg = self._poly_search(warped_binary, nzx, nzy, curvature=curv, map_hint=map_hint)
            mode_label  = "POLY"
        else:
            li, ri, dbg = self._sliding_window(warped_binary, nzx, nzy, map_hint=map_hint)
            mode_label  = "SLIDE"

        self.left_conf  = len(li)
        self.right_conf = len(ri)
        has_l = self.left_conf  >= self.MIN_PIX_OK
        has_r = self.right_conf >= self.MIN_PIX_OK

        if has_l:
            fl = np.polyfit(nzy[li], nzx[li], 2)
            self.left_fit  = fl
            curv_now = self.get_curvature(self.h // 2)
            alpha = self.EMA_ALPHA_TURN if curv_now > 0.002 else self.EMA_ALPHA
            self.sl        = self._ema(self.sl, fl, alpha)
            self.left_stale = 0
        else:
            self.left_stale += 1
            if self.left_stale > self.STALE_FIT_FRAMES:
                self.left_fit, self.sl = None, None

        if has_r:
            fr = np.polyfit(nzy[ri], nzx[ri], 2)
            self.right_fit  = fr
            curv_now = self.get_curvature(self.h // 2)
            alpha = self.EMA_ALPHA_TURN if curv_now > 0.002 else self.EMA_ALPHA
            self.sr         = self._ema(self.sr, fr, alpha)
            self.right_stale = 0
        else:
            self.right_stale += 1
            if self.right_stale > self.STALE_FIT_FRAMES:
                self.right_fit, self.sr = None, None

        # Collision guard: if both fits converge within 120 px of each other, drop the weaker one
        if has_l and has_r and self.left_fit is not None and self.right_fit is not None:
            sep = float(np.polyval(self.right_fit, self.h * 0.75) - np.polyval(self.left_fit, self.h * 0.75))
            if sep < 120:
                if self.left_conf < self.right_conf:
                    self.left_fit, self.sl, self.left_stale, has_l = None, None, self.STALE_FIT_FRAMES, False
                else:
                    self.right_fit, self.sr, self.right_stale, has_r = None, None, self.STALE_FIT_FRAMES, False

        if has_l and has_r:
            if not self._width_sane(self.left_fit, self.right_fit):
                if self.left_conf < self.right_conf:
                    self.left_fit, self.sl, self.left_stale, has_l = None, None, self.STALE_FIT_FRAMES, False
                else:
                    self.right_fit, self.sr, self.right_stale, has_r = None, None, self.STALE_FIT_FRAMES, False
            else:
                y_positions = [100, 200, 300, 400]
                widths = [np.polyval(self.sr, y) - np.polyval(self.sl, y) for y in y_positions]
                weighted_avg_width = np.average(widths, weights=[4, 3, 2, 1])
                self.estimated_lane_width = 0.8 * self.estimated_lane_width + 0.2 * weighted_avg_width

        self.mode = "TRACKING" if (has_l or has_r or self.sl is not None or self.sr is not None) else "SEARCH"
        return self.sl, self.sr, dbg, mode_label

    def get_target_x(self, y_eval, lane_width_px, extra_offset_px=0,
                     nav_state="NORMAL", frames_lost=0,
                     last_speed=0.0, last_steering=0.0, current_yaw=0.0,
                     imu_yaw_rate=0.0):
        sl, sr = self.sl, self.sr
        hw = lane_width_px / 2.0
        def ev(fit): return float(np.polyval(fit, y_eval))

        if nav_state == "ROUNDABOUT":
            if sl is not None: return ev(sl) + hw + extra_offset_px, "RBT_INNER"
            if sr is not None: return ev(sr) - hw + extra_offset_px, "RBT_OUTER"
            return None, "RBT_LOST"

        if nav_state.startswith("JUNCTION"):
            if nav_state == "JUNCTION_RIGHT":
                if sr is not None: return ev(sr) - (lane_width_px * 0.40) + extra_offset_px, "JCT_RIGHT_EDGE"
                elif sl is not None: return ev(sl) + (lane_width_px * 1.5) + extra_offset_px, "JCT_RIGHT_GHOST"
                else: return 320.0 + (lane_width_px * 0.8) + extra_offset_px, "JCT_RIGHT_BLIND"
            elif nav_state == "JUNCTION_LEFT":
                if sl is not None: return ev(sl) + (lane_width_px * 0.40) + extra_offset_px, "JCT_LEFT_EDGE"
                elif sr is not None: return ev(sr) - (lane_width_px * 1.5) + extra_offset_px, "JCT_LEFT_GHOST"
                else: return 320.0 - (lane_width_px * 0.8) + extra_offset_px, "JCT_LEFT_BLIND"
            return 320.0 + extra_offset_px, "JCT_WAITING_CHOICE"

        has_right = (sr is not None)
        has_left  = (sl is not None)

        if not has_right and not has_left:
            predicted_x, conf = self.dead_reckoner.predict_target(last_speed, last_steering, current_yaw)
            return predicted_x + extra_offset_px, f"DEAD_RECKONING_{conf:.2f}"

        if has_right:
            self.right_lost_frames = 0
            self.right_yaw_at_loss = current_yaw
            if has_left:
                base_x = (ev(sl) + ev(sr)) / 2.0
                anchor = "CENTER_DUAL"
            else:
                base_x = ev(sr) - hw
                anchor = "RIGHT_LANE_ONLY"
        else:
            self.right_lost_frames += 1
            if has_left:
                # Right line not visible — use left line + half lane width to stay centred.
                # This is the normal case on the BFMC track where only the inner boundary
                # (centre divider) is in the camera's field of view.
                base_x = ev(sl) + hw
                anchor = "LEFT_LANE_ONLY"
            elif self.right_lost_frames < 80: # ~4 seconds at 20 Hz
                # The user requested exactly a 5-degree left steer when BOTH lines drop.
                # So we aim the car 5 degrees to the left of wherever it was pointing when the line vanished.
                target_yaw = getattr(self, 'right_yaw_at_loss', current_yaw) - 5.0
                delta_yaw_deg = current_yaw - target_yaw
                
                # Because Stanley Controller chases a pixel target:
                # 320.0 is straight ahead. If we want to steer left, we place the pixel target to the left (< 320).
                # `delta_yaw_deg` positive means car is too far right, so we pull target left.
                base_x = 320.0 - (delta_yaw_deg * 20.0)
                anchor = "IMU_5_DEG_LEFT_FALLBACK"
            else:
                # After 4 seconds of blind 5-deg left steering, fallback to dead reckoning
                predicted_x, conf = self.dead_reckoner.predict_target(last_speed, last_steering, current_yaw)
                return predicted_x + extra_offset_px, f"DEAD_RECKONING_{conf:.2f}"

        # X-axis damping based on IMU yaw rate
        base_x -= (imu_yaw_rate * 5.0)

        self.dead_reckoner.last_valid_target    = base_x
        self.dead_reckoner.last_valid_curvature = self.get_curvature(y_eval)
        self.dead_reckoner.reset_lost_timer(current_yaw)
        return base_x + extra_offset_px, anchor

    def get_curvature(self, y_eval):
        fit = self.sr if self.sr is not None else self.sl
        if fit is None: return 0.0
        a, b = fit[0], fit[1]
        denom = (1.0 + (2.0 * a * y_eval + b) ** 2) ** 1.5
        return abs(2.0 * a) / max(denom, 1e-6)

    def get_signed_curvature(self, y_eval):
        fit = self.sr if self.sr is not None else self.sl
        if fit is None: return 0.0
        a, b = fit[0], fit[1]
        denom = (1.0 + (2.0 * a * y_eval + b) ** 2) ** 1.5
        return (2.0 * a) / max(denom, 1e-6)

    def _sliding_window(self, warped, nzx, nzy, map_hint: str = "STRAIGHT"):
        dbg  = cv2.cvtColor(warped, cv2.COLOR_GRAY2BGR)
        hist = np.sum(warped[self.h // 2:, :], axis=0)
        mid, margin = self.w // 2, self.SW_MARGIN   # true centre = 320

        shift = 0
        if map_hint == "LEFT":  shift = -60
        elif map_hint == "RIGHT": shift = 60

        # Strict left / right zones with a 60 px dead band at the centre
        l_lo = max(0,        shift)
        l_hi = max(l_lo + 1, mid - 30 + shift)
        r_lo = min(self.w,   mid + 30 + shift)
        r_hi = self.w

        lb = int(np.argmax(hist[l_lo:l_hi])) + l_lo if l_hi > l_lo else margin
        rb = int(np.argmax(hist[r_lo:r_hi])) + r_lo if r_hi > r_lo else self.w - margin

        l_val = int(hist[lb]) if 0 <= lb < self.w else 0
        r_val = int(hist[rb]) if 0 <= rb < self.w else 0

        # Ghost estimation: when only one side has real signal, project the other
        # from the known estimated lane width instead of letting both chase the same line.
        # When neither side has signal, park both at symmetric positions around centre
        # so windows don't snap to corners (argmax(zeros)==0 → left snaps to x=0).
        GHOST_MIN = 80  # histogram pixel-sum threshold to count as a real peak
        if l_val >= GHOST_MIN and r_val < GHOST_MIN:
            rb = int(np.clip(lb + self.estimated_lane_width, r_lo, self.w - margin))
        elif r_val >= GHOST_MIN and l_val < GHOST_MIN:
            lb = int(np.clip(rb - self.estimated_lane_width, margin, l_hi - 1))
        else:
            # No signal on either side — default to symmetric positions around mid
            half = int(self.estimated_lane_width // 2)
            lb = int(np.clip(mid - half, margin, mid - 30))
            rb = int(np.clip(mid + half, mid + 30, self.w - margin))

        wh = self.h // self.NWINDOWS
        lx, rx = lb, rb
        li, ri = [], []

        for win in range(self.NWINDOWS):
            y_lo, y_hi = self.h - (win + 1) * wh, self.h - win * wh
            xl0, xl1 = max(0, lx - self.SW_MARGIN), min(self.w, lx + self.SW_MARGIN)
            xr0, xr1 = max(0, rx - self.SW_MARGIN), min(self.w, rx + self.SW_MARGIN)

            cv2.rectangle(dbg, (xl0, y_lo), (xl1, y_hi), (0, 255, 0), 2)
            cv2.rectangle(dbg, (xr0, y_lo), (xr1, y_hi), (0, 255, 0), 2)

            gl = ((nzy >= y_lo) & (nzy < y_hi) & (nzx >= xl0)  & (nzx < xl1)).nonzero()[0]
            gr = ((nzy >= y_lo) & (nzy < y_hi) & (nzx >= xr0)  & (nzx < xr1)).nonzero()[0]
            li.append(gl); ri.append(gr)

            if len(gl) > self.MINPIX: lx = int(np.mean(nzx[gl]))
            if len(gr) > self.MINPIX: rx = int(np.mean(nzx[gr]))

        li, ri = np.concatenate(li) if len(li) else np.array([]), np.concatenate(ri) if len(ri) else np.array([])
        if len(li): dbg[nzy[li], nzx[li]] = [255, 80, 80]
        if len(ri): dbg[nzy[ri], nzx[ri]] = [80,  80, 255]
        return li, ri, dbg

    def _poly_search(self, warped, nzx, nzy, curvature=0.0, map_hint: str = "STRAIGHT"):
        dbg = cv2.cvtColor(warped, cv2.COLOR_GRAY2BGR)
        m = (self.POLY_MARGIN_CURV if curvature > 0.0015 else self.POLY_MARGIN_BASE)

        def band(fit): return ((nzx > np.polyval(fit, nzy) - m) & (nzx < np.polyval(fit, nzy) + m)).nonzero()[0]
        li = band(self.sl) if self.sl is not None else np.array([], dtype=int)
        ri = band(self.sr) if self.sr is not None else np.array([], dtype=int)

        # Only fall back to sliding window when BOTH lines are simultaneously absent.
        # If just one is missing, keep POLY mode so the stale fit holds position
        # and the found line's band cannot migrate to the other side.
        if len(li) < self.MIN_PIX_OK and len(ri) < self.MIN_PIX_OK:
            self.mode = "SEARCH"
            return self._sliding_window(warped, nzx, nzy, map_hint=map_hint)

        # Collision guard inside poly search: reject pixels claimed by the stronger
        # band if both bands overlap in x (< 120 px apart at mid-image).
        if len(li) > 0 and len(ri) > 0:
            li_cx = float(np.mean(nzx[li]))
            ri_cx = float(np.mean(nzx[ri]))
            if abs(ri_cx - li_cx) < 120:
                if len(li) < len(ri):
                    li = np.array([], dtype=int)
                else:
                    ri = np.array([], dtype=int)

        if len(li): dbg[nzy[li], nzx[li]] = [255, 80, 80]
        if len(ri): dbg[nzy[ri], nzx[ri]] = [80,  80, 255]
        return li, ri, dbg

    def _width_sane(self, lf, rf, y=400):
        if rf is None or lf is None: return False
        w = np.polyval(rf, y) - np.polyval(lf, y)
        return 150 < w < 550

    def _ema(self, prev, new, alpha=None):
        if alpha is None:
            alpha = self.EMA_ALPHA
        if prev is None: return new.copy()
        return alpha * new + (1.0 - alpha) * prev
