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
    RIGHT_LANE_BIAS_PX       = 25   # Shift target 25 pixels closer to the right edge
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
        self.estimated_lane_width = 280.0
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
                     last_speed=0.0, last_steering=0.0, current_yaw=0.0):
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
            if has_left:
                if lane_width_px >= self.WIDE_ROAD_PX:
                    base_x = (ev(sl) + ev(sr)) / 2.0 + self.RIGHT_LANE_BIAS_PX
                    anchor = "RL_DUAL"
                else:
                    base_x = (ev(sl) + ev(sr)) / 2.0 + self.RIGHT_LANE_BIAS_PX
                    anchor = "RL_DUAL"
            else:
                base_x = ev(sr) - hw + self.RIGHT_LANE_BIAS_PX
                anchor = "RL_FROM_EDGE"
        else:
            self.right_lost_frames += 1
            if has_left:
                # User requested to follow dashed center line immediately when right line drops
                # if dashed/dotted line is in the way, take that as reference
                base_x = ev(sl) + self.DIVIDER_FOLLOW_OFFSET_PX
                anchor = "CENTER_FOLLOW"
            elif self.right_lost_frames < 80: # ~4 seconds at 20 Hz
                # The user requested exactly a 5-degree left steer when BOTH lines drop.
                # So we aim the car 5 degrees to the left of wherever it was pointing when the line vanished.
                target_yaw = self.right_yaw_at_loss - 5.0
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

    def _sliding_window(self, warped, nzx, nzy, map_hint: str = "STRAIGHT"):
        dbg  = cv2.cvtColor(warped, cv2.COLOR_GRAY2BGR)
        hist = np.sum(warped[self.h // 2:, :], axis=0)
        mid, margin = int(self.w * 0.40), self.SW_MARGIN

        shift = 0
        if map_hint == "LEFT":  shift = -80
        elif map_hint == "RIGHT": shift = 80

        l_lo =  max(margin, margin + shift)
        l_hi =  max(l_lo + 1, mid - margin + shift)
        r_lo =  max(margin, mid + margin + shift)
        r_hi =  min(self.w - margin, self.w - margin)   

        lb = int(np.argmax(hist[l_lo:l_hi])) + l_lo if l_hi > l_lo else margin
        rb = int(np.argmax(hist[r_lo:r_hi])) + r_lo if r_hi > r_lo else mid + margin

        if abs(rb - lb) < 100:
            smoothed = np.convolve(hist.astype(float), np.ones(20) / 20, mode='same')
            p1 = int(np.argmax(smoothed))
            tmp = smoothed.copy()
            tmp[max(0, p1-40):min(self.w, p1+40)] = 0
            p2 = int(np.argmax(tmp))
            lb, rb = (min(p1, p2), max(p1, p2))

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

        if len(li) < self.MIN_PIX_OK and len(ri) < self.MIN_PIX_OK:
            self.mode = "SEARCH"
            return self._sliding_window(warped, nzx, nzy, map_hint=map_hint)

        if len(li): dbg[nzy[li], nzx[li]] = [255, 80, 80]
        if len(ri): dbg[nzy[ri], nzx[ri]] = [80,  80, 255]
        return li, ri, dbg

    def _width_sane(self, lf, rf, y=400):
        if rf is None or lf is None: return False
        w = np.polyval(rf, y) - np.polyval(lf, y)
        return 180 < w < 420

    def _ema(self, prev, new, alpha=None):
        if alpha is None:
            alpha = self.EMA_ALPHA
        if prev is None: return new.copy()
        return alpha * new + (1.0 - alpha) * prev
