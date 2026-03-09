import cv2
import numpy as np
import math
from dataclasses import dataclass
from perception.lane_tracker import HybridLaneTracker, DeadReckoningNavigator

@dataclass
class LaneResult:
    warped_binary:     np.ndarray
    lane_dbg:          np.ndarray
    sl:                object
    sr:                object
    target_x:          float
    lateral_error_px:  float
    anchor:            str
    confidence:        float
    lane_width_px:     float
    curvature:         float
    heading_rad:       float = 0.0
    heading_conf:      float = 0.0
    y_eval:            float = 320.0
    optical_yaw_rate:  float = 0.0
    optical_vel:       float = 0.0

class VisualOdometry:
    def __init__(self):
        self.feature_params = dict(maxCorners=50, qualityLevel=0.3, minDistance=7, blockSize=7)
        self.lk_params = dict(winSize=(15, 15), maxLevel=2, criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 10, 0.03))
        self.p0       = None
        self.old_gray = None

    def update(self, frame_bgr, dt: float):
        if dt <= 0: return 0.0, 0.0
        gray = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2GRAY)
        h, w = gray.shape
        roi  = gray[int(h * 0.6):, :]

        if self.p0 is None or len(self.p0) < 10:
            p0_roi = cv2.goodFeaturesToTrack(roi, mask=None, **self.feature_params)
            if p0_roi is not None:
                p0_roi[:, 0, 1] += int(h * 0.6)
                self.p0       = p0_roi
                self.old_gray = gray.copy()
            return 0.0, 0.0

        p1, st, _ = cv2.calcOpticalFlowPyrLK(self.old_gray, gray, self.p0, None, **self.lk_params)

        if p1 is None or st is None:
            self.p0 = None
            return 0.0, 0.0

        good_new = p1[st == 1]
        good_old = self.p0[st == 1]

        yaw_rate = vel = 0.0
        if len(good_new) > 3:
            dx = good_new[:, 0] - good_old[:, 0]
            dy = good_new[:, 1] - good_old[:, 1]
            yaw_rate = float(-np.median(dx) * 0.015 / dt)
            vel      = float( np.median(dy) * 0.008 / dt)

        self.old_gray = gray.copy()
        self.p0       = good_new.reshape(-1, 1, 2) if len(good_new) > 0 else None
        return yaw_rate, vel

class LaneDetector:
    def __init__(self):
        self.SRC_PTS = np.float32([[200, 260], [440, 260], [40, 450], [600, 450]])
        self.DST_PTS = np.float32([[150, 0], [490, 0], [150, 480], [490, 480]])
        self.M_forward = cv2.getPerspectiveTransform(self.SRC_PTS, self.DST_PTS)
        self.clahe = cv2.createCLAHE(clipLimit=3.0, tileGridSize=(8, 8))
        self.tracker = HybridLaneTracker(img_shape=(480, 640))
        self.vo = VisualOdometry()
        self.lost_frames = 0
        self.last_target_x = 320.0
        self._heading_ema = 0.0

    def process(self, raw_frame, dt: float = 0.033, extra_offset_px=0.0,
                nav_state="NORMAL", velocity_ms=0.0, last_steering=0.0,
                upcoming_curve: str = "STRAIGHT", pitch_rad: float = 0.0,
                current_yaw: float = 0.0) -> LaneResult:
        if raw_frame.shape[:2] != (480, 640):
            process_frame = cv2.resize(raw_frame, (640, 480))
        else:
            process_frame = raw_frame

        opt_yaw_rate, opt_vel = self.vo.update(process_frame, dt)

        if abs(pitch_rad) > 0.001:
            shift_px  = int(pitch_rad * 400)
            dyn_src   = self.SRC_PTS.copy()
            dyn_src[0][1] += shift_px
            dyn_src[1][1] += shift_px
            M_use = cv2.getPerspectiveTransform(dyn_src, self.DST_PTS)
        else:
            M_use = self.M_forward

        warped_colour = cv2.warpPerspective(process_frame, M_use, (640, 480))
        lab = cv2.cvtColor(warped_colour, cv2.COLOR_BGR2LAB)
        L = self.clahe.apply(lab[:, :, 0])
        
        mean_l = np.mean(L)
        if mean_l < 100:
            L = cv2.convertScaleAbs(L, alpha=1.0 + (100 - mean_l)/200, beta=int((100 - mean_l)*0.6))
        elif mean_l > 180:
            L = cv2.convertScaleAbs(L, alpha=1.0 - (mean_l - 180)/350, beta=int(-(mean_l - 180)*0.4))

        binary = cv2.adaptiveThreshold(L, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY_INV, 31, 15)
        warped_binary = cv2.morphologyEx(binary, cv2.MORPH_CLOSE, cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5)))
        
        map_hint = upcoming_curve if upcoming_curve in ("LEFT", "RIGHT") else "STRAIGHT"
        sl, sr, line_dbg, mode_label = self.tracker.update(warped_binary, map_hint=map_hint)
        
        y_eval = 320.0 # Aggressive lookahead
        lw = self.tracker.estimated_lane_width
        
        target_x, anchor = self.tracker.get_target_x(
            y_eval, lw, extra_offset_px, nav_state, self.lost_frames,
            velocity_ms, last_steering, current_yaw
        )
        if not hasattr(self, "_target_ema"):
            self._target_ema = target_x
        # Make EMA aggressive matching request earlier
        delta = abs(target_x - self._target_ema)
        alpha = 0.6 if delta > 15.0 else 0.3
        self._target_ema = (1.0 - alpha) * self._target_ema + alpha * target_x
        target_x = self._target_ema
        
        if target_x is None:
            self.lost_frames += 1
            self.tracker.dead_reckoner.accumulate(dt, current_yaw)
            target_x = self.last_target_x
        else:
            self.lost_frames = 0
            self.last_target_x = target_x

        curv = self.tracker.get_curvature(y_eval)
        conf = 1.0 if (sl is not None and sr is not None) else 0.5 if (sl is not None or sr is not None) else 0.0
        
        heading_rad = 0.0
        def _lane_heading(fit, y):
            return math.atan2(np.polyval(fit, y - 50) - np.polyval(fit, y), 50)
        if sl is not None and sr is not None:
            heading_rad = (_lane_heading(sl, y_eval) + _lane_heading(sr, y_eval)) / 2.0
        elif sl is not None:
            heading_rad = _lane_heading(sl, y_eval)
        elif sr is not None:
            heading_rad = _lane_heading(sr, y_eval)
        self._heading_ema = 0.7 * self._heading_ema + 0.3 * heading_rad
        heading_rad = self._heading_ema

        return LaneResult(
            warped_binary=warped_binary,
            lane_dbg=line_dbg,
            sl=sl, sr=sr,
            target_x=target_x,
            lateral_error_px=target_x - 320.0,
            anchor=anchor,
            confidence=conf,
            lane_width_px=lw,
            curvature=curv,
            heading_rad=heading_rad,
            heading_conf=conf,
            y_eval=y_eval,
            optical_yaw_rate=opt_yaw_rate,
            optical_vel=opt_vel,
        )
