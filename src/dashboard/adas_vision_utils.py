import cv2
import numpy as np

class AutonomousJunctionPlanner:
    def decide(self, warped_binary, left_fit, right_fit, lane_width_px):
        lroi=warped_binary[0:240,0:320]; rroi=warped_binary[0:240,320:640]; sroi=warped_binary[0:240,200:440]
        wts=np.linspace(2.0,0.5,240).reshape(-1,1)
        ls=np.sum(lroi*wts)/(320*240); rs=np.sum(rroi*wts)/(320*240); ss=np.sum(sroi*wts)/(240*240)
        scores={"LEFT":ls,"RIGHT":rs,"STRAIGHT":ss}
        best=max(scores,key=scores.get); total=sum(scores.values())
        conf=scores[best]/max(total,1e-6)
        return ("RIGHT",0.3) if conf<0.4 else (best,conf)

class JunctionDetector:
    ENTRY_FRAMES       = 5
    EXIT_FRAMES        = 8
    CROSS_ENERGY_RATIO = 1.4
    WIDTH_RATIO_HIGH   = 1.6
    MIN_BOT_ENERGY     = 500

    def __init__(self):
        self.state         = "NORMAL"
        self.entry_count   = 0
        self.exit_count    = 0
        self.frames_in_jct = 0
        self.planner       = AutonomousJunctionPlanner()

    def update(self, warped_binary, left_fit, right_fit, lane_width_px, active_labels):
        h, w = warped_binary.shape
        approaching_wide = False
        
        if left_fit is not None and right_fit is not None:
            if (np.polyval(right_fit,150) - np.polyval(left_fit,150)) > lane_width_px * 1.7:
                approaching_wide = True
        elif left_fit is not None:
            if np.polyval(left_fit,150) < max(0, 320 - lane_width_px * 1.7): approaching_wide = True
        elif right_fit is not None:
            if np.polyval(right_fit,150) > min(640, 320 + lane_width_px * 1.7): approaching_wide = True
            
        hist_bot = float(np.sum(warped_binary[h//2:, :]))
        hist_top = float(np.sum(warped_binary[:h//2, :]))
        
        cross_e = False
        if hist_bot > self.MIN_BOT_ENERGY and (hist_top / hist_bot) > 1.4: 
            cross_e = True
        if "crosswalk-sign" in active_labels: 
            cross_e = False
            
        evidence = approaching_wide or cross_e

        if self.state == "NORMAL":
            self.entry_count = self.entry_count + 1 if evidence else 0
            if self.entry_count >= self.ENTRY_FRAMES:
                self.state         = "JUNCTION_PROMPT"
                self.exit_count    = 0
                self.frames_in_jct = 0
                
        elif self.state.startswith("JUNCTION_"):
            self.frames_in_jct += 1
            self.exit_count = self.exit_count + 1 if not evidence else 0
            if self.exit_count >= self.EXIT_FRAMES and self.frames_in_jct > 25:
                self.state       = "NORMAL"
                self.entry_count = 0

        return self.state

class RoundaboutNavigator:
    ENTRY_WIDTH_RATIO  = 0.60
    EXIT_WIDTH_RATIO   = 0.82
    MIN_CIRCLE_FRAMES  = 25
    MAX_CIRCLE_FRAMES  = 120

    def __init__(self):
        self.state  = "NORMAL"
        self.frames = 0

    def update(self, left_fit, right_fit, lane_width_px, img_h=480):
        y = img_h - 50
        if left_fit is not None and right_fit is not None:
            lx    = np.polyval(left_fit,  y)
            rx    = np.polyval(right_fit, y)
            ratio = (rx - lx) / max(float(lane_width_px), 1.0)

            if self.state == "NORMAL":
                if ratio < self.ENTRY_WIDTH_RATIO:
                    self.state  = "ROUNDABOUT"
                    self.frames = 0
            elif self.state == "ROUNDABOUT":
                self.frames += 1
                normal_exit = (self.frames > self.MIN_CIRCLE_FRAMES and ratio > self.EXIT_WIDTH_RATIO)
                timeout_exit = self.frames > self.MAX_CIRCLE_FRAMES
                if normal_exit or timeout_exit:
                    self.state  = "NORMAL"
                    self.frames = 0
        elif self.state == "ROUNDABOUT":
            self.frames += 1
            if self.frames > self.MAX_CIRCLE_FRAMES:
                self.state  = "NORMAL"
                self.frames = 0
        return self.state

def detect_traffic_light_color(frame, x1, y1, x2, y2):
    h, w = frame.shape[:2]
    x1, y1 = max(0, x1), max(0, y1)
    x2, y2 = min(w, x2), min(h, y2)
    if y2 - y1 < 3 or x2 - x1 < 3: return "NONE"

    crop = frame[y1:y2, x1:x2]
    hsv = cv2.cvtColor(crop, cv2.COLOR_BGR2HSV)
    mask_red1 = cv2.inRange(hsv, np.array([0, 120, 150]), np.array([10, 255, 255]))
    mask_red2 = cv2.inRange(hsv, np.array([160, 120, 150]), np.array([180, 255, 255]))
    mask_red = cv2.bitwise_or(mask_red1, mask_red2)
    mask_yellow = cv2.inRange(hsv, np.array([15, 120, 150]), np.array([35, 255, 255]))
    mask_green = cv2.inRange(hsv, np.array([40, 120, 150]), np.array([90, 255, 255]))

    r_count = cv2.countNonZero(mask_red)
    y_count = cv2.countNonZero(mask_yellow)
    g_count = cv2.countNonZero(mask_green)
    max_count = max(r_count, y_count, g_count)
    box_area = (y2 - y1) * (x2 - x1)
    min_pixels = max(5, int(box_area * 0.02)) 

    if max_count < min_pixels: return "NONE"
    elif max_count == r_count: return "RED"
    elif max_count == y_count: return "YELLOW"
    else: return "GREEN"

def lbl(img, txt, x, y, scale=0.38, color=(230, 230, 230), t=1):
    cv2.putText(img, txt, (x, y), cv2.FONT_HERSHEY_SIMPLEX, scale, color, t, cv2.LINE_AA)

def annotate_bev(perc, ctrl, is_calibrating, auto_start_time):
    """V4 (VIZ-06) Standardized Lane View Overlay matching PerceptionResult structure."""
    dbg = perc.lane_dbg.copy() if getattr(perc, 'lane_dbg', None) is not None else np.zeros((480,640,3),np.uint8)

    def draw_poly(fit, color):
        if fit is None: return
        ys  = np.linspace(40,479,240).astype(np.float32)
        xs  = np.clip(np.polyval(fit,ys),0,639).astype(np.float32)
        pts = np.stack([xs,ys],axis=1).reshape(-1,1,2).astype(np.int32)
        cv2.polylines(dbg,[pts],False,color,3,cv2.LINE_AA)

    draw_poly(perc.sl,(255,80,80))
    draw_poly(perc.sr,(80,80,255))

    if perc.sl is not None and perc.sr is not None:
        lx = int(np.clip(np.polyval(perc.sl,400),0,639))
        rx = int(np.clip(np.polyval(perc.sr,400),0,639))
        cv2.line(dbg,(lx,400),(rx,400),(70,170,70),1,cv2.LINE_AA)
        lbl(dbg,f"w={getattr(perc, 'lane_width_px', 280):.0f}px",(lx+rx)//2-20,396,scale=0.34,color=(70,170,70))

    yrow = int(getattr(perc, 'y_eval', 400))
    anchor = getattr(ctrl, 'anchor', 'NONE')
    yc   = (50, 220, 50) if "DUAL" in anchor else ((50, 190, 255) if "DEAD" not in anchor else (50, 50, 230))
    for xi in range(0,640,18): cv2.line(dbg,(xi,yrow),(xi+9,yrow),yc,1,cv2.LINE_AA)

    tx = max(4, min(636, int(getattr(ctrl, 'target_x', 320))))
    for yi in range(360,440,12): cv2.line(dbg,(tx,yi),(tx,yi+6),(0,255,255),2,cv2.LINE_AA)
    cv2.line(dbg,(tx-12,yrow),(tx+12,yrow),(0,255,255),2,cv2.LINE_AA)

    curv = getattr(perc, 'curvature', 0.0)
    if curv > 1e-5:
        R = min(int(1.0/curv),1400)
        if R < 700:
            sign = 1 if (perc.sl is not None and perc.sl[0]>0) else -1
            cv2.ellipse(dbg,(tx+sign*R,400),(R,R),0,84,96,(190,70,170),2,cv2.LINE_AA)

    lbl(dbg,anchor,10,25,scale=0.50,color=(230, 230, 230))
    steer_deg = getattr(ctrl, 'steer_angle_deg', 0.0)
    la_px = getattr(ctrl, 'lookahead_px', 150)
    lbl(dbg,f"steer={steer_deg:+.1f}  la={la_px:.0f}px",10,50,scale=0.44,color=(70,225,70))
    conf = getattr(perc, 'confidence', 0.0)
    lbl(dbg,f"conf={conf:.2f}  curv={curv:.5f}",10,72,scale=0.37,color=(150,150,150))
    
    if is_calibrating:
        import time
        elapsed_auto = time.time() - auto_start_time
        cv2.putText(dbg, f"CALIBRATING: {5.0 - elapsed_auto:.1f}s", (80, 240), cv2.FONT_HERSHEY_SIMPLEX, 1.2, (0, 165, 255), 3, cv2.LINE_AA)
        
    return dbg