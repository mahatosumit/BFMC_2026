# config.py
"""
BFMC 2026 — Centralized Configuration Module
=============================================
All tunable parameters for the autonomous stack live here.
Change a value once and it propagates everywhere.

Sections
--------
  PATHS              Asset file locations
  PHYSICAL           Wheelbase, camera geometry, world dimensions
  MAP / LOCALISATION Scale factors, curve detection threshold
  V2X                Network addresses, reconnect timing
  CAMERA             Resolution, FPS, buffer size
  LANE DETECTION     Perspective transform, thresholding pipeline
  LANE TRACKER       Sliding window, EMA, width estimation
  CONTROL            Stanley gains, speed scheduling, loop timing
  TRAFFIC/BEHAVIOUR  Sign timers, zone multipliers, parking timing
  BATTERY            Voltage range for SOC estimation
  TELEMETRY          CSV fields, logging rate, video codec
  WEB DASHBOARD      Host, port, MJPEG FPS
  LOCAL DASHBOARD    Slider defaults
  UI THEME           Colours, fonts
  SIGN MAP           Sign-type → display info
"""

import os

# ─────────────────────────────────────────────────────────────────────────────
#  PATHS
# ─────────────────────────────────────────────────────────────────────────────
_ASSETS_DIR     = os.path.join(os.path.dirname(os.path.abspath(__file__)), "assets")

SVG_FILE        = os.path.join(_ASSETS_DIR, "Track.svg")
GRAPH_FILE      = os.path.join(_ASSETS_DIR, "Competition_track_graph.graphml")
SIGNS_DB_FILE   = os.path.join(_ASSETS_DIR, "signs_database.json")
CONFIG_FILE     = os.path.join(_ASSETS_DIR, "dashboard_config.json")
YOLO_MODEL_FILE = os.path.join(_ASSETS_DIR, "Niranjan.pt")

# ─────────────────────────────────────────────────────────────────────────────
#  PHYSICAL CONSTANTS
# ─────────────────────────────────────────────────────────────────────────────
WHEELBASE_M             = 0.23    # Bicycle model wheelbase (m)
CAMERA_FOCAL_LENGTH_PX  = 450.0   # Camera focal length (pixels)
REAL_SIGN_HEIGHT_M      = 0.08    # Physical sign height (m)
REAL_WIDTH_M            = 22.0    # Track world width (m)
REAL_HEIGHT_M           = 15.0    # Track world height (m)
DEFAULT_START_X         = 1.0     # Default spawn position X (m)
DEFAULT_START_Y         = 1.0     # Default spawn position Y (m)

# ─────────────────────────────────────────────────────────────────────────────
#  MAP / LOCALISATION
# ─────────────────────────────────────────────────────────────────────────────
FINAL_SCALE_X           = 1.0640  # SVG→world X scale factor
FINAL_SCALE_Y           = 1.0890  # SVG→world Y scale factor
FINAL_OFF_X             = 0       # World origin X offset (SVG units)
FINAL_OFF_Y             = 0       # World origin Y offset (SVG units)

# Minimum heading delta (degrees) to classify a path segment as a curve
CURVE_DETECT_MIN_ANGLE_DEG  = 25.0

USE_FUSED_LOCALIZATION  = True    # EKF fused pose (True) vs legacy path-interpolation
USE_SEMANTIC_LANDMARKS  = True    # Use YOLO labels to anchor EKF position

# ─────────────────────────────────────────────────────────────────────────────
#  V2X COMMUNICATION
# ─────────────────────────────────────────────────────────────────────────────
V2X_SERVER_HOST         = "127.0.0.1"
V2X_SERVER_PORT         = 5000
V2X_STREAM_PORT         = 9000
V2X_LOCSYS_PORT         = 4691
V2X_SIM_PORT            = 5007
V2X_RECONNECT_DELAY_S   = 3.0    # Seconds between TCP reconnect attempts
V2X_HEARTBEAT_S         = 0.1    # Telemetry push interval (s)

# ─────────────────────────────────────────────────────────────────────────────
#  CAMERA
# ─────────────────────────────────────────────────────────────────────────────
CAMERA_RESOLUTION       = (640, 480)   # Capture resolution (W, H)
CAMERA_FPS              = 30           # Target capture frame rate
CAMERA_BUFFER_FRAMES    = 2            # Frame queue depth (reduces latency)

# ─────────────────────────────────────────────────────────────────────────────
#  LANE DETECTION — Perspective Transform
#
#  Calibrated from BFMC indoor track video (white lane lines on dark carpet).
#  Left lane line sits at x≈0 at y=430 in the raw camera frame.
#  After warp:
#    DST left edge  (x=150)  → left lane marking
#    DST centre     (x=320)  → expected lane centre  ← Stanley zero-error point
#    DST right edge (x=490)  → estimated right boundary
# ─────────────────────────────────────────────────────────────────────────────
#
#  HOW TO TUNE THE BIRD'S-EYE VIEW (BEV)
#  ──────────────────────────────────────
#  The 4 SRC points define a trapezoid in the RAW camera frame (640×480).
#  The 4 DST points define where those 4 corners land in the BEV output (also 640×480).
#
#  SRC SHAPE RULES:
#    • Bottom row (high y) = road close to the car  → should be WIDE
#    • Top row   (low  y) = road far from the car   → should be NARROW
#      (matches the perspective convergence you see in the camera image)
#    • Left/right edges should sit just OUTSIDE the lane markings
#
#  TO ZOOM OUT  → widen SRC (increase x-spread at bottom, decrease top y value)
#  TO ZOOM IN   → narrow SRC (reduce x-spread or raise top y value)
#  TO SHIFT LEFT  → move both SRC left columns left
#  TO SHIFT RIGHT → move both SRC right columns right
#
#  DST is always the full output rectangle [[0,0],[640,0],[0,480],[640,480]]
#  unless you want black borders on the sides (then shrink DST x-range).
#
#  WORKFLOW:
#    1. Pause the car / use a still frame.
#    2. Print pixel coords of the two lane lines at y=top_row and y=bottom_row
#       (use cv2.imshow + mouse callback, or mark points in Paint).
#    3. Set SRC top-left  = (left_line_x_at_top  - margin, top_row)
#              top-right = (right_line_x_at_top  + margin, top_row)
#              bot-left  = (left_line_x_at_bot   - margin, bottom_row)
#              bot-right = (right_line_x_at_bot  + margin, bottom_row)
#    4. Restart and inspect the BEV — the lane lines should appear nearly vertical.
#
LANE_SRC_PTS = [
    [160, 265],   # top-left  — left of left line at y=265 (line at ~x=205)
    [370, 265],   # top-right — right of right line at y=265 (line at ~x=280)
    [ 60, 450],   # bottom-left  — left of left line at y=450 (line at ~x=115)
    [400, 450],   # bottom-right — right of right line at y=450 (line at ~x=330)
]
LANE_DST_PTS = [
    [  0,   0],   # full-width output — spreads lanes to BEV x≈110 (left) and x≈480 (right)
    [640,   0],
    [  0, 480],
    [640, 480],
]
# Slice [y0:y1, x0:x1] to zero-out camera-mount hardware in the warped image
# Lane lines at BEV x≈110 (left) and x≈480 (right) — mask only the bottom-center dead zone
LANE_CLIP_MASK_WARPED   = (slice(460, 480), slice(290, 360))

# ─────────────────────────────────────────────────────────────────────────────
#  LANE DETECTION — Binary Thresholding
#
#  Pipeline: raw L channel (no CLAHE) → brightness normalise → GaussianBlur
#            → global threshold → morphological close
#  CLAHE was removed because it amplified road-texture noise.
# ─────────────────────────────────────────────────────────────────────────────
LANE_BLUR_KERNEL        = (5, 5)  # Gaussian blur kernel size
LANE_THRESHOLD          = 155     # Global L-channel threshold (white lines: L≈200+)
LANE_BRIGHT_LOW         = 80      # Brighten if mean(L) below this
LANE_BRIGHT_HIGH        = 160     # Darken  if mean(L) above this
LANE_MORPH_KERNEL       = (5, 5)  # Morphological-close kernel (bridges dashed lines)

# ─────────────────────────────────────────────────────────────────────────────
#  LANE DETECTION — Target EMA & Lookahead
# ─────────────────────────────────────────────────────────────────────────────
LANE_TARGET_EMA_FAST    = 0.70    # EMA alpha when |Δtarget| > LANE_EMA_THR_FAST
LANE_TARGET_EMA_MED     = 0.30    # EMA alpha when LANE_EMA_THR_MED < |Δ| ≤ THR_FAST
LANE_TARGET_EMA_SLOW    = 0.05    # EMA alpha for micro-noise (|Δ| ≤ THR_MED)
LANE_EMA_THR_FAST       = 20.0    # Pixel delta → fast EMA
LANE_EMA_THR_MED        = 5.0     # Pixel delta → medium EMA
LANE_HEADING_EMA_ALPHA  = 0.15    # Heading EMA smoothing factor
LANE_LOOKAHEAD_Y_MIN    = 180.0   # Min lookahead row in bird's-eye (px)
LANE_LOOKAHEAD_Y_MAX    = 300.0   # Max lookahead row (added to min by curvature)

# ─────────────────────────────────────────────────────────────────────────────
#  LANE TRACKER — Sliding Window
# ─────────────────────────────────────────────────────────────────────────────
TRACKER_NWINDOWS            = 9    # Number of stacked windows per side
TRACKER_SW_MARGIN           = 60   # Half-width of each window (px)
TRACKER_MINPIX              = 50   # Min pixels to re-centre a window
TRACKER_POLY_MARGIN_BASE    = 60   # Polynomial search band — straight (px)
TRACKER_POLY_MARGIN_CURV    = 120  # Polynomial search band — curved (px)
TRACKER_MIN_PIX_OK          = 200  # Min accepted pixels for a valid lane fit
TRACKER_STALE_FIT_FRAMES    = 12   # Frames of no detection before fit is dropped

# ─────────────────────────────────────────────────────────────────────────────
#  LANE TRACKER — EMA & Width
# ─────────────────────────────────────────────────────────────────────────────
TRACKER_EMA_ALPHA           = 0.85   # Polynomial EMA — straight road
TRACKER_EMA_ALPHA_TURN      = 1.0    # Polynomial EMA — turn (instant update)
TRACKER_CURVATURE_TURN_THR  = 0.002  # Curvature > this → use turn EMA
TRACKER_ESTIMATED_LANE_W    = 340.0  # Default lane width in bird's-eye (px)
                                     # DST spans 150–490 = 340 px = one full lane
TRACKER_WIDTH_SANE_MIN      = 180    # Reject dual-lane fit narrower than this
TRACKER_WIDTH_SANE_MAX      = 420    # Reject dual-lane fit wider than this
TRACKER_WIDE_ROAD_PX        = 420    # Wide-road threshold (px)
TRACKER_SINGLE_LANE_PX      = 200    # Single-lane detection threshold (px)
TRACKER_RIGHT_LANE_BIAS_PX  = 0      # Static right-lane bias (0 = track centre)
TRACKER_DIVIDER_FOLLOW_OFF  = 145    # Divider-follow lateral offset (px)

# ─────────────────────────────────────────────────────────────────────────────
#  CONTROL — Stanley Controller
# ─────────────────────────────────────────────────────────────────────────────
STANLEY_K               = 2.5    # Cross-track error gain
STANLEY_KS              = 0.5    # Speed softening denominator
STANLEY_KD_YAW          = 0.45   # IMU yaw-rate damping gain (deg/deg·s⁻¹)
STANLEY_MAX_STEER_DEG   = 30.0   # Steering angle clamp (degrees)
STANLEY_MAX_STEER_RATE  = 60.0   # Max steer rate per frame (deg/frame)
STANLEY_DEADBAND_ERR_PX = 12.0   # Lateral error deadband on straights (px)

# ─────────────────────────────────────────────────────────────────────────────
#  CONTROL — Speed Scheduling
# ─────────────────────────────────────────────────────────────────────────────
SPEED_MIN_CURVE_FACTOR  = 0.45   # Minimum speed fraction at maximum curve
SPEED_BRAKING_DIST_M    = 1.8    # Braking starts this many metres before a curve
SPEED_STRAIGHT_BONUS    = 1.15   # Multiplier when |steer| < 5° (straight boost)
SPEED_STRAIGHT_MAX_MULT = 1.20   # Cap for the straight bonus
SPEED_ROUNDABOUT_MULT   = 0.50   # Speed fraction in roundabout nav state
SPEED_MIN_PWM           = 18.0   # Minimum PWM to actually move the car

# Straight-road filter: heavy smoothing prevents oscillation
CTRL_STRAIGHT_ALPHA     = 0.65   # Steer filter alpha on straights
CTRL_CURVE_ALPHA        = 0.20   # Steer filter alpha on curves

# ─────────────────────────────────────────────────────────────────────────────
#  CONTROL — Main Drive Loop
# ─────────────────────────────────────────────────────────────────────────────
LOOP_HZ                 = 20        # Target control-loop frequency (Hz)
LOOP_INTERVAL_MS        = 50        # Tkinter after() interval (ms)
AUTO_CALIBRATION_WAIT_S = 5.0       # IMU warm-up before autonomous drive
MANUAL_STEER_DEG        = 25.0      # Keyboard manual steering angle
SPEED_SMOOTH_ALPHA      = 0.20      # Speed ramp smoothing factor
STEER_SMOOTH_ALPHA      = 0.20      # Steer ramp smoothing factor
SPEED_DEAD_BAND         = 1.0       # Speeds below this are zeroed (PWM)
STEER_DEAD_BAND         = 0.5       # Steers below this are zeroed (degrees)
MAP_SIM_SPEED_SCALE     = 1.5       # Extra scale applied to map sim velocity

# ─────────────────────────────────────────────────────────────────────────────
#  TRAFFIC & BEHAVIOUR — Speed Modes
# ─────────────────────────────────────────────────────────────────────────────
HIGHWAY_SPEED_PWM       = 300       # PWM target in highway zone
HIGHWAY_SPEED_MULT      = 1.30      # Multiplier when highway mode is active
CROSSWALK_SPEED_MULT    = 0.80      # Speed fraction during crosswalk hold
PRIORITY_SPEED_MULT     = 0.80      # Speed fraction during priority-sign hold
CROSSWALK_HOLD_S        = 5.0       # Duration of crosswalk slow-zone (s)
PRIORITY_HOLD_S         = 10.0      # Duration of priority hold (s)
PARKING_WAIT_S          = 10.0      # Wait before starting reverse-parking (s)

# ─────────────────────────────────────────────────────────────────────────────
#  TRAFFIC & BEHAVIOUR — Detection Distances
# ─────────────────────────────────────────────────────────────────────────────
SIGN_DETECT_DEFAULT_M   = 5.0       # Default sign detection range (m)
SIGN_ACT_DEFAULT_M      = 2.0       # Default sign activation range (m)

# ─────────────────────────────────────────────────────────────────────────────
#  BATTERY
# ─────────────────────────────────────────────────────────────────────────────
BATTERY_MIN_V           = 3.0       # LiPo floor voltage → 0 %
BATTERY_MAX_V           = 4.2       # LiPo full voltage  → 100 %

# ─────────────────────────────────────────────────────────────────────────────
#  TELEMETRY LOGGING
#
#  The async TelemetryLogger writes one CSV row per LOG_CSV_INTERVAL_S,
#  throttling data rate to keep files readable post-mission.
# ─────────────────────────────────────────────────────────────────────────────
LOG_DIRECTORY           = "logs"    # Output directory for CSVs and recordings
LOG_CSV_INTERVAL_S      = 1.0       # One row per second (20 loops at 20 Hz)
LOG_VIDEO_CODEC         = "XVID"    # FourCC codec for camera recording
LOG_VIDEO_FPS           = 15.0      # Recording frame rate (lower → smaller files)
LOG_VIDEO_RES           = (640, 480)
LOG_MAX_QUEUE_SIZE      = 2000      # Max queued telemetry rows before drop

# Ordered CSV column list — only these fields are written.
# Keep this short so the CSV stays readable.
LOG_CSV_FIELDS = [
    "timestamp",
    "loop_hz",
    "mode",
    "speed_pwm",
    "steer_deg",
    "yaw_deg",
    "roll_deg",
    "pitch_deg",
    "car_x",
    "car_y",
    "lane_anchor",
    "target_x",
    "lateral_err_px",
    "lane_confidence",
    "active_sign",
    "yolo_labels",
]

# ─────────────────────────────────────────────────────────────────────────────
#  LOCAL DASHBOARD — Slider defaults
# ─────────────────────────────────────────────────────────────────────────────
DASH_BASE_SPEED_DEFAULT     = 150.0
DASH_SIM_SPEED_DEFAULT      = 1.0
DASH_STEER_MULT_DEFAULT     = 1.0
DASH_OVERTAKE_DIST_DEFAULT  = 1.2
DASH_OVERTAKE_TIME_DEFAULT  = 2.0
DASH_SIGN_DETECT_DEFAULT    = SIGN_DETECT_DEFAULT_M   # keeps them in sync
DASH_SIGN_ACT_DEFAULT       = SIGN_ACT_DEFAULT_M

# ─────────────────────────────────────────────────────────────────────────────
#  MAP NODE DISPLAY
#  Controls how graph nodes appear on the dashboard track map.
# ─────────────────────────────────────────────────────────────────────────────
MAP_NODE_SHOW           = True   # False → hide all nodes completely
MAP_NODE_OPACITY        = 0.10   # 0.0 = invisible  →  1.0 = fully opaque
MAP_NODE_MIN_SPACING_PX = 100     # skip interpolated nodes closer than this (px)
                                  # raise to thin out clutter, lower for denser dots

# ─────────────────────────────────────────────────────────────────────────────
#  UI THEME
# ─────────────────────────────────────────────────────────────────────────────
THEME = {
    "bg":      "#1e1e1e",
    "panel":   "#252526",
    "canvas":  "#111111",
    "fg":      "#cccccc",
    "accent":  "#007acc",
    "danger":  "#f44336",
    "success": "#4caf50",
    "warning": "#ff9800",
    "log_bg":  "#0d1117",
    "log_fg":  "#00ff00",
    "font_h":  ("Helvetica", 11, "bold"),
    "font_p":  ("Helvetica", 10),
    "sash":    "#333333",
}

# ─────────────────────────────────────────────────────────────────────────────
#  SIGN MAP
# ─────────────────────────────────────────────────────────────────────────────
SIGN_MAP = {
    "stop-sign":          {"name": "Stop",        "emoji": "🛑"},
    "crosswalk-sign":     {"name": "Crosswalk",   "emoji": "🚶"},
    "priority-sign":      {"name": "Priority",    "emoji": "🔶"},
    "parking-sign":       {"name": "Parking",     "emoji": "🅿️"},
    "highway-entry-sign": {"name": "Hwy Entry",   "emoji": "⬆️"},
    "highway-exit-sign":  {"name": "Hwy Exit",    "emoji": "↗️"},
    "pedestrian":         {"name": "Pedestrian",  "emoji": "🚸"},
    "traffic-light":      {"name": "Light",       "emoji": "🚦"},
    "roundabout-sign":    {"name": "Roundabout",  "emoji": "🔄"},
    "oneway-sign":        {"name": "Oneway",      "emoji": "⬆️"},
    "noentry-sign":       {"name": "No Entry",    "emoji": "⛔"},
    "car":                {"name": "Car",         "emoji": "🚙"},
}
