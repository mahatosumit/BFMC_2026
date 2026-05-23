# parking_config.py

# ── Detection ────────────────────────────────────────────────
CONF_THRESHOLD       = 0.35    # YOLO/ONNX confidence threshold
SIGN_CONFIRM_FRAMES  = 3       # consecutive frames to confirm parking sign

# ── Vision-based slot detection ──────────────────────────────
# Car is absent from the right ROI for this many consecutive frames
# → the gap is big enough to be a free parking slot.
# At 20 Hz:  8 frames = 0.4 s  (good for a 1-car-length gap at low speed)
SLOT_FREE_FRAMES     = 8

# Max time (seconds) to scan for a free slot before giving up
SCAN_TIMEOUT_S       = 25.0

# After spotting a free slot the car coasts this many seconds before stopping,
# so it is fully aligned with the spot (tune on the actual track).
ALIGN_COAST_S        = 0.40

# ── Speed multipliers ────────────────────────────────────────
PARKING_SPEED_MULTIPLIER = 0.28   # approach / scan speed (fraction of base)
NORMAL_SPEED_MULTIPLIER  = 1.0

# ── Open-loop parking maneuver ───────────────────────────────
# Used when no CSV file is found.  Tune these on the actual track.
MANEUVER_SPEED_PWM       = 50.0    # PWM for maneuver (reverse + realign)
MANEUVER_STEER_RIGHT_DEG = 25.0    # full-right lock  (degrees)
MANEUVER_STEER_LEFT_DEG  = -25.0   # full-left  lock  (degrees)
MANEUVER_T1_S            = 1.6     # right-lock reverse duration (s)
MANEUVER_T2_S            = 1.4     # left-lock  reverse duration (s)
MANEUVER_T3_S            = 0.4     # straighten + creep forward (s)

# ── Legacy / unused but kept for import compatibility ────────
MAX_SLOTS                = 5
SLOT_LENGTH_CM           = 50.0
DEBOUNCE_DURATION        = 1.0
IMU_DEADBAND             = 0.05
VELOCITY_DAMPING         = 0.98
DISTANCE_CALIBRATION_FACTOR = 1.0
REVERSE_TIMEOUT          = 18.0    # safety watchdog on trajectory execution
