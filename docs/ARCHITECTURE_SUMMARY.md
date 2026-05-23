# BFMC 2026 — Architecture Summary
## Team OPTINX | One-Page Engineering Reference

---

## System at a Glance

```
┌─────────────────────────────────────────────────────────────────────┐
│                     SENSE → PLAN → ACT  @  20 Hz                   │
│                                                                     │
│  SENSORS           PERCEPTION          CONTROL          ACTUATION   │
│  ──────────        ───────────         ────────         ─────────── │
│  CSI Camera   →   LaneDetector    →   Stanley    →    STM32        │
│  (640×480@30)     BEV+LAB+poly        K=2.5          Motor+Servo   │
│                                                                     │
│  IMU (STM32)  →   dead_reckoning  →   speed_sched →   PWM ±500    │
│  roll/pitch/yaw   yaw-delta nav        curve/brk        steer ±30° │
│                                                                     │
│  YOLO Thread  →   TrafficEngine   →   BehaviorFSM →   override     │
│  5-10 FPS         sign/ped/light       priority 0-4    speed/steer  │
└─────────────────────────────────────────────────────────────────────┘
```

---

## Data Flow Diagram

```
Camera Frame (BGR 640×480, every ~33ms)
    │
    ├──────────────────────────────────────────────────────────────────
    │  LANE DETECTION PIPELINE
    ├─── Perspective Warp (BEV, 640×480, M cached)
    ├─── LAB L-channel extraction
    ├─── Brightness normalise (mean L: 80–160 range)
    ├─── Gaussian Blur (5×5)
    ├─── Global Threshold (L > 155)
    ├─── Morphological Close (5×5)
    ├─── HybridLaneTracker:
    │      SEARCH: histogram → 9 sliding windows → polyfit
    │      TRACKING: poly band search (±60–120px)
    │      DEAD_RECKONING: IMU yaw delta → predicted center
    └─── LaneResult { target_x, lateral_err, confidence, curvature }
                        │
                        ▼
              StanleyController
              delta = heading_err + atan(K * CTE / (Ks + v))
                                  - Kd_yaw * imu_yaw_rate
                        │
                        ▼
              speed_schedule (curvature → PWM ramp)
                        │
    ├──────────────────────────────────────────────────────────────────
    │  AI / TRAFFIC PIPELINE (async thread)
    ├─── YOLO inference (~50–200ms, non-blocking)
    ├─── TrafficDecisionEngine → TrafficResult { state, multiplier }
    └─── BehaviorController (priority 0–4) → speed_pwm, steer_deg
                        │
    ├──────────────────────────────────────────────────────────────────
    │  PARKING PIPELINE (if parking_mode_active)
    └─── CSV playback → target_speed, target_steer (overrides AUTO)
                        │
    ├──────────────────────────────────────────────────────────────────
    │  KINEMATICS & LOGGING
    ├─── IMU delta → car_x, car_y update (dead-reckoning)
    ├─── Soft path snap → visited_path_nodes tracking
    ├─── CSV row → TelemetryLogger queue (1 Hz)
    └─── Video frame → TelemetryLogger queue (15 FPS)
                        │
                        ▼
              STM32 Commands
              #speed:N;;  #steer:N;;  (every 50ms)
```

---

## State Machine Overview

### Main Operational Mode

```
         ┌─────────┐  arrow keys  ┌──────────────┐
         │ MANUAL  │◄────────────►│ AUTONOMOUS   │
         └─────────┘  btn toggle  └──────┬───────┘
                                         │
                                   5s calibration
                                         │
                                   ┌─────▼──────┐
                                   │ LANE FOLLOW │
                                   └─────────────┘
```

### Behavior Priority Stack

```
Priority 0: EMERGENCY   → pedestrian on road, TTC < 1s → speed = 0
Priority 1: MANDATORY   → red light, STOP sign (3s) → speed = 0
Priority 2: LEGAL       → no-entry sign → speed = 0, refuse path
Priority 3: MISSION     → parking, roundabout, overtake → specialized
Priority 4: NORMAL      → crosswalk slow, yellow, highway boost
```

### Parking FSM

```
IDLE → [parking sign] → TRIGGERED (0.3s) → SCANNING (up to 6s)
     → SELECTED → APPROACHING → REVERSE_READY
     → TRAJECTORY_LOAD → REVERSE (main.py playback)
```

### Lane Tracker FSM

```
SEARCH ──[≥200px found]──► TRACKING ──[<MIN_PIX, 12 frames]──► SEARCH
                                │
                          [both lines stale]
                                │
                           DEAD_RECKONING ──[line found again]──► SEARCH
```

---

## Module Communication Summary

| Producer | Consumer | Data | Mechanism |
|----------|---------|------|-----------|
| `camera.py` | `lane_detector.py` | BGR frame | Thread queue (maxsize=1) |
| `lane_detector.py` | `controller.py` | `LaneResult` | Function return |
| `controller.py` | `main.py` | `ControlOutput` | Function return |
| `traffic_module.py` | `behavior_controller.py` | `TrafficResult` | Function return |
| `behavior_controller.py` | `main.py` | `BehaviorOutput` | Function return |
| `serial_handler.py` | `imu_sensor.py` | IMU values | `SHARED_STATE` dict |
| `imu_sensor.py` | `main.py`, `controller.py` | roll/pitch/yaw | Method call |
| `main.py` | `serial_handler.py` | speed, steer cmds | Method call |
| `main.py` | `v2x_client.py` | car pose | `update_state()` |
| `main.py` | `telemetry.py` | CSV fields, frames | Queue (async) |
| `map_engine.py` | `main.py` | sign_cmd, path | Method return |
| `yolo_thread` | `traffic_module.py` | YOLO results | Thread queue (maxsize=1) |

---

## Key Parameter Quick Reference

```python
# STANLEY CONTROLLER
STANLEY_K = 2.5        # Cross-track gain (oscillation: reduce)
STANLEY_KS = 0.5       # Speed softening
STANLEY_KD_YAW = 0.45  # IMU yaw damping

# SPEED
SPEED_MIN_CURVE_FACTOR = 0.45   # Min speed in curves
SPEED_STRAIGHT_BONUS = 1.15     # Speed on straights
HIGHWAY_SPEED_MULT = 1.30       # Highway zone multiplier

# LANE DETECTION
LANE_THRESHOLD = 155            # White line L-channel threshold
TRACKER_NWINDOWS = 9            # Sliding window count
TRACKER_ESTIMATED_LANE_W = 340  # Lane width in BEV (pixels)
TRACKER_STALE_FIT_FRAMES = 12   # Frames before fit dropped

# BEHAVIOR TIMERS
CROSSWALK_HOLD_S = 5.0          # Slow zone after crosswalk sign
PRIORITY_HOLD_S = 10.0          # Hold after priority sign
PARKING_WAIT_S = 10.0           # Wait before reverse parking

# SYSTEM
LOOP_HZ = 20                    # Target control frequency
AUTO_CALIBRATION_WAIT_S = 5.0   # IMU warmup before autonomous
```

---

## File Size & Complexity Reference

| File | Lines | Complexity | Notes |
|------|-------|-----------|-------|
| `main.py` | ~1070 | High | Orchestrator — most interactions here |
| `traffic/traffic_module.py` | ~758 | High | Most sign logic; many timer FSMs |
| `traffic/behavior_controller.py` | ~596 | High | Priority FSM; overtake; parking |
| `config.py` | 340 | Low | Constants only; safe to edit |
| `parking/parking.py` | 245 | Medium | FSM + trajectory |
| `hardware/serial_handler.py` | 313 | Medium | Protocol + threads |
| `perception/lane_tracker.py` | 325 | High | Core algorithm |
| `perception/lane_detector.py` | 194 | Medium | Pipeline wrapper |
| `control/controller.py` | 170 | Medium | Stanley + guards |
| `core/telemetry.py` | 177 | Low | Async logger |
| `v2x/v2x_client.py` | 120 | Low | TCP daemon |
| `hardware/imu_sensor.py` | 79 | Low | State reader |
| `perception/camera.py` | 134 | Low | Thread producer |

---

## Architecture Strengths

1. **Modular imports with graceful fallbacks** — all hardware modules have mock stubs; system runs without any hardware attached
2. **Async I/O throughout** — YOLO, telemetry, V2X, camera all run in background threads; main loop is never blocked
3. **Single source of truth** — all constants in `config.py`; no magic numbers scattered across modules
4. **Priority interrupt system** — pedestrian halt cannot be overridden by any lower-priority state
5. **Telemetry-first design** — CSV + video recording for every run enables post-mission debugging

## Architecture Weaknesses

1. **Tkinter as control timer** — `root.after(50ms)` jitters ±5ms; not real-time; unsuitable for safety-critical production
2. **No EKF fusion in main loop** — localizer module exists but not wired; position is dead-reckoning only
3. **CSV parking trajectory** — parking replay depends on pre-recorded file; not adaptive to new positions
4. **Rough sign distance estimation** — bounding-box-height heuristic; not calibrated per sign class
5. **Single camera, no LIDAR** — no depth information; obstacle avoidance is purely vision-based

---

*Architecture Summary — Team OPTINX — BFMC 2026*
