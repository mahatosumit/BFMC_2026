# BFMC 2026 — Full Technical Documentation
## Team OPTINX | Bosch Future Mobility Challenge 2026
### Autonomous Embedded Driving Platform for 1:10 Scale Vehicle

---

> **Platform:** Raspberry Pi 5 + STM32 Microcontroller  
> **Language:** Python 3.10+  
> **Competition:** Bosch Future Mobility Challenge (BFMC) 2026 — Cluj-Napoca, Romania  
> **Status:** Qualification Phase — May 2026  

---

## Table of Contents

1. [Project Overview](#1-project-overview)
2. [Workspace Structure](#2-workspace-structure)
3. [Module-by-Module Explanation](#3-module-by-module-explanation)
4. [System Architecture](#4-system-architecture)
5. [Execution Flow](#5-execution-flow)
6. [Dependency Documentation](#6-dependency-documentation)
7. [AI / Model Documentation](#7-ai--model-documentation)
8. [Configuration Documentation](#8-configuration-documentation)
9. [Debugging & Logging](#9-debugging--logging)
10. [Performance Optimization](#10-performance-optimization)
11. [Deployment Guide](#11-deployment-guide)
12. [Simulation & Testing](#12-simulation--testing)
13. [API / Communication Documentation](#13-api--communication-documentation)
14. [Safety & Reliability](#14-safety--reliability)
15. [Future Improvements](#15-future-improvements)

---

## 1. Project Overview

### Purpose

This project implements a **fully autonomous driving stack** for a 1:10 scale model vehicle competing in the **Bosch Future Mobility Challenge (BFMC) 2026**. The vehicle must navigate a structured indoor track, obey traffic laws, detect and respond to signs and obstacles, and execute complex maneuvers including parking — all without human intervention.

### Main Objectives

| Objective | Implementation |
|-----------|---------------|
| Lane Keeping | Stanley controller + camera-based lane detection |
| Traffic Sign Compliance | YOLOv8 inference + behavior FSM |
| Intersection Handling | Priority-based behavior controller |
| Parking | IMU-guided trajectory replay |
| Route Navigation | A* on GraphML track map |
| Vehicle-to-Infrastructure | TCP/UDP V2X telemetry protocol |
| Real-Time Safety | Priority interrupt system with pedestrian halt |

### Core Features

- **Lane Detection:** Bird's-eye view (BEV) perspective transform → LAB L-channel thresholding → polynomial sliding-window fitting → Stanley steering
- **YOLO-Based Perception:** YOLOv8n real-time detection of 12 object classes (signs, traffic lights, pedestrians, cars)
- **Behavior FSM:** Priority-layered finite state machine (Emergency > Mandatory > Legal > Mission > Normal)
- **Dead Reckoning:** IMU-based lane-center prediction when both lane lines are lost
- **Parking System:** IMU + camera slot detection with CSV trajectory playback
- **V2X Communication:** TCP telemetry streaming to infrastructure server
- **Telemetry Logging:** Async CSV + XVID video recording at 15 FPS
- **Tkinter Dashboard:** Real-time visualization with map, camera, BEV, and telemetry
- **Headless Mode:** Terminal-only operation for RPi deployment without display

### Technologies Used

| Category | Technology |
|----------|-----------|
| Language | Python 3.10+ |
| Computer Vision | OpenCV 4.x |
| AI Inference | Ultralytics YOLOv8 |
| Navigation | NetworkX (GraphML A*) |
| GUI | Tkinter + PIL |
| Serial Comms | PySerial |
| Networking | TCP/UDP sockets (asyncio/threading) |
| Logging | CSV + OpenCV VideoWriter |
| Camera | PiCamera2 (CSI) / Webcam fallback |
| Map Rendering | SVGLib, ReportLab |

### Hardware Integration

```
┌────────────────────────────────────────────────────────────┐
│                    VEHICLE HARDWARE                         │
│                                                            │
│  CSI Camera ──┐                                            │
│               ├──→ Raspberry Pi 5 (Main Compute)           │
│  IMU Sensor ──┘        │                                   │
│                        │ USB-Serial (115200 baud)           │
│                        ↓                                   │
│                   STM32 MCU                                │
│                   ├── DC Motor (speed PWM)                 │
│                   ├── Servo Motor (steering ±30°)          │
│                   └── Battery Monitor (LiPo)               │
└────────────────────────────────────────────────────────────┘
```

### Overall Architecture Summary

The system follows a **sense → plan → act** loop at **20 Hz**:

```
Camera Frame → Lane Detection → Controller → STM32 → Motors
     ↓              ↓              ↓
  YOLO AI    Behavior FSM    Telemetry Log
     ↓              ↓
  Sign FSM    V2X Server
     ↓
 Map Engine (A*)
```

---

## 2. Workspace Structure

```
BFMC_2026-main1/
│
├── main.py                        # Entry point — BFMC_App, 20 Hz control loop
├── config.py                      # Centralized configuration (all tunable constants)
├── simulate_video.py              # Offline lane detector test harness
├── SLOT_Romania.py                # Standalone parking slot detection script
│
├── core/
│   ├── __init__.py
│   └── telemetry.py               # Async CSV + video logger
│
├── perception/
│   ├── __init__.py
│   ├── camera.py                  # PiCamera2/webcam thread producer
│   ├── lane_detector.py           # Full lane detection pipeline (BEV → fit → target)
│   ├── lane_tracker.py            # Hybrid sliding-window + polynomial tracker
│   ├── perspective_transform.py   # BEV warp matrix calculator
│   └── preprocessing.py           # L-channel threshold utility (reusable)
│
├── control/
│   ├── __init__.py
│   ├── controller.py              # Stanley steering + speed scheduling
│   └── readme.md
│
├── hardware/
│   ├── __init__.py
│   ├── serial_handler.py          # STM32 USB-Serial interface
│   └── imu_sensor.py              # IMU data consumer thread
│
├── traffic/
│   ├── __init__.py
│   ├── traffic_module.py          # YOLO traffic decision engine
│   └── behavior_controller.py    # Priority-based behavior FSM
│
├── parking/
│   ├── __init__.py
│   ├── parking.py                 # Parking orchestrator
│   ├── parking_config.py          # Parking-specific constants
│   ├── parking_detector.py        # Slot/car detection logic
│   ├── parking_imu.py             # IMU-based distance tracking
│   ├── parking_slot_manager.py    # Slot occupancy state
│   ├── parking_state_machine.py   # Parking FSM
│   ├── parking_trajectory.py      # CSV trajectory loader
│   └── parking_dashboard.py       # Parking debug overlay
│
├── v2x/
│   ├── __init__.py
│   └── v2x_client.py             # Vehicle-to-Infrastructure TCP client
│
├── dashboard/
│   ├── __init__.py
│   ├── dashboard_ui.py            # Tkinter GUI layout
│   ├── map_engine.py              # GraphML navigation + SVG rendering
│   ├── adas_vision_utils.py       # BEV annotation, JunctionDetector, RoundaboutNav
│   ├── imu_3d_panel.py            # 3D IMU visualization panel
│   └── web_dashboard.py           # MJPEG HTTP stream server
│
├── localization/
│   ├── __init__.py
│   ├── types.py                   # EKF debug dataclass
│   ├── route_geometry.py          # Path geometry utilities
│   ├── graph_map_matcher.py       # Map matching algorithm
│   ├── map_world.py               # World coordinate mapper
│   ├── ekf_vehicle.py             # Extended Kalman Filter (bicycle model)
│   ├── fused_localizer.py         # EKF + map fusion coordinator
│   └── semantic_fusion.py         # YOLO landmark anchoring
│
├── servers/
│   ├── trafficCommunicationServer/
│   │   ├── TrafficCommunication.py  # V2X TCP server (port 5000)
│   │   ├── tcpServer.py
│   │   ├── udpStream.py
│   │   └── locsys_SIM.py
│   └── carsAndSemaphoreStreamSIM/
│       └── udpStreamSIM.py          # Car/semaphore simulator (UDP port 9000)
│
├── assets/
│   ├── Track.svg                   # Competition track map (SVG)
│   ├── Competition_track_graph.graphml  # Navigation graph (nodes + edges)
│   ├── signs_database.json          # 10 traffic sign geo-positions
│   ├── Niranjan.pt                  # YOLOv8 trained model weights
│   ├── imu_axis_config.json
│   └── dashboard_config.json
│
├── logs/                            # Runtime-generated (CSV + video)
├── docs/                            # Documentation
└── .vscode/sftp.json               # SFTP deploy config (RPi target)
```

### Folder Responsibilities

| Folder | Responsibility | Key Interactions |
|--------|---------------|-----------------|
| `core/` | Non-blocking async logging | Called by `main.py` every frame |
| `perception/` | Camera I/O + lane detection | Feeds `controller.py` target_x |
| `control/` | Steering + speed computation | Receives `LaneResult`, outputs PWM |
| `hardware/` | STM32 serial protocol | `main.py` calls set_speed/set_steering |
| `traffic/` | AI sign/obstacle handling | Feeds `behavior_controller` state |
| `parking/` | Parking orchestration | Provides trajectory to `main.py` playback |
| `v2x/` | Infrastructure telemetry | Daemon thread, receives car pose |
| `dashboard/` | Visualization + UI | All modules pipe state here |
| `localization/` | EKF pose estimation | Reads IMU + map; feeds car_x, car_y |
| `servers/` | V2X infrastructure simulation | Launched as subprocesses by `main.py` |

---

## 3. Module-by-Module Explanation

---

### 3.1 `main.py` — Application Entry Point

**Purpose:** Coordinates all subsystems in a 20 Hz Tkinter event loop (`root.after(50, control_loop)`). Acts as the top-level orchestrator: reads sensors, runs perception, computes control, outputs to hardware, and updates the UI.

#### Class: `BFMC_App`

| Method | Purpose | Inputs | Outputs |
|--------|---------|--------|---------|
| `__init__` | Initialize all subsystems | `root` (Tk), `args` (argparse) | Sets up camera, serial, IMU, V2X, UI |
| `control_loop` | Main 20 Hz loop | None (reads self.* state) | STM32 commands, UI updates, telemetry |
| `calibrate_to_start` | Align car pose to start node | None | Sets `_calib_imu_yaw`, `_calib_path_yaw` |
| `execute_parking_playback` | Load CSV trajectory for parking | `reverse: bool` | Populates `playback_queue` |
| `toggle_auto_mode` | Switch MANUAL ↔ AUTONOMOUS | None | `is_auto_mode`, resets calibration |
| `toggle_connection` | Connect/disconnect STM32 | None | `is_connected`, UI label update |
| `_estimate_upcoming_curve` | Look ahead on path for curve | `min_angle_deg` | `"STRAIGHT"`, `"LEFT"`, or `"RIGHT"` |
| `on_map_click` | Handle map canvas click | Tkinter `Event` | Teleport / set route node / place sign |
| `render_map` | Draw track map with car + path | None | PIL image → Tkinter canvas |
| `on_close` | Graceful shutdown | None | Stops all threads, disconnects hardware |

#### Control Loop Step-by-Step

```
1. Compute dt (time since last loop)
2. Read camera frame (Camera thread)
3. Run LaneDetector.process() → LaneResult
4. Run TrafficDecisionEngine.process() → TrafficResult (YOLO)
5. Run ParkingSystem.update() → parking_out
6. Update path sign states (distance + AI vision)
7. Compute controller output (if AUTO and calibrated)
8. Apply behavior output (speed multiplier, state FSM)
9. Handle parking playback override
10. Handle manual keyboard override
11. Smooth speed + steer (α=0.20)
12. Send to STM32 (set_speed, set_steering)
13. Push V2X telemetry
14. Integrate kinematics (dead-reckoning position)
15. Log telemetry row (rate-limited 1 Hz)
16. Write video frame (if recording)
17. Update all UI panels
18. Schedule next loop (root.after 50ms)
```

#### Internal State Variables

| Variable | Type | Description |
|----------|------|-------------|
| `is_auto_mode` | bool | Autonomous vs. manual mode |
| `is_playing_back` | bool | Parking CSV replay active |
| `is_waiting_for_reverse` | bool | Parking wait before reverse |
| `is_calibrating` | bool | 5s IMU warmup at startup |
| `in_highway_mode` | bool | Highway speed multiplier active |
| `crosswalk_timer` | float | Timestamp when crosswalk slowdown ends |
| `priority_timer` | float | Timestamp when priority hold ends |
| `car_x, car_y` | float | World position in metres |
| `car_yaw` | float | Heading in radians (path-aligned) |
| `path` | list | Current A* route node sequence |
| `path_signs` | list | Signs on current route with state |
| `playback_queue` | list | Pending parking commands |

---

### 3.2 `config.py` — Centralized Configuration

**Purpose:** Single source of truth for all tunable constants. Importing `from config import *` exposes all parameters to every module without circular dependencies.

#### Configuration Sections

| Section | Key Constants | Notes |
|---------|--------------|-------|
| PATHS | `SVG_FILE`, `GRAPH_FILE`, `YOLO_MODEL_FILE` | Absolute paths computed relative to `__file__` |
| PHYSICAL | `WHEELBASE_M=0.23`, `CAMERA_FOCAL_LENGTH_PX=450` | Bicycle model params |
| LANE DETECTION | `LANE_SRC_PTS`, `LANE_DST_PTS` | BEV perspective quad — must be calibrated per track |
| LANE THRESHOLD | `LANE_THRESHOLD=155`, `LANE_BRIGHT_LOW=80` | L-channel white detection |
| LANE TRACKER | `TRACKER_NWINDOWS=9`, `TRACKER_SW_MARGIN=60` | Sliding window params |
| STANLEY | `STANLEY_K=2.5`, `STANLEY_KS=0.5`, `STANLEY_KD_YAW=0.45` | Cross-track error gains |
| SPEED | `SPEED_MIN_CURVE_FACTOR=0.45`, `SPEED_BRAKING_DIST_M=1.8` | Speed scheduling |
| TRAFFIC | `CROSSWALK_HOLD_S=5.0`, `PARKING_WAIT_S=10.0` | Behavior timer durations |
| V2X | `V2X_SERVER_HOST`, `V2X_SERVER_PORT=5000` | Server connection |
| TELEMETRY | `LOG_CSV_INTERVAL_S=1.0`, `LOG_VIDEO_FPS=15.0` | Logging rate |

---

### 3.3 `perception/camera.py` — Camera Thread

**Purpose:** Produces camera frames in a background thread with a single-slot queue (always holds the **latest** frame, not a buffered sequence).

#### Class: `Camera`

| Method | Purpose | Inputs | Outputs |
|--------|---------|--------|---------|
| `__init__` | Detect camera (PiCamera2, webcam, dummy) | `sim_video` path | Sets up `_cap` or `_picam` |
| `_worker` | Thread: capture loop | None | Pushes frames to `_queue` |
| `read_frame` | Get latest frame | None | `np.ndarray (640×480 BGR)` or `None` |
| `stop` | Graceful shutdown | None | Joins thread, releases capture |

#### Threading Model

```
Main Thread                       Worker Thread
    │                                  │
    │ ← Camera._queue.get_nowait() ←── │ ← PiCamera2/VideoCapture.read()
    │                                  │
    │  (drops old frame on queue)      │  (blocks until new frame)
```

The queue has `maxsize=1`. The worker does `queue.put_nowait()`, which drops the old frame if main thread hasn't consumed it — ensuring **zero-latency** access to the newest frame.

**Fallback Hierarchy:**
1. **PiCamera2** (CSI camera on RPi) — 3 retries with 1s delay
2. **DirectShow webcam** (Windows: `cv2.CAP_DSHOW`)
3. **Default webcam** (Linux: index 0)
4. **Dummy generator** (black 640×480 frames — allows UI to run without hardware)

---

### 3.4 `perception/lane_detector.py` — Lane Detection Pipeline

**Purpose:** Converts a raw BGR camera frame into a `LaneResult` containing polynomial lane fits, target x-coordinate for steering, lateral error, curvature, and confidence.

#### Dataclass: `LaneResult`

```python
@dataclass
class LaneResult:
    warped_binary: np.ndarray      # 640×480 binary BEV mask
    lane_dbg: np.ndarray           # BGR debug overlay for UI
    sl, sr: np.poly1d              # Left/right 2nd-degree polynomial fits
    target_x: float                # Pixel x to steer toward (center = 320)
    lateral_error_px: float        # signed: positive = car is right of center
    anchor: str                    # Detection mode label (see below)
    confidence: float              # 1.0 dual | 0.5 single | 0.0 dead-reckon
    curvature: float               # Road curvature from polynomial
    heading_rad: float             # Lane tangent angle at lookahead point
    imu_yaw_rate: float            # IMU gyro yaw rate (deg/s)
```

#### Anchor Values (Detection Mode)

| Anchor | Meaning |
|--------|---------|
| `CENTER_DUAL` | Both lanes found, target = midpoint |
| `LEFT_LANE_ONLY` | Only left lane; target = left + half-width |
| `RIGHT_LANE_ONLY` | Only right lane; target = right - half-width |
| `DEAD_RECKONING` | No lanes; target from IMU yaw prediction |
| `GHOST_LEFT` | Right lane only + ghost projection |
| `GHOST_RIGHT` | Left lane only + ghost projection |
| `ROUNDABOUT_INNER/OUTER` | Roundabout navigation mode |
| `JUNCTION_RIGHT/LEFT/EDGE` | Intersection edge tracking |

#### Full Pipeline

```
Raw Frame (BGR, 640×480)
    │
    ▼
Resize to 640×480
    │
    ▼
Optical Flow (Lucas-Kanade for velocity estimate)
    │
    ▼
Perspective Warp → Bird's-Eye View (BEV)
  [Using LANE_SRC_PTS / LANE_DST_PTS from config]
    │
    ▼
LAB Colorspace → L-channel extraction
  [No CLAHE — removed because it amplified road texture noise]
    │
    ▼
Brightness Normalization
  [if mean(L) < 80: brighten | if mean(L) > 160: darken]
    │
    ▼
Gaussian Blur (5×5)
    │
    ▼
Global Threshold (L > 155)
    │
    ▼
Morphological Close (5×5 kernel)
  [Bridges gaps in dashed white lines]
    │
    ▼
HybridLaneTracker → Left + Right polynomial fits
    │
    ▼
EMA Smoothing on target_x (adaptive alpha: 0.70 fast / 0.30 med / 0.05 slow)
    │
    ▼
LaneResult (returned to main loop)
```

#### Function Summary

| Function | Purpose | Inputs | Outputs |
|----------|---------|--------|---------|
| `LaneDetector.__init__` | Initialize tracker, EMA state | None | — |
| `LaneDetector.process` | Full frame processing | frame, dt, velocity, steering, yaw, curve | `LaneResult` |
| `_warp_frame` | BEV perspective transform | BGR frame | warped BGR |
| `_threshold` | L-channel binary mask | BGR warped | binary mask |
| `_compute_target_x` | Lookahead target from fits | binary, fits, mode | target_x, anchor, confidence |
| `_ema_update` | Adaptive EMA smoother | raw target, prev EMA | smoothed target |

---

### 3.5 `perception/lane_tracker.py` — Hybrid Sliding-Window Tracker

**Purpose:** Implements the two core lane-finding algorithms — **histogram sliding-window search** (cold-start) and **polynomial band search** (warm-start tracking) — plus dead reckoning fallback.

#### Classes

**`DeadReckoningNavigator`**

Predicts lane center when both lines are lost, using IMU yaw change since last good detection.

```python
predicted_x = 320.0 - (delta_yaw_deg * 20.0)
```

Confidence decays from 1.0 → 0.0 over 3–5 seconds.

**`HybridLaneTracker`**

| Method | Purpose |
|--------|---------|
| `find_lanes` | Main entry: sliding window or poly search |
| `_sliding_window_search` | Histogram peak → stacked window search |
| `_poly_search` | Search within band around previous fit |
| `_accept_fit` | Sanity check on pixel count and width |
| `_ema_fit` | Exponential smooth polynomial coefficients |
| `get_target_x` | Compute target from fits + navigation mode |

#### Tracking State Machine

```
               ┌────────────────────────────────┐
               │          SEARCH                │
               │  (sliding window histogram)    │
               └──────────┬─────────────────────┘
                          │ ≥ MIN_PIX_OK pixels found
                          ▼
               ┌────────────────────────────────┐
               │         TRACKING               │
               │  (polynomial band search)      │
               └──────────┬─────────────────────┘
                          │ < MIN_PIX_OK for STALE_FIT_FRAMES
                          ▼
               ┌────────────────────────────────┐
               │       DEAD RECKONING           │
               │  (IMU yaw prediction)          │
               └────────────────────────────────┘
```

#### Width Sanity Guard

Dual-lane fits are **rejected** when:
- Separation < `TRACKER_WIDTH_SANE_MIN` (180 px) — lines collapsed
- Separation > `TRACKER_WIDTH_SANE_MAX` (420 px) — noise spike
- Only the weaker lane is dropped; the stronger is kept as single-lane

#### Navigation Modes

| Mode | Behavior | Use Case |
|------|---------|---------|
| `NORMAL` | Center of dual lanes; single with half-width offset | Default |
| `ROUNDABOUT_INNER` | Left line + width/2 (clockwise inner) | Roundabout |
| `ROUNDABOUT_OUTER` | Right line - width/2 (clockwise outer) | Roundabout |
| `JUNCTION_RIGHT` | Follow right edge | Turn right |
| `JUNCTION_LEFT` | Follow left edge | Turn left |

---

### 3.6 `control/controller.py` — Stanley Controller

**Purpose:** Computes steering angle from lane error and manages speed scheduling based on curvature, braking zones, and dead-reckoning confidence.

#### Stanley Control Law

```
cross_track_error = target_x - 320.0          # pixels, positive = right
heading_error     = lane_result.heading_rad    # radians
yaw_rate          = imu.get_yaw_rate()         # deg/s

delta = heading_error + arctan(K * cross_track_error / (Ks + v_ms))
      - Kd_yaw * yaw_rate                     # IMU damping term
```

Parameters: `K=2.5`, `Ks=0.5`, `Kd_yaw=0.45`

#### Classes

| Class | Purpose |
|-------|---------|
| `StanleyController` | Pure Stanley law (heading + lateral) |
| `DividerGuard` | Prevents contact with left divider / right edge |
| `Controller` | Wraps both + speed scheduling |

#### Speed Scheduling Logic

```
base_speed
    │
    ├── if |steer| < 5°: × SPEED_STRAIGHT_BONUS (1.15), capped at 1.20
    ├── if upcoming_curve and distance < BRAKING_DIST_M: ramp to MIN_CURVE_FACTOR (0.45)
    ├── if confidence < 0.5 (dead reckoning): × 0.40–0.80 penalty
    └── if DividerGuard active: × 0.75
```

#### DividerGuard Logic

Monitors left divider proximity and right edge proximity in bird's-eye view:
- Safe margin: 130 px from divider, 100 px from right edge
- Trigger threshold: gap < 115 px
- Correction gain: 0.35, max: 25°
- Speed reduction: proportional to gap error

#### Controller Output

```python
@dataclass
class ControlOutput:
    speed_pwm: float       # 0–255
    steer_angle_deg: float # ±30°
    reason: str            # Debug label
```

---

### 3.7 `hardware/serial_handler.py` — STM32 Interface

**Purpose:** Manages the USB-Serial connection to the STM32 microcontroller, implementing the BFMC communication protocol.

#### Protocol

```
Frame format:  #<command>:<value>;;\r\n

Commands:
  #speed:150;;     → set motor PWM to 150
  #steer:25;;      → set steering to 2.5° (value ×10 internally)
  #alive:0;;       → heartbeat keepalive
  #kl:30;;         → ignition ON (KL30 signal)
  #kl:0;;          → ignition OFF
  #brake:1;;       → emergency brake
  #imu:1;;         → enable IMU telemetry stream

IMU response:  @imu:<roll>;<pitch>;<yaw>;<ax>;<ay>;<az>;;
```

#### Connection Procedure

1. Enumerate serial ports
2. Filter by USB VID `0x0483` (STM32 manufacturer ID)
3. If not found, try `/dev/ttyAMA0` (RPi UART fallback)
4. Open at 115200 baud, 1.5s timeout
5. Send `#kl:30;;` to activate ignition
6. Start `read_loop` thread (parses incoming IMU data)
7. Start `heartbeat_loop` thread (sends `#alive:0;;` every 200ms)

#### Threading Architecture

```
Main Thread          read_loop Thread       heartbeat_loop Thread
    │                      │                       │
    │ set_speed(150) ──→   │                       │
    │ set_steering(10) →   │                       │
    │                      │ parse @imu:...        │
    │                      │ → SHARED_STATE dict   │
    │                      │                       │ → #alive:0;;  (200ms)
    │ ← SHARED_STATE ──── │                       │
    │   .yaw, .pitch, .roll│                       │
```

#### Class: `STM32_SerialHandler`

| Method | Purpose |
|--------|---------|
| `connect()` | Auto-detect + open serial port |
| `disconnect()` | Close port, stop threads |
| `set_speed(pwm)` | Send `#speed:N;;` command |
| `set_steering(deg)` | Send `#steer:N;;` (value ×10) |
| `set_light_state(state, on)` | Control indicators |
| `_read_loop()` | Parse IMU stream → SHARED_STATE |
| `_heartbeat_loop()` | 200ms keepalive sender |

---

### 3.8 `hardware/imu_sensor.py` — IMU Data Consumer

**Purpose:** Wraps the `SHARED_STATE` dict from `serial_handler` into a clean interface. Monitors connection health and exposes roll, pitch, yaw.

| Method | Returns |
|--------|---------|
| `get_yaw()` | Current yaw (degrees, wraps ±180) |
| `get_pitch()` | Current pitch (degrees) |
| `get_roll()` | Current roll (degrees) |
| `get_has_hardware()` | True if IMU data received < 1.5s ago |
| `start()` / `stop()` | Thread lifecycle |

**Connection Health:** If no IMU update arrives within 1.5s, `has_hardware` flips to `False` and `is_calibrated` reverts to prevent autonomous driving with stale orientation data.

---

### 3.9 `traffic/traffic_module.py` — YOLO Traffic Engine

**Purpose:** Runs YOLOv8 inference in a background thread and applies rule-based sign logic to produce `TrafficResult`.

#### Class: `ThreadedYOLODetector`

Wraps YOLO model in a daemon thread. Uses two queues:
- `_in_queue (maxsize=1)`: Always holds the latest frame
- `_out_queue (maxsize=1)`: Always holds the latest result

```python
detector = ThreadedYOLODetector("assets/Niranjan.pt")
results = detector.detect(frame)  # Non-blocking; returns None if no result yet
```

#### Class: `TrafficDecisionEngine`

**Output: `TrafficResult`**

| Field | Description |
|-------|-------------|
| `state` | SYS_GO, SYS_STOP, SYS_SLOW, SYS_APPROACH |
| `speed_multiplier` | 0.0 (full stop) to 1.3 (highway) |
| `zone_mode` | `"CITY"` or `"HIGHWAY"` |
| `parking_state` | NONE → SEEK → ENTER → WAIT → EXIT → DONE |
| `light_status` | `[RED]`, `[YELLOW]`, `[GREEN]`, `NONE` |
| `active_labels` | List of YOLO class names this frame |
| `sign_approach_m` | Estimated distance to nearest sign (metres) |

#### Sign Logic Summary

| Sign | Trigger | Action | Duration |
|------|---------|--------|---------|
| stop-sign | box_h > 30px | Full stop | 5 seconds |
| red-light | red detected | Full stop | Until green |
| yellow-light | yellow detected | 55% speed | Until change |
| crosswalk-sign | approach | 80% speed | 5s hold |
| pedestrian | on-road (x∈25%–75%) | Emergency stop | Until clear + 1.5s |
| priority-sign | approach | 80% speed | 10s |
| highway-entry | detected | Zone = HIGHWAY | Until exit sign |
| highway-exit | detected | Zone = CITY | Persistent |
| no-entry | detected | Full stop | Refuses path |
| roundabout-sign | approach | 50% speed | Persistent |
| parking-sign | approach | Trigger parking FSM | Until done |

#### Distance Estimation

```python
def _approx_dist_m(box_h):
    return (REAL_SIGN_HEIGHT_M * CAMERA_FOCAL_LENGTH_PX) / max(box_h, 1)
    # ≈ 120.0 / box_h  (assuming 0.08m sign, 450px focal)
```

Categories:
- `FAR`: box_h < 30px → pre-deceleration
- `APPROACH`: box_h 30–70px → sign-specific action
- `HALT`: box_h > 70px → full compliance

---

### 3.10 `traffic/behavior_controller.py` — Behavior FSM

**Purpose:** Priority-layered controller that overrides lane-following with traffic-law and mission behaviors.

#### Priority Levels

| Priority | Label | Condition | Action |
|----------|-------|-----------|--------|
| 0 | EMERGENCY | Pedestrian on road / collision TTC | Full stop (speed=0) |
| 1 | MANDATORY | Red light / STOP sign | Full stop (3s hold) |
| 2 | LEGAL | No-entry / bus lane | Stop or steer correction |
| 3 | MISSION | Parking / roundabout / overtake | Specialized FSM |
| 4 | NORMAL | Crosswalk slow / yellow light / highway | Speed multiplier |

#### OvertakeStateMachine

```
IDLE ──────────────────────────────────────────────┐
  │ dashed line detected + obstacle in path        │
  ▼                                                │
CHANGE_LEFT  (1.5s, steer bias: -12°)              │
  │                                                │
  ▼                                                │
PASS  (2.0s, 70% speed)                            │
  │                                                │
  ▼                                                │
CHANGE_RIGHT  (1.5s, steer bias: +12°)             │
  │                                                │
  └──────────────────────────────────────────────→─┘
```

#### Output: `BehaviorOutput`

```python
@dataclass
class BehaviorOutput:
    speed_pwm: float        # 0–255
    steer_deg: float        # ±30°
    priority: int           # 0–4
    state: str              # FSM state label
    reason: str
    zone_mode: str          # CITY or HIGHWAY
    maneuver: str           # NONE, OVERTAKE, PARKING, ROUNDABOUT
```

---

### 3.11 `parking/parking.py` — Parking Orchestrator

**Purpose:** Detects parking spots, manages slot occupancy, and triggers trajectory replay via `main.py`.

#### Parking FSM States

```
State 0: IDLE         → awaiting parking sign trigger
State 1: TRIGGERED    → 0.3s delay (debounce)
State 2: SCANNING     → scanning for clear spot (6s timeout)
State 3: SELECTED     → slot chosen
State 4: APPROACHING  → driving toward spot
State 5: REVERSE_READY → pre-reverse position
State 6: TRAJECTORY_LOAD → load parking CSV
State 7: REVERSE      → main.py playback takes over
```

#### Output Dict

```python
{
    "parking_completed": bool,
    "parking_failed": bool,
    "selected_slot": int,
    "selected_side": "left" | "right",
    "occupancy_status": {0: "empty", 1: "occupied", ...},
    "trajectory": [(speed, steer, direction, duration_fr), ...],
    "speed_multiplier": float,
    "parking_mode_active": bool,
    "parking_takeover": bool   # True → main.py switches to playback mode
}
```

---

### 3.12 `v2x/v2x_client.py` — V2X Telemetry

**Purpose:** Pushes vehicle state to the infrastructure server over TCP at 0.5 Hz.

#### Protocol Messages

```json
{"reqORinfo": "info", "type": "devicePos",   "value1": 4.17, "value2": 6.89}
{"reqORinfo": "info", "type": "deviceRot",   "value1": 45.2}
{"reqORinfo": "info", "type": "deviceSpeed", "value1": 150.0}
```

- Reconnects automatically on socket failure (3s delay)
- Runs as daemon thread — does not block shutdown
- `update_state()` is thread-safe via lock

---

### 3.13 `dashboard/map_engine.py` — Navigation Engine

**Purpose:** Loads the GraphML track map, renders the SVG background, computes A* paths, manages sign states, and produces a PIL image for the Tkinter canvas.

#### Key Methods

| Method | Purpose |
|--------|---------|
| `calc_path_nodes(start, end, pass_nodes)` | A* with intermediate waypoints |
| `get_path_signs(path)` | Returns signs on route in order |
| `update_sign_statuses(signs, labels, dist, ...)` | PENDING→DETECTING→ACTING→COMPLETED |
| `render_map(car_x, car_y, yaw, path, ...)` | Returns PIL image for display |
| `remove_sign(node_id)` | Delete sign from database |
| `save_signs()` | Persist signs_database.json |

#### Sign State Machine (per sign on route)

```
PENDING ──→ DETECTING (car within detect_dist=5m)
  ──→ ACTING (car within act_dist=2m AND YOLO confirms label)
  ──→ COMPLETED (action executed, no active blocks)
```

---

### 3.14 `core/telemetry.py` — Async Logger

**Purpose:** Non-blocking telemetry to CSV and video. The main 20 Hz loop never waits for disk I/O.

#### Architecture

```
control_loop()
    │ telemetry.log(**fields)      → _csv_queue  (rate-limited 1 Hz)
    │ telemetry.write_frame(frame) → _vid_queue  (every frame)
    │
    └── Worker Thread (daemon)
           │ drains _csv_queue → csv.DictWriter → logs/telemetry_*.csv
           └ drains _vid_queue → VideoWriter → logs/camera_*.avi
```

#### CSV Schema

```
timestamp | loop_hz | mode | speed_pwm | steer_deg | yaw_deg |
roll_deg | pitch_deg | car_x | car_y | lane_anchor | target_x |
lateral_err_px | lane_confidence | active_sign | yolo_labels
```

---

## 4. System Architecture

### High-Level Architecture

```
┌─────────────────────────────────────────────────────────────────┐
│                         RASPBERRY PI 5                          │
│                                                                 │
│  ┌──────────┐    ┌──────────────────────┐    ┌──────────────┐  │
│  │  Camera  │───▶│   Perception Layer   │───▶│  Controller  │  │
│  │ (CSI/USB)│    │  - lane_detector.py  │    │ Stanley+Speed│  │
│  └──────────┘    │  - lane_tracker.py   │    └──────┬───────┘  │
│                  └──────────────────────┘           │          │
│  ┌──────────┐    ┌──────────────────────┐           │          │
│  │   IMU    │───▶│    Traffic Layer     │    ┌──────▼───────┐  │
│  │  (STM32) │    │  - traffic_module.py │───▶│  Behavior    │  │
│  └──────────┘    │  - behavior_ctrl.py  │    │  Controller  │  │
│                  └──────────────────────┘    └──────┬───────┘  │
│  ┌──────────┐    ┌──────────────────────┐           │          │
│  │  YOLO    │───▶│   Planning Layer     │           │          │
│  │  Thread  │    │  - map_engine.py     │    ┌──────▼───────┐  │
│  └──────────┘    │  - parking.py        │    │  Hardware    │  │
│                  └──────────────────────┘    │  serial_hdlr │  │
│  ┌──────────┐    ┌──────────────────────┐    └──────┬───────┘  │
│  │  V2X     │    │   Logging Layer      │           │          │
│  │  Client  │    │  - telemetry.py      │    USB-Serial 115200 │
│  └──────────┘    └──────────────────────┘           │          │
└─────────────────────────────────────────────────────┼──────────┘
                                                       │
                                              ┌────────▼────────┐
                                              │    STM32 MCU    │
                                              │  Motor + Servo  │
                                              └─────────────────┘
```

### Data Flow: Sensor → Decision → Actuation

```
Camera Frame (BGR 640×480)
    │
    ├─── Lane Detector ──────────────────────────────────────────────┐
    │    BEV warp → threshold → sliding window → polynomial fit      │
    │    → LaneResult(target_x, confidence, curvature, heading)      │
    │                                                                │
    ├─── YOLO Detector (threaded) ───────────────────────────────┐   │
    │    YOLOv8n inference → bounding boxes + class labels       │   │
    │    → TrafficResult(state, speed_mult, active_labels)       │   │
    │                                                            │   │
    ├─── IMU Data ───────────────────────────────────────────┐   │   │
    │    STM32 telemetry → yaw, pitch, roll                  │   │   │
    │                                                        │   │   │
    │                                                        ▼   ▼   ▼
    │                                              BehaviorController
    │                                              Priority FSM decision
    │                                              → speed_pwm, steer_deg
    │                                                        │
    │                                              Stanley Controller
    │                                              target_x → steer_deg
    │                                              curvature → speed_pwm
    │                                                        │
    └──────────────────────────────────────────────────→ STM32 Command
                                                      #speed:N;;
                                                      #steer:N;;
```

### Frame-by-Frame Processing Summary

| Step | Module | Time Budget |
|------|--------|------------|
| Camera read | `camera.py` queue | < 1ms |
| Lane detection | `lane_detector.py` | ~8–15ms |
| YOLO inference | `traffic_module.py` (thread) | ~50–150ms async |
| Behavior compute | `behavior_controller.py` | < 2ms |
| Stanley compute | `controller.py` | < 1ms |
| Serial write | `serial_handler.py` | < 2ms |
| V2X push | `v2x_client.py` (thread) | async |
| Telemetry log | `telemetry.py` (thread) | async |
| UI update | `dashboard_ui.py` | ~5–10ms |
| **Total (sync)** | — | **~30–35ms (≤50ms budget)** |

---

## 5. Execution Flow

### Startup Sequence

```
1. python main.py [--headless] [--no-v2x]
         │
         ├── Parse CLI args
         ├── launch_v2x_servers() → subprocess Popen
         │     ├── TrafficCommunication.py (port 5000)
         │     └── udpStreamSIM.py (port 9000)
         │
         └── BFMC_App.__init__(root, args)
                  │
                  ├── DashboardUI(root, app)  [if not headless]
                  ├── MapEngine()             → load GraphML + SVG
                  ├── STM32_SerialHandler()   → auto-detect USB serial
                  ├── IMUSensor().start()     → start read thread
                  ├── V2XClient().start()     → start daemon thread
                  │
                  ├── Camera(sim_video=None)  → start worker thread
                  │     └── try PiCamera2 → try webcam → dummy
                  │
                  ├── LaneDetector()          → init BEV transform
                  ├── Controller()            → init Stanley + DividerGuard
                  │
                  ├── ThreadedYOLODetector("Niranjan.pt") → load model + start thread
                  ├── TrafficDecisionEngine(yolo) → init sign FSMs
                  ├── BehaviorController()    → init priority FSM
                  │
                  ├── ParkingSystem()         → init slot manager + trajectory
                  ├── TelemetryLogger()       → open CSV file
                  │
                  ├── bind keyboard events
                  ├── set_mode("DRIVE")
                  └── root.after(50, control_loop)  → schedule first tick
```

### 20 Hz Control Loop

```
control_loop() called by Tkinter every 50ms
         │
         ├── 1. Compute dt
         ├── 2. camera.read_frame() → frame
         ├── 3. LaneDetector.process(frame) → LaneResult
         ├── 4. TrafficDecisionEngine.process(frame) → TrafficResult
         ├── 5. ParkingSystem.update() → parking_out
         ├── 6. map_engine.update_sign_statuses() → active_sign_cmd
         │
         ├── 7. if is_auto_mode and calibrated:
         │       Controller.compute(LaneResult) → ctrl_out
         │       BehaviorController.compute(lane, traffic) → behav_out
         │       Apply multipliers (crosswalk, highway, pedestrian halt)
         │
         ├── 8. elif is_playing_back:
         │       Pop playback_queue → target_speed, target_steer
         │
         ├── 9. elif not is_auto_mode:
         │       Keyboard keys → target_speed, target_steer
         │
         ├── 10. Smooth speed/steer (EMA α=0.20)
         ├── 11. handler.set_speed() + handler.set_steering() → STM32
         ├── 12. v2x_client.update_state()
         ├── 13. Kinematics (IMU dead-reckoning + path snap)
         ├── 14. telemetry.log() [rate-limited 1 Hz]
         ├── 15. telemetry.write_frame() [if recording]
         ├── 16. Update UI labels + camera/BEV images + map
         └── 17. root.after(50, control_loop)  → reschedule
```

### Shutdown Flow

```
on_close() [called on window close or KeyboardInterrupt]
    │
    ├── camera.stop()          → set _running=False, join thread
    ├── yolo.stop()            → set _running=False, join thread
    ├── handler.set_speed(0)   → brake
    ├── handler.set_steering(0)
    ├── handler.disconnect()   → close serial, stop threads
    ├── imu.stop()             → stop read thread
    ├── v2x_client.stop()      → stop daemon thread
    ├── telemetry.stop()       → drain queues, close files
    └── root.destroy()         → kill Tkinter
```

---

## 6. Dependency Documentation

### Python Package Requirements

```bash
pip install -r requirements.txt
```

| Package | Version | Purpose |
|---------|---------|---------|
| `opencv-python` | ≥4.8 | Camera, BEV transform, morphology, optical flow |
| `numpy` | ≥1.24 | Array operations, polynomial fitting |
| `ultralytics` | ≥8.0 | YOLOv8 inference (`Niranjan.pt`) |
| `Pillow` | ≥10.0 | PIL images for Tkinter rendering, SVG → PNG |
| `pyserial` | ≥3.5 | USB-Serial communication with STM32 |
| `networkx` | ≥3.0 | GraphML loading, A* pathfinding |
| `svglib` | ≥1.5 | SVG → PIL conversion for track map |
| `reportlab` | ≥4.0 | Required by svglib |
| `picamera2` | RPi only | CSI camera capture (optional) |
| `torch` / `torchvision` | ≥2.0 | PyTorch backend for YOLO inference |

### System Requirements (Raspberry Pi 5)

```bash
sudo apt-get install python3-tk     # Tkinter (not pip-installable)
sudo apt-get install libopenblas-dev libatlas-base-dev  # NumPy acceleration
sudo apt-get install libcamera-dev python3-picamera2    # CSI camera
```

### Why Each Dependency

| Package | Engineering Reason |
|---------|-------------------|
| `opencv-python` | Perspective transform (BEV), binary thresholding, morphological close, Lucas-Kanade optical flow, VideoWriter for recording |
| `ultralytics` | Single-import YOLO inference with no custom C++ build required; auto-downloads YOLOv8n architecture |
| `networkx` | GraphML is the native competition track format; NetworkX provides A* out-of-the-box |
| `pyserial` | STM32 appears as CDC-ACM USB serial device; only pure-Python serial driver available on RPi |
| `svglib` | Competition provides track as SVG; rendering as PIL background for the dashboard map |
| `Pillow` | Required by Tkinter's `ImageTk` for frame display; also used by svglib |

---

## 7. AI / Model Documentation

### Model: `Niranjan.pt`

| Property | Value |
|----------|-------|
| Architecture | YOLOv8 nano (`yolov8n`) |
| Framework | Ultralytics / PyTorch |
| Input size | 640×640 (auto-letterboxed) |
| Confidence threshold | 0.25 |
| Device | CPU (RPi 5) |

### Detected Classes

| Class | BFMC Use |
|-------|---------|
| `stop-sign` | 5s stop on approach |
| `crosswalk-sign` | 80% speed + pedestrian watch |
| `priority-sign` | 80% speed hold |
| `parking-sign` | Trigger parking FSM |
| `highway-entry-sign` | Enter highway zone (130% speed) |
| `highway-exit-sign` | Return to city zone |
| `roundabout-sign` | 50% speed, navigate CCW |
| `noentry-sign` | Emergency stop, refuse path |
| `oneway-sign` | Path validation |
| `traffic-light` | Parse light color sub-class |
| `pedestrian` | On-road halt; crosswalk hold |
| `car` | Obstacle / overtake trigger |

### Inference Pipeline

```python
# ThreadedYOLODetector worker thread:
model = YOLO("assets/Niranjan.pt")
model.predict(frame, conf=0.25, device="cpu", verbose=False)
# → Results with .boxes (xyxy, conf, cls)
```

**Threading Strategy:**
- Frame goes into `_in_queue` (maxsize=1, overwrites stale)
- Worker pops frame, runs inference (~50–200ms on RPi 5 CPU)
- Result goes into `_out_queue` (maxsize=1, overwrites stale)
- Main thread calls `detect(frame)` → non-blocking; returns `None` if no result yet

**FPS Estimate (RPi 5, CPU):**
- YOLOv8n: ~5–10 FPS inference
- Since threaded, main control loop is never blocked — perception runs at 20 Hz, AI runs at 5–10 Hz asynchronously

### Lane Detection Algorithm (No ML)

The lane detector uses **classical computer vision** — no neural network:

#### 1. Perspective Transform (Bird's-Eye View)

```python
M = cv2.getPerspectiveTransform(src_pts, dst_pts)
warped = cv2.warpPerspective(frame, M, (640, 480))
```

The 4 source points define a trapezoid in the camera image (close = wide, far = narrow). The 4 destination points map to the full 640×480 output rectangle, "unfolding" the perspective into a top-down view where lane lines appear parallel.

#### 2. L-Channel Thresholding

```python
lab = cv2.cvtColor(warped, cv2.COLOR_BGR2LAB)
L = lab[:,:,0]
# Normalize brightness
if L.mean() < 80:  L = cv2.add(L, 30)   # brighten
if L.mean() > 160: L = cv2.subtract(L, 30)  # darken
binary = (L > 155).astype(np.uint8) * 255
```

White lane lines have L ≈ 200+. Dark carpet/road has L ≈ 50–100. Threshold at 155 gives clean separation. **No CLAHE** — CLAHE amplified road texture to false positives in testing.

#### 3. Morphological Close

```python
kernel = np.ones((5,5), np.uint8)
binary = cv2.morphologyEx(binary, cv2.MORPH_CLOSE, kernel)
```

Bridges gaps in **dashed white lines** (dashes ≈ 30px with 20px gaps in BEV). Close with 5×5 kernel fills gaps without widening lines significantly.

#### 4. Sliding Window Search

```
For each of 9 horizontal slices (y=0..480):
  1. Take column histogram of bottom slice
  2. Find peak → window center
  3. Collect pixels within ±60px of center
  4. If ≥ 50 pixels: re-center next window on their mean x
  5. Fit np.polyfit(pixels_y, pixels_x, deg=2) → polynomial
```

#### 5. Polynomial Tracking (Warm Start)

```python
# Search within ±60px of previous polynomial
x_fit = prev_poly(y) ± TRACKER_POLY_MARGIN_BASE
# Much faster than full sliding window; runs when fit is fresh
```

#### 6. Target Computation

```python
lookahead_y = 180 + curvature * 10000  # farther ahead on straights
target_x = left_poly(lookahead_y) + estimated_width / 2
# or: (left_poly(y) + right_poly(y)) / 2  if both lanes found
```

---

## 8. Configuration Documentation

### Perspective Transform Calibration

```python
# config.py — TUNE THESE VALUES FOR YOUR TRACK
LANE_SRC_PTS = [
    [160, 265],   # top-left  (far-left of left lane line at y=265)
    [370, 265],   # top-right (far-right of right lane line at y=265)
    [ 60, 450],   # bottom-left  (far-left at y=450)
    [400, 450],   # bottom-right (far-right at y=450)
]
```

**Calibration Procedure:**
1. Capture a still frame on the competition track
2. Use `cv2.imshow` with mouse callback to read pixel coordinates of lane lines at y=265 and y=450
3. Set SRC points with 10–20px margin outside the lane line edges
4. Restart and verify BEV shows near-vertical parallel lines

**Safe to Modify:** Yes — main calibration step before every competition run.

### Stanley Gains — Stability Impact

| Parameter | Value | Effect of Increase | Effect of Decrease |
|-----------|-------|-------------------|-------------------|
| `STANLEY_K` | 2.5 | Faster correction, may oscillate | Slower, may drift off center |
| `STANLEY_KS` | 0.5 | Reduces gain at low speed | Higher gain at low speed |
| `STANLEY_KD_YAW` | 0.45 | More IMU damping (smoother) | Less damping (more responsive) |

**Stability Risk:** Increasing K above 4.0 or KD_YAW above 1.0 can cause steering oscillation at speed.

### Speed Parameters

| Parameter | Value | Notes |
|-----------|-------|-------|
| `SPEED_MIN_CURVE_FACTOR` | 0.45 | Minimum speed in sharpest curves; below 0.35 may stall motor |
| `SPEED_BRAKING_DIST_M` | 1.8 | Braking starts 1.8m before curve; increase for faster approach |
| `SPEED_STRAIGHT_BONUS` | 1.15 | Safe up to 1.30 on smooth track; exceeding may cause lane drift |
| `HIGHWAY_SPEED_MULT` | 1.30 | Multiplied with base speed in highway zone |

### Lane Tracker Thresholds

| Parameter | Value | Notes |
|-----------|-------|-------|
| `TRACKER_MINPIX` | 50 | Min pixels to accept a lane — lower for sparse markings |
| `TRACKER_MIN_PIX_OK` | 200 | Min pixels for valid fit — affects SEARCH↔TRACKING transitions |
| `TRACKER_STALE_FIT_FRAMES` | 12 | 12 frames ÷ 20 Hz = 0.6s before dropping stale fit |
| `TRACKER_ESTIMATED_LANE_W` | 340 | Default half-track width in BEV pixels — calibrate from known track width |
| `TRACKER_WIDTH_SANE_MIN/MAX` | 180/420 | Reject dual fits outside this range — prevents ghost/noise lanes |

### Sign Behavior Timers

| Parameter | Value | Notes |
|-----------|-------|-------|
| `CROSSWALK_HOLD_S` | 5.0 | Hold slow speed 5s after crosswalk detected — safe to reduce to 3s |
| `PRIORITY_HOLD_S` | 10.0 | Hold after priority sign — reduce with caution |
| `PARKING_WAIT_S` | 10.0 | Wait before reverse — must be long enough for any pedestrians to clear |

---

## 9. Debugging & Logging

### Logging System

| Logger | Output | Rate | Format |
|--------|--------|------|--------|
| `TelemetryLogger` | `logs/telemetry_TIMESTAMP.csv` | 1 Hz | CSV (16 fields) |
| `TelemetryLogger` | `logs/camera_TIMESTAMP.avi` | 15 FPS | XVID compressed |
| Console (headless) | stdout | 20 Hz | Single-line overwrite (`\r`) |
| UI log panel | Tkinter text widget | Event-driven | Color-coded (SUCCESS/WARN/DANGER) |

### Console Output (Headless Mode)

```
[CTRL] Spd: 150 | Str: +3.2° | Yaw:  45.1° | Pos:(4.17,6.89) | 20Hz
```

### Debug Visualizations

| Window/Panel | Content | Toggle |
|-------------|---------|--------|
| Camera feed (UI left) | Raw frame or YOLO overlay | Always on |
| BEV panel (UI right) | Warped binary + polynomials + debug text | Always on |
| Map panel | Track + path + car position + signs | Always on |
| Parking dashboard | Slot occupancy + IMU trace | `toggle_parking_dashboard()` |
| IMU 3D panel | 3D orientation cube | `open_imu_panel()` button |

### BEV Debug Overlay Text

```
ANCHOR:   CENTER_DUAL        (lane detection mode)
Target X: 318.4              (pixel target for steering)
Lat Error: +2.3px            (lateral error from center)
STEER: +3.2 deg              (current steering output)
SPEED: 150 PWM               (current speed output)
STATE: SYS_GO                (behavior FSM state)
ZONE: CITY                   (highway or city)
YOLO: - stop-sign            (active detections)
      - pedestrian
```

### Common Failure Points

| Symptom | Likely Cause | Fix |
|---------|-------------|-----|
| Car veers to one side | BEV transform miscalibrated | Re-tune `LANE_SRC_PTS` |
| Oscillating steering | Stanley K too high | Reduce `STANLEY_K` to 1.5–2.0 |
| Lane lost on curves | Stale frame threshold | Reduce `TRACKER_STALE_FIT_FRAMES` |
| YOLO not detecting signs | Model or confidence issue | Check `YOLO_MODEL_FILE` path; try `conf=0.15` |
| STM32 not connecting | Wrong VID or port | Check `device_manager`, try manual port override |
| IMU shows 0.0 constantly | Serial not receiving `@imu:` | Verify `#imu:1;;` command sent on connect |
| Car stops after 5s at startup | IMU calibration wait | Normal behavior — wait for `AUTO_CALIBRATION_WAIT_S` |
| Parking trajectory wrong | CSV inversion bug | Re-record parking CSV on fresh run |
| V2X connection refused | Server not started | Run `TrafficCommunication.py` first, or use `--no-v2x` |

### Troubleshooting Guide

**Problem: Car oscillates in lane**
1. Reduce `STANLEY_K` (try 1.5)
2. Increase `STANLEY_KS` (try 1.0) — reduces gain at low speed
3. Increase `CTRL_STRAIGHT_ALPHA` (try 0.80) — heavier smoothing

**Problem: Car hugs one lane line**
1. Check `TRACKER_ESTIMATED_LANE_W` — set to measured BEV lane width
2. Verify `LANE_SRC_PTS` — asymmetric calibration causes center bias

**Problem: YOLO inference crashes**
1. Verify PyTorch version compatible with RPi 5 aarch64
2. Install: `pip install torch --index-url https://download.pytorch.org/whl/cpu`
3. Fallback: disable AI with `_AI_AVAILABLE = False` check

**Problem: Camera shows black frames**
1. Check `libcamera` daemon: `systemctl status libcamera`
2. Run `libcamera-hello` to verify CSI camera
3. Use `sim_video="path/to/test.mp4"` for offline testing

---

## 10. Performance Optimization

### Current Bottlenecks

| Bottleneck | Impact | Current Solution |
|-----------|--------|-----------------|
| YOLO inference (50–200ms) | Frame skipping | Threaded with maxsize=1 queue |
| BEV warp (8–15ms) | 20% of loop budget | Matrix pre-computed, cached |
| Tkinter UI rendering (5–10ms) | Variable latency | Deferred to end of loop |
| Serial write (2ms round-trip) | Low but blocking | Could be async |
| CSV write (disk I/O) | Non-blocking | Queued to worker thread |

### Optimization Recommendations

#### 1. GPU Acceleration (YOLO)
```python
# Use RPi 5's VideoCore VII via OpenCL
model.predict(frame, device="mps")  # if Metal/OpenCL available
# Or export to ONNX for faster CPU inference
model.export(format="onnx", opset=12)
```

#### 2. Reduce BEV Computation
```python
# Pre-scale input to 320×240 before warp (2× faster)
frame_small = cv2.resize(frame, (320, 240))
warped = cv2.warpPerspective(frame_small, M_small, (320, 240))
```

#### 3. Serial Write Async
```python
# Move set_speed/set_steering to a worker thread
# Only risk: 1-frame latency on commands (acceptable at 20 Hz)
```

#### 4. Reduce Tkinter UI Overhead
```python
# Render UI at 10 Hz instead of 20 Hz
if loop_count % 2 == 0:  update_ui()
```

#### 5. Process Frame at Reduced Resolution
The 640×480 BEV is larger than necessary for lane line fitting. A 320×240 BEV would cut warp + threshold time by 4×, with minimal accuracy loss.

---

## 11. Deployment Guide

See the separate [DEPLOYMENT_GUIDE.md](./DEPLOYMENT_GUIDE.md) for full step-by-step instructions.

### Quick Start (Raspberry Pi 5)

```bash
# 1. Clone and setup
git clone <repo-url> BFMC_2026
cd BFMC_2026
python -m venv .venv
source .venv/bin/activate
pip install opencv-python numpy ultralytics Pillow pyserial networkx svglib reportlab

# 2. Verify assets
ls assets/Niranjan.pt assets/Competition_track_graph.graphml assets/signs_database.json

# 3. Connect STM32 via USB

# 4. Run (with GUI)
python main.py

# 5. Run (headless, no display)
python main.py --headless

# 6. Run (no V2X servers)
python main.py --no-v2x
```

---

## 12. Simulation & Testing

### Offline Lane Detector Testing

```bash
python simulate_video.py --video path/to/track_recording.mp4
```

`simulate_video.py` runs `LaneDetector.process()` on each frame and shows:
- Raw frame side by side with BEV
- Polynomial fits overlaid
- Target X and lateral error printed per frame

**Recommended test sequence:**
1. Straight section → verify target_x ≈ 320 ± 15px
2. Left curve → verify steering goes negative (left)
3. Right curve → verify steering goes positive (right)
4. Dashed line section → verify morphological close bridges gaps

### Manual Drive Testing

1. Start `python main.py`
2. Click **CONNECT CAR** button
3. Use arrow keys (Up/Down speed, Left/Right steering)
4. Verify STM32 responds (wheels move)
5. Check BEV panel for lane line detection quality

### Autonomous Mode Validation

1. Set start/end nodes on map
2. Click **MODE: AUTONOMOUS**
3. Wait 5s for calibration
4. Monitor:
   - `lateral_err_px` should stay < ±30px on straights
   - `anchor` should read `CENTER_DUAL` most of the time
   - `loop_hz` should stay ≥ 18 Hz
5. If car drifts: adjust `LANE_SRC_PTS` (most common cause)

### V2X Simulation Testing

```bash
# In separate terminal:
python servers/trafficCommunicationServer/TrafficCommunication.py

# Then run car:
python main.py
# V2X position updates appear in server console
```

---

## 13. API / Communication Documentation

### STM32 Serial Protocol

**Direction:** RPi → STM32

```
Format: #<cmd>:<value>;;\r\n

#speed:150;;     Speed PWM (range: -500 to +500)
#steer:125;;     Steering (value = degrees × 10, so 125 = 12.5°)
#alive:0;;       Heartbeat (must arrive < 500ms or STM32 stops motors)
#kl:30;;         Ignition ON (KL30)
#kl:0;;          Ignition OFF
#brake:1;;       Emergency brake
#imu:1;;         Enable IMU stream
```

**Direction:** STM32 → RPi

```
Format: @<type>:<field1>;<field2>;...;;

@imu:<roll>;<pitch>;<yaw>;<accel_x>;<accel_y>;<accel_z>;;
  Example: @imu:1.2;-0.5;45.3;0.01;-0.02;9.81;;

@bat:<voltage>;<current>;;
  Example: @bat:3.8;1.2;;
```

### V2X TCP Protocol

**Port:** 5000 (TCP)

```json
// Position update
{"reqORinfo": "info", "type": "devicePos", "value1": 4.17, "value2": 6.89}

// Rotation update  
{"reqORinfo": "info", "type": "deviceRot", "value1": 45.2}

// Speed update
{"reqORinfo": "info", "type": "deviceSpeed", "value1": 150.0}

// Sign crossing notification
{"reqORinfo": "info", "type": "signCrossed", "value1": "stop-sign"}
```

### V2X UDP Stream

**Port:** 9000 (UDP)

Receives simulation data from the BFMC infrastructure:
- Semaphore states (traffic light colors per intersection)
- Other vehicle positions (for collision avoidance)

### Web Dashboard (Optional)

**Port:** 8080 (HTTP MJPEG)

```
GET /stream   → MJPEG video stream (camera + YOLO overlay)
GET /status   → JSON telemetry snapshot
```

---

## 14. Safety & Reliability

### Hardware Safety Guards

| Guard | Mechanism | Failure Mode |
|-------|-----------|-------------|
| STM32 Watchdog | 500ms heartbeat timeout | Motors stop if RPi freezes |
| IMU Timeout | 1.5s no-update → `has_hardware=False` | Autonomous mode disabled |
| Camera Fallback | Dummy frames on camera fail | Continues (no lane detection) |
| YOLO Thread | Non-blocking queue | AI failure → `SYS_GO` default (safe) |
| Parking Playback | Pedestrian check every frame | Stops if person detected during parking |

### Software Safety Guards

| Guard | Condition | Action |
|-------|-----------|--------|
| Pedestrian halt | YOLO detects pedestrian in center 50% of frame | `target_speed = 0` immediately |
| Red light halt | Traffic light = RED | `target_speed = 0` until GREEN |
| No-entry halt | `noentry-sign` detected | Full stop, refuse path advance |
| Calibration wait | `time.time() - auto_start_time < 5.0` | Speed = 0, steer = 0 for 5s |
| DividerGuard | Lane gap < 115px | Correction steer + 75% speed |
| Steer rate limit | `STANLEY_MAX_STEER_RATE = 60°/frame` | Prevents wheel jerk |
| Dead reckoning limit | confidence < threshold | 40% speed penalty |

### Recovery Mechanisms

1. **Lane Lost:** Dead reckoning for up to 3–5 seconds using IMU yaw
2. **V2X Offline:** Client silently retries every 3s; vehicle continues normally
3. **STM32 Disconnect:** `is_connected=False`; all commands skipped; UI shows DISCONNECTED
4. **Parking CSV Missing:** Error logged; parking state skipped; normal driving continues
5. **YOLO Model Fail:** `_AI_AVAILABLE=False`; behavior controller skipped; pure lane-follow continues

---

## 15. Future Improvements

### Short-Term (Competition-Ready)

1. **Perspective Transform Auto-Calibration**
   - Use checkerboard pattern to compute optimal `LANE_SRC_PTS` automatically
   - Eliminates manual recalibration between runs

2. **Better Sign Distance Estimation**
   - Current: `120 / box_h` (rough inverse-size approximation)
   - Better: Use known sign physical height + focal length for true pinhole distance
   - Even better: Train a separate depth estimation head on the YOLO model

3. **EKF Fusion Integration**
   - `localization/` has EKF code written but not wired into `main.py`
   - Wire `fused_localizer.py` into control loop for accurate world position
   - Replace IMU dead-reckoning kinematics with EKF-fused pose

4. **Roundabout Navigation**
   - Current: `ROUNDABOUT_INNER/OUTER` mode exists but trigger logic incomplete
   - Need: Entry/exit detection + CCW loop count + lane bias switching

5. **Parking System Completion**
   - Current parking FSM reaches state 6 but trajectory must be pre-recorded CSV
   - Better: Real-time IMU + ultrasonic distance for dynamic parking trajectory computation

### Medium-Term

6. **ROS2 Migration**
   - Replace direct serial protocol with `ros2_control` for standard hardware abstraction
   - Publish `/cmd_vel` instead of raw PWM; subscribe to `/imu/data` and `/camera/image_raw`
   - Enables reuse of ROS2 nav stack (costmaps, planners)

7. **ONNX / TFLite Export for YOLO**
   - Export `Niranjan.pt` → ONNX → run via OpenCV DNN module
   - Eliminates PyTorch dependency on RPi; ~2× faster inference

8. **Semantic Map Anchoring**
   - `localization/semantic_fusion.py` exists — wire YOLO detections to known sign positions
   - When YOLO sees a `stop-sign` at known GPS position, anchor EKF state

9. **Multi-Camera Support**
   - Add rear-facing camera for parking slot detection
   - Reduces dependency on YOLO for parking (camera is faster than IMU integration)

10. **Lane Change Completion**
    - Current overtake FSM uses fixed-time lane changes (1.5s)
    - Replace with lane-detection-based transitions: wait until target lane is detected before committing

### Long-Term

11. **SLAM Integration**
    - Replace GraphML + IMU dead-reckoning with online SLAM
    - Use ORB-SLAM3 or RTAB-Map for simultaneous mapping and localization
    - Enables recovery from path deviations without pre-mapped track

12. **End-to-End Learning**
    - Collect telemetry CSV + camera recordings (already implemented)
    - Train behavioral cloning model on expert driving logs
    - Use as fallback when classical lane detection fails

13. **Hardware Upgrade**
    - Replace RPi 5 with Jetson Nano / Orin for GPU inference
    - Add LIDAR for obstacle detection and true depth measurement
    - Add stereo camera for disparity-based lane width estimation

---

*Documentation generated: 2026-05-18 | Team OPTINX | BFMC 2026*
