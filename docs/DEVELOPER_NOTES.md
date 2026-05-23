# BFMC 2026 — Developer Notes
## Team OPTINX | Internal Engineering Reference

---

## Architecture Decisions & Rationale

### Why Tkinter at 20 Hz?

The control loop is driven by `root.after(50, control_loop)` — Tkinter's scheduler fires every 50ms. This was chosen because:
1. Tkinter is the GUI thread — UI updates must happen there anyway
2. Avoids threading complexity of a separate control thread
3. 50ms is more than enough for STM32 which has a 500ms watchdog

**Risk:** Tkinter event processing can add jitter. Monitor `loop_hz` in telemetry — if it drops below 15 Hz consistently, profile the UI update code.

### Why Not ROS2?

ROS2 was explicitly excluded (see `README.md` constraints). The custom protocol over USB-Serial is simpler to debug on competition day and has zero additional daemon dependencies. `serial_handler.py` replaces `ros2_control` adequately for this scale.

### Why LAB L-Channel, Not HSV?

Early testing used HSV Value channel. White lines under the BFMC track lighting varied significantly in V depending on track surface color and shadow. The L-channel in LAB is perceptually linear and more stable across different illumination angles. Threshold of 155 was empirically set on the indoor carpet track.

**No CLAHE:** CLAHE was tested first (adaptive histogram equalization) but it amplified dark carpet texture into false positive lane candidates. Global brightness normalization (80–160 range) was more reliable.

### Why EMA for Steering vs. Pure Stanley?

Stanley's output is noisy frame-to-frame because:
1. Camera noise → binary image noise → polynomial jitter → target_x jitter
2. Small `target_x` changes at 20 Hz create servo oscillation

EMA on `target_x` (α=0.70 fast, 0.30 medium, 0.05 slow based on delta magnitude) dampens noise without lagging on real lane changes.

**Additionally:** `CTRL_STRAIGHT_ALPHA=0.65` provides a second EMA on steer output specifically on straights, where oscillation is most visible.

### Why Single-Slot Camera Queue?

The camera runs at 30 FPS, the control loop at 20 FPS. Using a multi-frame buffer would introduce 33–133ms latency. The single-slot `put_nowait()` approach ensures the control loop always gets the **newest** frame with zero queue latency.

### Why Priority FSM Instead of Pure Reactive?

A pure reactive controller (largest stimulus wins) would fail when multiple signs are active simultaneously. The priority system ensures:
- **Pedestrian** always wins over **crosswalk hold** over **normal drive**
- **STOP sign** can't be overridden by **priority sign approaching**
- **Parking takeover** doesn't fight **autonomous lane follow**

---

## Known Issues & Workarounds

### Issue 1: `_calib_imu_yaw` drift over long runs

The IMU dead-reckoning kinematics (`car_x += v_ms * cos(car_yaw) * dt`) accumulates heading error over time. The system re-calibrates heading each time a new A* path is set via `calibrate_to_start()`.

**Workaround for long runs:** Set a new path via the NAV mode every ~2 minutes to re-anchor the digital twin position.

**Proper fix:** Wire `localization/fused_localizer.py` into the main loop for EKF pose updates.

### Issue 2: YOLO misses fast-moving vehicles

The YOLO thread processes frames at 5–10 FPS on RPi 5 CPU. Fast-moving cars may transit the frame between inference calls.

**Workaround:** `behavior_controller.py` keeps a 2-second detection memory — if a car was seen within 2s, it's assumed still present.

**Proper fix:** Export `Niranjan.pt` to ONNX, run via OpenCV DNN (no PyTorch overhead, ~2× faster).

### Issue 3: Parking CSV path hardcoded

`execute_parking_playback()` reads `"default_parking.csv"` from the working directory. The file must exist before a parking run.

**Recording a parking CSV:**
1. Drive manually through parking maneuver in a separate recording session
2. Telemetry CSV is logged → extract the parking-relevant rows
3. Rename and format as `default_parking.csv`

### Issue 4: `is_waiting_for_reverse` race condition

If the user toggles autonomous mode off during the 10-second parking wait, `is_waiting_for_reverse` stays True, blocking normal operation.

**Workaround:** `toggle_auto_mode()` should clear `is_waiting_for_reverse`. Currently it doesn't — add:
```python
def toggle_auto_mode(self):
    self.is_auto_mode = not self.is_auto_mode
    self.is_playing_back = False
    self.is_waiting_for_reverse = False  # ADD THIS LINE
    ...
```

### Issue 5: `_approx_dist_m` inaccuracy

```python
def _approx_dist_m(box_h):
    return 120.0 / max(box_h, 1)
```

This assumes a 0.08m sign at 450px focal. Real BFMC signs vary in size. At distances < 0.5m, box_h saturates to 400+px, giving ~0.3m which may still trigger HALT actions unnecessarily.

**Better formula:**
```python
REAL_SIGN_HEIGHT_M = 0.08   # measure actual sign
CAMERA_FOCAL_LENGTH_PX = 450.0  # from camera calibration
dist_m = (REAL_SIGN_HEIGHT_M * CAMERA_FOCAL_LENGTH_PX) / max(box_h, 1)
```
Both constants are in `config.py` — just use them directly.

---

## Module Interaction Map

```
main.py
 ├── perception/camera.py          (read_frame per loop)
 ├── perception/lane_detector.py   (process per loop)
 │    └── perception/lane_tracker.py (find_lanes per process)
 ├── control/controller.py         (compute per loop, if auto)
 ├── hardware/serial_handler.py    (set_speed/set_steering per loop)
 ├── hardware/imu_sensor.py        (get_yaw/roll/pitch per loop)
 ├── traffic/traffic_module.py     (process per loop, async YOLO)
 │    └── traffic/ThreadedYOLODetector (detect per loop, background)
 ├── traffic/behavior_controller.py (compute per loop, if auto)
 ├── parking/parking.py            (update per loop, if auto)
 ├── v2x/v2x_client.py             (update_state per loop, async)
 ├── core/telemetry.py             (log per loop, async CSV/video)
 ├── dashboard/map_engine.py       (update_sign_statuses + render per loop)
 └── dashboard/dashboard_ui.py     (update labels + images per loop)
```

---

## Adding a New Traffic Sign

1. **Train YOLO** — add class to `Niranjan.pt` training dataset
2. **Add to `config.py`**:
   ```python
   SIGN_MAP["new-sign"] = {"name": "New Sign", "emoji": "🆕"}
   ```
3. **Handle in `traffic_module.py`** — add case in `TrafficDecisionEngine.process()`:
   ```python
   elif "new-sign" in labels:
       return TrafficResult(state="SYS_SLOW", speed_multiplier=0.7, ...)
   ```
4. **Add to `signs_database.json`** — place on track with node + coordinates
5. **Test in simulate_video.py** — run a recording with the sign in view

---

## Performance Profiling

Add this to the top of `control_loop()` for timing breakdown:

```python
import time

t0 = time.time()
frame = self.camera.read_frame()
t1 = time.time()

lane_result = self.detector.process(frame, ...)
t2 = time.time()

t_res = self.traffic_engine.process(frame, ...)
t3 = time.time()

# ... rest of loop ...
t_end = time.time()

print(f"Camera:{(t1-t0)*1000:.1f}ms | Lane:{(t2-t1)*1000:.1f}ms | "
      f"Traffic:{(t3-t2)*1000:.1f}ms | Total:{(t_end-t0)*1000:.1f}ms")
```

Typical results on RPi 5:
- Camera read: 0.5ms (queue pop)
- Lane detector: 8–12ms
- Traffic engine: 1–2ms (YOLO result fetch, not inference)
- Behavior compute: 0.5ms
- Stanley: 0.3ms
- UI update: 5–10ms
- **Total:** 20–30ms (headroom: 20ms to 50ms target)

---

## Testing Without Hardware

The system has graceful fallbacks for all hardware:

```python
# Camera: dummy black frames if no camera found
# STM32: mock handler if pyserial not installed or port missing
# IMU: returns 0.0 for all values if no hardware
# V2X: silent client if server not running
```

This allows full offline testing with `python main.py --no-v2x`:
1. Lane detection runs on dummy frames (black) → shows dead reckoning mode
2. YOLO runs on dummy frames → no detections
3. Map renders correctly (no hardware needed)
4. Telemetry logs to CSV (no hardware needed)

For realistic testing, use `simulate_video.py` with a recorded track video.

---

## EKF Integration (Future Work)

`localization/` has:
- `ekf_vehicle.py` — bicycle model EKF (state: x, y, yaw, speed)
- `fused_localizer.py` — orchestrates EKF + map matching
- `semantic_fusion.py` — uses YOLO detections as EKF landmarks

**To wire into main.py:**

```python
# In __init__:
from localization.fused_localizer import FusedLocalizer
self.localizer = FusedLocalizer(self.map_engine.G)

# In control_loop, after IMU and YOLO:
pose = self.localizer.update(
    imu_yaw=self.imu.get_yaw(),
    speed_ms=self.current_speed / 1000.0,
    dt=dt,
    yolo_labels=ai_labels,
    sign_positions=self.path_signs
)
self.car_x = pose.x
self.car_y = pose.y
self.car_yaw = pose.yaw
```

Replace the current dead-reckoning block (lines 820–829 in `main.py`).

---

*Developer Notes — Team OPTINX — BFMC 2026*
