# BFMC 2026 — Troubleshooting Guide
## Team OPTINX | Competition Support Reference

---

## Quick Diagnostics Checklist

```
When something is wrong, check in order:

1. Is STM32 connected?       → UI label shows GREEN / "Connected"
2. Is camera working?        → Camera feed panel shows live image (not black)
3. Is IMU receiving data?    → IMU label shows non-zero R/P/Y values
4. Is loop_hz > 18?          → Hz display in top bar
5. Is YOLO loaded?           → Check console for "[SYS] Warning: Failed to load AI"
6. Is BEV showing white marks? → Lane lines visible as vertical white streaks
7. Is lateral error < ±30px? → BEV overlay text
```

---

## Category A: Hardware Issues

### A1 — STM32 Not Detected

**Symptom:** "DISCONNECTED" after clicking CONNECT CAR; no response

**Diagnosis:**
```bash
lsusb | grep 0483        # Should show STMicroelectronics entry
ls /dev/ttyACM*          # Should show /dev/ttyACM0 or similar
```

**Fixes:**
1. Check USB cable — try a different cable (data cable, not charge-only)
2. Verify STM32 firmware is flashed correctly
3. Add serial permissions: `sudo usermod -aG dialout $USER`, then re-login
4. Try manual port: open `hardware/serial_handler.py`, change fallback to your port:
   ```python
   FALLBACK_PORT = "/dev/ttyACM0"  # or "/dev/ttyUSB0"
   ```

---

### A2 — Camera Not Working (Black Screen)

**Symptom:** Camera feed panel is black; lane detection shows dead reckoning mode

**Diagnosis:**
```bash
libcamera-hello --timeout 5000   # Should open preview window
v4l2-ctl --list-devices           # Lists USB cameras
```

**Fixes:**

*CSI Camera:*
1. Check ribbon cable — must be fully seated, correct orientation (blue side)
2. Enable camera interface: `sudo raspi-config` → Interface Options → Camera
3. Install PiCamera2: `sudo apt-get install python3-picamera2`
4. Check `dmesg | grep camera` for errors

*USB Webcam fallback:*
1. Try `python3 -c "import cv2; cap=cv2.VideoCapture(0); print(cap.read()[0])"`
2. If False, try index 1, 2, etc.
3. In `camera.py`, change `cv2.VideoCapture(0)` to the correct index

---

### A3 — IMU Shows All Zeros

**Symptom:** IMU: R:0.0° P:0.0° Y:0.0° — values never change

**Diagnosis:**
- IMU data comes from STM32 telemetry stream
- STM32 must be connected and IMU enabled

**Fixes:**
1. Verify STM32 is connected first (see A1)
2. Ensure firmware sends `@imu:...;;` messages — check STM32 serial output:
   ```bash
   cat /dev/ttyACM0  # Should show @imu: lines streaming
   ```
3. Verify `#imu:1;;` command was sent on connect (check `serial_handler.connect()`)
4. Check 1.5s timeout in `imu_sensor.py` — if serial is slow, increase `TIMEOUT_S = 3.0`

---

### A4 — Battery Reading Shows 0%

**Symptom:** BAT: 0% (0.0V) in top bar

**Cause:** STM32 not sending `@bat:` messages, or voltage parser failing

**Fix:** Monitor raw serial: `cat /dev/ttyACM0 | grep bat` — if no output, check STM32 firmware battery reporting.

---

## Category B: Lane Detection Issues

### B1 — Car Veers Consistently to One Side

**Symptom:** Car drives offset from lane center; lateral error always positive or always negative

**Cause:** BEV perspective transform is miscalibrated — the center of the warped image doesn't correspond to the physical lane center.

**Fix:**
1. Capture a still frame on the track
2. Run: `python3 simulate_video.py --video calibration_frame.png`
3. Observe BEV panel — lane lines should be near x=110 (left) and x=480 (right)
4. If both lines are shifted right → move `LANE_SRC_PTS` right columns left
5. If both lines are shifted left → move `LANE_SRC_PTS` left columns right
6. Adjust until `target_x ≈ 320` when car is centered in lane

---

### B2 — Oscillating Steering on Straights

**Symptom:** Car weaves left-right at high frequency even on straight sections

**Cause:** Stanley gain K too high, or EMA smoothing too low

**Fixes (in order):**
1. Reduce `STANLEY_K` from 2.5 → 1.5 in `config.py`
2. Increase `CTRL_STRAIGHT_ALPHA` from 0.65 → 0.80
3. Increase `LANE_TARGET_EMA_SLOW` from 0.05 → 0.10 (smoother at low deltas)
4. Increase `STANLEY_KD_YAW` from 0.45 → 0.70 (more IMU damping)

---

### B3 — Lane Detection Lost in Curves

**Symptom:** Anchor shows DEAD_RECKONING on curves; car misses turns

**Cause:** Sliding window search misses lanes when they're at the edge of BEV

**Fixes:**
1. Check `TRACKER_SW_MARGIN` — increase from 60 → 80px for wider search
2. Reduce `TRACKER_MINPIX` from 50 → 30 (accept fewer pixels as valid)
3. Check `LANE_THRESHOLD` — if lighting is dim, reduce from 155 → 130
4. Reduce `LANE_BRIGHT_LOW` from 80 → 60 to allow more brightening
5. Increase `TRACKER_POLY_MARGIN_CURV` from 120 → 150 (wider tracking band on curves)

---

### B4 — BEV Shows Too Much Noise (False Lane Detections)

**Symptom:** BEV binary image has many white blobs; polynomial fits jump around

**Cause:** Lighting conditions creating false bright areas; threshold too low

**Fixes:**
1. Increase `LANE_THRESHOLD` from 155 → 170
2. Increase `LANE_MORPH_KERNEL` from (5,5) → (7,7) — closes noise clusters
3. Add ROI mask: in `lane_detector.py`, zero out the top 100 rows of BEV (sky/ceiling noise)
4. Check `LANE_CLIP_MASK_WARPED` — expand to mask more of the car hood area

---

### B5 — Dead Reckoning Not Working Correctly

**Symptom:** When lanes are lost, car turns wrong direction

**Cause:** IMU yaw polarity or scaling mismatch

**Fix:**
In `lane_tracker.py`, `DeadReckoningNavigator`:
```python
target = 320.0 - (delta_yaw * 20.0)
```
If car turns wrong way: change sign to `+ (delta_yaw * 20.0)`.
If correction is too strong: reduce 20.0 → 10.0.

---

## Category C: Traffic & Sign Issues

### C1 — YOLO Not Detecting Signs

**Symptom:** No detections in BEV panel YOLO section; `active_labels` always empty

**Diagnosis:**
```bash
python3 -c "
from ultralytics import YOLO
m = YOLO('assets/Niranjan.pt')
print('Classes:', m.names)
"
```

**Fixes:**
1. Verify model path: `ls assets/Niranjan.pt`
2. Try lower confidence: change `conf=0.25` → `conf=0.10` in `traffic_module.py`
3. Check if YOLO thread is running: add `print("YOLO detect called")` in `ThreadedYOLODetector._worker`
4. Check PyTorch: `python3 -c "import torch; print(torch.__version__)"`

---

### C2 — Car Doesn't Stop at Stop Signs

**Symptom:** Stop sign detected (shows in YOLO labels) but car doesn't halt

**Cause:** Sign detected but distance check fails (FAR category → no action)

**Diagnosis:** Add to `traffic_module.py`:
```python
print(f"Stop sign dist category: {cat}, box_h={box_h}")
```

**Fixes:**
1. Reduce `APPROACH` threshold from 30px → 20px in `_dist_cat()`
2. Check `sign_approach_m` in telemetry CSV — if always > 2.0m, sign is being categorized as FAR
3. Increase sign detect distance: `SIGN_ACT_DEFAULT_M` from 2.0 → 3.0 in `config.py`

---

### C3 — Red Light Not Halting Car

**Symptom:** Traffic light detected but car doesn't stop at red

**Diagnosis:** Check `light_status` in BEV overlay — should show `[RED]`

**Fixes:**
1. Check `_parse_light_color()` in `traffic_module.py` — verify HSV ranges for red detection
2. In `main.py` control loop, check halt logic:
   ```python
   if active_sign_cmd == "traffic-light" and "GREEN" not in light_status:
       target_speed = 0.0
   ```
   Ensure `light_status` is correctly passed from `t_res.light_status`

---

### C4 — Car Doesn't Respond to Pedestrian

**Symptom:** Pedestrian on track but car continues

**Cause:** Pedestrian bounding box not in the "on-road" zone check

**Fix in `traffic_module.py`:**
```python
# Current on-road check: x ∈ [25%, 75%], y > 40%
# Expand if needed:
is_on_road = (0.15 < cx_norm < 0.85) and (cy_norm > 0.30)
```

---

## Category D: System Performance

### D1 — Loop Hz Below 15 (Slow Loop)

**Symptom:** `loop_hz` display shows < 15; car behavior is sluggish

**Diagnosis:** Profile the loop (see DEVELOPER_NOTES.md profiling section)

**Fixes (in order of impact):**
1. Disable web dashboard if running: comment out `WebDashboard` init
2. Reduce camera resolution: `CAMERA_RESOLUTION = (320, 240)` in `config.py`
3. Skip UI update every other frame:
   ```python
   if self._loop_count % 2 == 0:
       self.render_map()
   ```
4. Reduce `LOG_VIDEO_FPS` from 15 → 10 (less disk I/O)
5. Check if WiFi is causing background load: `sudo iw dev wlan0 set power_save off`

---

### D2 — V2X Connection Refused

**Symptom:** `[V2X] Reconnecting...` messages; server not receiving data

**Fixes:**
1. Run server first: `python3 servers/trafficCommunicationServer/TrafficCommunication.py`
2. Check port 5000 is not blocked: `netstat -tlnp | grep 5000`
3. Verify `V2X_SERVER_HOST` in `config.py` matches server IP
4. Use `--no-v2x` flag for offline testing: `python3 main.py --no-v2x`

---

### D3 — High Memory Usage / OOM Kill

**Symptom:** System becomes unresponsive; `dmesg | grep oom` shows kills

**Causes:** YOLO model + PyTorch takes ~600MB RAM; with 4GB RPi this is fine, but 2GB models may OOM.

**Fixes:**
1. Check RAM: `free -h`
2. Swap: `sudo dphys-swapfile swapoff && sudo nano /etc/dphys-swapfile` → `CONF_SWAPSIZE=1024`
3. Export YOLO to ONNX to eliminate PyTorch overhead: `yolo export model=Niranjan.pt format=onnx`

---

## Category E: Map / Navigation

### E1 — A* Path Not Calculating

**Symptom:** "Path Calculated" not showing after setting start/end nodes

**Diagnosis:**
```bash
python3 -c "
import networkx as nx
G = nx.read_graphml('assets/Competition_track_graph.graphml')
print(f'Nodes: {len(G.nodes)}, Edges: {len(G.edges)}')
"
```

**Fixes:**
1. Verify graphml file exists and is readable
2. Check if start/end nodes exist in the graph: `print(list(G.nodes)[:10])`
3. Check if graph is connected: `print(nx.is_connected(G.to_undirected()))`
4. If disconnected, the A* will fail for nodes in different components

---

### E2 — Map Not Rendering (Grey Box)

**Symptom:** Map panel shows grey or empty

**Fixes:**
1. Check SVG file: `ls assets/Track.svg`
2. Verify svglib: `python3 -c "from svglib.svglib import svg2rlg; print('OK')"`
3. Install: `pip install svglib reportlab`
4. Check `map_engine.py` for any SVG parsing errors in console output

---

### E3 — Car Position Drifts Far from Track

**Symptom:** Digital twin (car icon) moves far off the track map

**Cause:** IMU dead-reckoning accumulates position error over time

**Fix:** 
1. Re-calibrate by setting a new A* path (click NAV → set start/end → auto-calibrate)
2. Reduce `MAP_SIM_SPEED_SCALE` from 1.5 → 1.0 if car moves too fast on map
3. Wire EKF localizer for accurate position (see DEVELOPER_NOTES.md)

---

## Emergency Recovery (Competition Day)

### Complete Restart Procedure

```bash
# Kill existing process
pkill -f main.py

# Clear any stale lock files
rm -f /tmp/bfmc_*.lock 2>/dev/null

# Restart
cd ~/BFMC_2026
source .venv/bin/activate
python3 main.py --headless 2>&1 | tee restart.log
```

### Manual Drive Override

If autonomous mode fails completely:
1. Press any key to exit auto mode (or SSH and `pkill -f main.py`)
2. Restart with: `python3 main.py`
3. Click CONNECT
4. Use arrow keys for manual control
5. Complete the run manually

### Disable YOLO (If Crashing)

If YOLO is causing crashes, disable AI:
```python
# In main.py, temporarily:
_AI_AVAILABLE = False
```
System continues with pure lane-following (no sign compliance).

---

*Troubleshooting Guide — Team OPTINX — BFMC 2026*
