# BFMC 2026 — Deployment Guide
## Team OPTINX | Raspberry Pi 5 Setup

---

## Prerequisites

| Requirement | Version | Notes |
|-------------|---------|-------|
| Raspberry Pi 5 | 4 GB RAM recommended | 8 GB for development |
| Raspberry Pi OS | Bookworm (64-bit) | Bullseye also works |
| Python | 3.10+ | `python3 --version` |
| STM32 firmware | BFMC official build | Flashed before deployment |
| CSI Camera | Official RPi camera v2/v3 | Or USB webcam as fallback |

---

## Step 1: Flash Raspberry Pi OS

```bash
# Use Raspberry Pi Imager:
# Image: Raspberry Pi OS (64-bit, Bookworm)
# Enable SSH, set hostname, username, password
# Enable SPI/I2C in advanced options if needed
```

After first boot:
```bash
sudo apt-get update && sudo apt-get upgrade -y
sudo raspi-config
# → Interface Options → Camera → Enable
# → Interface Options → Serial Port → Enable (no login shell, enable hardware)
```

---

## Step 2: Install System Dependencies

```bash
# GUI (Tkinter)
sudo apt-get install -y python3-tk

# Camera (PiCamera2 — CSI interface)
sudo apt-get install -y libcamera-dev python3-picamera2 python3-libcamera

# OpenCV system libs (speeds up pip install)
sudo apt-get install -y libopencv-dev python3-opencv

# OpenBLAS for NumPy acceleration
sudo apt-get install -y libopenblas-dev libatlas-base-dev

# Serial port access
sudo usermod -aG dialout $USER
# Log out and back in for group change to take effect

# Verify camera
libcamera-hello  # Should open preview window
```

---

## Step 3: Python Environment

```bash
cd ~
git clone <repo-url> BFMC_2026
cd BFMC_2026

python3 -m venv .venv
source .venv/bin/activate

# Upgrade pip
pip install --upgrade pip

# Core vision + utilities
pip install opencv-python numpy Pillow pyserial networkx svglib reportlab

# YOLOv8 (this auto-installs ultralytics + PyTorch)
pip install ultralytics

# If ultralytics doesn't pull torch on aarch64, install manually:
pip install torch torchvision --index-url https://download.pytorch.org/whl/cpu
```

**Verify install:**
```bash
python3 -c "import cv2; print('OpenCV:', cv2.__version__)"
python3 -c "import ultralytics; print('Ultralytics OK')"
python3 -c "import serial; print('PySerial OK')"
python3 -c "import networkx; print('NetworkX OK')"
```

---

## Step 4: Verify Assets

```bash
ls -la assets/
# Must have:
#   Niranjan.pt                     (YOLOv8 weights)
#   Competition_track_graph.graphml (navigation graph)
#   signs_database.json             (sign positions)
#   Track.svg                       (track map SVG)
```

If any file is missing, copy from development machine:
```bash
# From development machine:
scp assets/Niranjan.pt pi@<RPi_IP>:~/BFMC_2026/assets/
scp assets/Competition_track_graph.graphml pi@<RPi_IP>:~/BFMC_2026/assets/
```

---

## Step 5: Calibrate Perspective Transform

This is the **most critical** step before each competition run.

```bash
# Run with a static frame or recorded video
python3 simulate_video.py --video assets/test_frame.jpg

# OR use a fresh camera capture for calibration
python3 -c "
import cv2
cap = cv2.VideoCapture(0)
ret, frame = cap.read()
cv2.imwrite('calibration_frame.png', frame)
cap.release()
print('Saved calibration_frame.png')
"
```

Then open `calibration_frame.png` in any image editor, find pixel coordinates of lane lines at **y=265** and **y=450**, and update `config.py`:

```python
LANE_SRC_PTS = [
    [LEFT_LINE_X_AT_Y265 - 15, 265],   # top-left
    [RIGHT_LINE_X_AT_Y265 + 15, 265],  # top-right
    [LEFT_LINE_X_AT_Y450 - 15, 450],   # bottom-left
    [RIGHT_LINE_X_AT_Y450 + 15, 450],  # bottom-right
]
```

Restart and verify BEV panel shows near-vertical parallel lane lines.

---

## Step 6: STM32 Connection

```bash
# Verify STM32 is detected
lsusb | grep -i "STM\|0483"
# Should show: Bus XXX Device YYY: ID 0483:XXXX STMicroelectronics

# Check serial port
ls /dev/ttyUSB* /dev/ttyACM* 2>/dev/null
# Usually: /dev/ttyACM0

# Test serial manually (optional)
python3 -c "
import serial
s = serial.Serial('/dev/ttyACM0', 115200, timeout=1)
s.write(b'#kl:30;;\r\n')
print('Written OK')
s.close()
"
```

---

## Step 7: Run the System

### Standard Run (with display connected)

```bash
source .venv/bin/activate
python3 main.py
```

1. Click **CONNECT CAR** to initialize STM32
2. Wait for "Connected to STM32 Hardware successfully" in log panel
3. Use arrow keys to verify manual drive
4. Click **MODE: AUTONOMOUS** when ready

### Headless Run (no display)

```bash
source .venv/bin/activate
python3 main.py --headless
```

Console output:
```
[CTRL] Spd: 150 | Str: +3.2° | Yaw:  45.1° | Pos:(4.17,6.89) | 20Hz
```

### Without V2X (offline testing)

```bash
python3 main.py --no-v2x
```

---

## Step 8: Auto-Start on Boot (Optional)

Create a systemd service for competition day:

```bash
sudo nano /etc/systemd/system/bfmc.service
```

```ini
[Unit]
Description=BFMC 2026 Autonomous Stack
After=multi-user.target

[Service]
Type=simple
User=pi
WorkingDirectory=/home/pi/BFMC_2026
ExecStart=/home/pi/BFMC_2026/.venv/bin/python3 main.py --headless --no-v2x
Restart=on-failure
RestartSec=5
StandardOutput=journal
StandardError=journal

[Install]
WantedBy=multi-user.target
```

```bash
sudo systemctl enable bfmc.service
sudo systemctl start bfmc.service
sudo journalctl -u bfmc -f  # watch live logs
```

---

## Step 9: SSH Remote Monitoring

During track runs, monitor remotely:

```bash
# From laptop on same WiFi:
ssh pi@<RPi_IP>

# Watch system logs
journalctl -u bfmc -f

# Watch latest telemetry CSV
tail -f ~/BFMC_2026/logs/$(ls -t ~/BFMC_2026/logs/*.csv | head -1)

# Check CPU/RAM usage
htop
```

---

## Step 10: SFTP File Sync (VS Code)

The `.vscode/sftp.json` is pre-configured for deployment. In VS Code:
1. Install **SFTP** extension (Natizyskunk)
2. Press `Ctrl+Shift+P` → **SFTP: Config**
3. Verify host IP matches your RPi
4. Press `Ctrl+Shift+P` → **SFTP: Sync Local → Remote**

---

## Competition Day Checklist

```
Pre-Run:
  [ ] Battery charged (LiPo > 80%)
  [ ] STM32 USB cable connected
  [ ] CSI camera ribbon secure
  [ ] WiFi AP accessible (for SSH)
  [ ] config.py LANE_SRC_PTS calibrated for this track/lighting
  [ ] Assets verified (Niranjan.pt, graphml, signs_database.json)
  [ ] logs/ directory has write permission

Startup:
  [ ] python3 main.py (or service auto-started)
  [ ] STM32 connected (green LED in UI or "Connected" log line)
  [ ] IMU shows real values (R/P/Y not 0.0)
  [ ] Camera feed showing track in BEV panel
  [ ] Lane lines visible in BEV (white vertical marks)

Calibration:
  [ ] Set start node on map
  [ ] Click Calibrate (or auto-calibrate on NAV path set)
  [ ] IMU warmup 5s complete ("CALIBRATING" disappears)

Drive:
  [ ] Manual test → wheels respond correctly
  [ ] Autonomous → car centers in lane within 2-3 seconds
  [ ] Monitor: lateral_err_px < ±30px, loop_hz > 18 Hz

Post-Run:
  [ ] Download logs/ via SFTP for analysis
  [ ] Review telemetry CSV for anomalies
```

---

## Common Deployment Issues

| Issue | Fix |
|-------|-----|
| `tkinter` not found | `sudo apt-get install python3-tk` |
| PiCamera2 import error | `sudo apt-get install python3-picamera2` |
| STM32 not detected | Check `lsusb`, verify 0x0483 VID, try `ls /dev/ttyACM*` |
| Permission denied on serial | `sudo usermod -aG dialout $USER`, re-login |
| YOLO import fails | Check torch: `pip install torch --index-url https://download.pytorch.org/whl/cpu` |
| SVG track not rendering | `pip install svglib reportlab` |
| Loop hz drops below 15 | Disable web_dashboard, reduce CAMERA_FPS to 15 |
| NetworkX A* fails | Verify `.graphml` file has `x` and `y` node attributes |

---

*Deployment Guide — Team OPTINX — BFMC 2026*
