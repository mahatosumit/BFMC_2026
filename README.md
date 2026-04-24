# OPTINX BFMC 2026 – Autonomous Embedded Driving Platform

[![Bosch Future Mobility Challenge](https://img.shields.io/badge/BFMC-2026-blue.svg)](https://bosch-future-mobility-challenge-competition-regulation.readthedocs-hosted.com/)
[![Platform](https://img.shields.io/badge/Platform-Raspberry%20Pi%205%20%7C%20STM32-lightgrey.svg)]()
[![Python](https://img.shields.io/badge/Python-3.10%2B-blue.svg)]()

Welcome to **Team OPTINX**'s official repository for the **Bosch Future Mobility Challenge (BFMC) 2026**. 

This project implements a complete autonomous driving stack on a 1:10 scale vehicle. It features real-time lane detection, traffic sign and obstacle recognition using a lightweight ONNX YOLOv8 model, and precise motor/steering control through an STM32 microcontroller. The software is specifically tailored for edge deployment on a **Raspberry Pi 5**, balancing embedded constraints with advanced ADAS capabilities.

---

## 🏎️ Key Features

- **Hybrid Control System**: Run in fully manual (keyboard) mode, autonomous mode, or parking playback mode.
- **Real-Time Computer Vision**: Accelerated Lane detection (adaptive thresholding + CLAHE + Perspective Transforms) running smoothly on a Raspberry Pi.
- **Traffic Intelligence (AI)**: YOLOv8-ONNX based inference for recognizing traffic signs, traffic lights, and pedestrians. Integrated directly into the control pipeline for dynamic speed limits and halting.
- **Map Engine & Digital Twin**: A Tkinter-based Dashboard that features a real-time digital twin of the vehicle. It loads the official `.graphml` track, maps V2X signs, and uses kinematics simulation/dead reckoning.
- **V2X Communication**: Background servers for communicating with intelligent traffic lights and simulated vehicles.
- **Hardware Agnostic**: Fallbacks to simulated input if the STM32 serial connection or IMU sensor isn't physically available.

---

## 🏗️ Architecture

### Hardware
* **High-Level Controller (Raspberry Pi 5)**: Handles image processing, AI inference, path planning, dashboard UI, and decision-making logic.
* **Low-Level Controller (STM32)**: Handles hardware-level PWM generation for the steering servo and DC motor. Communicates with the Raspberry Pi over a USB-Serial connection. Captures real-time IMU telemetry.
* **Sensors**: Standard CSI Camera (vision) and BNO055 IMU (orientation & heading).

### Software Structure

```text
BFMC_QUAL/
├── main.py                     # Primary entry point; sets up UI, connections, and runs the 20Hz control loop
├── config.py                   # Centralized configuration (dimensions, theme, model paths)
├── launch_all.sh               # Shell script to start the V2X servers and the main app
├── README.md                   # This documentation file
│
├── dashboard/                  # UI and Digital Twin Module
│   ├── dashboard_ui.py         # Tkinter layout, sliders, and log panel
│   ├── map_engine.py           # Parses GraphML, pathfinding (A*/Dijkstra), sign placement
│   └── adas_vision_utils.py    # BEV (Bird's Eye View) render utilities and junction logic
│
├── perception/                 # Computer Vision Pipeline
│   ├── camera.py               # GStreamer/CSI camera stream handling
│   ├── lane_detector.py        # Optical Flow + CLAHE based lane extraction
│   ├── lane_tracker.py         # Hybrid Lane Tracker & Dead Reckoning
│   └── perspective_transform.py# IPM (Inverse Perspective Mapping) utilities
│
├── traffic/                    # Semantic Understanding & Behavior
│   ├── traffic_module.py       # YOLO ONNX Inference & Semantic Traffic logic
│   └── behavior_controller.py  # High-level state machine (Highway, Intersection, Stop)
│
├── control/                    # Vehicle Control Loop
│   └── controller.py           # Converts desired trajectory/lane target into Steering & PWM Speed
│
├── hardware/                   # Hardware Interfaces
│   ├── serial_handler.py       # Threaded Serial communication with STM32
│   └── imu_sensor.py           # Interface for Yaw, Pitch, Roll data
│
├── firmware_stm32/             # C++ Firmware for STM32 Microcontroller
│   └── main.cpp                
│
├── v2x/                        # Vehicle-to-Everything
│   └── v2x_client.py           # UDP Client for Traffic light statuses
│
├── servers/                    # V2X Infrastructure Servers (provided by BFMC)
└── assets/                     # Models (ONNX), Maps (SVG/GraphML), and config files
```

---

## 🚀 Setup & Installation

### 1. Prerequisites
- **Raspberry Pi 5** running a Debian-based OS (e.g., Ubuntu or RPi OS 64-bit).
- Python 3.10+
- STM32 setup with the compiled firmware (`firmware_stm32/main.cpp`) flashed onto it.

### 2. Python Dependencies
Clone the repository and install the required packages:
```bash
git clone https://github.com/Team-OPTINX/BFMC_QUAL.git
cd BFMC_QUAL
pip install -r requirements.txt # Ensure numpy, opencv-python, onnxruntime, networkx, Pillow are installed
```

### 3. Model Setup
Ensure your ONNX YOLOv8 model (`Niranjan.onnx` or equivalent) is located in the `assets/` directory as specified in `config.py`.

---

## 🎮 Usage 

To run the full stack including the V2X servers and the dashboard GUI:
```bash
./launch_all.sh
```

To run only the main application (with GUI):
```bash
python3 main.py
```

### Command Line Arguments
- `--headless`: Run without the Tkinter GUI (optimized for raw track performance).
- `--no_v2x`: Disable V2X communication servers.
- `--model PATH`: Override the default YOLO ONNX model path.

---

## 🕹️ Control & Dashboard Interface

Once the application launches, the **Dashboard** gives you full control over the digital twin and physical car:

1. **Connection**: Click `CONNECT CAR` to establish a serial link with the STM32.
2. **Mode Toggle**: Switch between **MANUAL** and **AUTONOMOUS** mode.
3. **Manual Controls**: 
   - `Up/Down` Arrows: Throttle / Reverse
   - `Left/Right` Arrows: Steering
4. **Digital Map**: 
   - *DRIVE Mode*: Click anywhere on the map nodes to teleport the digital twin.
   - *NAV Mode*: Define a Start, Pass-through, and End node to visualize the planned trajectory.
   - *SIGN Mode*: Add or Remove virtual traffic signs to test the AI reaction logic without physical signs.
5. **ADAS Tools**: Toggle ADAS Assist to enable dynamic speed adjustments and emergency stopping.

---

## ⚙️ Development Highlights
- **Performance**: The lane detection utilizes Visual Odometry (Optical Flow) to estimate yaw rates when lines are temporarily lost, keeping the vehicle stable.
- **AI Efficiency**: Ported from PyTorch to **ONNX Runtime** for optimal CPU efficiency on the Raspberry Pi 5.
- **Safety**: Hardcoded stop thresholds for red lights, pedestrians, and stop signs seamlessly override baseline PID outputs.

---
**Team OPTINX** | Ready for BFMC Qualification Round 2026.
