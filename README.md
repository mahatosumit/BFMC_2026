# BFMC 2026 – Autonomous Embedded Driving Platform

**Team:** OPTINX
**Competition:** Bosch Future Mobility Challenge 2026
Official Regulations:
[https://bosch-future-mobility-challenge-competition-regulation.readthedocs-hosted.com/](https://bosch-future-mobility-challenge-competition-regulation.readthedocs-hosted.com/)



## 1. Project Overview

This repository documents the development of Team OPTINX’s autonomous vehicle system for the Bosch Future Mobility Challenge (BFMC) 2026.

The system is built on the official 1:10 BFMC vehicle platform using a Raspberry Pi 5 as the high-level controller and an STM32 as the low-level real-time controller. Development follows the official qualification timeline and focuses on achieving incremental autonomous capabilities while maintaining system stability and embedded feasibility.

The project has completed the first two report stages and is currently in the Qualification Round phase.



## 2. Qualification Milestones and Status

Development has been aligned with the official competition milestones.

### First Report – 22 December 2025

Requirement:
The team should at least control the car with the given start-up code.

Status: Completed

Achievements:

* Official BFMC baseline restored and stabilized
* Raspberry Pi 5 migration completed
* STM32 communication verified
* Motor and steering actuation validated
* Baseline control confirmed under stable runtime

The vehicle operates reliably using the provided start-up framework.



### Second Report – 2 February 2026

Requirement:
The team should link input data to a rough output (e.g., camera to steering control).

Status: Completed

Achievements:

* CSI camera interface stabilized
* Image stream acquisition validated
* Visual input linked to steering command output
* Preliminary perception-to-control mapping implemented
* Serial command interface refined

The system is capable of processing visual input and generating corresponding steering commands.



### Qualification Round – 9 March 2026

Requirement:
The team should demonstrate autonomous features including:

* Lane keeping
* Basic reaction to standard obstacles (e.g., stopping at a stop sign)

Status: Ongoing

Current focus:

* Lane detection and lane-following integration
* Stop sign detection and controlled halt logic
* Closed-loop perception-to-control validation
* Runtime latency benchmarking
* Stability testing under track-like conditions



## 3. System Architecture

### Raspberry Pi 5 (High-Level Controller)

* CSI camera interface
* Vision preprocessing and model inference
* Decision logic generation
* Speed and steering command computation
* USB-Serial communication with STM32

### STM32 (Low-Level Controller)

* PWM motor control
* Steering servo actuation
* IMU handling
* Timing-critical execution
* Safety mechanisms

The architecture separates deterministic real-time actuation from high-level perception and planning, ensuring modularity and reliability.



## 4. Perception Development Status

### Dataset Analysis

Exploratory Data Analysis (EDA) has been conducted to:

* Evaluate class distribution
* Verify annotation consistency
* Assess image resolution and camera perspective alignment
* Identify imbalance and potential edge cases

Insights from EDA are guiding training configuration and augmentation strategy.



### Model Training

* Model architecture: YOLOv8 (nano variant for embedded feasibility)
* Image size: 640
* Batch size: 16
* Epochs: 100+
* Optimizer: AdamW
* Data augmentation enabled

Training metrics including Precision, Recall, mAP50, and mAP50-95 are monitored to evaluate convergence and class-level performance.

Models are currently validated offline. Runtime integration is proceeding in controlled stages aligned with Qualification Round objectives.



## 5. Engineering Constraints and Design Decisions

Key constraints encountered during development:

* CSI camera resource management limitations
* Runtime dependencies on official BFMC services
* Embedded performance constraints on Raspberry Pi 5

Engineering decisions:

* No ROS2 integration
* Direct USB-Serial communication
* Lightweight detection model selection
* Stability prioritized over architectural expansion
* Baseline compliance maintained



## 6. Qualification Objectives

Before the Qualification deadline, the system aims to demonstrate:

1. Stable lane keeping under varying curvature
2. Controlled steering response based on visual input
3. Reliable stop sign detection and vehicle halt
4. Deterministic behavior under repeated trials

Testing is focused on ensuring repeatable and stable performance under constrained track scenarios.



## 7. Repository Structure

```
BFMC_2026/
├── docs/
│   ├── architecture.md
│   ├── track_analysis.md
├── firmware_stm32/
│   ├── main.cpp
├── src/
│   ├── perception/
│   ├── control/
│   ├── communication/
│   ├── planning/
│   ├── main.py
├── models/
├── tools/
└── README.md
```

The repository is structured to maintain modular subsystem development and incremental validation.



## 8. Scope Clarification

At the current stage, the system:

* Meets baseline control requirements
* Establishes perception-to-control linkage
* Is actively preparing autonomous features for Qualification

Full competition readiness and advanced autonomous behaviors remain under development.



## 9. Team

Team Name: OPTINX


---

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
BFMC_2026/
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
git clone https://github.com/mahatosumit/BFMC_2026.git
cd BFMC_2026
pip install -r requirements.txt # Ensure numpy, opencv-python, onnxruntime, networkx, Pillow are installed
```

### 3. Model Setup
Ensure your ONNX YOLOv8 model (`model.onnx` or equivalent) is located in the `assets/` directory as specified in `config.py`.

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
**Team OPTINX** | Ready for BFMC Final Round 2026.
