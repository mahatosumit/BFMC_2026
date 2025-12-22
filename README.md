# BFMC 2025 – Autonomous Vehicle Platform

**Team:** OPTINX
**Competition:** Bosch Future Mobility Challenge 2025 (https://boschfuturemobility.com/)

---

## 1. Overview

This repository contains the hardware, firmware, and software development work for **BFMC 2025 (Bosch Future Mobility Challenge)**.

The project focuses on building a **modular autonomous vehicle development platform** using a **Raspberry Pi + STM32** architecture. Development is carried out incrementally, starting from hardware bring-up and baseline stabilization toward perception-assisted autonomy in later phases.

---

## 2. Current Phase Focus (Phase 1)

The current development phase prioritizes **system stability and compliance** rather than performance optimization.

Key focus areas:

* Hardware bring-up and validation
* Restoration and stabilization of the official BFMC baseline environment
* Raspberry Pi ↔ STM32 communication verification
* Dataset exploration and offline perception feasibility studies
* Preliminary race track analysis
* Preparation for the BFMC first submission

---

## 3. Project Objectives

1. Verify correct operation of the BFMC hardware kit
2. Establish a reliable Raspberry Pi–STM32 command interface
3. Design a scalable and modular software architecture
4. Perform **offline** perception model feasibility studies
5. Analyze BFMC race track constraints for design guidance
6. Transition incrementally from baseline control toward autonomous behavior

---

## 4. System Architecture (Current State)

### 4.1 High-Level Design

**Raspberry Pi**

* Camera interfacing and vision experimentation
* High-level decision logic (currently minimal / rule-based)
* Generation of speed and steering commands
* Communication with STM32 via USB-Serial

**STM32**

* Motor control
* Steering actuation
* Low-level timing-critical control
* Safety handling

> **Note:**
> No closed-loop autonomous control is currently claimed.
> FSM-based decision logic and vision-driven autonomy are planned for later phases.

---

## 5. Repository Structure

```
BFMC_2025/
├── docs/                     # Design documents, reports, track analysis
│   ├── architecture.md
│   ├── track_analysis.md
│   └── submission_notes.md
│
├── firmware_stm32/           # STM32 firmware (CubeMX / HAL)
│
├── src/                      # Raspberry Pi source code
│   ├── perception/           # Vision and detection modules (experimental)
│   ├── planning/             # FSM and decision logic (planned)
│   ├── control/              # Vehicle control logic
│   ├── communication/        # Pi–STM32 communication
│   └── main.py               # Entry point
│
├── models/                   # Trained models (weights excluded)
├── tools/                    # Utility scripts and debugging tools
├── tests/                    # Hardware and software test scripts
│
├── requirements.txt          # Python dependencies (non-system)
├── .gitignore
└── README.md
```

---

## 6. Hardware Validation

Initial hardware verification was performed after receiving the BFMC kit.

| Component | Test Description                 | Status |
| --------- | -------------------------------- | ------ |
| Motors    | Direction and basic response     | OK     |
| Steering  | PWM response and centering       | OK     |
| Camera    | OS-level detection (`libcamera`) | OK     |
| STM32     | USB/Serial communication         | OK     |
| Power     | Battery and regulation checks    | OK     |

> These tests confirm **basic operability**, not performance tuning.

---

## 7. Dataset and Perception Work

### 7.1 Dataset

* Source: BFMC-oriented dataset ((https://universe.roboflow.com/bfmc-final/my-first-project-t7joo))

### 7.2 Purpose

* Validate dataset suitability
* Assess feasibility of perception models
* Understand camera field-of-view constraints

### 7.3 Model

* YOLO-based object detection
* Training completed as an **offline proof-of-concept**

> The trained model is **not integrated** into the BFMC runtime pipeline.
> Real-time inference is intentionally deferred until system stability is ensured.

---

## 8. Race Track Analysis

Preliminary analysis of the BFMC track was conducted alongside hardware and software validation.

### Activities

* Study of expected track layout and constraints
* Estimation of lane width and turning radii
* Identification of critical regions:

  * Sharp curves
  * Intersections
  * Stop zones

### Design Impact

Track analysis informed:

* Camera mounting considerations
* Steering angle saturation limits
* Conservative speed constraints

---

## 9. Operating System & Environment Strategy

An initial attempt was made to work from a fresh OS and custom vision stack.

### Challenges Encountered

* CSI camera resource contention (`libcamera` single-process constraint)
* OpenCV and camera backend incompatibilities
* Runtime instability when deviating from BFMC reference services

### Engineering Decision

To ensure stability and competition compliance, the project was reverted to the **official BFMC baseline codebase and environment**.
All further development is now built on top of this validated baseline.

---

## 10. Current Project Status

### Completed

* Hardware bring-up and validation
* Raspberry Pi ↔ STM32 communication verification
* Identification of camera stack constraints
* Restoration and stabilization of BFMC baseline
* Offline perception feasibility study
* Preliminary race track analysis

### In Progress

* Code cleanup and refactoring for submission
* Definition of steering and speed command interfaces

### Planned

* FSM-based decision logic
* Lane detection and lane-following integration
* Centralized vision pipeline integration

---

## 11. Scope Clarification (Important)

This project does **not** currently claim:

* Full autonomy
* Closed-loop lane keeping
* Real-time perception deployment
* ROS2-based architecture
* Competition-ready performance

All functionality is developed incrementally, with validation at each stage.

---

## 12. Team

**Team Name:** OPTINX

