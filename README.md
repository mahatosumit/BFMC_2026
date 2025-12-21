# BFMC 2025 – Autonomous Vehicle Platform

**Team OPTINX**

---

## 1. Overview

This repository contains the hardware, firmware, and software development work for **BFMC 2025 (Bosch Future Mobility Challenge)**.

The project aims to build a **modular autonomous vehicle platform** using a **Raspberry Pi + STM32** architecture, with incremental progress toward perception-based autonomy.

### 1.1 Current Focus

The current development phase prioritizes:

* Hardware validation
* Stable BFMC baseline integration
* Dataset exploration and perception model training
* Preliminary race track analysis
* Preparation for the first BFMC submission

---

## 2. Project Objectives

1. Validate BFMC hardware kit functionality
2. Establish a reliable Raspberry Pi–STM32 control pipeline
3. Design a scalable and modular software architecture
4. Perform preliminary perception model training
5. Analyze BFMC race track constraints for design guidance
6. Transition incrementally from baseline control to autonomous behavior

---

## 3. System Architecture (Current)

### 3.1 High-Level Design

**Raspberry Pi**

* Vision processing
* Rule-based decision logic
* High-level control commands
* Communication with STM32

**STM32**

* Motor control
* Steering control
* Low-level actuation
* Safety handling

> **Note:**
> Autonomy is currently rule-based. FSM-based logic and learning-based control are planned for later phases.

---

## 4. Repository Structure

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
│   ├── perception/           # Vision and detection modules
│   ├── planning/             # FSM and decision logic (future)
│   ├── control/              # Vehicle control logic
│   ├── communication/        # Pi–STM32 communication
│   └── main.py               # Entry point
│
├── models/                   # Trained models (weights excluded)
├── tools/                    # Utility scripts and debugging tools
├── tests/                    # Hardware and software test scripts
│
├── requirements.txt          # Python dependencies
├── .gitignore
└── README.md
```

---

## 5. Hardware Validation

Initial hardware verification was performed after receiving the BFMC kit.

| Component | Test Description              | Status |
| --------- | ----------------------------- | ------ |
| Motors    | Direction and speed response  | OK     |
| Steering  | PWM response                  | OK     |
| Camera    | OS-level detection            | OK     |
| STM32     | USB/UART communication        | OK     |
| Power     | Battery and regulation checks | OK     |

---

## 6. Dataset and Model Work

### 6.1 Dataset

* Source: Roboflow BFMC dataset

### 6.2 Objectives

* Validate dataset usability
* Test feasibility of perception models
* Understand camera field-of-view constraints

### 6.3 Model

* YOLO-based object detection
* Training completed as proof-of-concept

> The trained model is **not yet integrated** into the BFMC runtime pipeline.
> Integration is planned after baseline software stabilization.

---

## 7. Race Track Analysis

Preliminary BFMC race track analysis was conducted in parallel with software and hardware validation.

### 7.1 Activities

* Study of BFMC track layout and constraints
* Estimation of lane width and minimum turn radius
* Identification of critical zones:

  * Sharp turns
  * Intersections
  * Stop regions

### 7.2 Design Impact

Track analysis informed:

* Camera mounting decisions
* Steering angle limits
* Speed constraints

---

## 8. Operating System and Environment

* Raspberry Pi OS (fresh installation attempted)

### 8.1 Challenges Encountered

* Camera stack incompatibility
* OpenCV and libcamera conflicts
* Runtime instability

### 8.2 Engineering Decision

To ensure submission readiness, the project reverted to the **official BFMC baseline codebase and environment**.

---

## 9. Current Project Status

### Completed

* Hardware validation
* Dataset exploration
* Offline perception model training
* Preliminary race track analysis
* BFMC baseline code restoration

### In Progress

* Code refactoring for submission
* Lane following integration

### Planned

* FSM-based autonomy development

---

## 10. Development Roadmap

### Short-Term

* Stable baseline control
* Clean software modularization
* Lane following integration

### Mid-Term

* FSM-based navigation
* Traffic sign handling
* Curvature-based speed control

### Long-Term

* Optimized perception pipeline
* Safety and recovery behaviors
* Competition track performance tuning

---

## 11. Scope Clarification

This project does **not** currently claim:

* Full autonomy
* ROS2 integration
* End-to-end learning-based control
* Final competition-ready performance

All features are developed incrementally with validation at each stage.

---

## 12. Team

**Team Name:** OPTINX


Say the next step.
