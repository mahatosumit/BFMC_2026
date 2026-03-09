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


