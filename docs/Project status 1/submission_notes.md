# BFMC 2026 – Phase 1 Project Status Report

**Team:** OPTINX
**Competition:** Bosch Future Mobility Challenge 2026
**Phase:** Phase 1 – System Bring-up, Failure Analysis & Baseline Stabilization
**Platform:** Raspberry Pi 5 + STM32 Nucleo-F401RE

---

## 1. Executive Summary

Phase 1 focused on establishing a **verified and competition-compliant engineering foundation**. Team OPTINX initiated development with a deliberate *fresh-start* approach to gain low-level understanding of the BFMC hardware, camera stack, and software interfaces.

This approach revealed several critical integration constraints—most notably **CSI camera resource contention** and **Python dependency incompatibilities**. Based on these findings, a disciplined, data-driven decision was made to revert to the **Official BFMC Reference Baseline**.

By the end of Phase 1, the system is stable, compliant, and fully validated, providing a reliable foundation for autonomous behavior development in Phase 2.

---

## 2. Hardware Validation Status

All hardware components supplied in the BFMC kit were rigorously tested to ensure electrical integrity, mechanical correctness, and communication reliability.

| Component               | Validation Protocol                              | Status   |
| ----------------------- | ------------------------------------------------ | -------- |
| **Drive System**        | ESC initialization and throttle range response   | **PASS** |
| **Steering System**     | Servo PWM mapping and center calibration         | **PASS** |
| **Vision System**       | CSI camera detection via `libcamera`             | **PASS** |
| **Embedded Controller** | STM32 firmware flashing and USB-Serial stability | **PASS** |
| **Power System**        | Voltage stability under peak actuation load      | **PASS** |

---

## 3. Engineering Journey: Challenges & Course Correction

### 3.1 The Fresh-Start Initiative

At the start of Phase 1, the team intentionally avoided relying on the BFMC reference code and instead attempted a custom OS and vision stack. The objective was to better understand system-level constraints and evaluate performance optimization opportunities on the Raspberry Pi 5.

---

### 3.2 Critical Failures Encountered

This approach exposed three primary integration blockers:

* **Camera Resource Contention**
  Repeated `libcamera` acquisition failures were observed. The CSI camera enforces strict **single-process ownership**, and custom scripts conflicted with background system services, resulting in pipeline lock errors.

* **Python Version Mismatch**
  The BFMC OS uses **Python 3.13**, while several critical vision and ML libraries (e.g., OpenCV, inference toolchains) exhibited installation issues or runtime instability on this version.

* **System Fragility**
  Deviations from the reference stack led to inconsistent control-loop timing and intermittent communication drops between the Raspberry Pi and STM32.

---

### 3.3 Decision to Revert to BFMC Baseline

To eliminate accumulating technical debt and ensure full compliance with competition standards, the team performed a **controlled rollback** to the official BFMC reference environment.

This rollback was treated as a **risk-containment step rather than a setback**, ensuring that all future development occurs on a known-stable, competition-validated foundation.

Key benefits of this decision include:

1. Guaranteed compatibility with BFMC evaluation tools
2. A single, authoritative camera pipeline
3. Reliable execution of dashboard, communication, and safety services

---

## 4. Current Software Architecture

A **dual-environment strategy** has been adopted to balance system stability with development flexibility.

---

### 4.1 BFMC System Environment (Python 3.13)

* **Status:** Unmodified and stable
* **Responsibilities:**

  * Runs BFMC core services
  * Handles hardware abstraction
  * Maintains exclusive ownership of the CSI camera
  * Ensures compliance with competition infrastructure

---

### 4.2 Vision Development Environment (`vision_env` – Python 3.11)

* **Status:** Isolated virtual environment

* **Responsibilities:**

  * Offline model training
  * Algorithm validation
  * Vision pipeline prototyping

* **Isolation Policy:**

  * No system services are launched from this environment
  * No camera access is attempted while BFMC services are running
  * No global or system-level dependencies are modified

This guarantees that experimental development cannot destabilize the vehicle runtime.

---

## 5. Perception & Track Analysis

### 5.1 Perception Feasibility Study

Although real-time inference is intentionally deferred to Phase 2, an offline feasibility study was completed:

* **Dataset:** BFMC-specific dataset (Roboflow)
* **Model:** YOLO-based object detection
* **Outcome:** Successful identification of track elements (lanes, signs) under controlled conditions

---

### 5.2 Track Constraints & Design Impact

Preliminary analysis of the BFMC track led to the following design decisions:

* **Camera Tilt Angle:** Optimized for ~1.5–2 m forward look-ahead
* **Steering Limits:** Software-defined saturation limits applied to prevent mechanical strain during tight curves

---

## 6. Project Status Summary & Roadmap

### Completed in Phase 1

* Full hardware verification and assembly
* Stable Raspberry Pi ↔ STM32 serial communication
* Identification and resolution of camera stack limitations
* Reversion to and stabilization of the official BFMC software baseline
* Repository restructuring for modular development

---

### In Progress

* Finalization of steering and speed command API
* Preparation for centralized vision pipeline integration within BFMC services

---

### Planned for Phase 2

* Lane Keeping Assist (LKA) with closed-loop control
* Finite State Machine (FSM) for high-level decision making
* Real-time perception integration using a centralized camera pipeline

---

## 7. Conclusion

Phase 1 followed a transparent **fail-fast and analyze-early** methodology. By attempting a fresh system build and identifying its limitations early, Team OPTINX gained a deeper understanding of the BFMC ecosystem and avoided late-stage instability.

All Phase-1 decisions were guided by the principle of **minimizing integration risk before introducing algorithmic complexity**. The project now enters Phase 2 with a **fully validated, stable, and competition-compliant baseline**, ready for autonomous behavior development.


