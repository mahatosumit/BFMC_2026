
# BFMC 2025 – Project Status Report (Phase 1)

**Team:** OPTINX
**Competition:** Bosch Future Mobility Challenge 2025
**Phase:** Initial Development & First Submission
**Platform:** Raspberry Pi 5 + Nucleo-F401RE (STM32)

## 1. Phase Objective

The primary objective of Phase 1 was to establish a robust and verifiable engineering baseline for the BFMC autonomous vehicle platform. The development effort focused on hardware verification, software environment stabilization, and preliminary system integration to ensure readiness for the initial submission deadline.

## 2. Hardware Validation Status

Upon receipt of the BFMC hardware kit, a comprehensive validation protocol was executed to verify the integrity of core electromechanical and computing subsystems.

| Component | Validation Protocol | Status |
| :--- | :--- | :--- |
| **Drive System** | DC motor response (RPM/Torque verification) | **Pass** |
| **Steering** | Servo PWM control and center calibration | **Pass** |
| **Vision System** | OS-level enumeration and frame capture | **Pass** |
| **Embedded Control** | STM32 UART communication & firmware flash | **Pass** |
| **Power Management** | Battery voltage stability & regulation checks | **Pass** |


## 3. Software Environment & Architecture

Initial development explored a custom Raspberry Pi OS configuration to maximize performance. However, significant compatibility challenges were identified during integration testing.

### 3.1 Challenges Identified

* **Camera Stack Incompatibility:** Conflicts between the legacy camera stack and the `libcamera` backend.
* **Dependency Conflicts:** Version mismatches between OpenCV and system-level libraries.
* **System Instability:** Inconsistent runtime behavior affecting control loop reliability.

### 3.2 Engineering Decision

To mitigate integration risks and ensure immediate system reliability, the engineering team elected to adopt the official BFMC baseline environment. This decision prioritized platform stability and competition compliance over experimental optimization, allowing for rapid development of the control pipeline.

## 4. Perception System Development

Feasibility studies were conducted to determine the viability of deep learning-based perception on the embedded platform.

* **Dataset Source:** Roboflow (BFMC specific dataset).
* **Objective:** Validate dataset quality and assess camera field-of-view (FOV) requirements.
* **Model Architecture:** YOLO-based object detection.
* **Status:** A proof-of-concept model has been successfully trained offline. Integration into the real-time inference pipeline is scheduled for Phase 2, following the stabilization of the control algorithms.

## 5. Race Track Analysis & Design Constraints

A theoretical analysis of the BFMC track layout was conducted parallel to software development to derive physical constraints for the vehicle controller.

### Key Findings

* **Geometric Constraints:** Estimation of lane widths and minimum turning radii.
* **Critical Zones:** Identification of high-risk areas including signalized intersections, crosswalks, and blind turns.

### Design Implications

Data from this analysis was used to define:

1. **Camera Mounting Angle:** Optimized for look-ahead distance vs. near-field detection.
2. **Steering Limits:** Capped to prevent mechanical binding while maintaining turning authority.
3. **Velocity Profiles:** established initial speed limits for straightaways vs. curves.

## 6. Repository Organization

The project codebase has been restructured into a modular architecture to facilitate parallel development. The structure separates **Perception**, **Control**, **Planning**, and **Communication** modules, ensuring maintainability and clear separation of concerns.

## 7. Project Status Summary

### Completed

* Comprehensive hardware validation.
* Stabilization of the software baseline.
* Offline training of the YOLO perception model.
* Detailed track analysis and constraint definition.
* Repository restructuring for scalability.

### In Progress

* Refactoring of control code for the first submission.
* Integration of vision-based Lane Keeping Assist (LKA).

### Planned (Phase 2)

* Implementation of Finite State Machine (FSM) for high-level decision logic.
* Real-time integration of the perception stack.
* Development of traffic sign handling behaviors.

## 8. Phase Conclusion

Phase 1 successfully achieved the goal of establishing a stable development platform. By validating all hardware subsystems and securing a reliable software environment, the team has mitigated early-stage technical risks. The project is now positioned to transition from baseline verification to the implementation of autonomous behaviors in the subsequent development phase.
