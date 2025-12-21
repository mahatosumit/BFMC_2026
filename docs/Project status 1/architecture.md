# System Architecture – BFMC 2025

Team: OPTINX  
Platform: Raspberry Pi + STM32  
Architecture Type: Distributed Embedded Control  

---

## 1. Architecture Overview

The BFMC vehicle platform follows a distributed architecture separating high-level decision making from low-level actuation.
This separation improves modularity, reliability, and future scalability.

The system is divided into two primary computational units:
- Raspberry Pi (high-level processing)
- STM32 microcontroller (low-level control)

---

## 2. Hardware Architecture

### 2.1 Raspberry Pi

The Raspberry Pi acts as the high-level processing unit responsible for:
- Vision processing
- Decision logic
- Behavior selection
- Communication with STM32

The Raspberry Pi runs the main control loop and coordinates perception and planning outputs.

---

### 2.2 STM32 Microcontroller

The STM32 microcontroller handles real-time, low-level control tasks including:
- Drive motor control
- Steering motor control
- PWM signal generation
- Safety-critical actuation

This separation ensures that timing-sensitive motor control is not affected by high-level processing delays.

---

## 3. Software Architecture

### 3.1 Raspberry Pi Software Stack

The Raspberry Pi software is organized into modular components:

- Perception Module  
  Handles camera input and visual processing.  
  (Currently under development; offline model training completed.)

- Planning Module  
  Responsible for decision-making logic.  
  Current implementation is rule-based; FSM-based logic planned.

- Control Interface  
  Translates high-level decisions into commands sent to STM32.

- Communication Module  
  Manages serial communication between Raspberry Pi and STM32.

---

### 3.2 STM32 Firmware Architecture

The STM32 firmware is implemented using STM32 HAL and CubeMX-generated structure.

Key responsibilities:
- Receive commands from Raspberry Pi
- Control motor drivers and steering actuators
- Maintain predictable real-time behavior

---

## 4. Data Flow Description

1. Camera captures frames and sends them to the Raspberry Pi
2. Perception module processes visual input
3. Planning logic determines vehicle behavior
4. Control commands are sent to STM32 via serial communication
5. STM32 executes motor and steering commands

---

## 5. Current Architecture Limitations

- Perception model not yet integrated into runtime pipeline
- Decision logic currently rule-based
- No closed-loop feedback from encoders in current phase

These limitations are intentional for Phase 1 stability and will be addressed in later phases.

---

## 6. Architectural Design Rationale

This architecture was chosen to:
- Ensure real-time motor control reliability
- Allow independent development of perception and control
- Simplify debugging and testing
- Support incremental autonomy development

---

## 7. Future Architectural Extensions

Planned extensions include:
- FSM-based planning layer
- Runtime perception integration
- Enhanced safety monitoring
- Sensor fusion (future scope)

---

## 8. Conclusion

The current architecture provides a stable and modular foundation for BFMC development.
It supports incremental feature addition while maintaining system reliability and clarity.
