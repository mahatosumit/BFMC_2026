# BFMC 2026 – Autonomous Embedded Driving Platform

**Team:** OPTINX  
**Competition:** Bosch Future Mobility Challenge 2026  
**Official Regulations:** [BFMC Documentation](https://bosch-future-mobility-challenge-competition-regulation.readthedocs-hosted.com/)

**Video Link** [Qualification Phase(https://youtu.be/07d6nmOjWjI/)]

---

## 1. Project Overview

During the current reporting period, Team OPTINX focused on transitioning from initial system integration to a stable autonomous driving system in preparation for the BFMC Qualification Round. A key milestone was the successful integration of the perception pipeline running on a Raspberry Pi 5 with the low-level actuation system implemented on an STM32 microcontroller. This architecture enables real-time interpretation of camera inputs and generation of steering and speed commands.

The primary objective of this phase was to achieve reliable lane keeping and consistent reactions to traffic elements. Extensive testing and parameter tuning were conducted to reduce steering oscillations, stabilize perception outputs, and minimize latency between perception and actuation. These improvements enabled the vehicle to complete repeated autonomous runs with stable trajectory tracking.

To support development and testing, a monitoring interface was implemented using a Tkinter-based dashboard. The interface visualizes perception outputs, telemetry data, and a digital twin of the vehicle state. The system includes a lightweight topological map engine capable of loading GraphML road graphs, computing node-to-node routes, and snapping the digital vehicle representation to valid track segments for accurate telemetry visualization and route monitoring. During experiments, the perception pipeline operates between 19 and 25 FPS, depending on scene complexity and lighting conditions.

## 2. Current Architecture & Perception Phase

The system follows a distributed architecture in which perception and behavioral logic run on the Raspberry Pi 5, while the STM32 microcontroller executes deterministic control of steering and motor actuation.

### Classical & Semantic Vision
The lane detection pipeline processes frames from the CSI camera and converts them into a Bird’s Eye View representation. Lane markings are extracted using adaptive thresholding and tracked through a sliding-window polynomial fitting method to estimate lane curvature and centerline. Semantic perception is performed using a quantized YOLOv8 neural network optimized for real-time inference on the Raspberry Pi. The model detects traffic signs, pedestrians, vehicles, and traffic lights. 

To maintain responsiveness, YOLO inference runs asynchronously so that neural network processing does not block the main control loop. Detected signs are processed through a chronological state pipeline that organizes traffic elements along the active driving route. Each element transitions through defined states (`PENDING → DETECTING → ACTING → COMPLETED`), ensuring that traffic behaviors are executed only when the vehicle reaches the appropriate spatial context and preventing premature rule activation.

### Kinematic Control
Vehicle control is handled through a Stanley Kinematic Controller that computes steering commands using cross-track error and heading deviation. Longitudinal speed control is governed by a rule-based traffic decision engine that adapts vehicle behavior to environmental conditions. The system differentiates between driving environments such as city and highway zones and applies dynamic speed multipliers to adjust the target velocity. During testing, the vehicle maintains approximately 20 PWM on standard road sections and 22 PWM on highway segments, ensuring stable navigation while preserving adequate reaction time for perception-driven decisions.

## 3. Autonomous Performance & Behaviors

Significant progress was achieved in perception, control, and system integration. 
- **Lane Trajectory:** Temporal smoothing using an Exponential Moving Average was introduced to stabilize polynomial lane estimates and reduce steering jitter. A fallback mechanism combining IMU-based dead reckoning and visual odometry was also implemented to maintain trajectory estimation when lane boundaries temporarily disappear.
- **Stanley Controller Tuning:** Adjustments eliminated oscillatory steering behavior and enabled smooth convergence toward the lane center. A boundary protection mechanism was also implemented to prevent the vehicle from drifting toward track edges.

### Validated Behaviors
The vehicle demonstrates reliable lane keeping, performs **autonomous parking using a CSV-driven trajectory playback module**, stops when pedestrians are detected, and executes lane-change maneuvers when a vehicle obstacle is present ahead. The system was also evaluated on ramp sections and tunnel environments, maintaining stable perception and control despite variations in elevation and lighting conditions.

### Temporal FSM Overrides
Additional behavioral constraints were introduced through time-based overrides within the control system. Certain semantic events such as crosswalks or priority zones **temporarily override standard control outputs, enforcing rule-compliant vehicle behavior during predefined time intervals.** System integration tasks were also completed to ensure reliable communication between hardware and software components. A Python daemon manages serial communication between the Raspberry Pi and STM32, while telemetry data including speed, steering angle, and system state are logged and transmitted through the BFMC network interface.

### Sustainability Considerations
To reduce material waste during development, the experimental track used for testing was constructed primarily from repurposed laboratory materials. Existing boards and surfaces were refurbished and repainted to recreate BFMC lane markings.

## 4. Encountered Issues & Mitigations

The most significant limitation was the computational load on the Raspberry Pi 5 when running both the lane detection pipeline and YOLO neural network simultaneously. This occasionally introduced latency spikes, which were mitigated through model quantization and asynchronous inference execution.

- **Lighting Calibration:** Lighting variations initially affected lane segmentation accuracy. To improve robustness, camera exposure parameters are calibrated during system initialization and then locked during operation. 
- **Deterministic Handshake:** Differences in processing frequency between the Pi and STM32 initially caused buffer overruns and delayed actuation. This issue was resolved by implementing a lightweight heartbeat synchronization protocol and improving the microcontroller command parser. 
- **IMU Injection:** Steering stability was further improved by combining perception-based lane estimation with IMU yaw feedback, enabling smoother trajectory correction.

## 5. Next Steps

The immediate priority is to perform repeated end-to-end validation runs to ensure stable lane keeping, reliable detection of traffic elements, and consistent autonomous operation. Following qualification, development will focus on improving behavioral robustness, refining intersection handling logic, and enhancing perception performance for dynamic obstacles such as pedestrians and moving vehicles. 

Future work will also target improved object detection consistency, reduced perception latency, and the integration of visual mapping with dead-reckoning techniques to strengthen localization robustness.
