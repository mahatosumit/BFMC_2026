1. Speed Parameters (in 

control/controller.py
)
Look inside the 

Controller
 class. These constants dictate how fast the car goes and how it brakes.

base_speed (passed from 

main.py
 dashboard slider): The default PWM target on straightaways (e.g., 150).
MIN_CURVE_SPEED_F = 0.45: When the car detects an upcoming curve or is turning, speed drops to 45% of base_speed. Lower this to 0.35 if it's drifting off the track on turns.
BRAKING_DISTANCE_M = 1.8: The car starts linearly decelerating 1.8 meters before it enters a sharp curve or intersection. Increase this (e.g. 2.5) to brake earlier.
MINIMUM_DRIVE_PWM = 18.0: A hard floor. No matter how much the PID algorithms say to slow down, it will never feed a PWM lower than 18 (so the motor doesn't stall).
Divider Follow speed penalty: speed *= 0.75 (Line 135). When the right line is missing and the car is hugging the left dashed divider line, speed drops by 25% for safety.
Dead Reckoning confidence speed drop: speed *= (0.4 + 0.4 * dr_conf) (Line 132). If both lines disappear and it is driving purely by IMU memory, speed dynamically drops heavily (down to ~40%-80% of normal) so it doesn't fly off the map blindly.
2. Steering Aggressiveness (in 

control/controller.py
)
Look inside the 

StanleyController
 class. This uses the Stanley Kinematic Model rather than a basic PID.

k = 3.5: The main gaining multiplier (Line 84).
Increase it (to 4.0 - 5.0): The car will steer much more aggressively/sharply to get back to the center of the lane.
Decrease it (to 2.0 - 2.5): The car will steer more lazily/smoothly.
MAX_STEER = 30.0: The absolute maximum angle (in degrees) the servo is allowed to turn physically.
MAX_STEER_RATE = 60.0: The maximum degrees the servo can jerk per second. Lower this to 30.0 if the car twitches back and forth violently on straightaways.
3. Lane Centering & Evasion (in 

control/controller.py
)
Look at the 

DividerGuard
 class. This acts like a virtual bumper to physically push the car away from the edges if Stanley fails.

DIVIDER_SAFE_PX = 130: The closest the car is allowed to get to the middle dashed line (130 pixels from the left camera edge). If it gets closer, the code mathematically "pushes" the steering wheel right.
EDGE_SAFE_PX = 100: The closest the car is allowed to get to the solid outside line on the right.
MAX_CORR = 25.0: The maximum emergency override string angle (25 degrees) it will add to save the car from hitting a line.
4. Dashboards Sliders (

main.py
)
Because of how the Tkinter app is set up, you don't actually need to open the code to tweak the three most important variables while testing! You can slide them inside the Drive Dynamics tab while the car is driving:

Base Speed (PWM): Overrides the target straightaway speed.
Steer Multiplier: Multiplies the final 

controller.py
 output (e.g., 0.8 drops the overall steering angles by 20%).
BEV Image Thresholding (Block Size & C): Tweak these if the camera isn't seeing the white lines correctly due to lighting conditions in the room.
