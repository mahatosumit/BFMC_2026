import math
from .parking_config import IMU_DEADBAND, VELOCITY_DAMPING, DISTANCE_CALIBRATION_FACTOR

class ParkingDistanceTracker:
    def __init__(self):
        self.distance_cm = 0.0

    def reset(self):
        self.distance_cm = 0.0

    def update(self, yaw_difference, dt, speed_pwm):
        """
        Distance must ONLY calculate during: STRAIGHT MOVEMENT. NOT while turning.
        Using yaw_difference to determine straight vs turning.
        """
        if yaw_difference < 5.0:
            # Straight movement
            # Mathematically convert speed (mm/s) to cm/s: speed_pwm / 10.0
            if abs(speed_pwm) > 0:
                speed_cm_s = (abs(speed_pwm) / 10.0) * DISTANCE_CALIBRATION_FACTOR
                self.distance_cm += speed_cm_s * dt
