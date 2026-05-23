import cv2
import numpy as np
from dataclasses import dataclass
from typing import Tuple, Optional, Dict, List
import logging
from enum import Enum

logger = logging.getLogger(__name__)

# ===================== ENUMS =====================

class LaneColor(Enum):
    WHITE_ON_DARK = 0
    DARK_ON_LIGHT = 1

# ===================== CONFIG =====================

@dataclass
class LaneDetectionConfig:
    # ? Pi-safe resolution (Quarter VGA)
    image_width: int = 320
    image_height: int = 240

    lane_color: LaneColor = LaneColor.WHITE_ON_DARK

    # Perspective (Standard "Trapezoid" for 320x240)
    # ?? CALIBRATE THESE ON THE ACTUAL CAR
    src_points: np.ndarray = None
    dst_points: np.ndarray = None

    # Processing
    roi_height_ratio: float = 0.55  # Cut off top 45% of image
    gaussian_blur_size: int = 5

    # Morphology
    kernel_size: int = 3
    morphology_iterations: int = 1

    # Sliding window
    n_windows: int = 9
    margin: int = 60
    min_pixels: int = 30

    # Polynomial
    polynomial_order: int = 2

    # Control smoothing
    smoothing_factor: float = 0.4
    blind_decay: float = 0.9

    debug_mode: bool = False

    def __post_init__(self):
        # Source Points (Trapezoid on image)
        if self.src_points is None:
            self.src_points = np.array([
                [20,  self.image_height],           # Bottom Left
                [80,  int(self.image_height * 0.6)], # Top Left
                [240, int(self.image_height * 0.6)], # Top Right
                [300, self.image_height]            # Bottom Right
            ], dtype=np.float32)

        # Dest Points (Rect on Bird's Eye)
        if self.dst_points is None:
            self.dst_points = np.array([
                [80, self.image_height],
                [80, 0],
                [self.image_width - 80, 0],
                [self.image_width - 80, self.image_height]
            ], dtype=np.float32)

# ===================== DETECTOR =====================

class BFMC_LaneDetector:

    def __init__(self, config: LaneDetectionConfig = None):
        self.config = config or LaneDetectionConfig()

        self.M = cv2.getPerspectiveTransform(self.config.src_points, self.config.dst_points)
        self.M_inv = cv2.getPerspectiveTransform(self.config.dst_points, self.config.src_points)

        self.left_fit = None
        self.right_fit = None
        self.prev_error = 0.0
        self.detection_confidence = 0.0

        self.debug_images: Dict[str, np.ndarray] = {}

    # ===================== PREPROCESS =====================

    def preprocess(self, frame: np.ndarray) -> Optional[np.ndarray]:
        """
        Combination of HLS Filtering (for Glare) + Adaptive Thresholding (for Edges)
        """
        if frame is None:
            return None

        # 1. HLS Filtering (Glare Removal)
        # Convert to HLS (Hue, Lightness, Saturation)
        hls = cv2.cvtColor(frame, cv2.COLOR_BGR2HLS)
        l_channel = hls[:, :, 1] # Lightness channel

        # Filter: Keep only bright white pixels (L > 160)
        # This removes shiny black asphalt reflections which are usually L < 120
        _, white_mask = cv2.threshold(l_channel, 160, 255, cv2.THRESH_BINARY)

        # 2. Standard Edge Detection
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        
        # Apply ROI (Zero out top half)
        h, w = gray.shape
        roi_top = int(h * self.config.roi_height_ratio)
        mask = np.zeros_like(gray)
        cv2.rectangle(mask, (0, roi_top), (w, h), 255, -1)
        gray = cv2.bitwise_and(gray, mask)

        # Gaussian Blur
        gray = cv2.GaussianBlur(gray, (self.config.gaussian_blur_size, self.config.gaussian_blur_size), 0)

        # Adaptive Threshold (Finds edges)
        binary_adaptive = cv2.adaptiveThreshold(
            gray, 255,
            cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
            cv2.THRESH_BINARY,
            21, 10
        )

        # 3. Combine: Valid pixels must be BOTH "Adaptive Edge" AND "Bright White"
        combined_binary = cv2.bitwise_and(binary_adaptive, white_mask)

        # 4. Cleanup Noise
        kernel = np.ones((self.config.kernel_size, self.config.kernel_size), np.uint8)
        combined_binary = cv2.morphologyEx(combined_binary, cv2.MORPH_OPEN, kernel, iterations=1)

        if self.config.debug_mode:
            self.debug_images["binary"] = combined_binary

        return combined_binary

    # ===================== WARP =====================

    def warp(self, img: np.ndarray) -> Optional[np.ndarray]:
        if img is None:
            return None
        warped = cv2.warpPerspective(
            img, self.M,
            (self.config.image_width, self.config.image_height)
        )
        if self.config.debug_mode:
            self.debug_images["warped"] = warped
        return warped

    # ===================== SLIDING WINDOW =====================

    def find_lane_pixels(self, binary: np.ndarray):
        h, w = binary.shape
        histogram = np.sum(binary[h // 2 :, :], axis=0)

        midpoint = w // 2
        leftx_base = np.argmax(histogram[:midpoint])
        rightx_base = np.argmax(histogram[midpoint:]) + midpoint

        window_height = h // self.config.n_windows
        nonzero = binary.nonzero()
        nonzeroy, nonzerox = nonzero

        leftx_current = leftx_base
        rightx_current = rightx_base

        left_inds, right_inds = [], []

        for window in range(self.config.n_windows):
            win_y_low = h - (window + 1) * window_height
            win_y_high = h - window * window_height

            win_xleft_low = leftx_current - self.config.margin
            win_xleft_high = leftx_current + self.config.margin
            win_xright_low = rightx_current - self.config.margin
            win_xright_high = rightx_current + self.config.margin

            good_left = (
                (nonzeroy >= win_y_low) & (nonzeroy < win_y_high) &
                (nonzerox >= win_xleft_low) & (nonzerox < win_xleft_high)
            ).nonzero()[0]

            good_right = (
                (nonzeroy >= win_y_low) & (nonzeroy < win_y_high) &
                (nonzerox >= win_xright_low) & (nonzerox < win_xright_high)
            ).nonzero()[0]

            left_inds.append(good_left)
            right_inds.append(good_right)

            if len(good_left) > self.config.min_pixels:
                leftx_current = int(np.mean(nonzerox[good_left]))
            if len(good_right) > self.config.min_pixels:
                rightx_current = int(np.mean(nonzerox[good_right]))

        try:
            left_inds = np.concatenate(left_inds)
            right_inds = np.concatenate(right_inds)
        except ValueError:
            return None, None, None, None

        return (
            nonzerox[left_inds], nonzeroy[left_inds],
            nonzerox[right_inds], nonzeroy[right_inds]
        )

    # ===================== MAIN PIPELINE =====================

    def process_frame(self, frame: np.ndarray) -> Dict:
        result = {"error": 0.0, "confidence": 0.0, "debug_images": {}}

        # 1. Preprocess & Warp
        binary = self.preprocess(frame)
        warped = self.warp(binary)

        # 2. Find Pixels
        lx, ly, rx, ry = self.find_lane_pixels(warped)

        # 3. Handle Blind/Lost Lane
        if lx is None or len(lx) < 10 or rx is None or len(rx) < 10:
            self.prev_error *= self.config.blind_decay
            result["error"] = float(self.prev_error)
            result["confidence"] = 0.0
            
            # Pass original frame as debug even if fail
            if self.config.debug_mode:
                result["debug_images"]["final"] = frame
                
            return result

        # 4. Fit Polynomials
        left_fit = np.polyfit(ly, lx, 2)
        right_fit = np.polyfit(ry, rx, 2)

        # 5. Calculate Error (Steering)
        h = self.config.image_height
        left_x_bottom = np.polyval(left_fit, h)
        right_x_bottom = np.polyval(right_fit, h)

        lane_center = (left_x_bottom + right_x_bottom) / 2
        vehicle_center = self.config.image_width / 2

        # Normalize error (-1.0 to 1.0)
        error = (lane_center - vehicle_center) / (self.config.image_width / 2)
        error = np.clip(error, -1.0, 1.0)

        # Smooth error
        error = (
            self.config.smoothing_factor * error +
            (1 - self.config.smoothing_factor) * self.prev_error
        )
        self.prev_error = error

        confidence = min(1.0, (len(lx) + len(rx)) / 300)

        result["error"] = float(error)
        result["confidence"] = float(confidence)

        # 6. Create Debug Visualization (Green Lane)
        if self.config.debug_mode:
            self._create_debug_image(frame, warped, left_fit, right_fit, result)

        return result

    def _create_debug_image(self, frame, warped, left_fit, right_fit, result):
        """Draws the detected green lane on the original frame"""
        # Create an image to draw the lines on
        warp_zero = np.zeros_like(warped).astype(np.uint8)
        color_warp = np.dstack((warp_zero, warp_zero, warp_zero))

        h = self.config.image_height
        ploty = np.linspace(0, h-1, h)

        try:
            left_fitx = left_fit[0]*ploty**2 + left_fit[1]*ploty + left_fit[2]
            right_fitx = right_fit[0]*ploty**2 + right_fit[1]*ploty + right_fit[2]

            # Recast the x and y points into usable format for cv2.fillPoly()
            pts_left = np.array([np.transpose(np.vstack([left_fitx, ploty]))])
            pts_right = np.array([np.flipud(np.transpose(np.vstack([right_fitx, ploty])))])
            pts = np.hstack((pts_left, pts_right))

            # Draw the lane onto the warped blank image
            cv2.fillPoly(color_warp, np.int_([pts]), (0, 255, 0))

            # Warp the blank back to original image space using inverse perspective matrix (Minv)
            newwarp = cv2.warpPerspective(color_warp, self.M_inv, (self.config.image_width, self.config.image_height)) 

            # Combine the result with the original image
            final = cv2.addWeighted(frame, 1, newwarp, 0.3, 0)
            
            result["debug_images"]["final"] = final
            
        except Exception as e:
            # Fallback if drawing fails
            result["debug_images"]["final"] = frame

    # ===================== CONTROL API =====================

    def get_lane_center_error(self) -> float:
        return float(self.prev_error)
