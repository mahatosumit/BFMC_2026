import cv2
import numpy as np

def preprocess_image(frame_bgr):
    """
    Applies CLAHE, adaptive thresholding, and morphological closing to extract lane lines.
    Assumes frame_bgr is BEV (warped).
    """
    clahe = cv2.createCLAHE(clipLimit=3.0, tileGridSize=(8, 8))
    lab = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2LAB)
    L = clahe.apply(lab[:, :, 0])
    
    mean_l = np.mean(L)
    if mean_l < 100:
        L = cv2.convertScaleAbs(L, alpha=1.0 + (100 - mean_l)/200, beta=int((100 - mean_l)*0.6))
    elif mean_l > 180:
        L = cv2.convertScaleAbs(L, alpha=1.0 - (mean_l - 180)/350, beta=int(-(mean_l - 180)*0.4))

    binary = cv2.adaptiveThreshold(L, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY_INV, 31, 15)
    warped_binary = cv2.morphologyEx(binary, cv2.MORPH_CLOSE, cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5)))
    
    return warped_binary
