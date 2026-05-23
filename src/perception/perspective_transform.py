import cv2
import numpy as np

class PerspectiveTransformer:
    def __init__(self):
        self.SRC_PTS = np.float32([[200, 260], [440, 260], [40, 450], [600, 450]])
        self.DST_PTS = np.float32([[150, 0], [490, 0], [150, 480], [490, 480]])
        self.M_forward = cv2.getPerspectiveTransform(self.SRC_PTS, self.DST_PTS)
        self.M_inv = cv2.getPerspectiveTransform(self.DST_PTS, self.SRC_PTS)

    def warp(self, frame):
        return cv2.warpPerspective(frame, self.M_forward, (640, 480))
        
    def unwarp(self, frame):
        return cv2.warpPerspective(frame, self.M_inv, (640, 480))
