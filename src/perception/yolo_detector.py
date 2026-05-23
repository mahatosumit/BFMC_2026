import numpy as np
import torch
from ultralytics import YOLO

class YOLODetector:
    """
    YOLO Perception Module (BFMC 2026 Optimized)
    --------------------------------------------
    Responsibilities:
    - Run object detection on the Pi 5.
    - Filter detections by Class Name (not just size).
    - Output specific 'Signals' for the FSM (Traffic Lights, Signs, Obstacles).
    """

    def __init__(self, model_path, img_w=640, img_h=640, device="cpu"):
        self.img_w = img_w
        self.img_h = img_h

        # Device selection: Use 'mps' for Mac, 'cuda' for NVIDIA, 'cpu' for Pi 5
        if device == "cuda" and not torch.cuda.is_available():
            print("[YOLO] CUDA not available, falling back to CPU")
            device = "cpu"
        
        self.device = device
        
        print(f"[YOLO] Loading model from {model_path}...")
        try:
            self.model = YOLO(model_path)
            self.model.to(self.device)
            
            # Fuse layers: Merges Conv2d + BatchNorm for ~15% speedup on CPU
            self.model.fuse() 
            print(f"[YOLO] Model loaded successfully on {self.device}")
            
            # Print classes so you can verify they match your dataset (uit-wywve)
            print(f"[YOLO] Detected Classes: {self.model.names}")
            self.names = self.model.names
            
        except Exception as e:
            print(f"[YOLO] CRITICAL ERROR: Could not load model. {e}")
            raise e

    def detect(self, frame):
        """
        Runs inference and returns a structured dictionary of signals.
        
        Args:
            frame (np.ndarray): BGR image from OpenCV.
            
        Returns:
            dict: {
                "sign_intensity": float (0.0-1.0),  # Area of STOP sign only
                "obstacle": bool,                   # True if Car/Pedestrian is close
                "traffic_light": str or None,       # "Red", "Green", "Yellow"
                "intersection": str or None,        # "CrossWalk", "Roundabout"
                "highway": str or None              # "Entry", "Exit"
            }
        """
        # 1. Initialize Default Signals (The "Safe State")
        signals = {
            "sign_intensity": 0.0,
            "obstacle": False,
            "traffic_light": None,
            "intersection": None,
            "highway": None
        }

        if frame is None:
            return signals

        # 2. Run Inference
        # imgsz=320 is CRITICAL for Pi 5 CPU speed (~15-20 FPS)
        # conf=0.40 filters out weak detections (ghosts)
        results = self.model(
            frame, 
            imgsz=320, 
            conf=0.40, 
            iou=0.45, 
            verbose=False, 
            device=self.device
        )

        # 3. Process Detections
        # We look for the "most critical" object in the frame
        
        for r in results:
            if r.boxes is None: continue
            
            for box in r.boxes:
                # Extract Data
                cls_id = int(box.cls[0])
                label = self.names[cls_id]
                confidence = float(box.conf[0])
                
                # Calculate Normalized Area (How close is it?)
                # box.xywh returns pixels: [center_x, center_y, width, height]
                x, y, w, h = box.xywh[0].cpu().numpy()
                area = (w * h) / (self.img_w * self.img_h)

                # --- LOGIC MAPPING (The "Brain") ---

                # A. STOP SIGNS (Distance Dependent)
                # Only trigger if we are somewhat close (> 0.02 area)
                if label == "Stop":
                    if area > signals["sign_intensity"]:
                        signals["sign_intensity"] = float(area)

                # B. TRAFFIC LIGHTS (State Dependent)
                # These override everything else usually
                elif label == "Redlight":
                    signals["traffic_light"] = "Red"
                elif label == "Greenlight":
                    signals["traffic_light"] = "Green"
                elif label == "Yellowlight":
                    signals["traffic_light"] = "Yellow"

                # C. OBSTACLES (Safety Critical)
                # Cars and Pedestrians
                elif label in ["Car", "Pedestrian"]:
                    # CRITICAL: Only stop if it's in our lane/close (area > 0.05)
                    # This prevents stopping for a car on the other side of the track.
                    if area > 0.05:
                        signals["obstacle"] = True

                # D. INTERSECTIONS (Trigger Blind Mode)
                elif label == "CrossWalk":
                    # Only trigger if close enough to lose lines
                    if area > 0.08:
                        signals["intersection"] = "CrossWalk"
                
                elif label == "Roundabout":
                    if area > 0.08:
                        signals["intersection"] = "Roundabout"

                # E. HIGHWAY SIGNS (Speed Control)
                elif label == "HighwayEntry":
                    signals["highway"] = "Entry"
                elif label == "HighwayEnd":
                    signals["highway"] = "Exit"

        return signals

# USAGE EXAMPLE (Put this in your main loop)

if __name__ == "__main__":
    import cv2
    
    detector = YOLODetector(model_path="best_bfmc.pt", device="cpu")
    
    cap = cv2.VideoCapture(0)
    
    while True:
        ret, frame = cap.read()
        if not ret: break
        
        # 1. DETECT
        signals = detector.detect(frame)
        
        # 2. DEBUG PRINT
        print(f"Signals: {signals}")
        
        # 3. VISUALIZE (Optional)
        cv2.imshow("Frame", frame)
        if cv2.waitKey(1) == 27: break
        
    cap.release()
    cv2.destroyAllWindows()
