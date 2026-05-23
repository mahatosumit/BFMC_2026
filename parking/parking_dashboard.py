import time
import numpy as np
try:
    import cv2
except ImportError:
    pass

class ParkingDashboard:
    def __init__(self):
        self.canvas_w = 1280
        self.canvas_h = 720
        self.window_name = "BFMC Parking Dashboard"
        self.update_every_n_frames = 2
        self.frame_count = 0
        
        # Colors (BGR for OpenCV)
        self.C_GREEN = (0, 255, 0)
        self.C_RED = (0, 0, 255)
        self.C_BLUE = (255, 0, 0)
        self.C_YELLOW = (0, 255, 255)
        self.C_CYAN = (255, 255, 0)
        self.C_PURPLE = (255, 0, 255)
        self.C_WHITE = (255, 255, 255)
        self.C_BLACK = (0, 0, 0)
        self.C_GRAY = (50, 50, 50)
        
        self.state_names = {
            0: "IDLE",
            1: "SIGN DETECTED",
            2: "SCANNING FOR FREE SLOT",
            3: "SLOT FOUND — COASTING",
            4: "STOPPING",
            5: "LOAD TRAJECTORY",
            6: "EXECUTING MANEUVER",
            7: "COMPLETE",
            8: "FAILED",
        }

        self.last_time = time.time()

    def update(self, debug_data):
        try:
            self.frame_count += 1
            if self.frame_count % self.update_every_n_frames != 0:
                return

            canvas = np.zeros((self.canvas_h, self.canvas_w, 3), dtype=np.uint8)
            
            # Calculate FPS
            current_time = time.time()
            fps = 1.0 / (current_time - self.last_time + 1e-6)
            self.last_time = current_time

            # --- 1. Normal Camera Feed Screen (Top Left 640x360) ---
            frame = debug_data.get("full_frame")
            if frame is None:
                frame = np.zeros((480, 640, 3), dtype=np.uint8)
                
            frame_disp = frame.copy()
            h, w = frame_disp.shape[:2]
            roi_y_start = int(h * 0.50)
            
            # Draw Horizontal ROI Split Line
            cv2.line(frame_disp, (0, roi_y_start), (w, roi_y_start), self.C_YELLOW, 1)
            cv2.putText(frame_disp, "ROI BOUNDARY (0.50)", (10, roi_y_start - 5), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.4, self.C_YELLOW, 1)

            # Draw Vertical Split Line
            mid_x = int(w / 2)
            cv2.line(frame_disp, (mid_x, 0), (mid_x, h), self.C_PURPLE, 1)
            cv2.putText(frame_disp, "L Split", (mid_x - 55, 15), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.4, self.C_PURPLE, 1)
            cv2.putText(frame_disp, "R Split", (mid_x + 5, 15), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.4, self.C_PURPLE, 1)
            
            # Sign detections — GREEN bounding boxes on full frame
            for d in debug_data.get("sign_detections", []):
                cx, cy, bw, bh = d.get('cx', 0), d.get('cy', 0), d.get('w', 0), d.get('h', 0)
                # Scale from 640x640 model space to original frame
                scale_x = w / 640.0
                scale_y = h / 640.0
                x1 = int((cx - bw/2) * scale_x)
                y1 = int((cy - bh/2) * scale_y)
                x2 = int((cx + bw/2) * scale_x)
                y2 = int((cy + bh/2) * scale_y)
                cv2.rectangle(frame_disp, (x1, y1), (x2, y2), self.C_GREEN, 2)
                cv2.putText(frame_disp, "Parking Sign", (x1, y1-25), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, self.C_GREEN, 1)
                cv2.putText(frame_disp, f"{d.get('confidence', 0):.2f}", (x1, y1-10), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, self.C_GREEN, 1)

            # Car detections — Draw on full frame below the ROI boundary line
            for car in debug_data.get("car_detections", []):
                cx = car.get('cx', 0)
                cy = car.get('cy', 0)
                bw = car.get('w', 0)
                bh = car.get('h', 0)
                roi_h = h - roi_y_start
                scale_x = w / 640.0
                scale_y = roi_h / 640.0
                x1 = int((cx - bw/2) * scale_x)
                y1 = int((cy - bh/2) * scale_y + roi_y_start)
                x2 = int((cx + bw/2) * scale_x)
                y2 = int((cy + bh/2) * scale_y + roi_y_start)
                color = self.C_BLUE if car.get('side') == 'left' else self.C_RED
                cv2.rectangle(frame_disp, (x1, y1), (x2, y2), color, 2)
                cv2.putText(frame_disp, "Car", (x1, max(y1-25, 10)), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)
                cv2.putText(frame_disp, f"{car.get('confidence', 0):.2f}", (x1, max(y1-10, 10)), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)

            # Resize camera frame to normal 640x360 dashboard aspect ratio
            frame_resized = cv2.resize(frame_disp, (640, 360))
            
            # Overlay State Text
            st_idx = debug_data.get("state", 0)
            
            if not debug_data.get("model_loaded", True):
                cv2.putText(frame_resized, "Parking Model: NOT LOADED", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, self.C_RED, 2)
            elif st_idx == -1:
                cv2.putText(frame_resized, f"Parking State: IDLE", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, self.C_YELLOW, 2)
            else:
                cv2.putText(frame_resized, f"State: {self.state_names.get(st_idx, '')}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, self.C_CYAN, 2)
                cv2.putText(frame_resized, f"Speed: x{debug_data.get('speed_multiplier', 1.0):.2f}", (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.6, self.C_CYAN, 2)
                cv2.putText(frame_resized, f"Dist: {debug_data.get('distance_cm', 0):.1f} cm", (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.6, self.C_CYAN, 2)
            
            # --- Parking Mode Indicator ---
            has_sign = any(d.get("class_id") == 1 for d in debug_data.get("sign_detections", []))
            is_active = (st_idx >= 1) or has_sign
            if is_active:
                cv2.rectangle(frame_resized, (320, 15), (630, 45), (0, 80, 0), -1)
                cv2.rectangle(frame_resized, (320, 15), (630, 45), self.C_GREEN, 1)
                cv2.putText(frame_resized, "Parking Mode Activated", (335, 36), cv2.FONT_HERSHEY_SIMPLEX, 0.55, self.C_GREEN, 2)
            
            canvas[0:360, 0:640] = frame_resized
            cv2.rectangle(canvas, (0, 0), (640, 360), self.C_CYAN, 2) # Normal view border

            # FPS overlay
            cv2.putText(canvas, f"FPS: {fps:.1f}", (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.6, self.C_GREEN, 2)

            # --- 3. Parking Status (Bottom Left 320x360) ---
            cv2.rectangle(canvas, (0, 360), (320, 720), self.C_GRAY, -1)
            cv2.rectangle(canvas, (0, 360), (320, 720), self.C_WHITE, 1)
            
            y_offset = 380
            st = debug_data.get("state", 0)
            cv2.putText(canvas, f"PARKING STATUS", (10, y_offset), cv2.FONT_HERSHEY_SIMPLEX, 0.7, self.C_WHITE, 2)
            
            mode_text = "ACTIVE" if st >= 1 else ("IDLE" if st == -1 else "INACTIVE")
            armed_text = "YES" if st >= 1 and st < 5 else "NO"
            takeover_text = "YES" if st >= 5 and st <= 7 else "NO"
            sign_text = "DETECTED" if st >= 1 else "NOT DETECTED"
            model_text = "LOADED" if debug_data.get("model_loaded", True) else "NOT LOADED"
            
            cv2.putText(canvas, f"Parking Mode: {mode_text}", (10, y_offset+40), cv2.FONT_HERSHEY_SIMPLEX, 0.6, self.C_GREEN if st>=1 else self.C_YELLOW, 2)
            cv2.putText(canvas, f"Parking Armed: {armed_text}", (10, y_offset+80), cv2.FONT_HERSHEY_SIMPLEX, 0.6, self.C_PURPLE if armed_text=="YES" else self.C_WHITE, 1)
            cv2.putText(canvas, f"Parking Takeover: {takeover_text}", (10, y_offset+120), cv2.FONT_HERSHEY_SIMPLEX, 0.6, self.C_RED if takeover_text=="YES" else self.C_WHITE, 2)
            cv2.putText(canvas, f"Parking Sign: {sign_text}", (10, y_offset+160), cv2.FONT_HERSHEY_SIMPLEX, 0.6, self.C_CYAN, 1)
            cv2.putText(canvas, f"Model: {model_text}", (10, y_offset+200), cv2.FONT_HERSHEY_SIMPLEX, 0.6, self.C_GREEN if debug_data.get("model_loaded", True) else self.C_RED, 1)
            
            cv2.putText(canvas, f"State {st}: {self.state_names.get(st, 'IDLE')}", (10, y_offset+240), cv2.FONT_HERSHEY_SIMPLEX, 0.5, self.C_WHITE, 1)
            cv2.putText(canvas, f"Target Stop: {debug_data.get('target_stop_distance', 0)} cm", (10, y_offset+280), cv2.FONT_HERSHEY_SIMPLEX, 0.5, self.C_YELLOW, 1)

            # Control Hierarchy Status
            cv2.rectangle(canvas, (0, 320), (320, 360), (40, 20, 20) if takeover_text == "YES" else (20, 40, 20), -1)
            if takeover_text == "YES":
                cv2.putText(canvas, "Parking TAKEOVER ACTIVE", (10, 345), cv2.FONT_HERSHEY_SIMPLEX, 0.6, self.C_RED, 2)
            else:
                cv2.putText(canvas, "Lane Following ACTIVE | Parking BACKGROUND", (10, 345), cv2.FONT_HERSHEY_SIMPLEX, 0.45, self.C_GREEN, 1)

            # --- 4. IMU Debug (Bottom Middle 320x360) ---
            cv2.rectangle(canvas, (320, 360), (640, 720), (30, 30, 30), -1)
            cv2.rectangle(canvas, (320, 360), (640, 720), self.C_WHITE, 1)
            
            main_imu = debug_data.get("main_live_imu", {})
            reset_imu = debug_data.get("parking_reset_imu", {})
            
            cv2.putText(canvas, "MAIN LIVE IMU", (330, 390), cv2.FONT_HERSHEY_SIMPLEX, 0.6, self.C_CYAN, 2)
            
            is_imu_waiting = not main_imu or all(main_imu.get(k, 0.0) == 0.0 for k in ["yaw", "pitch", "roll"])
            if is_imu_waiting:
                cv2.putText(canvas, "IMU WAITING", (330, 430), cv2.FONT_HERSHEY_SIMPLEX, 0.6, self.C_YELLOW, 2)
                cv2.putText(canvas, "Yaw:   0.0", (330, 480), cv2.FONT_HERSHEY_SIMPLEX, 0.6, self.C_WHITE, 1)
                cv2.putText(canvas, "Pitch: 0.0", (330, 520), cv2.FONT_HERSHEY_SIMPLEX, 0.6, self.C_WHITE, 1)
                cv2.putText(canvas, "Roll:  0.0", (330, 560), cv2.FONT_HERSHEY_SIMPLEX, 0.6, self.C_WHITE, 1)
            else:
                cv2.putText(canvas, f"Yaw:   {main_imu.get('yaw', 0):.0f}", (330, 480), cv2.FONT_HERSHEY_SIMPLEX, 0.6, self.C_WHITE, 1)
                cv2.putText(canvas, f"Pitch: {main_imu.get('pitch', 0):.1f}", (330, 520), cv2.FONT_HERSHEY_SIMPLEX, 0.6, self.C_WHITE, 1)
                cv2.putText(canvas, f"Roll:  {main_imu.get('roll', 0):.1f}", (330, 560), cv2.FONT_HERSHEY_SIMPLEX, 0.6, self.C_WHITE, 1)

            cv2.line(canvas, (470, 360), (470, 720), (100,100,100), 1)

            cv2.putText(canvas, "PARKING RESET IMU", (480, 390), cv2.FONT_HERSHEY_SIMPLEX, 0.6, self.C_YELLOW, 2)
            cv2.putText(canvas, f"Reset Ref: {reset_imu.get('reset_yaw', 0):.0f}", (480, 440), cv2.FONT_HERSHEY_SIMPLEX, 0.55, self.C_WHITE, 1)
            
            rel_yaw_val = int(round(reset_imu.get('current_yaw', 0)))
            rel_y_str = f"{rel_yaw_val:03d}"
            cv2.putText(canvas, f"Relative Yaw: {rel_y_str}", (480, 480), cv2.FONT_HERSHEY_SIMPLEX, 0.55, self.C_WHITE, 1)
            
            cv2.putText(canvas, f"Distance: {reset_imu.get('distance_cm', 0):.0f} cm", (480, 530), cv2.FONT_HERSHEY_SIMPLEX, 0.55, self.C_CYAN, 2)
            cv2.putText(canvas, f"Slot: {reset_imu.get('current_slot', 0)}", (480, 575), cv2.FONT_HERSHEY_SIMPLEX, 0.55, self.C_PURPLE, 2)

            # --- 5. Slot Occupancy Map (Bottom Right-TopLeft 320x180) ---
            cv2.rectangle(canvas, (640, 360), (960, 540), (20, 20, 20), -1)
            cv2.rectangle(canvas, (640, 360), (960, 540), self.C_WHITE, 1)
            
            cv2.putText(canvas, f"SLOT MAP & DETECTION", (650, 380), cv2.FONT_HERSHEY_SIMPLEX, 0.6, self.C_WHITE, 2)
            
            sign_dets = debug_data.get("sign_detections", [])
            sign_conf = sign_dets[0].get("confidence", 0.0) if sign_dets else 0.0
            cv2.putText(canvas, f"Parking Sign: {'DETECTED' if sign_dets else 'NONE'}", (650, 405), cv2.FONT_HERSHEY_SIMPLEX, 0.45, self.C_GREEN if sign_dets else self.C_WHITE, 1)
            cv2.putText(canvas, f"Confidence: {sign_conf:.2f}", (800, 405), cv2.FONT_HERSHEY_SIMPLEX, 0.45, self.C_CYAN if sign_dets else self.C_WHITE, 1)
            
            left_car = any(c.get('side') == 'left' for c in debug_data.get("car_detections", []))
            right_car = any(c.get('side') == 'right' for c in debug_data.get("car_detections", []))
            
            cv2.putText(canvas, f"Left Car: {'YES' if left_car else 'NO'}", (650, 425), cv2.FONT_HERSHEY_SIMPLEX, 0.45, self.C_BLUE if left_car else self.C_WHITE, 1)
            cv2.putText(canvas, f"Right Car: {'YES' if right_car else 'NO'}", (800, 425), cv2.FONT_HERSHEY_SIMPLEX, 0.45, self.C_RED if right_car else self.C_WHITE, 1)
            
            occ = debug_data.get("occupancy_map", {})
            for i in range(1, 4): # Display first 3 slots to save space
                slot_info = occ.get(i, {'left': False, 'right': False})
                l_char = "X" if slot_info['left'] else "_"
                r_char = "X" if slot_info['right'] else "_"
                cv2.putText(canvas, f"L{i} [{l_char}]", (650, 450 + i*25), cv2.FONT_HERSHEY_SIMPLEX, 0.5, self.C_RED if slot_info['left'] else self.C_GREEN, 1)
                cv2.putText(canvas, f"R{i} [{r_char}]", (800, 450 + i*25), cv2.FONT_HERSHEY_SIMPLEX, 0.5, self.C_RED if slot_info['right'] else self.C_GREEN, 1)

            # --- 6. Parking Decision Engine (Bottom Right-BotLeft 320x180) ---
            cv2.rectangle(canvas, (640, 540), (960, 720), (40, 40, 40), -1)
            cv2.rectangle(canvas, (640, 540), (960, 720), self.C_WHITE, 1)
            
            cv2.putText(canvas, f"DECISION ENGINE", (650, 560), cv2.FONT_HERSHEY_SIMPLEX, 0.6, self.C_WHITE, 2)
            cv2.putText(canvas, f"Curr Slot: {debug_data.get('current_slot', 1)}", (650, 590), cv2.FONT_HERSHEY_SIMPLEX, 0.6, self.C_WHITE, 1)
            
            sel_slot = debug_data.get("selected_slot")
            sel_side = debug_data.get("selected_side")
            if sel_slot is not None:
                cv2.putText(canvas, f"Selected: Slot {sel_slot}", (650, 620), cv2.FONT_HERSHEY_SIMPLEX, 0.6, self.C_GREEN, 2)
                cv2.putText(canvas, f"Side: {str(sel_side).upper()}", (650, 650), cv2.FONT_HERSHEY_SIMPLEX, 0.6, self.C_BLUE, 2)
            else:
                cv2.putText(canvas, f"Selected: NONE", (650, 620), cv2.FONT_HERSHEY_SIMPLEX, 0.6, self.C_YELLOW, 2)
                
            cv2.putText(canvas, f"Completed: {'YES' if debug_data.get('parking_completed') else 'NO'}", (800, 590), cv2.FONT_HERSHEY_SIMPLEX, 0.5, self.C_GREEN if debug_data.get('parking_completed') else self.C_WHITE, 1)
            cv2.putText(canvas, f"Failed: {'YES' if debug_data.get('parking_failed') else 'NO'}", (800, 620), cv2.FONT_HERSHEY_SIMPLEX, 0.5, self.C_RED if debug_data.get('parking_failed') else self.C_WHITE, 1)

            # --- 7. Reverse Parking Trajectory (Bottom Right-Right 320x360) ---
            cv2.rectangle(canvas, (960, 360), (1280, 720), (10, 10, 10), -1)
            cv2.rectangle(canvas, (960, 360), (1280, 720), self.C_WHITE, 1)
            
            cv2.putText(canvas, f"TRAJECTORY", (970, 380), cv2.FONT_HERSHEY_SIMPLEX, 0.6, self.C_WHITE, 2)
            traj = debug_data.get("trajectory")
            if traj:
                cv2.putText(canvas, f"Loaded Points: {len(traj)}", (970, 410), cv2.FONT_HERSHEY_SIMPLEX, 0.6, self.C_GREEN, 1)
                cv2.putText(canvas, f"Time  | Steer | Speed", (970, 440), cv2.FONT_HERSHEY_SIMPLEX, 0.5, self.C_CYAN, 1)
                # Display first few points
                for i in range(min(8, len(traj))):
                    pt = traj[i]
                    cv2.putText(canvas, f"{pt['time']:.1f} | {pt['steering']:.1f} | {pt['speed']:.2f}", 
                                (970, 465 + i*25), cv2.FONT_HERSHEY_SIMPLEX, 0.5, self.C_WHITE, 1)
                if len(traj) > 8:
                    cv2.putText(canvas, f"... and {len(traj)-8} more", (970, 465 + 8*25), cv2.FONT_HERSHEY_SIMPLEX, 0.5, self.C_GRAY, 1)
            else:
                cv2.putText(canvas, f"No Trajectory Loaded", (970, 410), cv2.FONT_HERSHEY_SIMPLEX, 0.6, self.C_YELLOW, 1)

            cv2.imshow(self.window_name, canvas)
            cv2.waitKey(1)
        except Exception as e:
            print(f"[Parking Dashboard Error] {e}")
