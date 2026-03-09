import tkinter as tk
from tkinter import messagebox
import time
import math
import numpy as np
import cv2
import csv
import json
import os
import sys
import subprocess
import argparse
from PIL import Image, ImageTk

# ── IMPORTS FROM PACKAGES ──────────────────────────────────
from config import *
from dashboard.dashboard_ui import DashboardUI
from dashboard.map_engine import MapEngine
from dashboard.adas_vision_utils import annotate_bev, JunctionDetector, RoundaboutNavigator

try:
    from hardware.serial_handler import STM32_SerialHandler
except ImportError:
    class STM32_SerialHandler:
        def __init__(self): self.running = False
        def connect(self): return True
        def disconnect(self): pass
        def set_speed(self, s): pass
        def set_steering(self, s): pass
        def set_light_state(self, state, on): pass

try:
    from hardware.imu_sensor import IMUSensor
except ImportError:
    class IMUSensor:
        def __init__(self): self.is_calibrated = True
        def start(self): pass
        def stop(self): pass
        def get_yaw(self): return 0.0

try:
    from v2x.v2x_client import V2XClient
except ImportError:
    class V2XClient:
        def __init__(self, *args, **kwargs): pass
        def start(self): pass
        def stop(self): pass
        def update_state(self, *args, **kwargs): pass

# ── AUTONOMOUS STACK IMPORTS (from tempfile) ───────────────
try:
    from perception.camera import Camera
    from perception.lane_detector import LaneDetector
    from control.controller import Controller
    _AUTO_DRIVE_AVAILABLE = True
except ImportError:
    _AUTO_DRIVE_AVAILABLE = False

try:
    from traffic.traffic_module import TrafficDecisionEngine, ThreadedYOLODetector
    from traffic.behavior_controller import BehaviorController
    _AI_AVAILABLE = True
except ImportError:
    _AI_AVAILABLE = False


# ─────────────────────────────────────────────────────────────
#  V2X LAUNCHER
# ─────────────────────────────────────────────────────────────
def launch_v2x_servers():
    """Launch V2X infrastructure as background subprocesses."""
    base_dir = os.path.dirname(os.path.abspath(__file__))
    server_dir = os.path.join(base_dir, "servers", "trafficCommunicationServer")
    sim_dir = os.path.join(base_dir, "servers", "carsAndSemaphoreStreamSIM")

    procs = []
    print("\n--- Starting V2X Servers ---")
    try:
        # 1. Traffic Communication Server
        p1 = subprocess.Popen([sys.executable, "TrafficCommunication.py"], cwd=server_dir,
                              stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        procs.append(p1)
        print("[V2X] TrafficCommunication Server started.")

        # 2. Semaphore + Car Simulator
        p2 = subprocess.Popen([sys.executable, "udpStreamSIM.py"], cwd=sim_dir,
                              stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        procs.append(p2)
        print("[V2X] Semaphore + Car Simulator started.")
    except Exception as e:
        print(f"[V2X] Warning: Could not start servers: {e}")
    return procs


# ─────────────────────────────────────────────────────────────
class MockCtrl: pass

# ─────────────────────────────────────────────────────────────
class BFMC_App:
    def __init__(self, root, args):
        self.root = root
        self.args = args
        self.headless = args.headless
        
        if not self.headless:
            self.root.title("TEAM OPTINX BFMC 2026")
            self.root.geometry("1400x850")
            self.root.minsize(1200, 700)
            self.root.configure(bg=THEME["bg"])
            self.ui = DashboardUI(self.root, self)
        
        self.map_engine = MapEngine()

        # Hardware setup
        self.handler = STM32_SerialHandler()
        self.is_connected = False
        
        self.imu = IMUSensor()
        self.imu.start()

        # V2X Client (Daemon thread)
        self.v2x_client = V2XClient(host=V2X_SERVER_HOST, port=V2X_SERVER_PORT)
        if not args.no_v2x:
            self.v2x_client.start()

        # Physics State
        self.car_x, self.car_y, self.car_yaw = 0.5, 0.5, 0.0
        self.current_speed, self.current_steer = 0.0, 0.0
        self.keys = {'Up': False, 'Down': False, 'Left': False, 'Right': False}
        self.last_ctrl_time = time.time()
        self.current_hz = 0.0
        self.is_calibrating = False
        self.last_logged_cmd = None

        # ADAS State
        self.adas_enabled = True
        self.in_highway_mode = False
        self.crosswalk_timer = 0.0
        self.priority_timer = 0.0

        # Routing State
        self.mode = "DRIVE"
        self.start_node = None; self.end_node = None; self.pass_nodes = []
        self.path = []
        self.visited_path_nodes = set()
        self.path_signs = []
        self.visible_signs = {}

        # Parking/Playback State
        self.is_playing_back = False
        self.is_parking_reverse_mode = False
        self.playback_queue = []; self.playback_cmd = None; self.playback_frames = 0

        # Autonomous Pipelines (Lane Detection)
        self.is_auto_mode    = False
        self.auto_start_time = 0.0
        
        self.camera = Camera(sim_video=None)
        self.detector = LaneDetector() if _AUTO_DRIVE_AVAILABLE else None
        self.controller = Controller() if _AUTO_DRIVE_AVAILABLE else None

        self.traffic_engine, self.behavior, self.yolo = None, None, None
        if _AI_AVAILABLE:
            try:
                self.yolo = ThreadedYOLODetector(YOLO_MODEL_FILE)
                self.traffic_engine = TrafficDecisionEngine(self.yolo)
                self.behavior = BehaviorController()
            except Exception as e:
                print(f"[SYS] Warning: Failed to load AI models: {e}")

        # Bindings & Loops
        if not self.headless:
            self.root.bind("<KeyPress>", self._on_key_press)
            self.root.bind("<KeyRelease>", self._on_key_release)
            # Bind clicks directly to map canvas actions
            self.ui.map_canvas.bind("<Button-1>", self.on_map_click)       # Left Click
        
        self.set_mode("DRIVE")
        self.control_loop()
        
        if not self.headless:
            self.render_map()

    def set_mode(self, m):
        self.mode = m
        if self.headless: return
        self.ui.var_main_mode.set(m)
        for w in self.ui.tool_frame.winfo_children():
            w.destroy()
        if m == "NAV":
            self.ui.build_nav_tools(self)
        elif m == "SIGN":
            self.ui.build_sign_tools(self)
            tk.Label(self.ui.tool_frame, text="Right-Click a node to DELETE sign", 
                     bg=THEME["panel"], fg="yellow", font=THEME["font_p"]).pack(side=tk.LEFT, padx=10)
        else:
            tk.Label(self.ui.tool_frame,
                     text="Drive Mode Active - Click Map to Teleport Digital Twin",
                     bg=THEME["panel"], fg=THEME["success"],
                     font=THEME["font_h"]).pack(side=tk.LEFT, padx=10, pady=5)

    def _get_nearest_node(self, event):
        """Helper function to convert canvas click to nearest GraphML node."""
        cx = self.ui.map_canvas.canvasx(event.x)
        cy = self.ui.map_canvas.canvasy(event.y)
        
        nearest_node = None
        min_dist = float('inf')
        
        for node_id, (px, py) in self.map_engine.node_pixels.items():
            dist = math.hypot(cx - px, cy - py)
            if dist < min_dist:
                min_dist = dist
                nearest_node = node_id
        return nearest_node

    def on_map_click(self, event):
        """Left click executes actions based on the current mode."""
        if self.headless: return
        nearest_node = self._get_nearest_node(event)
        if not nearest_node: return

        # 2. Map actions strictly to the nearest graph node
        if self.mode == "DRIVE":
            node_data = self.map_engine.G.nodes[nearest_node]
            self.car_x = float(node_data.get('x', self.car_x))
            self.car_y = float(node_data.get('y', self.car_y))
            self.render_map()

        elif self.mode == "NAV":
            nav_action = self.ui.var_path.get() if hasattr(self.ui, 'var_path') else "START"
            
            if nav_action == "START":
                self.start_node = nearest_node
                self.ui.log_event(f"Start Node Set: {nearest_node}", "SUCCESS")
            elif nav_action == "PASS":
                self.pass_nodes.append(nearest_node)
                self.ui.log_event(f"Pass Node Added: {nearest_node}", "SUCCESS")
            elif nav_action == "END":
                self.end_node = nearest_node
                self.ui.log_event(f"End Node Set: {nearest_node}", "SUCCESS")
                
            if self.start_node and self.end_node:
                self.path = self.map_engine.calc_path_nodes(self.start_node, self.end_node, self.pass_nodes)
                # Pull ONLY the signs that are on this newly calculated path
                self.path_signs = self.map_engine.get_path_signs(self.path)
                self.ui.log_event(f"Path Calculated. {len(self.path_signs)} signs on route.", "SUCCESS")
            self.render_map()

        elif self.mode == "SIGN":
            is_delete_mode = hasattr(self.ui, 'chk_del') and self.ui.chk_del.get()
            
            if is_delete_mode:
                if self.map_engine.remove_sign(nearest_node):
                    self.ui.log_event(f"🗑️ Sign deleted at Node: {nearest_node}", "WARN")
            else:
                sign_type = "stop-sign"
                if hasattr(self.ui, 'var_sign'):
                    sign_type = self.ui.var_sign.get()
                    
                self.map_engine.remove_sign(nearest_node) # Overwrite if exists
                x_val = float(self.map_engine.G.nodes[nearest_node].get('x', 0.0))
                y_val = float(self.map_engine.G.nodes[nearest_node].get('y', 0.0))
                new_sign = {"node": nearest_node, "type": sign_type, "x": x_val, "y": y_val}
                self.map_engine.signs.append(new_sign)
                self.map_engine.save_signs()
                self.ui.log_event(f"Sign '{sign_type}' placed at Node: {nearest_node}", "SUCCESS")
            self.render_map()

    def execute_parking_playback(self, reverse=False):
        """Reads default_parking.csv and enqueues commands. Handles steering inversion for reverse."""
        filename = "default_parking.csv"
        
        if not os.path.exists(filename):
            if not self.headless:
                self.ui.log_event(f"Error: {filename} not found!", "WARN")
            return

        commands = []
        try:
            with open(filename, 'r') as f:
                reader = csv.DictReader(f)
                for row in reader:
                    commands.append({
                        "speed": float(row.get("speed", 0.0)),
                        "steer": float(row.get("steering", row.get("steer", 0))),
                        "pwm": float(row.get("pwm", 0.0)),
                        "direction": int(row.get("direction", 1)),
                        "duration_fr": int(row.get("duration_fr", 1))
                    })
        except Exception as e:
            if not self.headless:
                self.ui.log_event(f"CSV Read Error: {e}", "DANGER")
            return

        self.playback_queue = []
        
        if reverse:
            # Read backwards and invert math for reverse kinematics
            for cmd in reversed(commands):
                self.playback_queue.append({
                    "speed": cmd["speed"],
                    "steer": -cmd["steer"],   # Invert steering angle
                    "pwm": cmd["pwm"],
                    "direction": -1 if cmd["direction"] == 1 else 1, # Reverse direction
                    "duration_fr": cmd["duration_fr"]
                })
            self.is_parking_reverse_mode = True
        else:
            self.playback_queue = commands
            self.is_parking_reverse_mode = False

        self.is_playing_back = True
        self.is_auto_mode = False 
        self.is_calibrating = False
        
        if not self.headless:
            mode_str = "REVERSE" if reverse else "FORWARD"
            self.ui.log_event(f"Starting {mode_str} parking playback...", "SUCCESS")

    def render_map(self):
        if self.headless: return
        pil = self.map_engine.render_map(
            self.car_x, self.car_y, self.car_yaw,
            self.path, self.visited_path_nodes, self.path_signs,
            True, self.start_node, self.pass_nodes, self.end_node
        )
        self.tk_map = ImageTk.PhotoImage(pil)
        self.ui.map_canvas.create_image(0, 0, anchor=tk.NW, image=self.tk_map)
        self.ui.map_canvas.config(scrollregion=self.ui.map_canvas.bbox(tk.ALL))

    def on_parking_toggle(self):
        pass

    def toggle_connection(self):
        if not self.is_connected:
            if self.handler.connect():
                self.is_connected = True
                if not self.headless:
                    self.ui.lbl_conn.config(text="🟢 CONNECTED", fg=THEME["success"])
                    self.ui.btn_connect.config(text="DISCONNECT", bg=THEME["danger"])
                    self.ui.log_event("🔗 Connected to STM32 Hardware successfully.", "SUCCESS")
        else:
            self.handler.disconnect(); self.is_connected = False
            if not self.headless:
                self.ui.log_event("🔌 Disconnected from STM32.", "WARN")
                self.ui.lbl_conn.config(text="⚫ DISCONNECTED", fg=THEME["danger"])
                self.ui.btn_connect.config(text="CONNECT CAR", bg=THEME["accent"])

    def toggle_auto_mode(self):
        self.is_auto_mode = not self.is_auto_mode
        self.is_playing_back = False 
        if self.is_auto_mode:
            self.auto_start_time = time.time()
            self.is_calibrating  = True
            for k in self.keys: self.keys[k] = False
            if not self.headless:
                self.ui.btn_auto.config(text="MODE: AUTONOMOUS", bg="#9b59b6")
                self.ui.log_event("🤖 Switched to AUTONOMOUS. Calibrating 5s …", "SUCCESS")
        else:
            self.is_calibrating  = False
            for k in self.keys: self.keys[k] = False
            if not self.headless:
                self.ui.btn_auto.config(text="MODE: MANUAL", bg="#444")
                self.ui.log_event("🖐 Switched to MANUAL mode.", "WARN")

    # ─────────────────────────────────────────────────────────
    # CONTROL LOOP  (20 Hz)
    # ─────────────────────────────────────────────────────────
    def control_loop(self):
        now = time.time()
        dt = max(now - self.last_ctrl_time, 0.001)
        self.last_ctrl_time = now

        base_speed = float(self.ui.slider_base_speed.get() if not self.headless else 50.0)
        steer_mult = float(self.ui.slider_steer_mult.get() if not self.headless else 1.0)

        target_speed, target_steer = 0.0, 0.0
        
        # 1. Grab Frame
        frame = self.camera.read_frame()
        lane_result = None
        t_res = None
        behav_out = None
        active_sign_cmd = None

        if frame is not None and self.detector and self.controller:
            # 2. Process Lane Detection
            yaw_deg = self.imu.get_yaw()
            lane_result = self.detector.process(
                frame, dt=dt, velocity_ms=max(self.current_speed / 1000.0, 0.0), 
                last_steering=self.current_steer, current_yaw=yaw_deg
            )
            
            # 3. Process AI & Semantic Traffic Rules
            ai_labels = []
            if self.traffic_engine:
                line_type = getattr(lane_result, 'lane_type', 'UNKNOWN')
                t_res = self.traffic_engine.process(frame, line_type)
                if t_res and hasattr(t_res, 'active_labels'):
                    ai_labels = t_res.active_labels
            
            # 4. Update Path Sign States (Distance + AI Vision)
            detect_dist = float(self.ui.slider_sign_detect.get() if not self.headless else 5.0)
            act_dist = float(self.ui.slider_sign_act.get() if not self.headless else 2.0)
            ai_dist = getattr(t_res, 'sign_approach_m', 99.0) if t_res else 99.0
            light_status = getattr(t_res, 'light_status', 'NONE') if t_res else 'NONE'
            
            # Form active blocking states to prevent premature sign completion
            self.active_blocks = {
                "crosswalk": time.time() < self.crosswalk_timer,
                "priority": time.time() < self.priority_timer,
                "pedestrian": any(label.lower() in ["pedestrian", "person"] for label in ai_labels),
                "parking": getattr(self, 'is_playing_back', False) or getattr(self, 'is_waiting_for_reverse', False)
            }
            
            teleport_node = None
            if not self.is_playing_back and not getattr(self, 'is_waiting_for_reverse', False):
                active_sign_cmd, self.path_signs, teleport_node = self.map_engine.update_sign_statuses(
                    self.path_signs, ai_labels, ai_dist, detect_dist=detect_dist, act_dist=act_dist, light_status=light_status, active_blocks=self.active_blocks
                )
            
            # --- TELEPORT CAR TO COMPLETED SIGN ---
            if teleport_node and teleport_node in self.map_engine.G.nodes:
                node_data = self.map_engine.G.nodes[teleport_node]
                self.car_x = float(node_data.get('x', self.car_x))
                self.car_y = float(node_data.get('y', self.car_y))
                # Sync path_distance to this node so the kinematics sim doesn't override it immediately
                if self.path and len(self.path) > 1:
                    acc_dist = 0.0
                    for i in range(len(self.path) - 1):
                        n1 = str(self.path[i])
                        if n1 == str(teleport_node):
                            self.path_distance = acc_dist
                            break
                        n2 = str(self.path[i+1])
                        if n1 in self.map_engine.G.nodes and n2 in self.map_engine.G.nodes:
                            x1, y1 = float(self.map_engine.G.nodes[n1].get('x', 0)), float(self.map_engine.G.nodes[n1].get('y', 0))
                            x2, y2 = float(self.map_engine.G.nodes[n2].get('x', 0)), float(self.map_engine.G.nodes[n2].get('y', 0))
                            acc_dist += math.hypot(x2 - x1, y2 - y1)
                if not self.headless:
                    self.ui.log_event(f"📍 Teleported to completed sign node: {teleport_node}", "SUCCESS")
            # --------------------------------------
            
            # --- OVERRIDE TIMERS FOR NEW LOGIC ---
            if active_sign_cmd:
                if "crosswalk" in active_sign_cmd.lower() or "pedestrian" in active_sign_cmd.lower():
                    self.crosswalk_timer = time.time() + 5.0
                elif "priority" in active_sign_cmd.lower():
                    self.priority_timer = time.time() + 10.0
                elif "park" in active_sign_cmd.lower():
                    if not getattr(self, 'has_parked_here', False):
                        self.execute_parking_playback(reverse=False)
                        self.has_parked_here = True
            
            if not active_sign_cmd or "park" not in active_sign_cmd.lower():
                self.has_parked_here = False
            # ------------------------------------
            
            if active_sign_cmd and not self.headless:
                if active_sign_cmd != self.last_logged_cmd:
                    self.ui.log_event(f"❗ Responding to active sign: {active_sign_cmd}", "WARN")
                    self.last_logged_cmd = active_sign_cmd
            elif not active_sign_cmd:
                self.last_logged_cmd = None

            # 5. Calculate Steering & Speed Control
            if self.is_auto_mode and not self.is_playing_back:
                if time.time() - self.auto_start_time > 5.0 and self.imu.is_calibrated:
                    self.is_calibrating = False
                    ctrl_out = self.controller.compute(lane_result, velocity_ms=max(self.current_speed / 1000.0, 0.0), base_speed=base_speed, dt=dt)
                    target_speed = ctrl_out.speed_pwm
                    target_steer = ctrl_out.steer_angle_deg * steer_mult
                    
                    if self.behavior:
                        behav_out = self.behavior.compute(
                            lane_result, t_res, dt, base_speed=base_speed, base_steer=ctrl_out.steer_angle_deg
                        )
                        # NOTE: If `active_sign_cmd` is active, it should ideally override here
                        target_speed = behav_out.speed_pwm
                        target_steer = behav_out.steer_deg
                        
                        # --- NEW SPEED MULTIPLIER AND IMMEDIATE OVERRIDE LOGIC ---
                        if active_sign_cmd and "highway" in active_sign_cmd.lower():
                            if "entry" in active_sign_cmd.lower():
                                self.in_highway_mode = True
                            elif "exit" in active_sign_cmd.lower():
                                self.in_highway_mode = False
                                
                        is_highway = False
                        if self.in_highway_mode or "highway" in getattr(behav_out, "zone_mode", "").lower():
                            is_highway = True

                        # Hard overrides (absolute halt)
                        halt_cmds = ["red-light"]
                        if active_sign_cmd in halt_cmds or (active_sign_cmd == "traffic-light" and "GREEN" not in light_status):
                            target_speed = 0.0
                            
                        # Dynamic Pedestrian Halt
                        if any(label.lower() in ["pedestrian", "person"] for label in ai_labels):
                            target_speed = 0.0

                        # Multipliers
                        if time.time() < self.crosswalk_timer:
                            target_speed *= 0.8
                        if time.time() < self.priority_timer:
                            target_speed *= 0.8
                        if is_highway:
                            target_speed *= 1.3
                        # --------------------------------------------------------
                else:
                    self.is_calibrating = True
                    target_speed, target_steer = 0.0, 0.0
                    
            # 6. Dashboard CAM + BEV Render
            if not self.headless:
                final_cam = t_res.yolo_debug_frame if (t_res and getattr(t_res, 'yolo_debug_frame', None) is not None) else frame
                final_cam = cv2.cvtColor(final_cam, cv2.COLOR_BGR2RGB)
                img = Image.fromarray(final_cam).resize((440, 330))
                self.ui.cam_label.imgtk = ImageTk.PhotoImage(image=img)
                self.ui.cam_label.configure(image=self.ui.cam_label.imgtk)

                if hasattr(lane_result, 'lane_dbg'):
                    dbg = lane_result.lane_dbg.copy()

                    cv2.putText(dbg, lane_result.anchor, (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)
                    cv2.putText(dbg, f"Target X: {lane_result.target_x:.1f}", (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)
                    cv2.putText(dbg, f"Lat Error: {lane_result.lateral_error_px:+.1f}px", (10, 75), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 200, 255), 1)
                    
                    steer_color = (100,255,100) if abs(self.current_steer)<15 else (100,100,255)
                    cv2.putText(dbg, f"STEER: {self.current_steer:+.1f} deg", (420, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.5, steer_color, 2)
                    cv2.putText(dbg, f"SPEED: {self.current_speed:.0f} PWM", (420, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 100, 100), 2)
                    
                    if t_res is not None and behav_out is not None:
                        cv2.putText(dbg, f"STATE: {behav_out.state}", (420, 75), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 2)
                        cv2.putText(dbg, f"ZONE: {behav_out.zone_mode}", (420, 100), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 100, 255), 2)
                        y_offset = 120
                        if ai_labels:
                            cv2.putText(dbg, "YOLO Detections:", (10, 100), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
                            for label in ai_labels:
                                cv2.putText(dbg, f"- {label}", (10, y_offset), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (100, 255, 100), 1)
                                y_offset += 20

                    bev = cv2.cvtColor(dbg, cv2.COLOR_BGR2RGB)
                    img_bev = Image.fromarray(bev).resize((440, 330))
                    self.ui.bev_label.imgtk = ImageTk.PhotoImage(image=img_bev)
                    self.ui.bev_label.configure(image=self.ui.bev_label.imgtk)

        # ── PARKING PLAYBACK OVERRIDE ─────────────────────────
        if self.is_playing_back:
            self.is_calibrating = False
            
            # If no current command is loaded or its duration is over, grab the next
            if self.playback_cmd is None or self.playback_frames <= 0:
                if self.playback_queue:
                    self.playback_cmd = self.playback_queue.pop(0)
                    self.playback_frames = self.playback_cmd.get("duration_fr", 1)
                else:
                    self.playback_cmd = None
            
            if self.playback_cmd:
                self.playback_frames -= 1
                cmd = self.playback_cmd
                
                # If PWM is available, use it directly with the direction multiplier.
                # Otherwise, fallback on raw speed. 
                # direction=1 (forward), direction=-1 (reverse)
                dir_mult = cmd.get("direction", 1)
                if dir_mult == 0: dir_mult = 1
                
                if cmd.get("pwm", 0) > 0:
                    target_speed = cmd["pwm"] * dir_mult
                else:
                    target_speed = cmd["speed"] * dir_mult
                    
                target_steer = cmd["steer"]
                
                # Log parking playback once per second
                if not getattr(self, '_last_park_log_time', 0) or time.time() - self._last_park_log_time > 1.0:
                    self._last_park_log_time = time.time()
                    if not self.headless:
                        self.ui.log_event(f"🅿️ Parking Steer: {target_steer:.1f}° | Spd: {target_speed:.1f}", "INFO")
            else:
                is_finishing_reverse = self.is_parking_reverse_mode
                self.is_playing_back = False
                self.is_parking_reverse_mode = False
                target_speed = 0.0
                target_steer = 0.0
                
                if not is_finishing_reverse and hasattr(self.ui, 'chk_parking') and self.ui.chk_parking.get():
                    self.is_waiting_for_reverse = True
                    self.reverse_timer = time.time() + 10.0
                    if not self.headless:
                        self.ui.log_event("Parking reached. Waiting 10s for Auto-Reverse...", "WARN")
                else:
                    if not self.headless:
                        self.ui.log_event("Parking sequence fully complete.", "SUCCESS")

        elif getattr(self, 'is_waiting_for_reverse', False):
            self.is_calibrating = False
            target_speed = 0.0
            target_steer = 0.0
            if time.time() > self.reverse_timer:
                self.is_waiting_for_reverse = False
                self.execute_parking_playback(reverse=True)

        # ── MANUAL OVERRIDES ──────────────────────────────────
        elif not self.is_auto_mode:
            self.is_calibrating = False
            target_speed = (base_speed  if self.keys['Up'] else (-base_speed if self.keys['Down'] else 0))
            target_steer = (-25 * steer_mult if self.keys['Left'] else (25 * steer_mult if self.keys['Right'] else 0))

        # ── SMOOTH APPLICATION ────────────────────────────────
        if target_speed == 0:
            self.current_speed = 0.0 
        else:
            self.current_speed += (target_speed - self.current_speed) * 0.2

        if target_steer == 0:
            self.current_steer = 0.0 
        else:
            self.current_steer += (target_steer - self.current_steer) * 0.2

        # ── HARDWARE OUTPUT ───────────────────────────────────
        if self.is_connected:
            if not self.imu.is_calibrated and (self.is_auto_mode):
                self.handler.set_speed(0)
                self.handler.set_steering(0)
            else:
                self.handler.set_speed(int(self.current_speed))
                self.handler.set_steering(self.current_steer)

        # ── V2X TELEMETRY PUSH ────────────────────────────────
        yaw_rad = math.radians(self.imu.get_yaw())
        self.v2x_client.update_state(
            x=self.car_x,
            y=self.car_y,
            yaw=self.imu.get_yaw(),
            speed=self.current_speed
        )

        # ── KINEMATICS SIMULATION (Map update) ────────────────
        if abs(self.current_speed) < 1:  self.current_speed = 0
        if abs(self.current_steer) < 0.5: self.current_steer = 0

        sim_mult = float(self.ui.slider_sim_speed.get() if not self.headless else 1.0)
        v_ms = (self.current_speed / 1000.0) * sim_mult * 1.5
        
        # --- NEW MAGNETIC PATH SNAP LOGIC ---
        if self.path and len(self.path) > 1:
            # Check if this is a fresh start to reset path distance
            path_tuple = tuple(self.path)
            if self.last_path_tuple != path_tuple:
                self.last_path_tuple = path_tuple
                self.path_distance = 0.0
                
            self.path_distance += v_ms * dt
            # Constrain distance
            if self.path_distance < 0: self.path_distance = 0
            
            # Find the segment currently at `self.path_distance`
            acc_dist = 0.0
            found_segment = False
            for i in range(len(self.path) - 1):
                n1 = str(self.path[i])
                n2 = str(self.path[i+1])
                
                if n1 not in self.map_engine.G.nodes or n2 not in self.map_engine.G.nodes:
                    continue
                    
                x1, y1 = float(self.map_engine.G.nodes[n1].get('x', 0)), float(self.map_engine.G.nodes[n1].get('y', 0))
                x2, y2 = float(self.map_engine.G.nodes[n2].get('x', 0)), float(self.map_engine.G.nodes[n2].get('y', 0))
                
                seg_len = math.hypot(x2 - x1, y2 - y1)
                
                if self.path_distance <= acc_dist + seg_len:
                    # Car is within this segment
                    ratio = (self.path_distance - acc_dist) / seg_len if seg_len > 0 else 0
                    self.car_x = x1 + ratio * (x2 - x1)
                    self.car_y = y1 + ratio * (y2 - y1)
                    self.car_yaw = math.atan2(y2 - y1, x2 - x1)
                    found_segment = True
                    self.visited_path_nodes.add(n1)
                    break
                    
                acc_dist += seg_len
            
            if not found_segment:
                # Car has reached the end of the path
                n_end = str(self.path[-1])
                if n_end in self.map_engine.G.nodes:
                    self.car_x = float(self.map_engine.G.nodes[n_end].get('x', 0))
                    self.car_y = float(self.map_engine.G.nodes[n_end].get('y', 0))
                self.current_speed = 0.0 # Force stop at end of path
        else:
            # Fallback unconstrained kinematics if no map path is active
            steer_rad = math.radians(self.current_steer)
            self.car_yaw -= (v_ms / max(WHEELBASE_M, 0.01)) * math.tan(steer_rad) * dt
            self.car_yaw  = (self.car_yaw + math.pi) % (2 * math.pi) - math.pi
            self.car_x   += v_ms * math.cos(self.car_yaw) * dt
            self.car_y   += v_ms * math.sin(self.car_yaw) * dt
            self.path_distance = 0.0
            self.last_path_tuple = None
        # ------------------------------------------------------

        # ── UI UPDATES ────────────────────────────────────────
        if not self.headless:
            hz = 1.0 / dt if dt > 0 else 0.0
            self.current_hz = 0.8 * self.current_hz + 0.2 * hz
            self.ui.lbl_hz.config(text=f"{self.current_hz:.1f} Hz", fg="cyan")
            
            mode_str = "AUTONOMOUS" if self.is_auto_mode else "MANUAL"
            if self.is_playing_back: mode_str = "REVERSE PARKING" if self.is_parking_reverse_mode else "PARKING PLAYBACK"
            if self.is_calibrating: mode_str = "CALIBRATING..."
            
            self.ui.lbl_telemetry.config(
                text=f"SPD: {int(self.current_speed)} | STR: {self.current_steer:.1f}° | LMT: {base_speed} | [{mode_str}]"
            )
            
            # --- Update Indicators ---
            active_keys = []
            if active_sign_cmd:
                cmd_l = active_sign_cmd.lower()
                if 'stop' in cmd_l: active_keys.append('stop_sign')
                elif 'no_entry' in cmd_l or 'no-entry' in cmd_l: active_keys.append('no_entry')
                elif 'pedestrian' in cmd_l or 'crosswalk' in cmd_l: active_keys.append('pedestrian')
                elif 'highway' in cmd_l: active_keys.append('highway')
                elif 'park' in cmd_l: active_keys.append('park')
                else: active_keys.append('caution')
                
            # Keep Indicators glowing if their internal logic blocks are still active
            if getattr(self, 'active_blocks', None):
                if self.active_blocks.get('crosswalk') or self.active_blocks.get('pedestrian'):
                    if 'pedestrian' not in active_keys: active_keys.append('pedestrian')
                if self.active_blocks.get('priority'):
                    if 'caution' not in active_keys: active_keys.append('caution')
                    
            if getattr(self, 'in_highway_mode', False) and 'highway' not in active_keys:
                active_keys.append('highway')
                
            if behav_out:
                ls = getattr(behav_out, 'light_status', '')
                if 'RED' in ls: active_keys.append('red_light')
                elif 'YELLOW' in ls: active_keys.append('yellow_light')
                elif 'GREEN' in ls: active_keys.append('green_light')
                if getattr(behav_out, 'parking_state', 'NONE') not in ('NONE', 'DONE'): active_keys.append('park')
                if getattr(behav_out, 'state', '') == 'SYS_LANE_CHANGE_LEFT': active_keys.append('overtake')
                if getattr(behav_out, 'zone_mode', '') == 'HIGHWAY': active_keys.append('highway')
            
            self.ui.update_indicators(active_keys)
            # -------------------------

            self.render_map()

        if self.headless:
            print(f"[CTRL] Spd:{int(self.current_speed):4d} | Str:{self.current_steer:5.1f}° | Yaw:{self.imu.get_yaw():5.1f}° | Pos:({self.car_x:.1f},{self.car_y:.1f}) | {1/dt:.0f}Hz", end="\r")

        if self.headless:
            time.sleep(0.05)
            self.control_loop()
        else:
            self.root.after(50, self.control_loop)

    def save_config(self): pass
    def load_config(self): pass

    def toggle_adas_mode(self):
        self.adas_enabled = not self.adas_enabled
        if not self.headless:
            if self.adas_enabled:
                self.ui.btn_adas.config(text="ADAS ASSIST: ON", bg="#9b59b6")
                self.ui.log_event("✅ ADAS ASSIST enabled.", "SUCCESS")
            else:
                self.ui.btn_adas.config(text="ADAS ASSIST: OFF", bg="#444")
                self.ui.log_event("⚠️ ADAS ASSIST DISABLED.", "WARN")

    def clear_route(self):
        self.start_node = None; self.end_node = None; self.pass_nodes = []; self.path = []
        self.path_signs = []
        self.visited_path_nodes.clear()
        if not self.headless:
            for item in self.ui.tree.get_children():
                self.ui.tree.delete(item)
            self.render_map()
            self.ui.log_event("🗑 Route & sign history cleared. Ready for new run.", "WARN")

    def _on_key_press(self, e):
        if self.is_auto_mode or self.is_playing_back: return
        if e.keysym in self.keys: self.keys[e.keysym] = True

    def _on_key_release(self, e):
        if e.keysym in self.keys: self.keys[e.keysym] = False

    def on_close(self):
        self.camera.stop()
        if self.yolo: self.yolo.stop()
        if self.is_connected:
            self.handler.set_speed(0)
            self.handler.set_steering(0)
            self.handler.disconnect()
        self.imu.stop()
        self.v2x_client.stop()
        if not self.headless:
            self.root.destroy()

# ─────────────────────────────────────────────────────────────
if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="BFMC 2026 Unified Autonomous Stack")
    parser.add_argument("--headless", action="store_true", help="Run in terminal only, no Tkinter GUI")
    parser.add_argument("--no-v2x", action="store_true", help="Do not start the background V2X servers")
    args = parser.parse_args()

    v2x_procs = []
    if not args.no_v2x:
        pass # V2X logic disabled/managed separately 

    try:
        if args.headless:
            class FakeRoot: pass
            app = BFMC_App(FakeRoot(), args)
        else:
            root = tk.Tk()
            app = BFMC_App(root, args)
            root.protocol("WM_DELETE_WINDOW", app.on_close)
            root.mainloop()

    except KeyboardInterrupt:
        print("\n[SYS] Interrupted by user.")
    except Exception as e:
        import traceback
        print("\n[SYS] FATAL ERROR:")
        traceback.print_exc()
    finally:
        if not args.headless:
            try: app.on_close()
            except: pass
        print("\n[SYS] Cleaning up V2X servers...")
        for p in v2x_procs:
            p.terminate()
