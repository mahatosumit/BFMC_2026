import tkinter as tk
from tkinter import ttk
from PIL import Image, ImageTk
from datetime import datetime
from config import THEME, SIGN_MAP

class DashboardUI:
    def __init__(self, root, controller):
        self.root = root
        self.controller = controller
        self.indicators = {}
        self._setup_layout()

    def _setup_layout(self):
        style = ttk.Style()
        style.configure('TPanedwindow', background=THEME["sash"])
        style.configure('TNotebook', background=THEME["panel"], borderwidth=0)
        style.configure('TNotebook.Tab', font=("Helvetica", 10, "bold"), padding=[10, 2])
        
        # --- Top Status Bar ---
        status = tk.Frame(self.root, bg=THEME["panel"], height=35)
        status.pack(side=tk.TOP, fill=tk.X)
        self.lbl_conn = tk.Label(status, text="⚫ DISCONNECTED", bg=THEME["panel"], fg=THEME["danger"], font=THEME["font_h"])
        self.lbl_conn.pack(side=tk.LEFT, padx=15)
        self.lbl_telemetry = tk.Label(status, text="SPD: 0 | STR: 0° | LMT: 150 | [MANUAL]", bg=THEME["panel"], fg=THEME["accent"], font=("Courier", 11, "bold"))
        self.lbl_telemetry.pack(side=tk.LEFT, padx=20)
        self.lbl_ai = tk.Label(status, text="AI: OFF", bg=THEME["panel"], fg="grey", font=("Courier", 10))
        self.lbl_ai.pack(side=tk.RIGHT, padx=15)

        # --- Main Layout Splitter ---
        main_panes = ttk.PanedWindow(self.root, orient=tk.HORIZONTAL)
        main_panes.pack(fill=tk.BOTH, expand=True, padx=5, pady=5)
        
        # --- LEFT PANEL (Cameras) ---
        left_panes = ttk.PanedWindow(main_panes, orient=tk.VERTICAL)
        main_panes.add(left_panes, weight=1)
        
        cam_frm = tk.LabelFrame(left_panes, text="Raw Camera (YOLO ADAS)", bg=THEME["panel"], fg="white", font=THEME["font_h"])
        self.cam_label = tk.Label(cam_frm, bg="black")
        self.cam_label.pack(fill=tk.BOTH, expand=True, padx=5, pady=5)
        placeholder = Image.new('RGB', (440, 330), color='black')
        self._ph_img = ImageTk.PhotoImage(placeholder)
        self.cam_label.configure(image=self._ph_img)
        left_panes.add(cam_frm, weight=1)

        bev_frm = tk.LabelFrame(left_panes, text="Bird's Eye View (VIZ-06)", bg=THEME["panel"], fg="white", font=THEME["font_h"])
        self.bev_label = tk.Label(bev_frm, bg="black")
        self.bev_label.pack(fill=tk.BOTH, expand=True, padx=5, pady=5)
        self.bev_label.configure(image=self._ph_img)
        left_panes.add(bev_frm, weight=1)
        
        # --- MIDDLE PANEL (Controls & Tuning) ---
        mid_col_panes = ttk.PanedWindow(main_panes, orient=tk.VERTICAL)
        main_panes.add(mid_col_panes, weight=1)
        mid_top_frm = tk.Frame(mid_col_panes, bg=THEME["bg"])
        mid_col_panes.add(mid_top_frm, weight=2)

        def make_slider(parent, text, row, from_, to_, res, default):
            tk.Label(parent, text=text, bg=THEME["panel"], fg="#ccc", font=("Helvetica", 10, "bold")).grid(row=row, column=0, sticky="w", padx=5, pady=2)
            s = tk.Scale(parent, from_=from_, to=to_, resolution=res, orient=tk.HORIZONTAL, bg=THEME["panel"], fg="white", highlightthickness=0, bd=0, length=150, sliderlength=15, width=12)
            s.set(default)
            s.grid(row=row, column=1, sticky="ew", padx=5, pady=2)
            return s

        # System Controls
        sys_frm = tk.LabelFrame(mid_top_frm, text="System Controls", bg=THEME["panel"], fg="white", font=THEME["font_h"])
        sys_frm.pack(fill=tk.X, padx=5, pady=(0, 5))
        sys_frm.columnconfigure(0, weight=1); sys_frm.columnconfigure(1, weight=1)
        
        tk.Button(sys_frm, text="💾 Save Config", bg="#2980b9", fg="white", relief="flat", font=("Helvetica", 9, "bold"), command=self.controller.save_config).grid(row=0, column=0, sticky="ew", padx=3, pady=3)
        tk.Button(sys_frm, text="📂 Load Config", bg="#27ae60", fg="white", relief="flat", font=("Helvetica", 9, "bold"), command=self.controller.load_config).grid(row=0, column=1, sticky="ew", padx=3, pady=3)
        self.btn_connect = tk.Button(sys_frm, text="CONNECT CAR", bg=THEME["accent"], fg="white", relief="flat", font=("Helvetica", 10, "bold"), command=self.controller.toggle_connection)
        self.btn_connect.grid(row=1, column=0, columnspan=2, sticky="ew", padx=3, pady=3)
        self.btn_auto = tk.Button(sys_frm, text="MODE: MANUAL", bg="#444", fg="white", relief="flat", font=THEME["font_h"], command=self.controller.toggle_auto_mode)
        self.btn_auto.grid(row=2, column=0, sticky="ew", padx=3, pady=3)
        self.btn_adas = tk.Button(sys_frm, text="ADAS ASSIST: ON", bg="#9b59b6", fg="white", relief="flat", font=THEME["font_h"], command=self.controller.toggle_adas_mode)
        self.btn_adas.grid(row=2, column=1, sticky="ew", padx=3, pady=3)

        # Drive Dynamics
        tab_drive = tk.Frame(mid_top_frm, bg=THEME["bg"])
        tab_drive.pack(fill=tk.BOTH, expand=True, padx=5, pady=3)

        # TAB 1: Drive Dynamics
        dyn_frm = tk.LabelFrame(tab_drive, text="Drive Dynamics", bg=THEME["panel"], fg="white", font=THEME["font_h"])
        dyn_frm.pack(fill=tk.X, padx=5, pady=5)
        dyn_frm.columnconfigure(1, weight=1)
        self.slider_base_speed = make_slider(dyn_frm, "Base Speed (PWM):", 0, 0, 500, 1, 150)
        self.slider_sim_speed = make_slider(dyn_frm, "Map Sim Mult:", 1, 0.1, 3.0, 0.1, 1.0)
        self.slider_steer_mult = make_slider(dyn_frm, "Steer Multiplier:", 2, 0.1, 3.0, 0.1, 1.0)
        self.slider_overtake_dist = make_slider(dyn_frm, "Overtake Dist (m):", 3, 0.5, 5.0, 0.1, 1.2)
        self.slider_overtake_time = make_slider(dyn_frm, "Overtake Time (s):", 4, 1.0, 5.0, 0.2, 2.0)
        self.slider_sign_detect = make_slider(dyn_frm, "Sign Detect (m):", 5, 1.0, 10.0, 0.5, 5.0)
        self.slider_sign_act = make_slider(dyn_frm, "Sign Act (m):", 6, 0.5, 5.0, 0.1, 2.0)

        adas_frm = tk.LabelFrame(tab_drive, text="Active ADAS Responses", bg=THEME["panel"], fg="white", font=THEME["font_h"])
        adas_frm.pack(fill=tk.BOTH, expand=True, padx=5, pady=3)
        self.chk_parking = tk.BooleanVar(value=False)
        self.btn_parking = tk.Checkbutton(adas_frm, text="🏁 AUTO-REVERSE PARKING (10s)", variable=self.chk_parking, bg=THEME["danger"], fg="white", selectcolor="#333", font=("Helvetica", 10, "bold"))
        self.btn_parking.pack(side=tk.TOP, fill=tk.X, padx=5, pady=2)
        
        states = [("🛑 STOP", "stop_sign"), ("⛔ NO ENTRY", "no_entry"), ("🚸 PEDESTRIAN", "pedestrian"), ("🔴 RED LGT", "red_light"), ("🟡 YEL LGT", "yellow_light"), ("🟢 GRN LGT", "green_light"), ("⚠️ CAUTION", "caution"), ("🛣️ HIGHWAY", "highway"), ("🅿️ AUTO-PARK", "park"), ("🏎️ OVERTAKE", "overtake")]
        grid_frm = tk.Frame(adas_frm, bg=THEME["panel"])
        grid_frm.pack(fill=tk.BOTH, expand=True, padx=2, pady=4)
        for i, (text, key) in enumerate(states):
            f = tk.Frame(grid_frm, bg=THEME["panel"])
            f.grid(row=i//2, column=i%2, sticky="w", padx=4, pady=3)
            c = tk.Canvas(f, width=14, height=14, bg=THEME["panel"], highlightthickness=0)
            c.pack(side=tk.LEFT)
            dot = c.create_oval(2, 2, 12, 12, fill="#333333", outline="#555555")
            tk.Label(f, text=text, bg=THEME["panel"], fg="white", font=("Helvetica", 9, "bold")).pack(side=tk.LEFT, padx=4)
            self.indicators[key] = (c, dot)

        # Focus Pad
        self.drive_pad = tk.Label(mid_top_frm, text="🎯 CLICK HERE TO DRIVE 🎯", bg="#8e44ad", fg="white", font=("Helvetica", 11, "bold"), relief="raised", pady=10, cursor="target")
        self.drive_pad.pack(fill=tk.X, padx=5, pady=5)
        self.drive_pad.bind("<Button-1>", lambda e: self.drive_pad.focus_set())
        self.drive_pad.config(takefocus=True)

        # Logs
        log_frm = tk.LabelFrame(mid_col_panes, text="System Log (Resizable)", bg=THEME["panel"], fg="white", font=THEME["font_h"])
        mid_col_panes.add(log_frm, weight=1)
        log_head = tk.Frame(log_frm, bg=THEME["panel"]); log_head.pack(fill=tk.X, padx=5, pady=2)
        tk.Label(log_head, text="Events:", bg=THEME["panel"], fg="#aaa", font=("Courier", 9, "bold")).pack(side=tk.LEFT)
        self.lbl_hz = tk.Label(log_head, text="OFFLINE", bg=THEME["panel"], fg="gray", font=("Courier", 10, "bold"))
        self.lbl_hz.pack(side=tk.RIGHT)
        self.log_text = tk.Text(log_frm, height=4, bg="black", fg="#00ff00", font=("Courier", 10), state="disabled", relief="flat")
        scroll_log = ttk.Scrollbar(log_frm, command=self.log_text.yview)
        self.log_text.configure(yscrollcommand=scroll_log.set)
        self.log_text.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=5, pady=5)
        scroll_log.pack(side=tk.RIGHT, fill=tk.Y)
        self.log_text.tag_config("INFO", foreground="#cccccc"); self.log_text.tag_config("WARN", foreground="orange")
        self.log_text.tag_config("CRITICAL", foreground="#ff3333"); self.log_text.tag_config("SUCCESS", foreground="#00ff00")

        # --- RIGHT PANEL (Map) ---
        right_frame = tk.Frame(main_panes, bg=THEME["bg"])
        main_panes.add(right_frame, weight=2)
        mode_frm = tk.LabelFrame(right_frame, text="Interactive Map Mode", bg=THEME["panel"], fg="white", font=THEME["font_h"])
        mode_frm.pack(fill=tk.X, padx=5, pady=2)
        self.var_main_mode = tk.StringVar(value="DRIVE")
        for txt, val in [("🚗 DRIVE", "DRIVE"), ("🗺️ PLAN PATH", "NAV"), ("🛑 PLACE SIGNS", "SIGN")]:
            tk.Radiobutton(mode_frm, text=txt, variable=self.var_main_mode, value=val, command=lambda v=val: self.controller.set_mode(v), bg=THEME["panel"], fg="white", selectcolor="#444", indicatoron=0, font=("Helvetica", 10, "bold")).pack(side=tk.LEFT, padx=5, pady=5, fill=tk.X, expand=True)

        self.tool_frame = tk.Frame(right_frame, bg=THEME["panel"], height=40)
        self.tool_frame.pack(fill=tk.X, padx=5, pady=(0, 5))
        
        right_panes = ttk.PanedWindow(right_frame, orient=tk.VERTICAL)
        right_panes.pack(fill=tk.BOTH, expand=True, padx=5)
        map_container = tk.Frame(right_panes, bg=THEME["bg"])
        right_panes.add(map_container, weight=3)
        self.map_canvas = tk.Canvas(map_container, bg="black", highlightthickness=0)
        
        # NOTE: Map bindings are now injected inside main.py initialization directly, allowing 
        # both Left & Right click without crossing files.
        self.map_canvas.bind("<B1-Motion>", self.controller.on_map_click) 
        
        map_scroll_y = ttk.Scrollbar(map_container, orient=tk.VERTICAL, command=self.map_canvas.yview)
        map_scroll_x = ttk.Scrollbar(map_container, orient=tk.HORIZONTAL, command=self.map_canvas.xview)
        self.map_canvas.config(yscrollcommand=map_scroll_y.set, xscrollcommand=map_scroll_x.set)
        map_scroll_y.pack(side=tk.RIGHT, fill=tk.Y); map_scroll_x.pack(side=tk.BOTTOM, fill=tk.X)
        self.map_canvas.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)

        table_frame = tk.LabelFrame(right_panes, text="Route Manifest & Live Status", bg=THEME["panel"], fg="white", font=THEME["font_h"])
        right_panes.add(table_frame, weight=1)
        self.tree = ttk.Treeview(table_frame, columns=("type", "loc", "status"), show="headings", height=5)
        self.tree.heading("type", text="Sign Type"); self.tree.heading("loc", text="Location"); self.tree.heading("status", text="Real-time Status")
        self.tree.column("type", width=120, stretch=tk.YES); self.tree.column("loc", width=80, stretch=tk.YES); self.tree.column("status", width=120, stretch=tk.YES)
        style.theme_use("clam")
        style.configure("Treeview", background="#333", foreground="white", fieldbackground="#333", borderwidth=0, font=("Helvetica", 10))
        style.configure("Treeview.Heading", background="#444", foreground="white", relief="flat", font=("Helvetica", 10, "bold"))
        style.map("Treeview", background=[("selected", THEME["accent"])])
        scrollbar = ttk.Scrollbar(table_frame, orient=tk.VERTICAL, command=self.tree.yview)
        self.tree.configure(yscroll=scrollbar.set)
        self.tree.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=5, pady=5)
        scrollbar.pack(side=tk.RIGHT, fill=tk.Y)
        
        # Kick off background self-updater to prevent manual refresh issues
        self.root.after(500, self.auto_update_table)

    def auto_update_table(self):
        """Automatically checks for new signs in the controller's state & redraws if needed."""
        if hasattr(self.controller, 'path_signs'):
            self.update_sign_table(self.controller.path_signs)
        # Re-schedule to run twice a second
        self.root.after(500, self.auto_update_table)

    def log_event(self, message, level="INFO"):
        timestamp = datetime.now().strftime("%H:%M:%S")
        self.log_text.config(state="normal")
        self.log_text.insert(tk.END, f"[{timestamp}] {message}\n", level)
        self.log_text.see(tk.END)
        self.log_text.config(state="disabled")

    def update_sign_table(self, path_signs):
        """Updates the TreeView efficiently, avoiding UI flickering by comparing current data."""
        # Grab current data state to prevent flickering loop redraws
        current_data = []
        for item in self.tree.get_children():
            current_data.append(tuple(self.tree.item(item, "values")))
        
        # Build requested data state
        new_data = []
        for s in path_signs:
            s_type = s['type']
            emoji = SIGN_MAP.get(s_type, {"emoji": "?"}).get('emoji', '?')
            name = SIGN_MAP.get(s_type, {"name": s_type}).get('name', s_type)
            status_text = s.get('status', "⏳ PENDING")
            new_data.append((f"{emoji} {name}", f"Node {s['node']}", status_text))
            
        # Only rewrite the table if there is an actual update to data or statuses
        if tuple(current_data) == tuple(new_data):
            return 
            
        for item in self.tree.get_children(): 
            self.tree.delete(item)
            
        for data in new_data:
            row_id = self.tree.insert("", "end", values=data)
            if "🔴" in data[2] or "🟢" in data[2]: 
                self.tree.item(row_id, tags=("live",))
        self.tree.tag_configure("live", background="#440000", foreground="#ff5555")

    def update_indicators(self, active_keys):
        """Update dashboard glowing indicators based on a list of active keys."""
        for key, (c, dot) in self.indicators.items():
            if key in active_keys:
                c.itemconfig(dot, fill=THEME["success"])
            else:
                c.itemconfig(dot, fill="#333333")

    def build_nav_tools(self, controller):
        self.var_path = tk.StringVar(value="START")
        tk.Radiobutton(self.tool_frame, text="🟢 Start", variable=self.var_path, value="START", bg=THEME["panel"], fg="white", selectcolor="#444", indicatoron=0, font=("Helvetica", 9, "bold")).pack(side=tk.LEFT, padx=5, pady=5)
        tk.Radiobutton(self.tool_frame, text="🔵 Pass", variable=self.var_path, value="PASS", bg=THEME["panel"], fg="white", selectcolor="#444", indicatoron=0, font=("Helvetica", 9, "bold")).pack(side=tk.LEFT, padx=5, pady=5)
        tk.Radiobutton(self.tool_frame, text="🔴 End", variable=self.var_path, value="END", bg=THEME["panel"], fg="white", selectcolor="#444", indicatoron=0, font=("Helvetica", 9, "bold")).pack(side=tk.LEFT, padx=5, pady=5)
        tk.Button(self.tool_frame, text="Clear Route", bg=THEME["danger"], fg="white", relief="flat", font=("Helvetica", 9, "bold"), command=controller.clear_route).pack(side=tk.RIGHT, padx=10, pady=5)

    def build_sign_tools(self, controller):
        self.var_sign = tk.StringVar(value="stop-sign")
        opt = ttk.Combobox(self.tool_frame, textvariable=self.var_sign, values=list(SIGN_MAP.keys()), state="readonly", width=18)
        opt.pack(side=tk.LEFT, padx=5, pady=5)
        self.chk_del = tk.BooleanVar(value=False)
        tk.Checkbutton(self.tool_frame, text="Delete Mode", variable=self.chk_del, bg=THEME["panel"], fg=THEME["danger"], selectcolor="#333", font=("Helvetica", 9, "bold")).pack(side=tk.LEFT, padx=10, pady=5)
        tk.Button(self.tool_frame, text="Save DB", bg=THEME["success"], fg="white", relief="flat", font=("Helvetica", 9, "bold"), command=controller.map_engine.save_signs).pack(side=tk.RIGHT, padx=10, pady=5)
