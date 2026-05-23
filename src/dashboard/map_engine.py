import networkx as nx
import json
import os
import math
import numpy as np
from PIL import Image, ImageDraw, ImageFont
from config import (GRAPH_FILE, SVG_FILE, SIGNS_DB_FILE, THEME, SIGN_MAP,
                     REAL_WIDTH_M, REAL_HEIGHT_M, FINAL_SCALE_X, FINAL_SCALE_Y,
                     FINAL_OFF_X, FINAL_OFF_Y)

class MapEngine:
    def __init__(self):
        self.G = nx.Graph()
        self.signs = []
        self.svg_w, self.svg_h = 600, 600
        self.pil_bg = Image.new('RGB', (600, 600), color='#1e1e1e')
        self.ppm_x = 1.0
        self.ppm_y = 1.0
        self.node_pixels = {}
        
        self._load_map_and_graph()
        self.load_signs()

    def _load_map_and_graph(self):
        if not os.path.exists(GRAPH_FILE):
            self.G.add_node("1", x=5.0, y=5.0)
            self.G.add_node("2", x=15.0, y=5.0)
            self.G.add_edge("1", "2")
        else:
            self.G = nx.read_graphml(GRAPH_FILE)

        try:
            from svglib.svglib import svg2rlg
            from reportlab.graphics import renderPM
            if os.path.exists(SVG_FILE):
                d = svg2rlg(SVG_FILE)
                s = 600 / d.width
                d.width *= s; d.height *= s; d.scale(s, s)
                self.pil_bg = renderPM.drawToPIL(d, bg=0x111111)
                self.svg_w, self.svg_h = int(d.width), int(d.height)
        except BaseException as e:
            self.svg_w, self.svg_h = 600, 600
            self.pil_bg = Image.new('RGB', (self.svg_w, self.svg_h), color='#1e1e1e')
            draw = ImageDraw.Draw(self.pil_bg)
            draw.text((20, 20), "Fallback Map Background\n(Missing Track.svg)", fill="gray")

        self.ppm_x = self.svg_w / REAL_WIDTH_M
        self.ppm_y = self.svg_h / REAL_HEIGHT_M
        
        # Convert node names to string during cache generation for type safety
        self.node_pixels = {str(n): self.to_pixel(float(d.get('x', 0)), float(d.get('y', 0))) for n, d in self.G.nodes(data=True)}

    def to_pixel(self, x, y): 
        return int((float(x)*self.ppm_x*FINAL_SCALE_X)+FINAL_OFF_X), int(self.svg_h-((float(y)*self.ppm_y*FINAL_SCALE_Y)+FINAL_OFF_Y))
    
    def to_meter(self, x, y): 
        return (x-FINAL_OFF_X)/(self.ppm_x*FINAL_SCALE_X), (self.svg_h-y-FINAL_OFF_Y)/(self.ppm_y*FINAL_SCALE_Y)

    def save_signs(self):
        with open(SIGNS_DB_FILE, 'w') as f: json.dump(self.signs, f, indent=4)
        
    def load_signs(self):
        if os.path.exists(SIGNS_DB_FILE):
            with open(SIGNS_DB_FILE, 'r') as f: self.signs = json.load(f)
            migration_map = {
                "Stop": "stop-sign", "Crosswalk": "crosswalk-sign", "Priority": "priority-sign",
                "Parking": "parking-sign", "Highway Entry": "highway-entry-sign",
                "Highway Exit": "highway-exit-sign", "Pedestrian": "pedestrian",
                "Traffic Light": "traffic-light", "Roundabout": "roundabout-sign",
                "Oneway": "oneway-sign", "No Entry": "noentry-sign"
            }
            for s in self.signs:
                if s['type'] in migration_map: s['type'] = migration_map[s['type']]

    def remove_sign(self, node_id):
        """Removes a sign at the specified node and saves the database."""
        initial_len = len(self.signs)
        self.signs = [s for s in self.signs if str(s['node']) != str(node_id)]
        if len(self.signs) < initial_len:
            self.save_signs()
            return True
        return False

    def calc_path_nodes(self, start_node, end_node, pass_nodes=None):
        try:
            if pass_nodes:
                full_path = nx.shortest_path(self.G, start_node, pass_nodes[0])
                for i in range(len(pass_nodes) - 1):
                    segment = nx.shortest_path(self.G, pass_nodes[i], pass_nodes[i+1])
                    full_path += segment[1:] # Skip the duplicated pass node
                if pass_nodes[-1] != end_node:
                    last_segment = nx.shortest_path(self.G, pass_nodes[-1], end_node)
                    full_path += last_segment[1:] # Skip the duplicated pass node
                return full_path
            else: 
                return nx.shortest_path(self.G, start_node, end_node)
        except: return []

    def get_path_signs(self, path):
        """Returns a list of sign objects that are located exactly on the given path, IN STRICT CHRONOLOGICAL ORDER."""
        path_signs = []
        seen_nodes = set()
        
        # Iterate over the explicit node-to-node chronological list
        for node in path:
            if node in seen_nodes:
                continue # Skip any accidental duplicates in the path array
            seen_nodes.add(node)
            
            node_str = str(node)
            # Find the sign (if one exists) installed at this node
            for s in self.signs:
                if str(s['node']) == node_str:
                    ps = s.copy()
                    ps['status'] = '⏳ PENDING'
                    ps['distance'] = float('inf')
                    path_signs.append(ps)
                    break # Only one sign per physical node
                    
        return path_signs

    def update_sign_statuses(self, path_signs, ai_detections, ai_distance, detect_dist=5.0, act_dist=2.0, light_status="NONE", active_blocks=None):
        """
        Updates the status of signs on the route based entirely on AI vision distance.
        The car goes one by one through the signs in the list.
        Returns (active_command, updated_signs, teleport_node_id).
        """
        active_command = None
        teleport_node = None
        
        for ps in path_signs:
            if ps.get('status') != '✅ COMPLETED':
                base_type = ps['type'].replace('-sign', '').lower()
                is_detected = any(base_type in str(label).lower() for label in ai_detections)
                
                if is_detected:
                    ps['distance'] = ai_distance
                    if ai_distance <= act_dist:
                        ps['status'] = '🔴 ACTING'
                        active_command = ps['type']
                    elif ai_distance <= detect_dist:
                        ps['status'] = '🟢 DETECTING'
                else:
                    # If it was acting and we lose sight, we likely passed it...
                    if ps.get('status') == '🔴 ACTING':
                        # BUT we ONLY mark it completed if the main loop isn't still actively processing it!
                        # (e.g. crosswalk 5s timer still ticking, or pedestrian still physically blocking)
                        is_blocked = False
                        if active_blocks:
                            if "crosswalk" in ps['type'].lower() or "pedestrian" in ps['type'].lower():
                                is_blocked = active_blocks.get("pedestrian", False) or active_blocks.get("crosswalk", False)
                            elif "priority" in ps['type'].lower():
                                is_blocked = active_blocks.get("priority", False)
                        
                        if not is_blocked:
                            ps['status'] = '✅ COMPLETED'
                            teleport_node = ps['node']
                
                # Enforce sequential one-by-one processing by breaking after the first pending sign
                break
                    
        return active_command, path_signs, teleport_node

    def render_map(self, car_x, car_y, car_yaw, path, visited_nodes, path_signs, is_connected, start_node, pass_nodes, end_node):
        pil = self.pil_bg.copy()
        draw = ImageDraw.Draw(pil)
        
        if path:
            for i in range(len(path)-1):
                n1, n2 = path[i], path[i+1]
                color = THEME["danger"] if (n1 in visited_nodes and n2 in visited_nodes) else THEME["accent"]
                p1 = self.node_pixels.get(str(n1))
                p2 = self.node_pixels.get(str(n2))
                if p1 and p2: draw.line([p1, p2], fill=color, width=4)
        
        try: font = ImageFont.truetype("seguiemj.ttf", 20) 
        except: font = ImageFont.load_default()
        
        path_nodes = set([str(n) for n in path])
        for s in self.signs:
            # Using str() lookup to fix JSON saving/loading ID type mismatches
            p = self.node_pixels.get(str(s['node']))
            if not p: continue
            
            s_type = s['type']
            emoji = SIGN_MAP.get(s_type, {"emoji": "?"})['emoji']
            outline = None
            
            if str(s['node']) in path_nodes:
                status = "⏳ PENDING"
                for ps in path_signs:
                    if str(ps['node']) == str(s['node']): 
                        status = ps.get('status', '⏳ PENDING')
                        break
                        
                if "✅" in status: outline = THEME["danger"]       
                elif "🔴" in status or "🟢" in status: outline = "#00ffff"        
                else: outline = THEME["success"]                             
            
            if outline: draw.ellipse([p[0]-14, p[1]-14, p[0]+14, p[1]+14], outline=outline, width=3)
            try: draw.text((p[0]-10, p[1]-10), emoji, font=font, fill="white", embedded_color=True)
            except: draw.text((p[0]-10, p[1]-10), emoji, font=font, fill="white")
            
        def mark(n, c): 
            if n and str(n) in self.node_pixels:
                p = self.node_pixels[str(n)]
                draw.ellipse([p[0]-6, p[1]-6, p[0]+6, p[1]+6], fill=c)
                
        mark(start_node, THEME["success"])
        if pass_nodes:
            for n in pass_nodes:
                mark(n, "cyan")
        mark(end_node, THEME["danger"])
        
        # Calculate visual yaw to make the car always face the path (or fallback to real yaw)
        visual_yaw = car_yaw
        if path and len(path) > 1:
            min_dist = float('inf')
            target_node = path[1]
            
            # Find the segment the car is currently closest to
            for i in range(len(path) - 1):
                n1, n2 = str(path[i]), str(path[i+1])
                if n1 not in self.G.nodes: continue
                
                n1x = float(self.G.nodes[n1].get('x', 0))
                n1y = float(self.G.nodes[n1].get('y', 0))
                dist = math.hypot(car_x - n1x, car_y - n1y)
                
                if dist < min_dist:
                    min_dist = dist
                    target_node = n2
            
            # Point to the next node in that segment
            if str(target_node) in self.G.nodes:
                tx = float(self.G.nodes[str(target_node)].get('x', 0))
                ty = float(self.G.nodes[str(target_node)].get('y', 0))
                # Prevent erratic spinning if the car is directly exactly on top of the node
                if math.hypot(tx - car_x, ty - car_y) > 0.1:
                    visual_yaw = math.atan2(ty - car_y, tx - car_x)

        # Draw the car ALWAYS (removed `if is_connected:` requirement)
        # Use an orange color if disconnected to alert the user about hardware failure/battery
        car_color = "cyan" if is_connected else "orange"
        
        cx, cy = self.to_pixel(car_x, car_y)
        hx = cx + math.cos(-visual_yaw) * 20
        hy = cy + math.sin(-visual_yaw) * 20
        
        draw.ellipse([cx-8, cy-8, cx+8, cy+8], fill=car_color, outline="white", width=2)
        draw.line([cx, cy, hx, hy], fill="white", width=2)
            
        return pil
