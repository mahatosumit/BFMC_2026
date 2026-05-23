import networkx as nx
import json
import os
import math
import numpy as np
from PIL import Image, ImageDraw, ImageFont
from config import (GRAPH_FILE, SVG_FILE, SIGNS_DB_FILE, THEME, SIGN_MAP,
                     REAL_WIDTH_M, REAL_HEIGHT_M, FINAL_SCALE_X, FINAL_SCALE_Y,
                     FINAL_OFF_X, FINAL_OFF_Y,
                     MAP_NODE_SHOW, MAP_NODE_OPACITY, MAP_NODE_MIN_SPACING_PX)

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

    def render_map(
        self,
        car_x,
        car_y,
        car_yaw,
        path,
        visited_nodes,
        path_signs,
        is_connected,
        start_node,
        pass_nodes,
        end_node,
        loc_viz=None,
    ):
        pil = self.pil_bg.copy()
        draw = ImageDraw.Draw(pil)

        # ── graph node overlay ────────────────────────────────
        if MAP_NODE_SHOW and MAP_NODE_OPACITY > 0.0:
            alpha      = max(0, min(255, int(MAP_NODE_OPACITY * 255)))
            spacing2   = MAP_NODE_MIN_SPACING_PX ** 2
            node_layer = Image.new("RGBA", pil.size, (0, 0, 0, 0))
            ndraw      = ImageDraw.Draw(node_layer)
            _last_cell: dict = {}

            for nid, (px, py) in self.node_pixels.items():
                try:
                    is_orig = int(nid) <= 600   # IDs > 600 are Catmull-Rom interpolated
                except ValueError:
                    is_orig = True

                if is_orig:
                    # Junction / key waypoint — always draw, slightly larger
                    ndraw.ellipse([px-3, py-3, px+3, py+3],
                                  fill=(96, 112, 128, alpha),
                                  outline=(144, 168, 184, alpha))
                else:
                    # Interpolated node — skip if too close to last drawn one
                    cell = (px // MAP_NODE_MIN_SPACING_PX, py // MAP_NODE_MIN_SPACING_PX)
                    last = _last_cell.get(cell)
                    if last and (px - last[0]) ** 2 + (py - last[1]) ** 2 < spacing2:
                        continue
                    _last_cell[cell] = (px, py)
                    ndraw.ellipse([px-1, py-1, px+1, py+1],
                                  fill=(58, 74, 84, alpha))

            pil  = Image.alpha_composite(pil.convert("RGBA"), node_layer).convert("RGB")
            draw = ImageDraw.Draw(pil)   # refresh draw handle after composite
        # ─────────────────────────────────────────────────────

        if path:
            for i in range(len(path) - 1):
                n1, n2 = path[i], path[i + 1]
                color = THEME["danger"] if (n1 in visited_nodes and n2 in visited_nodes) else THEME["accent"]
                p1 = self.node_pixels.get(str(n1))
                p2 = self.node_pixels.get(str(n2))
                if p1 and p2:
                    lw = 4
                    if loc_viz and loc_viz.get("seg_idx", -1) == i:
                        draw.line([p1, p2], fill="#ffeb3b", width=6)
                    else:
                        draw.line([p1, p2], fill=color, width=lw)
        
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
        
        # Car arrow uses raw IMU yaw — no path-facing override
        car_color = "cyan" if is_connected else "orange"
        cx, cy = self.to_pixel(car_x, car_y)
        hx = cx + math.cos(-car_yaw) * 20
        hy = cy + math.sin(-car_yaw) * 20

        if loc_viz:
            try:
                pxp, pyp = self.to_pixel(float(loc_viz["x_pred"]), float(loc_viz["y_pred"]))
                draw.ellipse([pxp - 5, pyp - 5, pxp + 5, pyp + 5], outline="#ff9800", width=2)
            except (KeyError, TypeError, ValueError):
                pass
            cov = loc_viz.get("P_xy")
            if cov and len(cov) == 2:
                a, b_ = float(cov[0][0]), float(cov[0][1])
                c = float(cov[1][1])
                tr = a + c
                det = max(a * c - b_ * b_, 0.0)
                disc = max(tr * tr * 0.25 - det, 0.0)
                l1 = max(tr * 0.5 + math.sqrt(disc), 1e-9)
                l2 = max(tr * 0.5 - math.sqrt(disc), 1e-9)
                rx = max(4.0, math.sqrt(l1) * self.ppm_x * 2.45)
                ry = max(4.0, math.sqrt(l2) * self.ppm_y * 2.45)
                draw.ellipse([cx - rx, cy - ry, cx + rx, cy + ry], outline="#00bcd4", width=1)
            if loc_viz.get("lost"):
                draw.rectangle([cx - 14, cy - 14, cx + 14, cy + 14], outline="#f44336", width=2)

        draw.ellipse([cx - 8, cy - 8, cx + 8, cy + 8], fill=car_color, outline="white", width=2)
        draw.line([cx, cy, hx, hy], fill="white", width=2)

        return pil
