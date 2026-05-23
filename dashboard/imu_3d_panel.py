"""
IMU 3D orientation panel — pure Tkinter Canvas, zero external deps beyond numpy.
Renders a single colored cube with painter's-algorithm face sorting and
EMA-smoothed angles for fluid motion.
"""
import tkinter as tk
import math
import numpy as np

try:
    from config import THEME
except ImportError:
    THEME = {"bg": "#1e1e1e", "panel": "#252526", "accent": "#007acc", "success": "#4caf50"}


# ── Rotation ───────────────────────────────────────────────────────────────────
def _rot_zyx(yaw_deg, pitch_deg, roll_deg):
    y = math.radians(yaw_deg)
    p = math.radians(pitch_deg)
    r = math.radians(roll_deg)
    cy, sy = math.cos(y), math.sin(y)
    cp, sp = math.cos(p), math.sin(p)
    cr, sr = math.cos(r), math.sin(r)
    Rz = np.array([[ cy, -sy, 0], [ sy,  cy, 0], [0, 0, 1]])
    Ry = np.array([[ cp,   0, sp], [  0,  1, 0], [-sp, 0, cp]])
    Rx = np.array([[  1,   0,  0], [  0, cr, -sr], [0, sr, cr]])
    return Rz @ Ry @ Rx


# ── Cube geometry ──────────────────────────────────────────────────────────────
#  Vertex layout (right-hand, Z-up):
#    4---5        7---6
#    |   |   →   |   |   top ring
#    0---1        3---2   bottom ring
_S = 1.15   # half-size
_VERTS = np.array([
    [-_S, -_S, -_S], [ _S, -_S, -_S], [ _S,  _S, -_S], [-_S,  _S, -_S],
    [-_S, -_S,  _S], [ _S, -_S,  _S], [ _S,  _S,  _S], [-_S,  _S,  _S],
], dtype=float)

# Each face: (vertex-indices, fill-colour, dim-factor for back-face shade)
_FACES = [
    ([4, 5, 6, 7], "#42a5f5", 1.00),   # +Z  top     — blue
    ([0, 3, 2, 1], "#1a237e", 0.55),   # -Z  bottom  — dark blue
    ([1, 5, 4, 0], "#ef5350", 0.85),   # -Y  front   — red
    ([2, 6, 7, 3], "#7b1fa2", 0.60),   # +Y  back    — purple
    ([1, 2, 6, 5], "#66bb6a", 0.80),   # +X  right   — green
    ([0, 4, 7, 3], "#f57c00", 0.65),   # -X  left    — orange
]

# Fixed camera tilt: 28° down, 38° right — gives a clean 3/4 view
_CAM_R = (
    np.array([[1, 0, 0],
              [0, math.cos(math.radians(-28)), -math.sin(math.radians(-28))],
              [0, math.sin(math.radians(-28)),  math.cos(math.radians(-28))]]) @
    np.array([[ math.cos(math.radians(38)), 0, math.sin(math.radians(38))],
              [0, 1, 0],
              [-math.sin(math.radians(38)), 0, math.cos(math.radians(38))]])
)

# Light direction (camera-space) for face shading
_LIGHT = np.array([0.4, -0.6, 0.7])
_LIGHT = _LIGHT / np.linalg.norm(_LIGHT)


def _hex_shade(hex_color: str, factor: float) -> str:
    """Darken a hex colour by factor (0‒1)."""
    h = hex_color.lstrip("#")
    r, g, b = int(h[0:2], 16), int(h[2:4], 16), int(h[4:6], 16)
    return "#{:02x}{:02x}{:02x}".format(
        int(r * factor), int(g * factor), int(b * factor)
    )


def _project(pts, cx, cy, fov=190.0, cam_z=4.5):
    """Perspective-project Nx3 camera-space points → list of (x, y) tuples."""
    z = pts[:, 2] + cam_z
    z = np.where(z < 0.01, 0.01, z)
    x2d = cx + fov * pts[:, 0] / z
    y2d = cy - fov * pts[:, 1] / z
    return list(zip(x2d.tolist(), y2d.tolist()))


# ── Panel ─────────────────────────────────────────────────────────────────────
class IMU3DPanel:
    """
    Pop-up Toplevel with a smooth, real-time 3D orientation cube.
    Instantiate from the main Tkinter thread only.
    """
    _W, _H   = 420, 360   # initial canvas size
    _ALPHA   = 0.18        # EMA smoothing (lower = smoother, more lag)
    _FPS     = 60          # target update rate (ms interval = 1000/FPS)

    def __init__(self, parent: tk.Misc, imu_sensor):
        self._imu   = imu_sensor
        self._alive = True

        # Calibration offsets
        self._cal_yaw = self._cal_pitch = self._cal_roll = 0.0

        # EMA state (smoothed display values)
        self._sy = self._sp = self._sr = 0.0

        # ── Window ────────────────────────────────────────────
        self.win = tk.Toplevel(parent)
        self.win.title("IMU 3D Orientation")
        self.win.geometry("440x530")
        self.win.minsize(380, 460)
        self.win.configure(bg=THEME["bg"])
        self.win.protocol("WM_DELETE_WINDOW", self._on_close)

        # ── Canvas ────────────────────────────────────────────
        self._canvas = tk.Canvas(self.win, bg="#0d1117", highlightthickness=0,
                                 width=self._W, height=self._H)
        self._canvas.pack(fill=tk.BOTH, expand=True, padx=6, pady=(6, 4))
        self._canvas.bind("<Configure>", lambda e: self._set_size(e.width, e.height))
        self._cx, self._cy = self._W / 2, self._H / 2

        # ── Numeric readout ────────────────────────────────────
        ro = tk.Frame(self.win, bg=THEME["panel"], pady=5)
        ro.pack(fill=tk.X, padx=6)
        _f = ("Courier", 12, "bold")
        self._lbl_y = tk.Label(ro, text="YAW:   +0.0°", bg=THEME["panel"],
                               fg="#00e5ff", font=_f, anchor="w", width=14)
        self._lbl_p = tk.Label(ro, text="PITCH: +0.0°", bg=THEME["panel"],
                               fg="#76ff03", font=_f, anchor="w", width=14)
        self._lbl_r = tk.Label(ro, text="ROLL:  +0.0°", bg=THEME["panel"],
                               fg="#ff9100", font=_f, anchor="w", width=14)
        for lbl in (self._lbl_y, self._lbl_p, self._lbl_r):
            lbl.pack(side=tk.LEFT, expand=True)

        # ── Calibrate button ───────────────────────────────────
        btn_row = tk.Frame(self.win, bg=THEME["bg"])
        btn_row.pack(fill=tk.X, padx=6, pady=(4, 8))
        self._btn = tk.Button(
            btn_row,
            text="CALIBRATE  —  Set current pose as (0°, 0°, 0°)",
            bg=THEME["accent"], fg="white", relief="flat",
            font=("Helvetica", 10, "bold"), activebackground="#005fa3",
            command=self._calibrate,
        )
        self._btn.pack(fill=tk.X, ipady=5)

        self._schedule()

    # ── Public ────────────────────────────────────────────────
    def lift(self):
        self.win.lift()
        self.win.focus_force()

    # ── Private ───────────────────────────────────────────────
    def _set_size(self, w, h):
        self._cx, self._cy = w / 2, h / 2

    def _calibrate(self):
        self._cal_yaw   = self._imu.get_yaw()
        self._cal_pitch = self._imu.get_pitch()
        self._cal_roll  = self._imu.get_roll()
        # Snap smoothed values to zero immediately
        self._sy = self._sp = self._sr = 0.0
        self._btn.config(text="CALIBRATED  ✓  —  origin reset", bg=THEME["success"])
        self.win.after(1800, lambda: self._btn.config(
            text="CALIBRATE  —  Set current pose as (0°, 0°, 0°)", bg=THEME["accent"]
        ))

    def _on_close(self):
        self._alive = False
        self.win.destroy()

    def _schedule(self):
        if not self._alive:
            return
        try:
            self._render()
        except Exception:
            pass
        self.win.after(1000 // self._FPS, self._schedule)

    def _render(self):
        # Raw angles relative to calibration
        raw_y = self._imu.get_yaw()   - self._cal_yaw
        raw_p = self._imu.get_pitch() - self._cal_pitch
        raw_r = self._imu.get_roll()  - self._cal_roll

        # EMA smoothing
        a = self._ALPHA
        self._sy += a * (raw_y - self._sy)
        self._sp += a * (raw_p - self._sp)
        self._sr += a * (raw_r - self._sr)

        yaw, pitch, roll = self._sy, self._sp, self._sr

        # Update labels
        self._lbl_y.config(text=f"YAW:  {yaw:+7.1f}°")
        self._lbl_p.config(text=f"PITCH:{pitch:+7.1f}°")
        self._lbl_r.config(text=f"ROLL: {roll:+7.1f}°")

        cv = self._canvas
        cx, cy = self._cx, self._cy
        cv.delete("all")

        # Combined rotation: camera tilt then IMU rotation
        R = _CAM_R @ _rot_zyx(yaw, pitch, roll)

        # Transform all 8 cube vertices
        verts_cam = (_VERTS @ R.T)   # shape (8, 3)
        pts2d = _project(verts_cam, cx, cy)

        # Build face draw list with depth and shading
        draw_list = []
        for indices, base_color, dim in _FACES:
            face_verts = verts_cam[indices]             # (4, 3)
            depth = float(face_verts[:, 2].mean())      # avg camera-Z

            # Face normal from cross product of two edges
            v0, v1, v2 = face_verts[0], face_verts[1], face_verts[2]
            normal = np.cross(v1 - v0, v2 - v0)
            n_len  = np.linalg.norm(normal)
            if n_len > 1e-6:
                normal /= n_len
            # Simple diffuse shading
            diffuse = max(0.0, float(np.dot(normal, _LIGHT)))
            shade   = 0.35 + 0.65 * diffuse
            color   = _hex_shade(base_color, shade * dim)

            poly = [pts2d[i] for i in indices]
            draw_list.append((depth, poly, color))

        # Painter's algorithm: draw back faces first
        draw_list.sort(key=lambda x: x[0])
        for _, poly, color in draw_list:
            flat = [coord for pt in poly for coord in pt]
            cv.create_polygon(flat, fill=color, outline="#1a1a2e", width=2)

        # ── HUD ───────────────────────────────────────────────
        hw = (hasattr(self._imu, "get_has_hardware") and
              self._imu.get_has_hardware())
        status = "IMU: HARDWARE" if hw else "IMU: SIMULATED"
        s_col  = "#4caf50" if hw else "#ff9800"
        cv.create_text(8, 8, anchor="nw",
                       text=f"Y {yaw:+6.1f}°  P {pitch:+6.1f}°  R {roll:+6.1f}°",
                       fill="#cccccc", font=("Courier", 9))
        cv.create_text(cx * 2 - 8, 8, anchor="ne",
                       text=status, fill=s_col, font=("Courier", 9, "bold"))
