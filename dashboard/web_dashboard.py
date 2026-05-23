"""
dashboard/web_dashboard.py
==========================
Full-featured Flask web dashboard — light theme, 3-column layout matching the
Tkinter UI, tunable CSS colours, BEV + map streams, CSS 3D IMU cube.

Routes
------
  GET  /              Full dashboard HTML
  GET  /stream        MJPEG camera stream
  GET  /stream/bev    MJPEG Bird's-Eye-View stream
  GET  /api/map       Latest map frame (JPEG)
  GET  /api/state     JSON telemetry snapshot
  GET  /api/log       JSON array of recent log lines
  POST /api/command   {"action": str, "value": optional}
"""

import io
import threading
import time
from collections import deque
from typing import Any, Dict, List

import cv2
import numpy as np

from config import WEB_DASHBOARD_HOST, WEB_DASHBOARD_PORT, WEB_DASHBOARD_FPS

try:
    from flask import Flask, Response, jsonify, request
    _FLASK_AVAILABLE = True
except ImportError:
    _FLASK_AVAILABLE = False


# ── Shared state ──────────────────────────────────────────────────────────────
_state: Dict[str, Any] = {
    "mode": "MANUAL", "speed_pwm": 0, "steer_deg": 0.0,
    "yaw_deg": 0.0, "roll_deg": 0.0, "pitch_deg": 0.0,
    "car_x": 0.0, "car_y": 0.0,
    "lane_anchor": "—", "target_x": 320.0, "lateral_err_px": 0.0,
    "lane_confidence": 0.0, "active_sign": "—", "yolo_labels": [],
    "battery_pct": 0, "loop_hz": 0.0, "is_recording": False,
    "base_speed": 150.0, "steer_mult": 1.0,
    "sign_detect_m": 5.0, "sign_act_m": 2.0,
    "is_connected": False, "imu_connected": False,
    "ai_enabled": True, "loc_node": "—",
    "active_indicators": [], "route_signs": [],
    "start_node": "", "end_node": "", "pass_nodes": [],
}
_state_lock = threading.Lock()

_cam_frame: bytes = b""
_cam_lock  = threading.Lock()
_cam_event = threading.Event()

_bev_frame: bytes = b""
_bev_lock  = threading.Lock()
_bev_event = threading.Event()

_map_frame: bytes = b""
_map_lock  = threading.Lock()

_log_lines: deque = deque(maxlen=120)
_log_lock  = threading.Lock()

_pending_commands: List[Dict] = []
_cmd_lock = threading.Lock()


# ── Public API ────────────────────────────────────────────────────────────────
def push_telemetry(**kwargs) -> None:
    with _state_lock:
        _state.update(kwargs)

def push_frame(bgr_frame: np.ndarray) -> None:
    global _cam_frame
    _, buf = cv2.imencode(".jpg", bgr_frame, [cv2.IMWRITE_JPEG_QUALITY, 72])
    with _cam_lock:
        _cam_frame = buf.tobytes()
    _cam_event.set()

def push_bev_frame(bgr_frame: np.ndarray) -> None:
    global _bev_frame
    _, buf = cv2.imencode(".jpg", bgr_frame, [cv2.IMWRITE_JPEG_QUALITY, 72])
    with _bev_lock:
        _bev_frame = buf.tobytes()
    _bev_event.set()

def push_map_image(pil_img) -> None:
    global _map_frame
    buf = io.BytesIO()
    pil_img.save(buf, format="JPEG", quality=82)
    with _map_lock:
        _map_frame = buf.getvalue()

def push_log(message: str, level: str = "INFO") -> None:
    ts = time.strftime("%H:%M:%S")
    with _log_lock:
        _log_lines.append({"ts": ts, "level": level, "msg": message})

def pop_commands() -> List[Dict]:
    with _cmd_lock:
        cmds = list(_pending_commands)
        _pending_commands.clear()
    return cmds


# ── HTML ──────────────────────────────────────────────────────────────────────
_HTML = """<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1.0">
<title>BFMC 2026 — Dashboard</title>
<style>
/* ── CSS custom properties (all colours tunable via Settings) ── */
:root {
  --bg:     #f0f2f5;  --panel:  #ffffff;  --fg:     #212121;
  --accent: #1565c0;  --danger: #c62828;  --success:#2e7d32;
  --warn:   #e65100;  --border: #dde1e7;  --card:   #f8f9fa;
  --header: #1a237e;  --text2:  #616161;
}
*{box-sizing:border-box;margin:0;padding:0}
body{background:var(--bg);color:var(--fg);font-family:-apple-system,'Segoe UI',Arial,sans-serif;
     font-size:13px;height:100vh;display:flex;flex-direction:column;overflow:hidden}

/* ── Status bar ── */
#sb{background:var(--header);color:#fff;padding:0 14px;height:44px;
    display:flex;align-items:center;gap:12px;flex-shrink:0;
    box-shadow:0 2px 6px rgba(0,0,0,.35);font-size:12px;white-space:nowrap}
#sb-title{font-weight:800;font-size:15px;letter-spacing:2px;margin-right:4px}
#sb-right{margin-left:auto;display:flex;align-items:center;gap:10px}
.sbsep{color:rgba(255,255,255,.3)}
#sb-mode{padding:2px 9px;border-radius:10px;font-weight:700;font-size:11px;
         background:rgba(255,255,255,.15)}
.sb-btn{background:rgba(255,255,255,.15);border:1px solid rgba(255,255,255,.3);
        color:#fff;padding:3px 9px;border-radius:4px;cursor:pointer;
        font-size:11px;font-weight:700}
.sb-btn:hover{background:rgba(255,255,255,.28)}
#sb-hz{color:#80cbc4;font-weight:700}

/* ── 3-column grid ── */
#main{display:grid;grid-template-columns:460px 310px 1fr;
      gap:8px;padding:8px;flex:1;overflow:hidden;min-height:0}
.col{display:flex;flex-direction:column;gap:8px;overflow-y:auto;min-height:0}

/* ── Panel card ── */
.panel{background:var(--panel);border:1px solid var(--border);border-radius:8px;
       padding:10px 12px;box-shadow:0 1px 4px rgba(0,0,0,.08);flex-shrink:0}
.pt{font-size:10px;font-weight:700;color:var(--text2);text-transform:uppercase;
    letter-spacing:1.2px;margin-bottom:8px;display:flex;align-items:center;gap:6px}

/* ── Camera ── */
.cam-wrap{background:#111;border-radius:6px;overflow:hidden;position:relative}
.cam-wrap img{width:100%;display:block;height:240px;object-fit:contain;background:#111}
.cam-tag{position:absolute;top:5px;left:5px;background:rgba(0,0,0,.55);
         color:#fff;font-size:10px;padding:2px 6px;border-radius:3px;font-weight:700}

/* ── Telemetry cards ── */
.tg{display:grid;grid-template-columns:repeat(4,1fr);gap:5px;margin-bottom:8px}
.tc{background:var(--card);border:1px solid var(--border);border-radius:6px;
    padding:8px 4px;text-align:center}
.tv{font-size:17px;font-weight:800;color:var(--accent)}
.tl{font-size:9px;color:var(--text2);margin-top:2px;font-weight:600}

/* Lane bar */
.lbw{background:var(--card);border:1px solid var(--border);height:14px;
     border-radius:7px;position:relative;overflow:hidden;margin:4px 0}
.lbc{position:absolute;left:50%;top:0;width:1px;height:100%;background:var(--border)}
.lbp{position:absolute;top:2px;width:10px;height:10px;border-radius:50%;
     background:var(--accent);transform:translateX(-50%);
     transition:left .12s ease;box-shadow:0 0 4px var(--accent)}

/* ── Buttons ── */
.btn{padding:7px 11px;border:none;border-radius:5px;cursor:pointer;font-weight:700;
     font-size:12px;font-family:inherit;transition:opacity .12s,transform .08s;
     display:inline-flex;align-items:center;gap:4px}
.btn:hover{opacity:.87} .btn:active{transform:scale(.97)}
.btn-full{width:100%;justify-content:center}
.btn-d{background:var(--danger);color:#fff}  .btn-a{background:var(--accent);color:#fff}
.btn-s{background:var(--success);color:#fff} .btn-w{background:var(--warn);color:#fff}
.btn-g{background:#9e9e9e;color:#fff}
.btn-o{background:transparent;border:1.5px solid var(--accent);color:var(--accent)}
.btn-row{display:flex;gap:5px;flex-wrap:wrap;margin-bottom:5px}
.estop{background:var(--danger);color:#fff;width:100%;padding:13px;font-size:14px;
       font-weight:800;border:none;border-radius:6px;cursor:pointer;letter-spacing:1px;
       animation:epulse 2s infinite}
@keyframes epulse{0%,100%{box-shadow:0 0 0 0 rgba(198,40,40,.4)}
                  50%{box-shadow:0 0 0 9px rgba(198,40,40,0)}}

/* ── Sliders ── */
.sr{display:flex;align-items:center;gap:8px;margin:5px 0}
.sr label{width:128px;font-size:11px;color:var(--text2);font-weight:600}
.sr input[type=range]{flex:1;accent-color:var(--accent);cursor:pointer}
.sv{width:36px;text-align:right;font-weight:700;font-size:12px;color:var(--accent);
    font-variant-numeric:tabular-nums}

/* ── ADAS indicators ── */
.ig{display:grid;grid-template-columns:1fr 1fr;gap:3px}
.ind{display:flex;align-items:center;gap:6px;padding:5px 7px;border-radius:5px;
     background:var(--card);border:1px solid var(--border);font-size:11px;font-weight:600}
.dot{width:9px;height:9px;border-radius:50%;background:#bdbdbd;flex-shrink:0;
     transition:background .2s,box-shadow .2s}
.dot.on{background:var(--success);box-shadow:0 0 7px var(--success)}

/* ── Log ── */
#log-box{background:#0d1117;border-radius:5px;height:160px;overflow-y:auto;
         padding:7px;font-family:'Courier New',monospace;font-size:11px;
         line-height:1.6;border:1px solid var(--border)}
.log-INFO{color:#9e9e9e} .log-SUCCESS{color:#66bb6a}
.log-WARN{color:#ffa726} .log-CRITICAL,.log-DANGER{color:#ef5350}

/* ── Mode tabs (right panel) ── */
.mtabs{display:flex;gap:5px;margin-bottom:6px}
.mtab{flex:1;padding:7px;border-radius:5px;border:1.5px solid var(--border);
      background:var(--card);cursor:pointer;font-weight:700;font-size:11px;
      text-align:center;transition:background .15s;color:var(--fg)}
.mtab.active{background:var(--accent);border-color:var(--accent);color:#fff}
.toolbar{min-height:34px;display:flex;align-items:center;gap:6px;
         flex-wrap:wrap;margin-bottom:6px;font-size:12px}

/* ── Map ── */
#map-img{width:100%;border-radius:6px;background:var(--card);
         border:1px solid var(--border);display:block;min-height:240px;object-fit:contain}
#map-ph{height:240px;display:none;align-items:center;justify-content:center;
        color:var(--text2);background:var(--card);border-radius:6px;
        border:1px solid var(--border)}

/* ── Route table ── */
#rt{width:100%;border-collapse:collapse;font-size:11px}
#rt th{background:var(--card);padding:5px 8px;text-align:left;font-size:10px;
       font-weight:700;color:var(--text2);text-transform:uppercase;
       border-bottom:2px solid var(--border)}
#rt td{padding:5px 8px;border-bottom:1px solid var(--border)}
#rt tr.live td{background:#fff3e0;color:var(--warn)}
#rt tr:hover td{background:var(--card)}

/* ── IMU 3D Popup ── */
#imu-pop{position:fixed;top:54px;right:14px;width:300px;background:#fff;
         border:1.5px solid var(--border);border-radius:10px;
         box-shadow:0 8px 32px rgba(0,0,0,.18);z-index:1000;
         display:none;flex-direction:column;padding:12px}
#imu-pop.open{display:flex}
.imu-ph{display:flex;align-items:center;justify-content:space-between;margin-bottom:10px}
.imu-title{font-weight:700;font-size:13px;color:var(--header)}
.pop-close{background:none;border:none;cursor:pointer;font-size:18px;
           color:var(--text2);line-height:1}

/* CSS 3D cube */
#imu-scene{width:150px;height:150px;perspective:520px;margin:0 auto 12px}
#imu-cube{width:100%;height:100%;transform-style:preserve-3d;
          transform:rotateX(-20deg) rotateY(30deg)}
.face{position:absolute;width:150px;height:150px;border:2px solid rgba(255,255,255,.3);
      display:flex;align-items:center;justify-content:center;
      font-weight:900;font-size:14px;color:#fff;
      text-shadow:0 1px 3px rgba(0,0,0,.4)}
.f-top  {background:rgba(66,165,245,.9); transform:rotateX( 90deg) translateZ(75px)}
.f-bot  {background:rgba(21,101,192,.9); transform:rotateX(-90deg) translateZ(75px)}
.f-fwd  {background:rgba(239,83,80,.9);  transform:translateZ(75px)}
.f-bck  {background:rgba(123,31,162,.9); transform:rotateY(180deg) translateZ(75px)}
.f-rgt  {background:rgba(102,187,106,.9);transform:rotateY( 90deg) translateZ(75px)}
.f-lft  {background:rgba(245,124,0,.9);  transform:rotateY(-90deg) translateZ(75px)}

.imu-rds{display:grid;grid-template-columns:1fr 1fr 1fr;gap:5px;margin-bottom:8px}
.imu-rc{background:var(--card);border:1px solid var(--border);
        border-radius:5px;padding:6px 4px;text-align:center}
.imu-rv{font-size:15px;font-weight:800}
.imu-rl{font-size:9px;color:var(--text2);font-weight:700;margin-top:2px}
#imu-hw{text-align:center;font-size:11px;font-weight:700;margin-bottom:8px}

/* ── Settings Popup ── */
#cfg-pop{position:fixed;top:54px;right:14px;width:286px;background:#fff;
         border:1.5px solid var(--border);border-radius:10px;
         box-shadow:0 8px 32px rgba(0,0,0,.18);z-index:999;
         display:none;flex-direction:column;padding:14px}
#cfg-pop.open{display:flex}
.cr{display:flex;align-items:center;justify-content:space-between;margin:5px 0}
.cr label{font-size:12px;font-weight:600}
.cr input[type=color]{width:40px;height:27px;border:none;border-radius:4px;cursor:pointer}
hr.sep{border:none;border-top:1px solid var(--border);margin:8px 0}
</style>
</head>
<body>

<!-- ═══════════════════════════ STATUS BAR ═════════════════════════════════ -->
<header id="sb">
  <span id="sb-title">⚙ BFMC 2026</span>
  <span class="sbsep">|</span>
  <span id="sb-conn" style="color:#ef9a9a">⚫ DISCONNECTED</span>
  <span class="sbsep">|</span>
  <span id="sb-mode">MANUAL</span>
  <span class="sbsep">|</span>
  <span id="sb-tele">SPD: 0 | STR: 0.0°</span>
  <div id="sb-right">
    <span id="sb-rec" style="color:rgba(255,255,255,.4)">⬜ NOT REC</span>
    <span class="sbsep">|</span>
    <span id="sb-ai">AI: OFF</span>
    <span class="sbsep">|</span>
    <span id="sb-bat">BAT: --%</span>
    <span class="sbsep">|</span>
    <span id="sb-imu" style="color:#b2dfdb">IMU: -- | R:0° P:0° Y:0°</span>
    <span class="sbsep">|</span>
    <span id="sb-hz">0.0 Hz</span>
    <span class="sbsep">|</span>
    <button class="sb-btn" onclick="toggleImu()">⊕ IMU 3D</button>
    <button class="sb-btn" onclick="toggleCfg()" title="Theme">⚙</button>
  </div>
</header>

<!-- ═════════════════════════ MAIN 3-COLUMN GRID ══════════════════════════ -->
<div id="main">

  <!-- ── LEFT: Cameras + Telemetry ──────────────────────────────────────── -->
  <div class="col">
    <div class="panel">
      <div class="pt">📷 Raw Camera — YOLO ADAS</div>
      <div class="cam-wrap">
        <img id="cam" src="/stream" onerror="retryStream(this,'/stream')">
        <div class="cam-tag">LIVE</div>
      </div>
    </div>

    <div class="panel">
      <div class="pt">🦅 Bird's Eye View — VIZ-06</div>
      <div class="cam-wrap">
        <img id="bev" src="/stream/bev" onerror="retryStream(this,'/stream/bev')">
        <div class="cam-tag">BEV</div>
      </div>
    </div>

    <div class="panel">
      <div class="pt">📊 Live Telemetry</div>
      <div class="tg">
        <div class="tc"><div class="tv" id="t-spd">0</div><div class="tl">Speed PWM</div></div>
        <div class="tc"><div class="tv" id="t-str">0°</div><div class="tl">Steering</div></div>
        <div class="tc"><div class="tv" id="t-yaw">0°</div><div class="tl">Yaw (IMU)</div></div>
        <div class="tc"><div class="tv" id="t-cnf">0.00</div><div class="tl">Lane Conf</div></div>
        <div class="tc"><div class="tv" id="t-tx">320</div><div class="tl">Target X</div></div>
        <div class="tc"><div class="tv" id="t-le">0</div><div class="tl">Lat Err px</div></div>
        <div class="tc"><div class="tv" id="t-pit">0°</div><div class="tl">Pitch</div></div>
        <div class="tc"><div class="tv" id="t-rol">0°</div><div class="tl">Roll</div></div>
      </div>
      <div style="font-size:10px;color:var(--text2);font-weight:600;margin-bottom:3px">Lateral Position</div>
      <div class="lbw"><div class="lbc"></div><div class="lbp" id="lbp" style="left:50%"></div></div>
      <div style="margin-top:6px;font-size:11px;color:var(--text2)">
        Lane: <b id="t-anc">—</b> &nbsp;|&nbsp;
        Sign: <span id="t-sgn" style="color:var(--warn);font-weight:700">—</span> &nbsp;|&nbsp;
        YOLO: <span id="t-yol" style="color:var(--accent)">—</span>
      </div>
    </div>
  </div>

  <!-- ── MIDDLE: Controls ────────────────────────────────────────────────── -->
  <div class="col">

    <div class="panel" style="padding:8px">
      <button class="estop" onclick="cmd('e_stop')">🛑 EMERGENCY STOP</button>
    </div>

    <div class="panel">
      <div class="pt">🔧 System Controls</div>
      <div class="btn-row">
        <button class="btn btn-a btn-full" id="btn-conn" onclick="cmd('toggle_connection')">⚡ Connect Car</button>
      </div>
      <div class="btn-row">
        <button class="btn btn-o btn-full" onclick="cmd('toggle_auto')">🤖 Toggle Auto Mode</button>
      </div>
      <div class="btn-row">
        <button class="btn btn-g btn-full" id="btn-adas" onclick="cmd('toggle_adas')">ADAS ASSIST: ON</button>
      </div>
      <div class="btn-row">
        <button class="btn btn-a" onclick="cmd('save_config')" style="flex:1">💾 Save Config</button>
        <button class="btn btn-g" onclick="cmd('load_config')" style="flex:1">📂 Load Config</button>
      </div>
      <div class="btn-row">
        <button class="btn btn-s" id="btn-rec" onclick="toggleRec()" style="flex:1">⏺ Start Recording</button>
      </div>
    </div>

    <div class="panel">
      <div class="pt">🎛 Drive Dynamics</div>
      <div class="sr"><label>Base Speed (PWM)</label>
        <input type="range" id="sl-spd" min="0" max="500" step="5" value="150"
          oninput="slUpd('sl-spd','sv-spd',v=>Math.round(v),'set_base_speed')">
        <span class="sv" id="sv-spd">150</span></div>
      <div class="sr"><label>Map Sim Mult</label>
        <input type="range" id="sl-sim" min="0.1" max="3.0" step="0.1" value="1.0"
          oninput="slUpd('sl-sim','sv-sim',v=>parseFloat(v).toFixed(1))">
        <span class="sv" id="sv-sim">1.0</span></div>
      <div class="sr"><label>Steer Multiplier</label>
        <input type="range" id="sl-str" min="0.1" max="3.0" step="0.1" value="1.0"
          oninput="slUpd('sl-str','sv-str',v=>parseFloat(v).toFixed(1),'set_steer_mult')">
        <span class="sv" id="sv-str">1.0</span></div>
      <div class="sr"><label>Overtake Dist (m)</label>
        <input type="range" id="sl-od" min="0.5" max="5.0" step="0.1" value="1.2"
          oninput="slUpd('sl-od','sv-od',v=>parseFloat(v).toFixed(1))">
        <span class="sv" id="sv-od">1.2</span></div>
      <div class="sr"><label>Overtake Time (s)</label>
        <input type="range" id="sl-ot" min="1.0" max="5.0" step="0.2" value="2.0"
          oninput="slUpd('sl-ot','sv-ot',v=>parseFloat(v).toFixed(1))">
        <span class="sv" id="sv-ot">2.0</span></div>
      <div class="sr"><label>Sign Detect (m)</label>
        <input type="range" id="sl-sd" min="1.0" max="10.0" step="0.5" value="5.0"
          oninput="slUpd('sl-sd','sv-sd',v=>parseFloat(v).toFixed(1),'set_sign_detect')">
        <span class="sv" id="sv-sd">5.0</span></div>
      <div class="sr"><label>Sign Act (m)</label>
        <input type="range" id="sl-sa" min="0.5" max="5.0" step="0.1" value="2.0"
          oninput="slUpd('sl-sa','sv-sa',v=>parseFloat(v).toFixed(1),'set_sign_act')">
        <span class="sv" id="sv-sa">2.0</span></div>
    </div>

    <div class="panel">
      <div class="pt">🚦 Active ADAS Responses</div>
      <div style="margin-bottom:7px">
        <label style="display:flex;align-items:center;gap:7px;cursor:pointer;
                      font-size:12px;font-weight:700;color:var(--danger)">
          <input type="checkbox" id="chk-park" onchange="cmd('toggle_parking_auto')">
          🏁 AUTO-REVERSE PARKING
        </label>
      </div>
      <div class="ig">
        <div class="ind"><div class="dot" id="i-stop_sign"></div>🛑 STOP</div>
        <div class="ind"><div class="dot" id="i-no_entry"></div>⛔ NO ENTRY</div>
        <div class="ind"><div class="dot" id="i-pedestrian"></div>🚸 PEDESTRIAN</div>
        <div class="ind"><div class="dot" id="i-red_light"></div>🔴 RED LGT</div>
        <div class="ind"><div class="dot" id="i-yellow_light"></div>🟡 YEL LGT</div>
        <div class="ind"><div class="dot" id="i-green_light"></div>🟢 GRN LGT</div>
        <div class="ind"><div class="dot" id="i-caution"></div>⚠️ CAUTION</div>
        <div class="ind"><div class="dot" id="i-highway"></div>🛣️ HIGHWAY</div>
        <div class="ind"><div class="dot" id="i-park"></div>🅿️ AUTO-PARK</div>
        <div class="ind"><div class="dot" id="i-overtake"></div>🏎️ OVERTAKE</div>
      </div>
    </div>

    <div class="panel" style="flex:1">
      <div class="pt">
        📋 System Log
        <span id="hz-b" style="margin-left:auto;font-weight:800;color:var(--accent);font-size:11px">0.0 Hz</span>
        <span id="loc-b" style="font-size:10px;color:var(--text2);margin-left:8px">LOC: —</span>
      </div>
      <div id="log-box"></div>
    </div>
  </div>

  <!-- ── RIGHT: Map ──────────────────────────────────────────────────────── -->
  <div class="col">

    <div class="panel">
      <div class="pt">🗺 Interactive Map Mode</div>
      <div class="mtabs">
        <div class="mtab active" id="mt-DRIVE" onclick="setMode('DRIVE')">🚗 DRIVE</div>
        <div class="mtab" id="mt-NAV"   onclick="setMode('NAV')">🗺️ PLAN PATH</div>
        <div class="mtab" id="mt-SIGN"  onclick="setMode('SIGN')">🛑 PLACE SIGNS</div>
      </div>
      <div class="toolbar" id="toolbar">
        <span style="color:var(--success);font-weight:600">Drive Mode — car position shown on map</span>
      </div>
    </div>

    <div class="panel">
      <div class="pt">🗺 Track Map <span style="margin-left:auto;font-size:10px;color:var(--text2)" id="map-ts"></span></div>
      <img id="map-img" src="/api/map" alt="Map"
           onload="document.getElementById('map-ph').style.display='none';this.style.display='block'"
           onerror="this.style.display='none';document.getElementById('map-ph').style.display='flex'">
      <div id="map-ph">🗺 Map not available — run with --web flag</div>
    </div>

    <div class="panel" style="flex:1">
      <div class="pt">📋 Route Manifest &amp; Live Status</div>
      <div style="overflow-x:auto">
        <table id="rt">
          <thead><tr><th>Sign Type</th><th>Node</th><th>Status</th></tr></thead>
          <tbody id="rt-body">
            <tr><td colspan="3" style="text-align:center;color:var(--text2);padding:16px">No route planned</td></tr>
          </tbody>
        </table>
      </div>
    </div>
  </div>
</div>

<!-- ═══════════════════════════ IMU 3D POPUP ══════════════════════════════ -->
<div id="imu-pop">
  <div class="imu-ph">
    <span class="imu-title">⊕ IMU 3D Orientation</span>
    <button class="pop-close" onclick="toggleImu()">✕</button>
  </div>
  <div id="imu-scene">
    <div id="imu-cube">
      <div class="face f-top">TOP</div><div class="face f-bot">BOT</div>
      <div class="face f-fwd">FWD</div><div class="face f-bck">BCK</div>
      <div class="face f-rgt">RGT</div><div class="face f-lft">LFT</div>
    </div>
  </div>
  <div class="imu-rds">
    <div class="imu-rc"><div class="imu-rv" id="iv-y" style="color:#00b0d4">+0.0°</div><div class="imu-rl">YAW</div></div>
    <div class="imu-rc"><div class="imu-rv" id="iv-p" style="color:#4caf50">+0.0°</div><div class="imu-rl">PITCH</div></div>
    <div class="imu-rc"><div class="imu-rv" id="iv-r" style="color:#ff6d00">+0.0°</div><div class="imu-rl">ROLL</div></div>
  </div>
  <div id="imu-hw"><span style="color:var(--text2)">IMU: checking…</span></div>
  <button class="btn btn-a btn-full" onclick="calImu()">🎯 CALIBRATE — Set as (0°, 0°, 0°)</button>
</div>

<!-- ══════════════════════ SETTINGS (COLOUR THEME) ════════════════════════ -->
<div id="cfg-pop">
  <div style="display:flex;align-items:center;justify-content:space-between;margin-bottom:10px">
    <span style="font-weight:700;font-size:13px;color:var(--header)">⚙ Theme Colours</span>
    <button class="pop-close" onclick="toggleCfg()">✕</button>
  </div>
  <div class="cr"><label>Background</label>    <input type="color" id="c-bg"  value="#f0f2f5" oninput="ac('--bg',this.value)"></div>
  <div class="cr"><label>Panel</label>         <input type="color" id="c-pn"  value="#ffffff" oninput="ac('--panel',this.value)"></div>
  <div class="cr"><label>Text</label>          <input type="color" id="c-fg"  value="#212121" oninput="ac('--fg',this.value)"></div>
  <div class="cr"><label>Accent (Blue)</label> <input type="color" id="c-ac"  value="#1565c0" oninput="ac('--accent',this.value)"></div>
  <div class="cr"><label>Danger (Red)</label>  <input type="color" id="c-dn"  value="#c62828" oninput="ac('--danger',this.value)"></div>
  <div class="cr"><label>Success (Green)</label><input type="color" id="c-sc" value="#2e7d32" oninput="ac('--success',this.value)"></div>
  <div class="cr"><label>Warning (Orange)</label><input type="color" id="c-wn" value="#e65100" oninput="ac('--warn',this.value)"></div>
  <div class="cr"><label>Border</label>        <input type="color" id="c-bd"  value="#dde1e7" oninput="ac('--border',this.value)"></div>
  <div class="cr"><label>Card BG</label>       <input type="color" id="c-cd"  value="#f8f9fa" oninput="ac('--card',this.value)"></div>
  <div class="cr"><label>Header Bar</label>    <input type="color" id="c-hd"  value="#1a237e" oninput="ac('--header',this.value)"></div>
  <hr class="sep">
  <div class="btn-row">
    <button class="btn btn-s" onclick="saveTheme()" style="flex:1">💾 Save</button>
    <button class="btn btn-g" onclick="resetTheme()" style="flex:1">↺ Reset</button>
  </div>
</div>

<script>
// ── Theme ─────────────────────────────────────────────────────────────────
const DT = {'--bg':'#f0f2f5','--panel':'#ffffff','--fg':'#212121',
            '--accent':'#1565c0','--danger':'#c62828','--success':'#2e7d32',
            '--warn':'#e65100','--border':'#dde1e7','--card':'#f8f9fa','--header':'#1a237e'};
const CM = {'--bg':'c-bg','--panel':'c-pn','--fg':'c-fg','--accent':'c-ac',
            '--danger':'c-dn','--success':'c-sc','--warn':'c-wn',
            '--border':'c-bd','--card':'c-cd','--header':'c-hd'};

function ac(p,v){document.documentElement.style.setProperty(p,v)}
function saveTheme(){const t={};for(const k of Object.keys(DT))
  t[k]=getComputedStyle(document.documentElement).getPropertyValue(k).trim();
  localStorage.setItem('bfmc_t',JSON.stringify(t))}
function resetTheme(){for(const[k,v] of Object.entries(DT)){ac(k,v);
  document.getElementById(CM[k]).value=v}localStorage.removeItem('bfmc_t')}
function loadTheme(){const s=localStorage.getItem('bfmc_t');if(!s)return;
  const t=JSON.parse(s);for(const[k,id] of Object.entries(CM))
  if(t[k]){ac(k,t[k]);document.getElementById(id).value=t[k]}}
loadTheme();

function toggleCfg(){
  document.getElementById('cfg-pop').classList.toggle('open');
  document.getElementById('imu-pop').classList.remove('open')}
function toggleImu(){
  document.getElementById('imu-pop').classList.toggle('open');
  document.getElementById('cfg-pop').classList.remove('open')}

// ── Command ───────────────────────────────────────────────────────────────
function cmd(a,v){fetch('/api/command',{method:'POST',
  headers:{'Content-Type':'application/json'},
  body:JSON.stringify({action:a,value:v!==undefined?v:null})}).catch(()=>{})}

// ── Sliders ───────────────────────────────────────────────────────────────
function slUpd(si,vi,fmt,ca){const v=document.getElementById(si).value;
  document.getElementById(vi).textContent=fmt(v);if(ca)cmd(ca,parseFloat(v))}
function syncSl(si,vi,v,fmt){const el=document.getElementById(si);
  if(document.activeElement!==el){el.value=v;
  document.getElementById(vi).textContent=fmt(v)}}

// ── Recording ─────────────────────────────────────────────────────────────
let _rec=false;
function toggleRec(){cmd(_rec?'stop_recording':'start_recording')}

// ── Mode selector ─────────────────────────────────────────────────────────
const TBARS={
  DRIVE:'<span style="color:var(--success);font-weight:600">🚗 Drive Mode — car position shown on map</span>',
  NAV:  '<input id="node-inp" type="text" placeholder="Node ID" style="width:90px;padding:4px 8px;border:1px solid var(--border);border-radius:4px;font-size:12px">'+
        '<button class="btn btn-s" onclick="cmd(\'nav_set_start\',document.getElementById(\'node-inp\').value)">🟢 Start</button>'+
        '<button class="btn btn-a" onclick="cmd(\'nav_set_pass\',document.getElementById(\'node-inp\').value)">🔵 Pass</button>'+
        '<button class="btn btn-d" onclick="cmd(\'nav_set_end\',document.getElementById(\'node-inp\').value)">🔴 End</button>'+
        '<button class="btn btn-g" onclick="cmd(\'recalc_route\')" title="Recalculate path">⟳</button>'+
        '<button class="btn btn-g" onclick="cmd(\'clear_route\')" style="margin-left:auto">🗑 Clear</button>',
  SIGN: '<select id="sgn-sel" class="btn btn-o" style="cursor:pointer">'+
        '<option value="stop-sign">🛑 Stop</option>'+
        '<option value="crosswalk-sign">🚶 Crosswalk</option>'+
        '<option value="priority-sign">🔶 Priority</option>'+
        '<option value="parking-sign">🅿️ Parking</option>'+
        '<option value="highway-entry-sign">⬆️ Hwy Entry</option>'+
        '<option value="highway-exit-sign">↗️ Hwy Exit</option>'+
        '<option value="pedestrian">🚸 Pedestrian</option>'+
        '<option value="traffic-light">🚦 Traffic Light</option>'+
        '<option value="roundabout-sign">🔄 Roundabout</option>'+
        '<option value="noentry-sign">⛔ No Entry</option></select>'+
        '<label style="font-size:11px;font-weight:700;display:flex;align-items:center;gap:4px">'+
        '<input type="checkbox" id="del-mode"> Delete Mode</label>'+
        '<button class="btn btn-s" onclick="cmd(\'save_signs\')" style="margin-left:auto">💾 Save DB</button>'
};
function setMode(m){document.querySelectorAll('.mtab').forEach(t=>t.classList.remove('active'));
  document.getElementById('mt-'+m).classList.add('active');
  document.getElementById('toolbar').innerHTML=TBARS[m];cmd('set_mode',m)}

// ── IMU 3D Cube ───────────────────────────────────────────────────────────
let calY=0,calP=0,calR=0,sY=0,sP=0,sR=0;
const A=0.18;
function calImu(){const s=_last;calY=s.yaw_deg||0;calP=s.pitch_deg||0;calR=s.roll_deg||0;
  sY=sP=sR=0;const b=document.querySelector('#imu-pop .btn-a');
  b.textContent='✓ CALIBRATED';setTimeout(()=>{b.textContent='🎯 CALIBRATE — Set as (0°, 0°, 0°)'},2000)}
function updCube(y,p,r){sY+=A*(y-sY);sP+=A*(p-sP);sR+=A*(r-sR);
  const c=document.getElementById('imu-cube');if(!c)return;
  c.style.transform=`rotateX(${-sP-20}deg) rotateY(${sY+30}deg) rotateZ(${sR}deg)`;
  const fmt=v=>(v>=0?'+':'')+v.toFixed(1)+'°';
  document.getElementById('iv-y').textContent=fmt(sY);
  document.getElementById('iv-p').textContent=fmt(sP);
  document.getElementById('iv-r').textContent=fmt(sR)}

// ── Map refresh ───────────────────────────────────────────────────────────
let _mts=0;
function refMap(){const now=Date.now();if(now-_mts<1200)return;_mts=now;
  const ni=new Image();ni.src='/api/map?t='+now;
  ni.onload=()=>{const i=document.getElementById('map-img');
    i.src=ni.src;i.style.display='block';
    document.getElementById('map-ph').style.display='none';
    document.getElementById('map-ts').textContent=new Date().toLocaleTimeString()};
  ni.onerror=()=>{document.getElementById('map-img').style.display='none';
    document.getElementById('map-ph').style.display='flex'}}

// ── Stream reconnect ──────────────────────────────────────────────────────
function retryStream(el,src){setTimeout(()=>{el.src=src+'?t='+Date.now()},2000)}

// ── Main state poll ───────────────────────────────────────────────────────
let _last={};
async function poll(){
  try{
    const r=await fetch('/api/state');const s=await r.json();_last=s;
    const spd=Math.round(s.speed_pwm||0),str=(s.steer_deg||0).toFixed(1);
    const yaw=(s.yaw_deg||0).toFixed(1),pit=(s.pitch_deg||0).toFixed(1),rol=(s.roll_deg||0).toFixed(1);
    document.getElementById('t-spd').textContent=spd;
    document.getElementById('t-str').textContent=str+'°';
    document.getElementById('t-yaw').textContent=yaw+'°';
    document.getElementById('t-pit').textContent=pit+'°';
    document.getElementById('t-rol').textContent=rol+'°';
    document.getElementById('t-cnf').textContent=(s.lane_confidence||0).toFixed(2);
    document.getElementById('t-tx').textContent=Math.round(s.target_x||320);
    document.getElementById('t-le').textContent=(s.lateral_err_px||0).toFixed(0);
    document.getElementById('t-anc').textContent=s.lane_anchor||'—';
    document.getElementById('t-sgn').textContent=s.active_sign||'—';
    const yl=Array.isArray(s.yolo_labels)?s.yolo_labels.join(', '):(s.yolo_labels||'');
    document.getElementById('t-yol').textContent=yl||'—';

    // Lane cursor
    const pct=Math.max(0,Math.min(100,((s.target_x||320)-150)/340*100));
    document.getElementById('lbp').style.left=pct+'%';

    // Status bar
    const mode=(s.mode||'MANUAL').toUpperCase();
    const me=document.getElementById('sb-mode');me.textContent=mode;
    me.style.background=mode.includes('AUTO')?'rgba(106,0,173,.7)':
      mode.includes('PARK')?'rgba(198,40,40,.7)':'rgba(255,255,255,.15)';
    document.getElementById('sb-tele').textContent=`SPD: ${spd} | STR: ${str}°`;
    document.getElementById('sb-hz').textContent=(s.loop_hz||0).toFixed(1)+' Hz';
    document.getElementById('hz-b').textContent=(s.loop_hz||0).toFixed(1)+' Hz';
    document.getElementById('loc-b').textContent='LOC: '+(s.loc_node||'—');

    const bp=s.battery_pct||0;const be=document.getElementById('sb-bat');
    be.textContent='BAT: '+bp+'%';
    be.style.color=bp>50?'#a5d6a7':bp>20?'#ffcc80':'#ef9a9a';

    const ic=s.imu_connected;const ie=document.getElementById('sb-imu');
    ie.textContent=(ic?'IMU: OK':'IMU: LOST')+` | R:${rol}° P:${pit}° Y:${yaw}°`;
    ie.style.color=ic?'#a5d6a7':'#ef9a9a';
    document.getElementById('imu-hw').innerHTML=ic?
      '<span style="color:var(--success);font-weight:800">● IMU: HARDWARE</span>':
      '<span style="color:var(--warn);font-weight:800">● IMU: SIMULATED</span>';

    const conn=s.is_connected;const ce=document.getElementById('sb-conn');
    ce.textContent=conn?'🟢 CONNECTED':'⚫ DISCONNECTED';
    ce.style.color=conn?'#a5d6a7':'#ef9a9a';
    const bc=document.getElementById('btn-conn');
    if(bc){bc.textContent=conn?'🔌 Disconnect':'⚡ Connect Car';
      bc.className='btn btn-full '+(conn?'btn-d':'btn-a')}

    document.getElementById('sb-ai').textContent=s.ai_enabled?'AI: ON':'AI: OFF';
    document.getElementById('sb-ai').style.color=s.ai_enabled?'#a5d6a7':'rgba(255,255,255,.45)';
    const ba=document.getElementById('btn-adas');
    if(ba){ba.textContent=s.ai_enabled?'ADAS ASSIST: ON':'ADAS ASSIST: OFF';
      ba.className='btn btn-full '+(s.ai_enabled?'btn-g':'btn-o')}

    _rec=s.is_recording;
    const re=document.getElementById('sb-rec'),rb=document.getElementById('btn-rec');
    if(_rec){re.textContent='⏺ RECORDING';re.style.color='#ef9a9a';
      rb.textContent='⏹ Stop Recording';rb.className='btn btn-d'}
    else{re.textContent='⬜ NOT REC';re.style.color='rgba(255,255,255,.4)';
      rb.textContent='⏺ Start Recording';rb.className='btn btn-s'}

    // Sliders sync
    syncSl('sl-spd','sv-spd',s.base_speed||150,v=>Math.round(v));
    syncSl('sl-str','sv-str',s.steer_mult||1.0,v=>parseFloat(v).toFixed(1));
    syncSl('sl-sd','sv-sd',s.sign_detect_m||5.0,v=>parseFloat(v).toFixed(1));
    syncSl('sl-sa','sv-sa',s.sign_act_m||2.0,v=>parseFloat(v).toFixed(1));

    // ADAS dots
    const ak=s.active_indicators||[];
    document.querySelectorAll('.dot[id^="i-"]').forEach(d=>{
      const k=d.id.replace('i-','');d.className='dot'+(ak.includes(k)?' on':'')});

    // IMU cube
    updCube((s.yaw_deg||0)-calY,(s.pitch_deg||0)-calP,(s.roll_deg||0)-calR);

    // Route table
    const signs=s.route_signs||[];const tb=document.getElementById('rt-body');
    if(!signs.length){tb.innerHTML='<tr><td colspan="3" style="text-align:center;color:var(--text2);padding:14px">No route planned</td></tr>'}
    else tb.innerHTML=signs.map(sg=>{
      const live=(sg.status||'').includes('ACTIVE');
      return`<tr class="${live?'live':''}">`+
        `<td>${sg.emoji||''} ${sg.name||sg.type}</td>`+
        `<td>Node ${sg.node}</td><td>${sg.status||'⏳ PENDING'}</td></tr>`
    }).join('');

    refMap();
  }catch(e){}
}

// ── Log poll ──────────────────────────────────────────────────────────────
async function pollLog(){try{const r=await fetch('/api/log');const ls=await r.json();
  const b=document.getElementById('log-box');
  b.innerHTML=ls.slice(-80).map(l=>`<div class="log-${l.level}">[${l.ts}] ${l.msg}</div>`).join('');
  b.scrollTop=b.scrollHeight}catch(e){}}

setInterval(poll,250);setInterval(pollLog,1000);poll();pollLog();
</script>
</body>
</html>"""


# ── Flask app ─────────────────────────────────────────────────────────────────
def _make_app() -> "Flask":
    app = Flask(__name__)
    app.logger.disabled = True
    import logging
    logging.getLogger("werkzeug").setLevel(logging.ERROR)

    _fi = 1.0 / max(WEB_DASHBOARD_FPS, 1)

    @app.route("/")
    def index():
        return _HTML, 200, {"Content-Type": "text/html"}

    def _mjpeg_gen(lock, frame_ref, event):
        last = 0.0
        while True:
            event.wait(timeout=1.0)
            event.clear()
            now = time.monotonic()
            if now - last < _fi:
                continue
            last = now
            with lock:
                data = frame_ref[0]
            if not data:
                continue
            yield b"--frame\r\nContent-Type: image/jpeg\r\n\r\n" + data + b"\r\n"

    @app.route("/stream")
    def stream():
        def gen():
            last = 0.0
            while True:
                _cam_event.wait(timeout=1.0); _cam_event.clear()
                now = time.monotonic()
                if now - last < _fi: continue
                last = now
                with _cam_lock:
                    data = _cam_frame
                if data:
                    yield b"--frame\r\nContent-Type: image/jpeg\r\n\r\n" + data + b"\r\n"
        return Response(gen(), mimetype="multipart/x-mixed-replace; boundary=frame")

    @app.route("/stream/bev")
    def stream_bev():
        def gen():
            last = 0.0
            while True:
                _bev_event.wait(timeout=1.0); _bev_event.clear()
                now = time.monotonic()
                if now - last < _fi: continue
                last = now
                with _bev_lock:
                    data = _bev_frame
                if data:
                    yield b"--frame\r\nContent-Type: image/jpeg\r\n\r\n" + data + b"\r\n"
        return Response(gen(), mimetype="multipart/x-mixed-replace; boundary=frame")

    @app.route("/api/map")
    def api_map():
        with _map_lock:
            data = _map_frame
        if not data:
            return Response("", status=204)
        return Response(data, mimetype="image/jpeg",
                        headers={"Cache-Control": "no-cache"})

    @app.route("/api/state")
    def api_state():
        with _state_lock:
            snap = dict(_state)
        return jsonify(snap)

    @app.route("/api/log")
    def api_log():
        with _log_lock:
            lines = list(_log_lines)
        return jsonify(lines)

    @app.route("/api/command", methods=["POST"])
    def api_command():
        data   = request.get_json(silent=True) or {}
        action = str(data.get("action", "")).strip()
        value  = data.get("value")
        if action:
            with _cmd_lock:
                _pending_commands.append({"action": action, "value": value})
        return jsonify({"ok": True})

    return app


# ── WebDashboard class ────────────────────────────────────────────────────────
class WebDashboard:
    def __init__(self):
        if not _FLASK_AVAILABLE:
            print("[WebDashboard] Flask not installed — pip install flask")
            self._app = None
            return
        self._app = _make_app()

    def start(self) -> None:
        if self._app is None:
            return
        t = threading.Thread(
            target=self._app.run,
            kwargs={"host": WEB_DASHBOARD_HOST, "port": WEB_DASHBOARD_PORT,
                    "threaded": True, "use_reloader": False, "debug": False},
            daemon=True, name="web-dashboard",
        )
        t.start()
        print(f"[WebDashboard] http://{WEB_DASHBOARD_HOST}:{WEB_DASHBOARD_PORT}")

    push_telemetry  = staticmethod(push_telemetry)
    push_frame      = staticmethod(push_frame)
    push_bev_frame  = staticmethod(push_bev_frame)
    push_map_image  = staticmethod(push_map_image)
    push_log        = staticmethod(push_log)
    pop_commands    = staticmethod(pop_commands)
