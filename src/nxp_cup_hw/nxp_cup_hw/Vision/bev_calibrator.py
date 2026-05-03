#!/usr/bin/env python3
"""
bev_calibrator.py
-----------------
Live BEV calibration tool. Serves a web UI at http://<navqplus-ip>:8090

Left panel  : raw camera frame with the source trapezoid drawn on it
Right panel : warped BEV result using current corner values
Controls    : 8 sliders (one per x/y coordinate of each corner)
Copy button : prints the final BEV_SRC_POINTS_BASE array to paste into code

ROS parameters (all tunable live via the UI):
  src_bl_x, src_bl_y   bottom-left  corner  (nearest, left)
  src_br_x, src_br_y   bottom-right corner  (nearest, right)
  src_tr_x, src_tr_y   top-right    corner  (farthest, right)
  src_tl_x, src_tl_y   top-left     corner  (farthest, left)

Add to setup.py:
  "bev_calibrator = nxp_cup_hw.Vision.bev_calibrator:main"
"""

import json
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import parse_qs, urlparse

import cv2
import numpy as np

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage

BEV_W    = 400
BEV_H    = 300
HTTP_PORT = 8090

# Default source points — tune via UI
DEFAULTS = {
    "src_bl_x": 80.0,  "src_bl_y": 479.0,
    "src_br_x": 560.0, "src_br_y": 479.0,
    "src_tr_x": 610.0, "src_tr_y": 280.0,
    "src_tl_x": 30.0,  "src_tl_y": 280.0,
}

# Shared state
_lock       = threading.Lock()
_raw_jpg    = b""
_bev_jpg    = b""
_raw_seq    = 0
_bev_seq    = 0
_params     = dict(DEFAULTS)
_img_w      = 640
_img_h      = 480


def _src_points():
    with _lock:
        p = dict(_params)
    return np.float32([
        [p["src_bl_x"], p["src_bl_y"]],
        [p["src_br_x"], p["src_br_y"]],
        [p["src_tr_x"], p["src_tr_y"]],
        [p["src_tl_x"], p["src_tl_y"]],
    ])


def _dst_points():
    return np.float32([
        [0,         BEV_H - 1],
        [BEV_W - 1, BEV_H - 1],
        [BEV_W - 1, 0],
        [0,         0],
    ])


def _process(bgr):
    global _raw_jpg, _bev_jpg, _raw_seq, _bev_seq, _img_w, _img_h

    h, w = bgr.shape[:2]
    with _lock:
        _img_w, _img_h = w, h

    src = _src_points()

    # Draw trapezoid on raw frame
    raw_vis = bgr.copy()
    pts = src.astype(np.int32)
    cv2.polylines(raw_vis, [pts], True, (0, 255, 255), 2, cv2.LINE_AA)
    colors = [(0,200,60),(0,200,60),(0,100,255),(0,100,255)]
    labels = ["BL","BR","TR","TL"]
    for i, (pt, col, lbl) in enumerate(zip(pts, colors, labels)):
        cv2.circle(raw_vis, tuple(pt), 6, col, -1)
        cv2.putText(raw_vis, lbl, (pt[0]+6, pt[1]-6),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, col, 1, cv2.LINE_AA)

    # BEV warp
    M = cv2.getPerspectiveTransform(src, _dst_points())
    bev = cv2.warpPerspective(bgr, M, (BEV_W, BEV_H))

    # Draw grid on BEV for reference
    for x in range(0, BEV_W, 50):
        cv2.line(bev, (x, 0), (x, BEV_H), (40, 40, 40), 1)
    for y in range(0, BEV_H, 50):
        cv2.line(bev, (0, y), (BEV_W, y), (40, 40, 40), 1)

    enc = [cv2.IMWRITE_JPEG_QUALITY, 80]
    ok1, buf1 = cv2.imencode(".jpg", raw_vis, enc)
    ok2, buf2 = cv2.imencode(".jpg", bev,     enc)

    with _lock:
        if ok1: _raw_jpg = buf1.tobytes(); _raw_seq += 1
        if ok2: _bev_jpg = buf2.tobytes(); _bev_seq += 1


# ── HTTP handler ──────────────────────────────────────────────

_HTML = r"""<!doctype html>
<html><head><meta charset="utf-8">
<title>BEV Calibrator</title>
<style>
  @import url('https://fonts.googleapis.com/css2?family=Share+Tech+Mono&display=swap');
  *{box-sizing:border-box;margin:0;padding:0}
  :root{
    --bg:#0b0e11;--panel:#12171d;--border:#1f2d3a;
    --accent:#00e5ff;--accent2:#ff6b35;--text:#c8d8e8;--dim:#4a6070;
  }
  body{background:var(--bg);color:var(--text);font-family:'Share Tech Mono',monospace;
       min-height:100vh;display:flex;flex-direction:column;gap:0}
  header{padding:10px 20px;border-bottom:1px solid var(--border);
         display:flex;align-items:center;gap:16px}
  header h1{font-size:1em;letter-spacing:.15em;color:var(--accent);text-transform:uppercase}
  header span{color:var(--dim);font-size:.8em}
  .streams{display:grid;grid-template-columns:1fr 1fr;gap:2px;background:var(--border)}
  .stream-panel{background:var(--panel);display:flex;flex-direction:column}
  .stream-label{padding:4px 10px;font-size:.7em;letter-spacing:.1em;
                color:var(--dim);border-bottom:1px solid var(--border);text-transform:uppercase}
  .stream-panel img{width:100%;object-fit:contain;display:block}
  .controls{padding:16px 20px;background:var(--panel);border-top:2px solid var(--border);
            display:grid;grid-template-columns:1fr 1fr;gap:12px 32px}
  .corner{border:1px solid var(--border);padding:10px 14px;border-radius:2px}
  .corner-title{font-size:.7em;letter-spacing:.12em;text-transform:uppercase;margin-bottom:8px}
  .corner-title.bl{color:#00e5ff}.corner-title.br{color:#00e5ff}
  .corner-title.tr{color:#ff6b35}.corner-title.tl{color:#ff6b35}
  .row{display:flex;align-items:center;gap:10px;margin-bottom:6px}
  .row label{width:24px;font-size:.75em;color:var(--dim)}
  .row input[type=range]{flex:1;accent-color:var(--accent);height:3px;cursor:pointer}
  .row input[type=number]{width:60px;background:#0b0e11;border:1px solid var(--border);
                           color:var(--text);font-family:inherit;font-size:.8em;
                           padding:2px 4px;text-align:right}
  .footer{padding:10px 20px;border-top:1px solid var(--border);
          display:flex;align-items:center;gap:12px}
  button{background:transparent;border:1px solid var(--accent);color:var(--accent);
         font-family:inherit;font-size:.8em;letter-spacing:.08em;
         padding:6px 14px;cursor:pointer;text-transform:uppercase;transition:all .15s}
  button:hover{background:var(--accent);color:#000}
  button.reset{border-color:var(--dim);color:var(--dim)}
  button.reset:hover{background:var(--dim);color:#000}
  #output{flex:1;font-size:.72em;color:#4a8060;white-space:pre;overflow:auto;
          user-select:text;cursor:text;padding:4px 8px;
          border:1px solid var(--border);border-radius:2px;max-height:80px}
  #status{font-size:.7em;color:var(--dim)}
</style>
</head><body>

<header>
  <h1>⬡ BEV Calibrator</h1>
  <span>adjust trapezoid corners — bottom=near, top=far</span>
  <span id="status" style="margin-left:auto">connecting…</span>
</header>

<div class="streams">
  <div class="stream-panel">
    <div class="stream-label">Raw camera + trapezoid</div>
    <img id="raw" src="/stream/raw">
  </div>
  <div class="stream-panel">
    <div class="stream-label">BEV warp result</div>
    <img id="bev" src="/stream/bev">
  </div>
</div>

<div class="controls" id="controls"></div>

<div class="footer">
  <button onclick="copyValues()">Copy array</button>
  <button class="reset" onclick="resetDefaults()">Reset defaults</button>
  <div id="output"></div>
</div>

<script>
const DEFAULTS = {
  src_bl_x:80,  src_bl_y:479,
  src_br_x:560, src_br_y:479,
  src_tr_x:610, src_tr_y:280,
  src_tl_x:30,  src_tl_y:280,
};

const CORNERS = [
  { id:'bl', label:'Bottom-Left  (near)',  cls:'bl', keys:['src_bl_x','src_bl_y'] },
  { id:'br', label:'Bottom-Right (near)',  cls:'br', keys:['src_br_x','src_br_y'] },
  { id:'tr', label:'Top-Right    (far)',   cls:'tr', keys:['src_tr_x','src_tr_y'] },
  { id:'tl', label:'Top-Left     (far)',   cls:'tl', keys:['src_tl_x','src_tl_y'] },
];

let imgW = 640, imgH = 480;
let vals = {...DEFAULTS};

// Fetch image dims
fetch('/dims').then(r=>r.json()).then(d=>{imgW=d.w;imgH=d.h;buildUI()});

function buildUI(){
  const ctrl = document.getElementById('controls');
  ctrl.innerHTML = '';
  CORNERS.forEach(c=>{
    const div = document.createElement('div');
    div.className = 'corner';
    div.innerHTML = `<div class="corner-title ${c.cls}">${c.label}</div>`;
    c.keys.forEach(k=>{
      const isY = k.endsWith('_y');
      const max = isY ? imgH : imgW;
      const row = document.createElement('div');
      row.className = 'row';
      row.innerHTML = `
        <label>${isY?'Y':'X'}</label>
        <input type="range" id="r_${k}" min="0" max="${max}" step="1" value="${vals[k]}">
        <input type="number" id="n_${k}" min="0" max="${max}" value="${vals[k]}" style="width:64px">
      `;
      div.appendChild(row);
      row.querySelector(`#r_${k}`).oninput = e => sync(k, e.target.value, 'n');
      row.querySelector(`#n_${k}`).oninput = e => sync(k, e.target.value, 'r');
    });
    ctrl.appendChild(div);
  });
}

function sync(key, value, other){
  vals[key] = parseFloat(value);
  document.getElementById(`${other}_${key}`).value = value;
  sendParams();
}

let _timer = null;
function sendParams(){
  clearTimeout(_timer);
  _timer = setTimeout(()=>{
    fetch('/set', {method:'POST',
      headers:{'Content-Type':'application/json'},
      body: JSON.stringify(vals)
    }).then(r=>r.json()).then(d=>{
      document.getElementById('status').textContent = d.ok ? 'ok' : 'err';
    });
  }, 60);
}

function copyValues(){
  const p = vals;
  const arr = `BEV_SRC_POINTS_BASE = np.float32([\n` +
    `    [${p.src_bl_x.toFixed(1)}, ${p.src_bl_y.toFixed(1)}],   # bottom-left\n` +
    `    [${p.src_br_x.toFixed(1)}, ${p.src_br_y.toFixed(1)}],   # bottom-right\n` +
    `    [${p.src_tr_x.toFixed(1)}, ${p.src_tr_y.toFixed(1)}],   # top-right\n` +
    `    [${p.src_tl_x.toFixed(1)}, ${p.src_tl_y.toFixed(1)}],   # top-left\n` +
    `])`;
  const box = document.getElementById('output');
  box.textContent = arr;
  box.style.color = '#00e5ff';
  // best-effort clipboard — works on https, silently skips on http
  try { navigator.clipboard.writeText(arr); } catch(_){}
  // fallback: select the text so user can Ctrl+C
  const sel = window.getSelection();
  const range = document.createRange();
  range.selectNodeContents(box);
  sel.removeAllRanges();
  sel.addRange(range);
}

function resetDefaults(){
  vals = {...DEFAULTS};
  CORNERS.forEach(c => c.keys.forEach(k=>{
    const r = document.getElementById(`r_${k}`);
    const n = document.getElementById(`n_${k}`);
    if(r) r.value = vals[k];
    if(n) n.value = vals[k];
  }));
  sendParams();
}

// Reconnect streams if they stall
['raw','bev'].forEach(id=>{
  const img = document.getElementById(id);
  setInterval(()=>{ img.src = `/stream/${id}?t=${Date.now()}`; }, 5000);
});
</script>
</body></html>"""


class _Handler(BaseHTTPRequestHandler):
    def log_message(self, *a): pass

    def do_GET(self):
        path = urlparse(self.path).path

        if path == "/":
            body = _HTML.encode()
            self.send_response(200)
            self.send_header("Content-Type", "text/html; charset=utf-8")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers(); self.wfile.write(body)

        elif path == "/dims":
            with _lock:
                body = json.dumps({"w": _img_w, "h": _img_h}).encode()
            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers(); self.wfile.write(body)

        elif path in ("/stream/raw", "/stream/bev"):
            is_raw = path.endswith("/raw")
            self.send_response(200)
            self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
            self.send_header("Cache-Control", "no-cache")
            self.end_headers()
            last_seq = -1
            try:
                while True:
                    with _lock:
                        seq = _raw_seq if is_raw else _bev_seq
                        jpg = _raw_jpg  if is_raw else _bev_jpg
                    if seq != last_seq and jpg:
                        last_seq = seq
                        hdr = (b"--frame\r\nContent-Type: image/jpeg\r\nContent-Length: "
                               + str(len(jpg)).encode() + b"\r\n\r\n")
                        self.wfile.write(hdr + jpg + b"\r\n")
                        self.wfile.flush()
                    else:
                        time.sleep(0.02)
            except (BrokenPipeError, ConnectionResetError):
                pass

        else:
            self.send_error(404)

    def do_POST(self):
        path = urlparse(self.path).path
        if path == "/set":
            length = int(self.headers.get("Content-Length", 0))
            data   = json.loads(self.rfile.read(length))
            with _lock:
                for k, v in data.items():
                    if k in _params:
                        _params[k] = float(v)
            body = b'{"ok":true}'
            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers(); self.wfile.write(body)
        else:
            self.send_error(404)


# ── ROS node ──────────────────────────────────────────────────

class BevCalibratorNode(Node):

    def __init__(self):
        super().__init__("bev_calibrator")

        for k, v in DEFAULTS.items():
            self.declare_parameter(k, v)
            _params[k] = self.get_parameter(k).get_parameter_value().double_value

        self.create_subscription(
            CompressedImage, "/camera/image_raw/compressed",
            self._on_image, 10
        )
        self.get_logger().info(
            f"BEV Calibrator ready → http://<navqplus-ip>:{HTTP_PORT}"
        )

    def _on_image(self, msg: CompressedImage):
        np_arr = np.frombuffer(msg.data, np.uint8)
        bgr    = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        if bgr is not None:
            bgr = cv2.rotate(bgr, cv2.ROTATE_180)
            _process(bgr)


def main(args=None):
    rclpy.init(args=args)
    node = BevCalibratorNode()

    srv = ThreadingHTTPServer(("0.0.0.0", HTTP_PORT), _Handler)
    threading.Thread(target=srv.serve_forever, daemon=True).start()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()