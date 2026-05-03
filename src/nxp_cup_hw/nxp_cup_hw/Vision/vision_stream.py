#!/usr/bin/env python3
"""
vision_stream.py
----------------
NXP Cup – Vision Stream Dashboard
Reads from the nxp_cup_vision package.

  Top-left  : Debug image from nxp_cup_vision + EdgeVectors overlay
  Top-right : Twist comparison (encoder / IMU / fused)
  Bottom-left: IMU raw (ax, ay, az, gz)
  Bottom-right: Encoder odometry (vx, wz)

http://<navqplus-ip>:8081
"""

import argparse, json, math, sys, threading, time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import cv2
import numpy as np

try:
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import Imu, CompressedImage
    from synapse_msgs.msg import EdgeVectors
except ImportError:
    print("[ERROR] source /opt/ros/humble/setup.bash first")
    sys.exit(1)

DEFAULT_PORT = 8081
JPEG_QUALITY = 75
CANVAS_W     = 640
CANVAS_H     = 480

# ── Shared state ──────────────────────────────────────────────

class _Slot:
    def __init__(self):
        self._lock = threading.Lock()
        self._jpg  = b""
        self._seq  = 0
    def push(self, jpg):
        with self._lock:
            self._jpg = jpg
            self._seq += 1
    def get_if_newer(self, s):
        with self._lock:
            if self._seq != s:
                return self._seq, self._jpg
        return s, None

_slot_cam        = _Slot()
_enc_params      = [cv2.IMWRITE_JPEG_QUALITY, JPEG_QUALITY]
_frame_lock      = threading.Lock()
_latest_frame    = None          # raw BGR from CompressedImage
_ev_lock         = threading.Lock()
_latest_ev       = None          # EdgeVectors msg
_sensor_lock     = threading.Lock()
_imu_data        = None
_fused_odom_data = None
_enc_odom_data   = None

# ── Overlay ───────────────────────────────────────────────────

def overlay_edge_vectors(canvas, ev: EdgeVectors):
    """
    Draw EdgeVectors lines onto the canvas.
    Coords are in BEV space (400x300), scaled to canvas size.
    """
    cw, ch = canvas.shape[1], canvas.shape[0]
    sx = cw / 400.0
    sy = ch / 300.0

    def pt(p):
        return (int(p.x * sx), int(p.y * sy))

    if ev.vector_count >= 1:
        p0 = pt(ev.vector_1[0]); p1 = pt(ev.vector_1[1])
        cv2.line(canvas, p0, p1, (60, 60, 255), 3, cv2.LINE_AA)
        cv2.circle(canvas, p0, 6, (60, 60, 255), -1)
        cv2.circle(canvas, p1, 6, (60, 60, 255), -1)

    if ev.vector_count >= 2:
        p0 = pt(ev.vector_2[0]); p1 = pt(ev.vector_2[1])
        cv2.line(canvas, p0, p1, (255, 60, 60), 3, cv2.LINE_AA)
        cv2.circle(canvas, p0, 6, (255, 60, 60), -1)
        cv2.circle(canvas, p1, 6, (255, 60, 60), -1)

    l_ok = "OK" if ev.vector_count >= 1 else "--"
    r_ok = "OK" if ev.vector_count >= 2 else "--"
    cv2.putText(canvas, f"L:{l_ok}  R:{r_ok}", (6, ch - 8),
                cv2.FONT_HERSHEY_SIMPLEX, 0.45, (240, 240, 100), 1, cv2.LINE_AA)
    return canvas


def build_cam_frame(canvas_w, canvas_h):
    with _frame_lock:
        raw = _latest_frame
    canvas = cv2.resize(raw, (canvas_w, canvas_h)) if raw is not None \
        else np.zeros((canvas_h, canvas_w, 3), dtype=np.uint8)
    if raw is None:
        cv2.putText(canvas, "Waiting for nxp_cup_vision…", (10, canvas_h // 2),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (80, 80, 120), 1, cv2.LINE_AA)
        return canvas
    with _ev_lock:
        ev = _latest_ev
    if ev is not None:
        canvas = overlay_edge_vectors(canvas, ev)
    return canvas


def push_cam(canvas_w, canvas_h):
    ok, jpg = cv2.imencode(".jpg", build_cam_frame(canvas_w, canvas_h), _enc_params)
    if ok:
        _slot_cam.push(jpg.tobytes())


# ── Dashboard HTML ────────────────────────────────────────────

def _make_dashboard():
    return r"""<!doctype html>
<html><head><meta charset="utf-8"><title>NXP Cup Dashboard</title>
<script src="https://cdnjs.cloudflare.com/ajax/libs/Chart.js/4.4.1/chart.umd.min.js"></script>
<style>
*{box-sizing:border-box;margin:0;padding:0}
body{background:#080c10;color:#ccc;font:11px 'Courier New',monospace;height:100vh;
     display:grid;grid-template-columns:1fr 1fr;grid-template-rows:1fr 1fr;gap:3px;padding:3px}
.panel{background:#0d1117;border:1px solid #1e2d3d;border-radius:3px;
       display:flex;flex-direction:column;overflow:hidden;min-height:0}
.panel-title{font-size:.7em;color:#4a9eff;padding:3px 8px;
             border-bottom:1px solid #1e2d3d;flex-shrink:0;letter-spacing:.05em}
.panel-title span{color:#888;font-weight:normal;font-size:.95em}
.panel img{width:100%;flex:1;object-fit:contain;min-height:0}
.panel .chart-wrap{flex:1;min-height:0;position:relative;padding:4px}
.panel canvas{position:absolute;inset:4px}
</style>
</head><body>

<div class="panel">
  <div class="panel-title">CAMERA <span>nxp_cup_vision + EdgeVectors overlay</span></div>
  <img src="/stream">
</div>

<div class="panel">
  <div class="panel-title">TWIST COMPARISON <span id="h-twist">encoder / imu / fused</span></div>
  <div class="chart-wrap"><canvas id="c-twist"></canvas></div>
</div>

<div class="panel">
  <div class="panel-title">IMU RAW <span id="h-imu">ax ay az gz</span></div>
  <div class="chart-wrap"><canvas id="c-imu"></canvas></div>
</div>

<div class="panel">
  <div class="panel-title">ENCODER ODOMETRY <span id="h-enc">vx  wz</span></div>
  <div class="chart-wrap"><canvas id="c-enc"></canvas></div>
</div>

<script>
const N = 80;
const DARK_GRID  = '#1a2030';
const TICK_COLOR = '#556070';

function mkChart(id, defs, y2) {
    const datasets = defs.map(d => ({
        label: d.label, data: [],
        borderColor: d.color, backgroundColor: 'transparent',
        borderWidth: 1.5, pointRadius: 0, tension: 0.25,
        spanGaps: true, yAxisID: d.y2 ? 'y2' : 'y',
    }));
    const scales = {
        x: { display: false },
        y: { position: 'left',
             ticks: { color: TICK_COLOR, font: { size: 9 }, maxTicksLimit: 5 },
             grid: { color: DARK_GRID } }
    };
    if (y2) scales.y2 = {
        position: 'right',
        ticks: { color: TICK_COLOR, font: { size: 9 }, maxTicksLimit: 5 },
        grid: { drawOnChartArea: false }
    };
    return new Chart(document.getElementById(id), {
        type: 'line', data: { labels: [], datasets },
        options: { animation: false, responsive: true, maintainAspectRatio: false,
                   interaction: { mode: 'index', intersect: false },
                   plugins: { legend: { labels: { color: '#8090a0', boxWidth: 8,
                                                  font: { size: 9 }, padding: 6 } } },
                   scales }
    });
}

const twistChart = mkChart('c-twist', [
    { label: 'vx enc',    color: '#f07030', y2: false },
    { label: 'vx fused',  color: '#ffcc00', y2: false },
    { label: 'wz enc',    color: '#3090f0', y2: true  },
    { label: 'wz imu/gz', color: '#30d0d0', y2: true  },
    { label: 'wz fused',  color: '#80ff80', y2: true  },
], true);

const imuChart = mkChart('c-imu', [
    { label: 'ax', color: '#ff5555' },
    { label: 'ay', color: '#55ff55' },
    { label: 'az', color: '#5599ff' },
    { label: 'gz', color: '#ffaa00' },
]);

const encChart = mkChart('c-enc', [
    { label: 'vx enc', color: '#f07030', y2: false },
    { label: 'wz enc', color: '#3090f0', y2: true  },
], true);

function push(chart, ...vals) {
    chart.data.labels.push(Date.now());
    if (chart.data.labels.length > N) chart.data.labels.shift();
    chart.data.datasets.forEach((ds, i) => {
        ds.data.push(vals[i] != null ? vals[i] : null);
        if (ds.data.length > N) ds.data.shift();
    });
    chart.update('none');
}

let t = 0;
setTimeout(() => {
    setInterval(async () => {
        try {
            const resp = await fetch('/data?t=' + (t++));
            const d    = await resp.json();
            push(twistChart,
                d.enc  ? d.enc.vx   : null,
                d.fused? d.fused.vx : null,
                d.enc  ? d.enc.wz   : null,
                d.imu  ? d.imu.gz   : null,
                d.fused? d.fused.wz : null
            );
            if (d.imu) push(imuChart, d.imu.ax, d.imu.ay, d.imu.az, d.imu.gz);
            if (d.enc) push(encChart, d.enc.vx, d.enc.wz);
            if (d.fused)
                document.getElementById('h-twist').textContent =
                    `vx enc:${(d.enc?d.enc.vx:0).toFixed(2)} fused:${d.fused.vx.toFixed(2)}  `+
                    `wz gz:${(d.imu?d.imu.gz:0).toFixed(3)} fused:${d.fused.wz.toFixed(3)}`;
            if (d.imu)
                document.getElementById('h-imu').textContent =
                    `ax:${d.imu.ax.toFixed(2)} ay:${d.imu.ay.toFixed(2)} `+
                    `az:${d.imu.az.toFixed(2)} gz:${d.imu.gz.toFixed(3)}`;
            if (d.enc)
                document.getElementById('h-enc').textContent =
                    `vx:${d.enc.vx.toFixed(3)} m/s   wz:${d.enc.wz.toFixed(3)} r/s`;
        } catch(e) { console.error(e); }
    }, 150);
}, 1000);
</script>
</body></html>"""


# ── HTTP handler ──────────────────────────────────────────────

class _Handler(BaseHTTPRequestHandler):
    _canvas_w = CANVAS_W
    _canvas_h = CANVAS_H

    def log_message(self, *a): pass

    def do_GET(self):
        if self.path.startswith('/data'):
            with _sensor_lock:
                imu   = dict(_imu_data)        if _imu_data        else None
                fused = dict(_fused_odom_data)  if _fused_odom_data else None
                enc   = dict(_enc_odom_data)    if _enc_odom_data   else None
            body = json.dumps({'imu': imu, 'fused': fused, 'enc': enc}).encode()
            self.send_response(200)
            self.send_header('Content-Type',   'application/json')
            self.send_header('Content-Length', str(len(body)))
            self.send_header('Cache-Control',  'no-store')
            self.end_headers()
            self.wfile.write(body)

        elif self.path == '/stream':
            self.send_response(200)
            self.send_header('Content-Type', 'multipart/x-mixed-replace; boundary=frame')
            self.send_header('Cache-Control', 'no-cache')
            self.end_headers()
            seq = -1
            try:
                while True:
                    seq, jpg = _slot_cam.get_if_newer(seq)
                    if jpg:
                        hdr = (b'--frame\r\nContent-Type: image/jpeg\r\nContent-Length: '
                               + str(len(jpg)).encode() + b'\r\n\r\n')
                        self.wfile.write(hdr + jpg + b'\r\n')
                        self.wfile.flush()
                    else:
                        time.sleep(0.01)
            except (BrokenPipeError, ConnectionResetError):
                pass

        elif self.path in ('/', '/index.html'):
            body = _make_dashboard().encode('utf-8')
            self.send_response(200)
            self.send_header('Content-Type',   'text/html; charset=utf-8')
            self.send_header('Content-Length', str(len(body)))
            self.send_header('Cache-Control',  'no-store')
            self.end_headers()
            self.wfile.write(body)

        else:
            self.send_error(404)


def start_server(port, canvas_w, canvas_h):
    _Handler._canvas_w = canvas_w
    _Handler._canvas_h = canvas_h
    srv = ThreadingHTTPServer(('0.0.0.0', port), _Handler)
    threading.Thread(target=srv.serve_forever, daemon=True).start()


# ── ROS 2 node ────────────────────────────────────────────────

class VisionStreamNode(Node):
    def __init__(self, canvas_w, canvas_h):
        super().__init__('vision_stream')
        self._cw = canvas_w
        self._ch = canvas_h
        be = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                        durability=DurabilityPolicy.VOLATILE, depth=5)

        # RELIABLE depth=10 — must match publisher (nxp_track_vision)
        self.create_subscription(
            CompressedImage, '/nxp_cup/debug_image', self._on_image, 10)
        self.create_subscription(
            EdgeVectors, '/edge_vectors', self._on_edge_vectors, 10)
        self.create_subscription(
            Imu,      '/nxp_cup/imu',          self._on_imu,        be)
        self.create_subscription(
            Odometry, '/nxp_cup/wheel_odom',   self._on_fused_odom, be)
        self.create_subscription(
            Odometry, '/nxp_cup/encoder_odom', self._on_enc_odom,   be)

        self.get_logger().info('VisionStreamNode ready')

    def _on_image(self, msg: CompressedImage):
        global _latest_frame
        np_arr = np.frombuffer(msg.data, np.uint8)
        bgr    = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        if bgr is None:
            return
        with _frame_lock:
            _latest_frame = bgr
        push_cam(self._cw, self._ch)

    def _on_edge_vectors(self, msg: EdgeVectors):
        global _latest_ev
        with _ev_lock:
            _latest_ev = msg
        push_cam(self._cw, self._ch)

    def _on_imu(self, msg):
        global _imu_data
        with _sensor_lock:
            _imu_data = {
                'ax': msg.linear_acceleration.x,
                'ay': msg.linear_acceleration.y,
                'az': msg.linear_acceleration.z,
                'gx': msg.angular_velocity.x,
                'gy': msg.angular_velocity.y,
                'gz': msg.angular_velocity.z,
                'stamp': time.time(),
            }

    def _on_fused_odom(self, msg):
        global _fused_odom_data
        q = msg.pose.pose.orientation
        with _sensor_lock:
            _fused_odom_data = {
                'vx':  msg.twist.twist.linear.x,
                'wz':  msg.twist.twist.angular.z,
                'x':   msg.pose.pose.position.x,
                'y':   msg.pose.pose.position.y,
                'yaw': math.atan2(2*(q.w*q.z + q.x*q.y),
                                  1 - 2*(q.y**2 + q.z**2)),
                'stamp': time.time(),
            }

    def _on_enc_odom(self, msg):
        global _enc_odom_data
        with _sensor_lock:
            _enc_odom_data = {
                'vx':  msg.twist.twist.linear.x,
                'wz':  msg.twist.twist.angular.z,
                'stamp': time.time(),
            }


# ── Main ──────────────────────────────────────────────────────

def _watchdog(cw, ch):
    while True:
        time.sleep(0.5)
        push_cam(cw, ch)


def parse_args():
    import rclpy.utilities
    argv = rclpy.utilities.remove_ros_args(sys.argv[1:])
    p = argparse.ArgumentParser()
    p.add_argument('--port',     type=int, default=DEFAULT_PORT)
    p.add_argument('--canvas-w', type=int, default=CANVAS_W)
    p.add_argument('--canvas-h', type=int, default=CANVAS_H)
    return p.parse_args(argv)


def main():
    rclpy.init()
    args = parse_args()
    start_server(args.port, args.canvas_w, args.canvas_h)
    print(f'\n[Dashboard]  http://<navqplus-ip>:{args.port}\n')
    threading.Thread(target=_watchdog, args=(args.canvas_w, args.canvas_h),
                     daemon=True).start()
    node = VisionStreamNode(args.canvas_w, args.canvas_h)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()