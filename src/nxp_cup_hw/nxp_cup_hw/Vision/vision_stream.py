#!/usr/bin/env python3
"""
NXP Cup – Vision Stream Dashboard
===================================
2x2 grid: camera+chains | IMU plots | odometry plots | encoder plots
http://<navqplus-ip>:8081
"""

import argparse, json, math, sys, threading, time
from http.server import BaseHTTPRequestHandler, HTTPServer, ThreadingHTTPServer

import cv2
import numpy as np

try:
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
    from std_msgs.msg import String, Int32MultiArray
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import Imu, Image
    from cv_bridge import CvBridge
except ImportError:
    print("[ERROR] source /opt/ros/humble/setup.bash first")
    sys.exit(1)

DEFAULT_PORT  = 8081
DEFAULT_SCALE = 2
JPEG_QUALITY  = 75
BEV_W, BEV_H  = 400, 300

# ── Shared state ──────────────────────────────────────────────────────────────

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

_slot_cam      = _Slot()
_enc           = [cv2.IMWRITE_JPEG_QUALITY, JPEG_QUALITY]
_frame_lock    = threading.Lock()
_latest_frame  = None
_chain_lock    = threading.Lock()
_latest_chains = None
_sensor_lock   = threading.Lock()
_imu_data      = None
_odom_data     = None
_ticks_data    = None

C_LEFT     = (80,  80,  255)
C_RIGHT    = (80,  200,  80)
C_CENTER   = (255, 180,   0)
C_CROSSLINK= (100, 100, 100)
C_CORRIDOR = (40,   80,  40)
C_DOT_L    = (140, 140, 255)
C_DOT_R    = (140, 255, 140)
C_TEXT     = (220, 220, 220)

# ── Camera frame builder ──────────────────────────────────────────────────────

def overlay_chains(canvas, data, bev_w, bev_h):
    cw, ch = canvas.shape[1], canvas.shape[0]
    def p(x, y): return (int(x/bev_w*cw), int(y/bev_h*ch))
    left   = [tuple(pt) for pt in data.get("left",   [])]
    right  = [tuple(pt) for pt in data.get("right",  [])]
    center = [tuple(pt) for pt in data.get("center", [])]
    if len(left) > 1 and len(right) > 1:
        poly = [p(x,y) for x,y in left] + [p(x,y) for x,y in reversed(right)]
        ov = canvas.copy()
        cv2.fillPoly(ov, [np.array(poly, dtype=np.int32)], C_CORRIDOR)
        canvas = cv2.addWeighted(canvas, 0.55, ov, 0.45, 0)
    n = data.get("n_strips", 24)
    if left and right:
        def near(c, yt): return min(c, key=lambda pt: abs(pt[1]-yt))
        for sy in np.linspace(0, bev_h, n, endpoint=False):
            cv2.line(canvas, p(*near(left,sy)), p(*near(right,sy)), C_CROSSLINK, 1)
    def draw(chain, cl, cd, t=2):
        pts = [p(x,y) for x,y in chain]
        for i in range(1, len(pts)): cv2.line(canvas, pts[i-1], pts[i], cl, t, cv2.LINE_AA)
        for pt in pts: cv2.circle(canvas, pt, 4, cd, -1)
    draw(left,   C_LEFT,   C_DOT_L)
    draw(right,  C_RIGHT,  C_DOT_R)
    draw(center, C_CENTER, C_CENTER, 2)
    valid = data.get("valid", {})
    age   = (time.time()-data["stamp"])*1000 if data.get("stamp") else 0
    hud   = [
        f"L {len(left):>2}pts {'OK' if valid.get('left') else '--'}  R {len(right):>2}pts {'OK' if valid.get('right') else '--'}  C {len(center):>2}pts",
        f"Corridor {int(data.get('corridor_area',0))}px  age {age:.0f}ms",
    ]
    for i, t in enumerate(hud):
        cv2.putText(canvas, t, (6, 15+i*15), cv2.FONT_HERSHEY_SIMPLEX, 0.40, C_TEXT, 1, cv2.LINE_AA)
    return canvas

def build_cam_frame(canvas_w, canvas_h):
    with _frame_lock: raw = _latest_frame
    canvas = cv2.resize(raw, (canvas_w, canvas_h)) if raw is not None \
        else np.zeros((canvas_h, canvas_w, 3), dtype=np.uint8)
    if raw is None:
        cv2.putText(canvas, "Waiting for camera...", (10, canvas_h//2),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (80,80,120), 1, cv2.LINE_AA)
    with _chain_lock: chains = _latest_chains
    if chains:
        canvas = overlay_chains(canvas, chains, chains.get("bev_w", BEV_W), chains.get("bev_h", BEV_H))
    return canvas

def push_cam(canvas_w, canvas_h):
    ok, jpg = cv2.imencode(".jpg", build_cam_frame(canvas_w, canvas_h), _enc)
    if ok: _slot_cam.push(jpg.tobytes())

# ── HTML ──────────────────────────────────────────────────────────────────────

def _make_dashboard():
    return """<!doctype html>
<html><head><meta charset="utf-8"><title>NXP Cup Vision</title>
<script src="https://cdnjs.cloudflare.com/ajax/libs/Chart.js/4.4.1/chart.umd.min.js"></script>
<style>
*{box-sizing:border-box;margin:0;padding:0}
body{background:#080808;color:#ccc;font:12px monospace;height:100vh;
     display:grid;grid-template-columns:1fr 1fr;grid-template-rows:1fr 1fr;gap:4px;padding:4px}
.panel{background:#111;border:1px solid #222;border-radius:4px;display:flex;flex-direction:column;overflow:hidden;min-height:0}
.panel h3{font-size:.75em;color:#6af;padding:4px 8px;border-bottom:1px solid #222;flex-shrink:0}
.panel img{width:100%;flex:1;object-fit:contain;min-height:0}
.panel canvas{flex:1;min-height:0;width:100%}
</style>
</head><body>
<div class="panel"><h3>BEV Camera + Lane Chains</h3><img src="/stream"></div>
<div class="panel"><h3 id="h-imu">IMU Acceleration (m/s\u00b2)</h3><canvas id="c-accel"></canvas></div>
<div class="panel"><h3>Odometry vx (m/s) &amp; wz (r/s)</h3><canvas id="c-odom"></canvas></div>
<div class="panel"><h3>Encoder Ticks</h3><canvas id="c-enc"></canvas></div>
<script>
const N = 60;
function mkChart(id, defs) {
    return new Chart(document.getElementById(id), {
        type: 'line',
        data: {
            labels: [],
            datasets: defs.map(d => ({
                label: d.label,
                data: [],
                borderColor: d.color,
                backgroundColor: 'transparent',
                borderWidth: 2,
                pointRadius: 0,
                tension: 0.2,
                spanGaps: true
            }))
        },
        options: {
            animation: false,
            responsive: true,
            maintainAspectRatio: false,
            interaction: { mode: 'index', intersect: false },
            plugins: { legend: { labels: { color:'#aaa', boxWidth:10, font:{size:10} } } },
            scales: {
                x: { display: false },
                y: { beginAtZero: false, ticks:{ color:'#888', font:{size:10} }, grid:{ color:'#1a1a1a' } }
            }
        }
    });
}
const charts = {
    accel: mkChart('c-accel', [{label:'ax',color:'#f66'},{label:'ay',color:'#6f6'},{label:'az',color:'#66f'}]),
    odom:  mkChart('c-odom',  [{label:'vx',color:'#fa0'},{label:'wz',color:'#0af'}]),
    enc:   mkChart('c-enc',   [{label:'Left',color:'#f8a'},{label:'Right',color:'#8fa'}])
};
function addPoint(chart, ...vals) {
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
        if (d.imu)   addPoint(charts.accel, d.imu.ax, d.imu.ay, d.imu.az);
        if (d.odom)  addPoint(charts.odom,  d.odom.vx, d.odom.wz);
        if (d.ticks) addPoint(charts.enc,   d.ticks.left, d.ticks.right);
        if (d.imu)   document.getElementById('h-imu').textContent =
            'IMU  ax:' + d.imu.ax.toFixed(2) + '  ay:' + d.imu.ay.toFixed(2) + '  az:' + d.imu.az.toFixed(2);
    } catch(e) { console.error(e); }
}, 150);
}, 1000);
</script>
</body></html>"""

# ── HTTP handler ──────────────────────────────────────────────────────────────

class _Handler(BaseHTTPRequestHandler):
    _canvas_w = BEV_W * DEFAULT_SCALE
    _canvas_h = BEV_H * DEFAULT_SCALE

    def log_message(self, *a): pass

    def do_GET(self):
        if self.path.startswith('/data'):
            with _sensor_lock:
                imu   = dict(_imu_data)   if _imu_data   else None
                odom  = dict(_odom_data)  if _odom_data  else None
                ticks = dict(_ticks_data) if _ticks_data else None
            body = json.dumps({'imu': imu, 'odom': odom, 'ticks': ticks}).encode()
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

# ── ROS 2 node ────────────────────────────────────────────────────────────────

class VisionStreamNode(Node):
    def __init__(self, canvas_w, canvas_h):
        super().__init__('nxp_vision_stream')
        self._cw     = canvas_w
        self._ch     = canvas_h
        self._bridge = CvBridge()
        be = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                        durability=DurabilityPolicy.VOLATILE, depth=5)
        self.create_subscription(Image,            '/nxp_cup/debug_image', self._on_image,  10)
        self.create_subscription(String,           '/nxp_cup/lane_chains', self._on_chains, 10)
        self.create_subscription(Imu,              '/nxp_cup/imu',         self._on_imu,    be)
        self.create_subscription(Odometry,         '/nxp_cup/wheel_odom',  self._on_odom,   be)
        self.create_subscription(Int32MultiArray,  '/nxp_cup/wheel_ticks', self._on_ticks,  be)
        self.get_logger().info('VisionStreamNode ready')

    def _on_image(self, msg):
        global _latest_frame
        try: bgr = self._bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception: return
        with _frame_lock: _latest_frame = bgr
        push_cam(self._cw, self._ch)

    def _on_chains(self, msg):
        global _latest_chains
        try: data = json.loads(msg.data)
        except Exception: return
        with _chain_lock: _latest_chains = data
        push_cam(self._cw, self._ch)

    def _on_imu(self, msg):
        global _imu_data
        with _sensor_lock:
            _imu_data = {'ax': msg.linear_acceleration.x, 'ay': msg.linear_acceleration.y,
                         'az': msg.linear_acceleration.z, 'gx': msg.angular_velocity.x,
                         'gy': msg.angular_velocity.y,    'gz': msg.angular_velocity.z,
                         'stamp': time.time()}

    def _on_odom(self, msg):
        global _odom_data
        q = msg.pose.pose.orientation
        with _sensor_lock:
            _odom_data = {'vx': msg.twist.twist.linear.x,  'wz': msg.twist.twist.angular.z,
                          'x':  msg.pose.pose.position.x,  'y':  msg.pose.pose.position.y,
                          'yaw': math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y**2+q.z**2)),
                          'stamp': time.time()}

    def _on_ticks(self, msg):
        global _ticks_data
        with _sensor_lock:
            _ticks_data = {'left':  msg.data[0] if len(msg.data) > 0 else 0,
                           'right': msg.data[1] if len(msg.data) > 1 else 0,
                           'stamp': time.time()}

def _watchdog(cw, ch):
    while True:
        time.sleep(0.5)
        push_cam(cw, ch)

def parse_args():
    import rclpy.utilities
    argv = rclpy.utilities.remove_ros_args(sys.argv[1:])
    p = argparse.ArgumentParser()
    p.add_argument('--port',  type=int, default=DEFAULT_PORT)
    p.add_argument('--scale', type=int, default=DEFAULT_SCALE)
    return p.parse_args(argv)

def main():
    rclpy.init()
    args     = parse_args()
    canvas_w = BEV_W * args.scale
    canvas_h = BEV_H * args.scale
    start_server(args.port, canvas_w, canvas_h)
    print(f'\n[Dashboard]  http://<navqplus-ip>:{args.port}\n')
    threading.Thread(target=_watchdog, args=(canvas_w, canvas_h), daemon=True).start()
    node = VisionStreamNode(canvas_w, canvas_h)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()