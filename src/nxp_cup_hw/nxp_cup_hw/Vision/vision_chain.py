#!/usr/bin/env python3
"""
NXP Cup – Edge Vector Vision Node
Publishes /edge_vectors (JSON) and /nxp_cup/debug_image.
Detects two dark lines on a bright surface.
No hard track-width constant.

ROS parameter
-------------
use_bev : bool (default True)
    Apply BEV perspective warp before detection.
    Set via launch file: parameters=[{'use_bev': True}]
"""

import argparse, itertools, json, math, os, sys, threading, time
import cv2
import numpy as np

try:
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import String
    from sensor_msgs.msg import Image
    from cv_bridge import CvBridge
    _ROS_AVAILABLE = True
except ImportError:
    _ROS_AVAILABLE = False

from http.server import BaseHTTPRequestHandler, HTTPServer

# ── Config ────────────────────────────────────────────────────
CAM_DEVICE    = "/dev/video3"
CAM_W, CAM_H  = 640, 480
CAM_FPS       = 30
DARK_THRESH   = 80
MIN_LINE_COLS = 3
COL_GAP       = 3
N_STRIPS      = 24
N_BOTTOM      = 5
PROC_SCALE    = 0.5
MJPEG_PORT    = 8080
JPEG_QUALITY  = 70
ROS_NODE_NAME = "nxp_track_vision"
ROS_FRAME_ID  = "camera"
ARROW_LEN     = 90

# ── BEV config (from reference b3rb_ros_edge_vectors node) ───
USE_BEV      = True        # overridden by ROS param 'use_bev' at startup
BASE_CAM_W   = 640.0
BASE_CAM_H   = 480.0
BEV_W        = 400
BEV_H        = 300
IGNORE_TOP_ROWS_RATIO = 0.15

BEV_SRC_POINTS_BASE = np.float32([
    [ 80.0, 479.0],
    [560.0, 479.0],
    [610.0, 280.0],
    [ 30.0, 280.0],
])

BEV_DST_POINTS = np.float32([
    [0.0,         BEV_H - 1.0],
    [BEV_W - 1.0, BEV_H - 1.0],
    [BEV_W - 1.0, 0.0],
    [0.0,         0.0],
])

_thresh_rt = None

# ── Inter-frame lane anchors ──────────────────────────────────
_anchor_left_x  = None
_anchor_right_x = None

# ── Camera ────────────────────────────────────────────────────
def open_camera(device, w, h, fps):
    pipe = (f"v4l2src device={device} ! "
            f"video/x-raw,framerate={fps}/1,width={w},height={h} ! "
            f"videoconvert ! video/x-raw,format=BGR ! appsink drop=1")
    cap = cv2.VideoCapture(pipe, cv2.CAP_GSTREAMER)
    if not cap.isOpened():
        cap = cv2.VideoCapture(device)
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, w)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, h)
        cap.set(cv2.CAP_PROP_FPS, fps)
    if not cap.isOpened():
        sys.exit(f"[ERROR] Cannot open {device}")
    return cap

# ── BEV transform ─────────────────────────────────────────────
def apply_bev(bgr):
    """Warp frame into BEV_W x BEV_H bird's-eye space. Returns (bev_img, bev_mask)."""
    h, w = bgr.shape[:2]
    sx, sy = w / BASE_CAM_W, h / BASE_CAM_H
    src = BEV_SRC_POINTS_BASE.copy()
    src[:, 0] = np.clip(src[:, 0] * sx, 0, w - 1)
    src[:, 1] = np.clip(src[:, 1] * sy, 0, h - 1)
    M = cv2.getPerspectiveTransform(src, BEV_DST_POINTS)
    src_mask = np.zeros((h, w), dtype=np.uint8)
    cv2.fillConvexPoly(src_mask, src.astype(np.int32), 255)
    masked   = cv2.bitwise_and(bgr, bgr, mask=src_mask)
    bev_img  = cv2.warpPerspective(masked,   M, (BEV_W, BEV_H))
    bev_mask = cv2.warpPerspective(src_mask, M, (BEV_W, BEV_H))
    return bev_img, bev_mask

# ── Segmentation ──────────────────────────────────────────────
def segment_lines(bgr, bev_mask=None):
    gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
    t = _thresh_rt if _thresh_rt is not None else DARK_THRESH
    _, dark = cv2.threshold(gray, t, 255, cv2.THRESH_BINARY_INV)
    adapt = cv2.adaptiveThreshold(gray, 255,
                cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY_INV, 21, 8)
    mask = cv2.bitwise_or(dark, adapt)
    if bev_mask is not None:
        mask   = cv2.bitwise_and(mask, bev_mask)
        ignore = int(bgr.shape[0] * IGNORE_TOP_ROWS_RATIO)
        mask[:ignore, :] = 0
    k3 = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
    k5 = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 3))
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN,  k3, iterations=1)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, k5, iterations=2)
    if bev_mask is not None:
        mask = cv2.bitwise_and(mask, bev_mask)
        mask[:ignore, :] = 0
    return mask

# ── Group finder ──────────────────────────────────────────────
def _find_groups(col_arr):
    if len(col_arr) == 0:
        return []
    groups = []
    start = int(col_arr[0]); prev = int(col_arr[0])
    for c in col_arr[1:]:
        c = int(c)
        if c - prev > COL_GAP:
            if prev - start + 1 >= MIN_LINE_COLS:
                groups.append((start, prev, (start + prev) // 2))
            start = c
        prev = c
    if prev - start + 1 >= MIN_LINE_COLS:
        groups.append((start, prev, (start + prev) // 2))
    return groups

# ── Chain extraction with anchor-based tracking ───────────────
def extract_chains(binary):
    global _anchor_left_x, _anchor_right_x
    h, w    = binary.shape
    strip_h = max(1, h // N_STRIPS)
    left_x  = _anchor_left_x  if _anchor_left_x  is not None else w // 4
    right_x = _anchor_right_x if _anchor_right_x is not None else (3 * w) // 4
    left_raw, right_raw = [], []

    for i in range(N_STRIPS - 1, -1, -1):
        y0 = i * strip_h; y1 = min(y0 + strip_h, h); cy = (y0 + y1) // 2
        strip        = binary[y0:y1, :]
        col_presence = strip.max(axis=0)
        dark_cols    = np.where(col_presence > 0)[0]
        groups       = _find_groups(dark_cols)
        lpt = rpt    = None

        if len(groups) == 0:
            pass
        elif len(groups) == 1:
            cx = groups[0][2]
            if abs(cx - left_x) <= abs(cx - right_x):
                lpt = (cx, cy); left_x  = cx
            else:
                rpt = (cx, cy); right_x = cx
        else:
            best_l    = min(groups,                    key=lambda g: abs(g[2] - left_x))
            remaining = [g for g in groups if g is not best_l]
            best_r    = min(remaining,                 key=lambda g: abs(g[2] - right_x))
            if best_l[2] > best_r[2]:
                best_l, best_r = best_r, best_l
            lpt = (best_l[2], cy); left_x  = best_l[2]
            rpt = (best_r[2], cy); right_x = best_r[2]

        left_raw.append(lpt); right_raw.append(rpt)

    for pt in left_raw:
        if pt is not None: _anchor_left_x  = pt[0]; break
    for pt in right_raw:
        if pt is not None: _anchor_right_x = pt[0]; break

    return left_raw, right_raw

def bottom_pts(raw, n=N_BOTTOM):
    out = []
    for pt in raw:
        if pt is not None:
            out.append(pt)
            if len(out) == n: break
    return out

# ── Collinear triplet & vector ────────────────────────────────
def _area(p1, p2, p3):
    return abs((p2[0]-p1[0])*(p3[1]-p1[1]) - (p3[0]-p1[0])*(p2[1]-p1[1]))

def best_triplet(pts):
    if len(pts) < 3: return None
    best_a, best = math.inf, None
    for t in itertools.combinations(pts, 3):
        a = _area(*t)
        if a < best_a: best_a, best = a, list(t)
    return best

def triplet_to_vec(triplet):
    s = sorted(triplet, key=lambda p: p[1], reverse=True)
    vx, vy = s[2][0] - s[0][0], s[2][1] - s[0][1]
    mag = math.hypot(vx, vy)
    if mag < 1e-6: return None
    return {"x0": float(s[0][0]), "y0": float(s[0][1]),
            "dx": vx/mag,         "dy": vy/mag}

# ── JSON payload ──────────────────────────────────────────────
def _side(vec, tri):
    if vec is None:
        return {"valid": False, "x0": 0.0, "y0": 0.0, "dx": 0.0, "dy": 0.0, "triplet": []}
    return {"valid": True,
            "x0": round(vec["x0"], 2), "y0": round(vec["y0"], 2),
            "dx": round(vec["dx"], 5), "dy": round(vec["dy"], 5),
            "triplet": [[int(x), int(y)] for x, y in tri]}

def build_json(lv, rv, lt, rt, img_w, img_h, stamp):
    return json.dumps({"stamp": stamp, "img_w": img_w, "img_h": img_h,
                        "left":  _side(lv, lt or []),
                        "right": _side(rv, rt or [])})

# ── Debug visualisation ───────────────────────────────────────
def draw_debug(frame, mask, lpts, rpts, lt, rt, lv, rv, fps):
    vis = frame.copy()
    tint = np.zeros_like(vis); tint[mask > 0] = (0, 200, 60)
    vis = cv2.addWeighted(vis, 1.0, tint, 0.28, 0)
    for pt in lpts: cv2.circle(vis, pt, 4, (200, 80, 255), -1)
    if lt:
        s = sorted(lt, key=lambda p: p[1], reverse=True)
        for i in range(len(s)-1): cv2.line(vis, s[i], s[i+1], (255, 60, 60), 1, cv2.LINE_AA)
        for pt in s: cv2.circle(vis, pt, 7, (255, 60, 60), 2)
    if lv:
        ox, oy = int(lv["x0"]), int(lv["y0"])
        cv2.arrowedLine(vis, (ox, oy),
                        (int(ox + lv["dx"]*ARROW_LEN), int(oy + lv["dy"]*ARROW_LEN)),
                        (255, 40, 40), 2, cv2.LINE_AA, tipLength=0.25)
    for pt in rpts: cv2.circle(vis, pt, 4, (255, 80, 200), -1)
    if rt:
        s = sorted(rt, key=lambda p: p[1], reverse=True)
        for i in range(len(s)-1): cv2.line(vis, s[i], s[i+1], (60, 60, 255), 1, cv2.LINE_AA)
        for pt in s: cv2.circle(vis, pt, 7, (60, 60, 255), 2)
    if rv:
        ox, oy = int(rv["x0"]), int(rv["y0"])
        cv2.arrowedLine(vis, (ox, oy),
                        (int(ox + rv["dx"]*ARROW_LEN), int(oy + rv["dy"]*ARROW_LEN)),
                        (40, 40, 255), 2, cv2.LINE_AA, tipLength=0.25)
    h = vis.shape[0]
    if _anchor_left_x  is not None: cv2.line(vis, (int(_anchor_left_x),  h-8), (int(_anchor_left_x),  h), (255, 40, 40), 2)
    if _anchor_right_x is not None: cv2.line(vis, (int(_anchor_right_x), h-8), (int(_anchor_right_x), h), (40, 40, 255), 2)
    lok = "OK" if lv else "--"; rok = "OK" if rv else "--"
    for i, txt in enumerate([
        f"FPS {fps:.1f}  [{'BEV' if USE_BEV else 'RAW'}]",
        f"L={len(lpts)}pts {lok}   R={len(rpts)}pts {rok}",
        f"N_BOTTOM={N_BOTTOM}  STRIPS={N_STRIPS}",
    ]):
        cv2.putText(vis, txt, (8, 18+i*19), cv2.FONT_HERSHEY_SIMPLEX, 0.50, (240,240,240), 1, cv2.LINE_AA)
    return vis

# ── MJPEG server ──────────────────────────────────────────────
class _LatestFrame:
    def __init__(self):
        self._lock, self._data, self._seq = threading.Lock(), b"", 0
    def push(self, jpg):
        with self._lock: self._data, self._seq = jpg, self._seq + 1
    def get_if_newer(self, last):
        with self._lock:
            if self._seq != last: return self._seq, self._data
        return last, None

_lf = _LatestFrame()

_PAGE = (
    "HTTP/1.0 200 OK\r\nContent-Type: text/html\r\n\r\n"
    "<!doctype html><html><head><title>NXP Vision</title>"
    "<style>body{margin:0;background:#0a0a0a;display:flex;flex-direction:column;"
    "align-items:center;font-family:monospace;color:#0f0}"
    "img{max-width:100%;border:1px solid #0f0}</style></head><body>"
    "<h2>Edge Vector Vision</h2><img src='/stream'/>"
    "<p>LEFT arrow=red | RIGHT arrow=blue | triplet=circles | ticks=anchors</p>"
    "</body></html>"
).encode()

class _MJPEGHandler(BaseHTTPRequestHandler):
    def log_message(self, *a): pass
    def do_GET(self):
        if self.path == "/":
            self.wfile.write(_PAGE); return
        if self.path == "/stream":
            self.send_response(200)
            self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
            self.send_header("Cache-Control", "no-cache")
            self.end_headers()
            seq = -1
            try:
                while True:
                    seq, jpg = _lf.get_if_newer(seq)
                    if jpg:
                        hdr = (b"--frame\r\nContent-Type: image/jpeg\r\nContent-Length: "
                               + str(len(jpg)).encode() + b"\r\n\r\n")
                        self.wfile.write(hdr + jpg + b"\r\n")
                        self.wfile.flush()
                    else:
                        time.sleep(0.01)
            except (BrokenPipeError, ConnectionResetError): pass
            return
        self.send_error(404)

def start_mjpeg(port):
    srv = HTTPServer(("0.0.0.0", port), _MJPEGHandler)
    threading.Thread(target=srv.serve_forever, daemon=True).start()
    print(f"[MJPEG] http://<navqplus-ip>:{port}")

# ── ROS 2 node ────────────────────────────────────────────────
class TrackVisionNode(Node):
    def __init__(self):
        super().__init__(ROS_NODE_NAME)
        self.pub_vec = self.create_publisher(String, "/edge_vectors",        10)
        self.pub_raw = self.create_publisher(Image,  "/nxp_cup/debug_image", 10)
        self.bridge  = CvBridge()
        # ── ROS parameter — set in launch file ────────────────
        self.declare_parameter('use_bev', True)
        self.use_bev = self.get_parameter('use_bev').get_parameter_value().bool_value
        self.get_logger().info(
            f"TrackVisionNode ready → /edge_vectors  /nxp_cup/debug_image  use_bev={self.use_bev}"
        )

    def publish(self, json_str, raw_bgr):
        now = self.get_clock().now().to_msg()
        msg = String(); msg.data = json_str
        self.pub_vec.publish(msg)
        img = self.bridge.cv2_to_imgmsg(raw_bgr, encoding="bgr8")
        img.header.stamp = now; img.header.frame_id = ROS_FRAME_ID
        self.pub_raw.publish(img)

# ── Args ──────────────────────────────────────────────────────
def parse_args():
    argv = sys.argv[1:]
    if _ROS_AVAILABLE:
        import rclpy.utilities
        argv = rclpy.utilities.remove_ros_args(argv)
    p = argparse.ArgumentParser()
    p.add_argument("--device",       default=CAM_DEVICE)
    p.add_argument("--width",        type=int, default=CAM_W)
    p.add_argument("--height",       type=int, default=CAM_H)
    p.add_argument("--fps",          type=int, default=CAM_FPS)
    p.add_argument("--port",         type=int, default=MJPEG_PORT)
    p.add_argument("--no-flip",      action="store_true")
    p.add_argument("--no-ros",       action="store_true")
    p.add_argument("--sim",          action="store_true")
    p.add_argument("--jpeg-quality", type=int, default=JPEG_QUALITY)
    return p.parse_args(argv)

# ── Main ──────────────────────────────────────────────────────
def main():
    global _thresh_rt, USE_BEV
    args     = parse_args()
    use_ros  = _ROS_AVAILABLE and not args.no_ros
    ros_node = None

    if use_ros:
        rclpy.init()
        ros_node = TrackVisionNode()
        USE_BEV  = ros_node.use_bev          # ← from launch file parameter
        executor = rclpy.executors.SingleThreadedExecutor()
        executor.add_node(ros_node)
        threading.Thread(target=executor.spin, daemon=True).start()
        print(f"[ROS2] Node started  use_bev={USE_BEV}")
    else:
        print("[INFO] MJPEG-only mode")

    _sim_frame, _sim_lock = None, threading.Lock()
    if args.sim:
        if not _ROS_AVAILABLE:
            sys.exit("[ERROR] --sim requires rclpy")
        from sensor_msgs.msg import Image as _Img
        from cv_bridge import CvBridge as _CB
        _br = _CB()
        class _SimCam(rclpy.node.Node):
            def __init__(self):
                super().__init__("nxp_vision_sim_cam")
                self.create_subscription(_Img, "/camera/image_raw", self._cb, 10)
            def _cb(self, msg):
                nonlocal _sim_frame
                try:
                    bgr = _br.imgmsg_to_cv2(msg, "bgr8")
                    with _sim_lock: _sim_frame = bgr
                except Exception: pass
        sc = _SimCam()
        if use_ros:
            executor.add_node(sc)
        else:
            rclpy.init()
            e2 = rclpy.executors.SingleThreadedExecutor()
            e2.add_node(sc)
            threading.Thread(target=e2.spin, daemon=True).start()
        cap = None
        print("[Camera] Sim — /camera/image_raw")
    else:
        cap = open_camera(args.device, args.width, args.height, args.fps)

    do_flip = not args.no_flip
    cfg_path = os.path.join(os.path.dirname(__file__), "camera_config.json")
    if not os.path.exists(cfg_path):
        cfg_path = "camera_config.json"
    if os.path.exists(cfg_path):
        try:
            with open(cfg_path) as f:
                cfg = json.load(f)
            if "dark_thresh" in cfg:
                _thresh_rt = int(cfg["dark_thresh"])
            if "flip" in cfg and not args.no_flip:
                do_flip = bool(cfg["flip"])
            print(f"[Config] thresh={_thresh_rt}  flip={do_flip}  bev={USE_BEV}")
        except Exception as e:
            print(f"[Config] Failed: {e}")
    else:
        print(f"[Config] No camera_config.json — bev={USE_BEV}")

    start_mjpeg(args.port)

    enc    = [cv2.IMWRITE_JPEG_QUALITY, args.jpeg_quality]
    prev_t = time.time()
    fps_d  = 0.0
    count  = 0

    print("[Running] Ctrl-C to stop")
    try:
        while True:
            if args.sim:
                with _sim_lock: frame = _sim_frame
                if frame is None: time.sleep(0.02); continue
            else:
                ret, frame = cap.read()
                if not ret: time.sleep(0.02); continue

            if do_flip:
                frame = cv2.rotate(frame, cv2.ROTATE_180)

            if USE_BEV:
                proc_frame, bev_mask = apply_bev(frame)
            else:
                proc_frame, bev_mask = frame, None

            img_h, img_w = proc_frame.shape[:2]
            now   = time.time()
            fps_d = 0.9 * fps_d + 0.1 / max(now - prev_t, 1e-6)
            prev_t = now; count += 1

            ph = max(1, int(img_h * PROC_SCALE))
            pw = max(1, int(img_w * PROC_SCALE))
            small      = cv2.resize(proc_frame, (pw, ph))
            small_mask = cv2.resize(bev_mask, (pw, ph)) if bev_mask is not None else None

            mask_s     = segment_lines(small, small_mask)
            lr_s, rr_s = extract_chains(mask_s)

            scale = 1.0 / PROC_SCALE
            def sc_pt(p): return (int(p[0]*scale), int(p[1]*scale)) if p else None
            lr_full = [sc_pt(p) for p in lr_s]
            rr_full = [sc_pt(p) for p in rr_s]

            lpts = bottom_pts(lr_full)
            rpts = bottom_pts(rr_full)
            lt   = best_triplet(lpts)
            rt   = best_triplet(rpts)
            lv   = triplet_to_vec(lt) if lt else None
            rv   = triplet_to_vec(rt) if rt else None

            stamp    = time.time()
            json_str = build_json(lv, rv, lt, rt, img_w, img_h, stamp)

            mask_full = cv2.resize(mask_s, (img_w, img_h), interpolation=cv2.INTER_NEAREST)
            debug     = draw_debug(proc_frame, mask_full, lpts, rpts, lt, rt, lv, rv, fps_d)

            if use_ros and ros_node is not None:
                ros_node.publish(json_str, debug)

            ok, jpg = cv2.imencode(".jpg", debug, enc)
            if ok: _lf.push(jpg.tobytes())

            if count % 60 == 0:
                print(f"fps={fps_d:.1f}  "
                      f"L={'OK' if lv else '--'}  R={'OK' if rv else '--'}  "
                      f"anchors=({_anchor_left_x},{_anchor_right_x})")

    except KeyboardInterrupt:
        print("\n[Stopped]")
    finally:
        if cap is not None: cap.release()
        if use_ros: rclpy.shutdown()


if __name__ == "__main__":
    main()