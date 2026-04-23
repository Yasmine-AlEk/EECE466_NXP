#!/usr/bin/env python3
"""
NXP Cup – Lane Chain Viewer
============================
Subscribes to the topics published by nxp_track_vision.py and streams
a rendered visualisation over MJPEG HTTP so you can watch it in any
browser over SSH — no display, no X11 needed.

Subscriptions
-------------
  /nxp_cup/lane_chains    std_msgs/String   JSON chain data  (primary)
  /nxp_cup/debug_image    sensor_msgs/Image raw BEV debug frame (optional)

MJPEG stream
------------
  http://<navqplus-ip>:8081          main rendered view
  http://<navqplus-ip>:8081/raw      raw debug_image from vision node
  http://<navqplus-ip>:8081/both     side-by-side rendered + raw

Run
---
  source /opt/ros/humble/setup.bash
  python3 nxp_chain_viewer.py [--port 8081] [--no-raw]

  --port      HTTP port (default 8081, so it doesn't clash with vision node)
  --no-raw    Don't subscribe to debug_image (saves bandwidth)
  --scale N   Scale factor for rendered canvas (default 2 → 800×600)
"""

import argparse
import json
import sys
import threading
import time

import cv2
import numpy as np

try:
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import String
    from sensor_msgs.msg import Image
    from cv_bridge import CvBridge
    _ROS_OK = True
except ImportError:
    print("[ERROR] rclpy not found. Source your ROS 2 workspace first:")
    print("        source /opt/ros/humble/setup.bash")
    sys.exit(1)

from http.server import BaseHTTPRequestHandler, HTTPServer

# ─────────────────────────────────────────────────────────────────────────────
#  CONFIG
# ─────────────────────────────────────────────────────────────────────────────

DEFAULT_PORT   = 8081
DEFAULT_SCALE  = 2          # rendered canvas = BEV dims × this factor
JPEG_QUALITY   = 72
STALE_TIMEOUT  = 2.0        # seconds before "NO SIGNAL" overlay appears


# ─────────────────────────────────────────────────────────────────────────────
#  SHARED STATE  (written by ROS callbacks, read by HTTP threads)
# ─────────────────────────────────────────────────────────────────────────────

class _FrameSlot:
    """Lock-free latest-frame store with sequence number for polling."""
    def __init__(self):
        self._lock = threading.Lock()
        self._jpg  = b""
        self._seq  = 0

    def push(self, jpg: bytes):
        with self._lock:
            self._jpg  = jpg
            self._seq += 1

    def get_if_newer(self, last_seq):
        with self._lock:
            if self._seq != last_seq:
                return self._seq, self._jpg
        return last_seq, None


_slot_rendered = _FrameSlot()   # rendered chain visualisation
_slot_raw      = _FrameSlot()   # raw debug_image from vision node
_slot_both     = _FrameSlot()   # side-by-side

_enc = [cv2.IMWRITE_JPEG_QUALITY, JPEG_QUALITY]

_last_chain_t  = 0.0            # epoch time of last /lane_chains message


# ─────────────────────────────────────────────────────────────────────────────
#  RENDERER
# ─────────────────────────────────────────────────────────────────────────────

# Colours
C_BG         = (18,  18,  18)
C_GRID       = (40,  40,  40)
C_LEFT       = (80,  80,  255)    # blue
C_RIGHT      = (80,  200, 80)     # green  (different from vision node so it's clear)
C_CENTER     = (255, 180, 0)      # amber
C_CROSSLINK  = (80,  80,  80)
C_CORRIDOR   = (40,  60,  40)
C_DOT_L      = (140, 140, 255)
C_DOT_R      = (140, 255, 140)
C_TEXT       = (220, 220, 220)
C_WARN       = (0,   60,  200)


def _pt(x, y, bev_w, bev_h, canvas_w, canvas_h):
    """Map BEV pixel coords to canvas pixel coords."""
    cx = int(x / bev_w * canvas_w)
    cy = int(y / bev_h * canvas_h)
    return (cx, cy)


def render_chains(data: dict, canvas_w: int, canvas_h: int) -> np.ndarray:
    """
    Draw a clean bird's-eye visualisation of the lane chain data.

    Layout (canvas, origin top-left, y grows downward):
        bottom = nearest to car
        top    = farthest ahead
    """
    bev_w = data.get("bev_w", 400)
    bev_h = data.get("bev_h", 300)

    canvas = np.full((canvas_h, canvas_w, 3), C_BG, dtype=np.uint8)

    def p(x, y):
        return _pt(x, y, bev_w, bev_h, canvas_w, canvas_h)

    # ── Grid ─────────────────────────────────────────────────────────────────
    n_strips = data.get("n_strips", 24)
    for i in range(n_strips + 1):
        y = int(i / n_strips * canvas_h)
        cv2.line(canvas, (0, y), (canvas_w, y), C_GRID, 1)
    for col in range(0, canvas_w, canvas_w // 8):
        cv2.line(canvas, (col, 0), (col, canvas_h), C_GRID, 1)

    # Car position indicator at the bottom
    car_x = canvas_w // 2
    cv2.arrowedLine(canvas,
                    (car_x, canvas_h - 4),
                    (car_x, canvas_h - 28),
                    (180, 180, 180), 2, tipLength=0.4)
    cv2.putText(canvas, "CAR", (car_x - 16, canvas_h - 6),
                cv2.FONT_HERSHEY_SIMPLEX, 0.38, (160, 160, 160), 1)

    left   = [tuple(pt) for pt in data.get("left",   [])]
    right  = [tuple(pt) for pt in data.get("right",  [])]
    center = [tuple(pt) for pt in data.get("center", [])]

    # ── Corridor fill ─────────────────────────────────────────────────────────
    if len(left) > 1 and len(right) > 1:
        poly = (
            [p(x, y) for x, y in left] +
            [p(x, y) for x, y in reversed(right)]
        )
        overlay = canvas.copy()
        cv2.fillPoly(overlay, [np.array(poly, dtype=np.int32)], C_CORRIDOR)
        canvas = cv2.addWeighted(canvas, 0.6, overlay, 0.4, 0)

    # ── Cross-links (depth alignment between left and right) ─────────────────
    # Re-derive raw pairs from the original strip data embedded in JSON.
    # Since we only have the simplified chains, we draw cross-links between
    # nearest-neighbour pairs by matching y-coordinate.
    def nearest(chain, y_target):
        if not chain:
            return None
        return min(chain, key=lambda pt: abs(pt[1] - y_target))

    if left and right:
        sample_ys = np.linspace(0, bev_h, n_strips, endpoint=False)
        for sy in sample_ys:
            lp = nearest(left,  sy)
            rp = nearest(right, sy)
            if lp and rp:
                cv2.line(canvas, p(*lp), p(*rp), C_CROSSLINK, 1)

    # ── Chains ────────────────────────────────────────────────────────────────
    def draw_chain(chain, col_line, col_dot, thickness=2):
        pts = [p(x, y) for x, y in chain]
        for i in range(1, len(pts)):
            cv2.line(canvas, pts[i - 1], pts[i], col_line, thickness, cv2.LINE_AA)
        for pt in pts:
            cv2.circle(canvas, pt, 5, col_dot, -1)

    draw_chain(left,   C_LEFT,   C_DOT_L)
    draw_chain(right,  C_RIGHT,  C_DOT_R)
    draw_chain(center, C_CENTER, C_CENTER, thickness=2)

    # Direction arrows along centre path
    if len(center) >= 2:
        step = max(1, len(center) // 6)
        for i in range(0, len(center) - 1, step):
            cv2.arrowedLine(canvas,
                            p(*center[i]), p(*center[i + 1]),
                            C_CENTER, 1, tipLength=0.4)

    # ── HUD ──────────────────────────────────────────────────────────────────
    valid  = data.get("valid", {})
    area   = data.get("corridor_area", 0)
    stamp  = data.get("stamp", 0)
    age    = time.time() - stamp if stamp else 0

    hud = [
        f"LEFT  {len(left):>2} pts  {'OK' if valid.get('left')  else 'MISSING'}",
        f"RIGHT {len(right):>2} pts  {'OK' if valid.get('right') else 'MISSING'}",
        f"CTR   {len(center):>2} pts",
        f"Corridor {int(area)} px",
        f"Msg age  {age*1000:.0f} ms",
    ]
    for i, txt in enumerate(hud):
        col = (80, 200, 80) if "OK" in txt else C_TEXT
        cv2.putText(canvas, txt, (8, 18 + i * 18),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.46, col, 1, cv2.LINE_AA)

    # Legend
    legend = [
        (C_LEFT,   "Left chain"),
        (C_RIGHT,  "Right chain"),
        (C_CENTER, "Centre path"),
    ]
    for i, (col, label) in enumerate(legend):
        x0 = canvas_w - 130
        y0 = 18 + i * 18
        cv2.line(canvas, (x0, y0 - 4), (x0 + 20, y0 - 4), col, 2)
        cv2.putText(canvas, label, (x0 + 26, y0),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.40, col, 1, cv2.LINE_AA)

    return canvas


def no_signal_frame(canvas_w, canvas_h, reason="Waiting for /nxp_cup/lane_chains …"):
    canvas = np.full((canvas_h, canvas_w, 3), (10, 10, 30), dtype=np.uint8)
    cv2.putText(canvas, "NO SIGNAL", (canvas_w // 2 - 90, canvas_h // 2 - 14),
                cv2.FONT_HERSHEY_SIMPLEX, 1.2, C_WARN, 2, cv2.LINE_AA)
    cv2.putText(canvas, reason, (canvas_w // 2 - 200, canvas_h // 2 + 18),
                cv2.FONT_HERSHEY_SIMPLEX, 0.48, (120, 120, 180), 1, cv2.LINE_AA)
    return canvas


# ─────────────────────────────────────────────────────────────────────────────
#  MJPEG SERVER
# ─────────────────────────────────────────────────────────────────────────────

_ROUTES = {
    "/":        "rendered",
    "/rendered":"rendered",
    "/raw":     "raw",
    "/both":    "both",
}

def _make_page(port):
    return (
        "HTTP/1.0 200 OK\r\nContent-Type: text/html\r\n\r\n"
        "<!doctype html><html><head><title>NXP Chain Viewer</title>"
        "<style>body{margin:0;background:#0a0a0f;"
        "font-family:monospace;color:#8cf;display:flex;"
        "flex-direction:column;align-items:center;}"
        "h2{margin:8px;font-size:1em;}"
        "img{max-width:100%;}"
        "nav a{color:#8cf;margin:0 12px;text-decoration:none;font-size:.85em;}"
        "</style></head><body>"
        "<h2>NXP Cup – Lane Chain Viewer</h2>"
        "<nav>"
        f"<a href='http://localhost:{port}/'>Rendered</a>"
        f"<a href='http://localhost:{port}/raw'>Raw BEV</a>"
        f"<a href='http://localhost:{port}/both'>Side-by-side</a>"
        "</nav><br>"
        "<img src='/stream'/>"
        "</body></html>"
    ).encode()


class _Handler(BaseHTTPRequestHandler):
    _port = DEFAULT_PORT

    def log_message(self, *a):
        pass

    def do_GET(self):
        route = _ROUTES.get(self.path)

        if route is None and self.path != "/stream":
            self.send_error(404)
            return

        # HTML page for non-stream requests
        if self.path in _ROUTES and self.path != "/stream":
            self.wfile.write(_make_page(self._port))
            return

        # Pick which slot to stream based on Referer or path
        # Default slot is rendered; /raw and /both override
        slot = _slot_rendered
        if "raw"  in self.headers.get("Referer", "") or self.path.endswith("raw"):
            slot = _slot_raw
        elif "both" in self.headers.get("Referer", "") or self.path.endswith("both"):
            slot = _slot_both

        self.send_response(200)
        self.send_header("Content-Type",
                         "multipart/x-mixed-replace; boundary=frame")
        self.send_header("Cache-Control", "no-cache")
        self.end_headers()

        seq = -1
        try:
            while True:
                seq, jpg = slot.get_if_newer(seq)
                if jpg:
                    hdr = (
                        b"--frame\r\nContent-Type: image/jpeg\r\nContent-Length: "
                        + str(len(jpg)).encode() + b"\r\n\r\n"
                    )
                    self.wfile.write(hdr + jpg + b"\r\n")
                    self.wfile.flush()
                else:
                    time.sleep(0.01)
        except (BrokenPipeError, ConnectionResetError):
            pass


def start_server(port):
    _Handler._port = port
    srv = HTTPServer(("0.0.0.0", port), _Handler)
    threading.Thread(target=srv.serve_forever, daemon=True).start()


# ─────────────────────────────────────────────────────────────────────────────
#  ROS 2 NODE
# ─────────────────────────────────────────────────────────────────────────────

class ChainViewerNode(Node):
    def __init__(self, canvas_w, canvas_h, subscribe_raw):
        super().__init__("nxp_chain_viewer")
        self._cw = canvas_w
        self._ch = canvas_h
        self._bridge = CvBridge() if subscribe_raw else None
        self._raw_ok = False

        self.create_subscription(
            String, "/nxp_cup/lane_chains",
            self._on_chains, 10
        )

        if subscribe_raw:
            self.create_subscription(
                Image, "/nxp_cup/debug_image",
                self._on_raw_image, 10
            )

        self.get_logger().info(
            f"Subscribed to /nxp_cup/lane_chains"
            + (" + /nxp_cup/debug_image" if subscribe_raw else "")
        )

    # ── Callbacks ─────────────────────────────────────────────────────────────

    def _on_chains(self, msg: String):
        global _last_chain_t
        _last_chain_t = time.time()

        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError as e:
            self.get_logger().warn(f"JSON parse error: {e}")
            return

        # Rendered visualisation
        rendered = render_chains(data, self._cw, self._ch)
        self._push_rendered(rendered, data)

    def _on_raw_image(self, msg: Image):
        try:
            bgr = self._bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:
            self.get_logger().warn(f"cv_bridge error: {e}")
            return

        self._raw_ok = True
        ok, jpg = cv2.imencode(".jpg", bgr, _enc)
        if ok:
            _slot_raw.push(jpg.tobytes())

        # Update both slot if we have a rendered frame too
        self._update_both(bgr)

    # ── Helpers ───────────────────────────────────────────────────────────────

    def _push_rendered(self, rendered, data):
        ok, jpg = cv2.imencode(".jpg", rendered, _enc)
        if ok:
            _slot_rendered.push(jpg.tobytes())
        # Try to update "both" slot
        self._update_both(None)

    def _update_both(self, raw_bgr_override):
        """Compose rendered + raw side by side and push to _slot_both."""
        # Get latest rendered as numpy (re-render is cheap; we just re-read
        # from the rendered slot by decoding the last JPEG)
        with _slot_rendered._lock:
            r_jpg = _slot_rendered._jpg
        with _slot_raw._lock:
            raw_jpg = _slot_raw._jpg

        if not r_jpg:
            return

        rendered = cv2.imdecode(np.frombuffer(r_jpg, np.uint8), cv2.IMREAD_COLOR)
        if rendered is None:
            return

        if raw_jpg:
            raw = cv2.imdecode(np.frombuffer(raw_jpg, np.uint8), cv2.IMREAD_COLOR)
            if raw is not None:
                # Scale raw to same height as rendered
                rh = rendered.shape[0]
                raw_rw = int(raw.shape[1] * rh / raw.shape[0])
                raw = cv2.resize(raw, (raw_rw, rh))
                both = np.hstack([rendered, raw])
            else:
                both = rendered
        else:
            both = rendered

        ok, jpg = cv2.imencode(".jpg", both, _enc)
        if ok:
            _slot_both.push(jpg.tobytes())


# ─────────────────────────────────────────────────────────────────────────────
#  STALE-SIGNAL WATCHDOG
# ─────────────────────────────────────────────────────────────────────────────

def _watchdog(canvas_w, canvas_h):
    """Push a 'no signal' frame if no chain message arrives for STALE_TIMEOUT."""
    while True:
        time.sleep(0.5)
        if time.time() - _last_chain_t > STALE_TIMEOUT:
            frame = no_signal_frame(canvas_w, canvas_h)
            ok, jpg = cv2.imencode(".jpg", frame, _enc)
            if ok:
                jpg_bytes = jpg.tobytes()
                _slot_rendered.push(jpg_bytes)
                _slot_both.push(jpg_bytes)


# ─────────────────────────────────────────────────────────────────────────────
#  MAIN
# ─────────────────────────────────────────────────────────────────────────────

def parse_args():
    p = argparse.ArgumentParser(
        description="NXP Cup lane-chain viewer – MJPEG stream over SSH"
    )
    p.add_argument("--port",   type=int, default=DEFAULT_PORT,
                   help=f"HTTP port (default {DEFAULT_PORT})")
    p.add_argument("--scale",  type=int, default=DEFAULT_SCALE,
                   help="Canvas scale factor relative to BEV (default 2)")
    p.add_argument("--no-raw", action="store_true",
                   help="Don't subscribe to debug_image topic")
    return p.parse_args()


def main():
    args    = parse_args()
    bev_w   = 400
    bev_h   = 300
    canvas_w = bev_w * args.scale
    canvas_h = bev_h * args.scale

    # Start MJPEG server
    start_server(args.port)
    print(f"\n[Viewer]  http://<navqplus-ip>:{args.port}")
    print(f"          Routes:  /  (rendered)  /raw  /both")
    print(f"          Find IP: ip addr show | grep 'inet '\n")

    # Pre-fill with no-signal frame so the browser shows something immediately
    ns = no_signal_frame(canvas_w, canvas_h)
    ok, jpg = cv2.imencode(".jpg", ns, _enc)
    if ok:
        b = jpg.tobytes()
        _slot_rendered.push(b)
        _slot_both.push(b)

    # Stale-signal watchdog
    threading.Thread(
        target=_watchdog, args=(canvas_w, canvas_h), daemon=True
    ).start()

    # ROS 2
    rclpy.init()
    node = ChainViewerNode(canvas_w, canvas_h,
                           subscribe_raw=not args.no_raw)

    print("[Running]  Ctrl-C to stop\n")
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\n[Stopped]")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()