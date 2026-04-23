#!/usr/bin/env python3
"""
NXP Cup – NavQPlus  Full Track Vision Node
==========================================
Replaces the Pixy-camera single-vector model with a full lane-chain pipeline:

  CSI camera (GStreamer)
        |
  Bird's-eye warp  (perspective transform)
        |
  Track segmentation  (white-line isolation, corridor mask)
        |
  Strip-centroid chain extraction  (N strips, bottom→top)
        |
  Douglas-Peucker simplification  (compact polyline)
        |
  ROS 2 publishers
        ├─ /nxp_cup/lane_chains     std_msgs/String  (JSON, primary output)
        ├─ /nxp_cup/left_path       nav_msgs/Path
        ├─ /nxp_cup/right_path      nav_msgs/Path
        ├─ /nxp_cup/center_path     nav_msgs/Path    (midline, ready for Pure Pursuit)
        └─ /nxp_cup/debug_image     sensor_msgs/Image
  MJPEG HTTP stream on :8080   (browser debug view, no display needed)

JSON message schema
-------------------
{
  "stamp":      float,          # seconds since epoch
  "bev_w":      int,            # BEV canvas width  (pixels)
  "bev_h":      int,            # BEV canvas height (pixels)
  "n_strips":   int,            # number of depth slices
  "left":  [[x,y], ...],        # chain, index 0 = nearest, index N = farthest
  "right": [[x,y], ...],        # same ordering, same strip indices
  "center":[[x,y], ...],        # midpoint between paired left/right points
  "valid": {"left": bool, "right": bool},
  "corridor_area": float        # pixel area of segmented track corridor
}

Point coordinates are in BEV pixel space (origin = top-left, y grows downward).
Corresponding indices across left/right/center arrays sit at the same strip depth,
so the receiver can directly zip() them to build cross-track segments.

Run
---
    python3 nxp_track_vision.py [--device /dev/video3] [--port 8080] [--flip]
                                [--calibrate] [--no-ros]

    --no-ros    Run without ROS 2 (MJPEG stream + stdout JSON only).
                Useful for initial tuning without a full ROS 2 workspace.

Requirements
------------
    sudo apt install python3-opencv python3-numpy
    # For ROS 2 publishing:
    source /opt/ros/humble/setup.bash   (or foxy / iron)
"""

# ── stdlib ────────────────────────────────────────────────────────────────────
import argparse
import json
import sys
import threading
import time
import math
import os

# ── third-party ───────────────────────────────────────────────────────────────
import cv2
import numpy as np

# ── ROS 2  (optional – graceful fallback) ─────────────────────────────────────
try:
    import rclpy
    from rclpy.node import Node
    from rclpy.clock import Clock
    from std_msgs.msg     import String
    from nav_msgs.msg     import Path
    from geometry_msgs.msg import PoseStamped
    from sensor_msgs.msg  import Image
    from cv_bridge        import CvBridge
    _ROS_AVAILABLE = True
except ImportError:
    _ROS_AVAILABLE = False

from http.server import BaseHTTPRequestHandler, HTTPServer

# ═════════════════════════════════════════════════════════════════════════════
#  CONFIGURATION  ── edit here or use CLI
# ═════════════════════════════════════════════════════════════════════════════

# Camera ──────────────────────────────────────────────────────────────────────
CAM_DEVICE   = "/dev/video3"    # v4l2-ctl --list-devices  to confirm
CAM_W, CAM_H = 640, 480
CAM_FPS      = 30

# Bird's-eye-view (BEV) ───────────────────────────────────────────────────────
# Source trapezoid in raw-image pixels  [BL, BR, TR, TL]
# Run with  --calibrate  to set these interactively.
BEV_SRC = np.float32([
    [160, 480],
    [480, 480],
    [540, 280],
    [100, 280],
])
BEV_W, BEV_H = 400, 300           # output canvas

BEV_DST = np.float32([
    [0,     BEV_H],
    [BEV_W, BEV_H],
    [BEV_W, 0    ],
    [0,     0    ],
])

# Track segmentation ──────────────────────────────────────────────────────────
# Tune BRIGHT_THRESH if your track tape is grey rather than bright white.
BRIGHT_THRESH  = 110
# Minimum contiguous white-pixel columns to count as a line in a strip
MIN_LINE_COLS  = 3

# Chain extraction ────────────────────────────────────────────────────────────
N_STRIPS         = 24       # depth slices across BEV (more = finer resolution)
DP_EPSILON       = 4.0      # Douglas-Peucker simplification tolerance (pixels)
# Half-width (pixels in BEV) around last known line position to search
TRACK_SEARCH_W   = 60

# Processing resolution ───────────────────────────────────────────────────────
# Segmentation runs on a downscaled BEV; results are scaled back up.
PROC_SCALE = 0.5            # 0.5 → 200×150  (fast on ARM)

# MJPEG ───────────────────────────────────────────────────────────────────────
MJPEG_PORT    = 8080
JPEG_QUALITY  = 70

# ROS 2 ───────────────────────────────────────────────────────────────────────
ROS_NODE_NAME = "nxp_track_vision"
ROS_FRAME_ID  = "bev_camera"


# ═════════════════════════════════════════════════════════════════════════════
#  CAMERA
# ═════════════════════════════════════════════════════════════════════════════

def open_camera(device, w, h, fps):
    pipeline = (
        f"v4l2src device={device} ! "
        f"video/x-raw,framerate={fps}/1,width={w},height={h} ! "
        f"videoconvert ! video/x-raw,format=BGR ! appsink drop=1"
    )
    cap = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)
    if not cap.isOpened():
        print("[WARN] GStreamer failed, falling back to direct V4L2")
        cap = cv2.VideoCapture(device)
        cap.set(cv2.CAP_PROP_FRAME_WIDTH,  w)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, h)
        cap.set(cv2.CAP_PROP_FPS,          fps)
    if not cap.isOpened():
        print(f"[ERROR] Cannot open {device}")
        sys.exit(1)
    return cap


# ═════════════════════════════════════════════════════════════════════════════
#  PERSPECTIVE / BEV
# ═════════════════════════════════════════════════════════════════════════════

def build_warp(src, dst):
    M     = cv2.getPerspectiveTransform(src, dst)
    M_inv = cv2.getPerspectiveTransform(dst, src)
    return M, M_inv

def warp(frame, M):
    return cv2.warpPerspective(frame, M, (BEV_W, BEV_H))


# ═════════════════════════════════════════════════════════════════════════════
#  TRACK SEGMENTATION
# ═════════════════════════════════════════════════════════════════════════════

_bright_thresh_runtime = None  # overridden by camera_config.json

def segment_lines(bev_bgr):
    """
    Returns a binary mask (uint8, same HxW as input) where white pixels
    represent detected track-line candidates.

    Strategy: combine a global brightness threshold with an adaptive one to
    handle uneven lighting, then clean up with morphology.
    """
    gray = cv2.cvtColor(bev_bgr, cv2.COLOR_BGR2GRAY)

    # Global: catch clearly bright tape
    _t = _bright_thresh_runtime if _bright_thresh_runtime is not None else BRIGHT_THRESH
    _, bright = cv2.threshold(gray, _t, 255, cv2.THRESH_BINARY)

    # Adaptive: catch tape that's bright relative to its local surroundings
    adapt = cv2.adaptiveThreshold(
        gray, 255,
        cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY,
        blockSize=21, C=-8
    )

    mask = cv2.bitwise_or(bright, adapt)

    # Remove tiny speckles, close gaps within a line
    k3 = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
    k5 = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 3))   # wider than tall
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN,  k3, iterations=1)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, k5, iterations=2)
    return mask


def build_corridor_mask(left_chain, right_chain, h, w):
    """
    Fill the polygon between the two chains to produce a corridor mask.
    Returns a uint8 mask or None if either chain is empty.
    """
    if len(left_chain) < 2 or len(right_chain) < 2:
        return None
    # Polygon: go up the left side, down the right side
    poly = (
        [(int(x), int(y)) for x, y in left_chain] +
        [(int(x), int(y)) for x, y in reversed(right_chain)]
    )
    mask = np.zeros((h, w), dtype=np.uint8)
    cv2.fillPoly(mask, [np.array(poly, dtype=np.int32)], 255)
    return mask


# ═════════════════════════════════════════════════════════════════════════════
#  CHAIN EXTRACTION
# ═════════════════════════════════════════════════════════════════════════════

def _histogram_base(binary):
    """Find initial left/right x positions from bottom-half histogram."""
    h, w = binary.shape
    hist = np.sum(binary[h // 2:, :], axis=0).astype(np.float32)
    mid  = w // 2
    lx   = int(np.argmax(hist[:mid]))       if hist[:mid].max()  > 0 else mid // 2
    rx   = int(np.argmax(hist[mid:]) + mid) if hist[mid:].max()  > 0 else mid + mid // 2
    return lx, rx


def extract_chains(binary, n_strips=N_STRIPS):
    """
    Divide the BEV binary image into n_strips horizontal bands.
    For each band, find the centroid of white pixels on the left side
    and the centroid on the right side, tracking the boundary between
    them adaptively as the lines curve.

    Returns:
        left_chain  : list of (x, y) ordered near→far (index 0 = bottom)
        right_chain : list of (x, y) ordered near→far
    Both lists have the same length; a None entry means that side was
    not visible in that strip.
    """
    h, w      = binary.shape
    strip_h   = max(1, h // n_strips)

    lx, rx    = _histogram_base(binary)
    left_raw  = []
    right_raw = []

    for i in range(n_strips - 1, -1, -1):   # bottom strip first
        y0 = i * strip_h
        y1 = min(y0 + strip_h, h)
        cy = (y0 + y1) // 2
        strip = binary[y0:y1, :]

        # Adaptive split point – midpoint of last known l/r positions
        split = (lx + rx) // 2

        # Search within ±TRACK_SEARCH_W of last known position
        l_lo = max(0,    lx - TRACK_SEARCH_W)
        l_hi = min(split, lx + TRACK_SEARCH_W)
        r_lo = max(split, rx - TRACK_SEARCH_W)
        r_hi = min(w,     rx + TRACK_SEARCH_W)

        left_cols  = np.where(strip[:, l_lo:l_hi].max(axis=0) > 0)[0]
        right_cols = np.where(strip[:, r_lo:r_hi].max(axis=0) > 0)[0]

        lpt = rpt = None

        if len(left_cols) >= MIN_LINE_COLS:
            cx = l_lo + int(np.median(left_cols))
            lpt = (cx, cy)
            lx  = cx                         # track for next strip

        if len(right_cols) >= MIN_LINE_COLS:
            cx = r_lo + int(np.median(right_cols))
            rpt = (cx, cy)
            rx  = cx

        left_raw.append(lpt)
        right_raw.append(rpt)

    return left_raw, right_raw


def _douglas_peucker(points, epsilon):
    """
    Thin wrapper around cv2.approxPolyDP for a list of (x,y) tuples.
    Returns a simplified list of (x, y).
    """
    if len(points) < 2:
        return points
    arr = np.array(points, dtype=np.float32).reshape(-1, 1, 2)
    simplified = cv2.approxPolyDP(arr, epsilon, closed=False)
    return [(int(p[0][0]), int(p[0][1])) for p in simplified]


def build_paired_chains(left_raw, right_raw, bev_w, bev_h):
    """
    From the raw per-strip optional points, produce:
      - left_chain, right_chain : lists of (x, y), Nones removed, simplified
      - center_chain            : midpoints of corresponding valid pairs
      - paired_left, paired_right: same-length lists aligned by strip index,
                                   None where a side is missing

    All coordinates are in full BEV pixel space.
    """
    # --- scale back up if we ran on a downscaled image ---
    scale = 1.0 / PROC_SCALE

    def scale_pt(pt):
        if pt is None:
            return None
        return (int(pt[0] * scale), int(pt[1] * scale))

    left_raw  = [scale_pt(p) for p in left_raw]
    right_raw = [scale_pt(p) for p in right_raw]

    # Clamp to BEV bounds
    def clamp(pt):
        if pt is None:
            return None
        return (max(0, min(bev_w - 1, pt[0])),
                max(0, min(bev_h - 1, pt[1])))

    left_raw  = [clamp(p) for p in left_raw]
    right_raw = [clamp(p) for p in right_raw]

    # Build centre chain from corresponding pairs
    center_raw = []
    for lp, rp in zip(left_raw, right_raw):
        if lp is not None and rp is not None:
            cx = (lp[0] + rp[0]) // 2
            cy = (lp[1] + rp[1]) // 2
            center_raw.append((cx, cy))
        else:
            center_raw.append(None)

    def compact(chain):
        """Drop Nones and simplify."""
        pts = [p for p in chain if p is not None]
        if len(pts) < 2:
            return pts
        return _douglas_peucker(pts, DP_EPSILON)

    left_chain   = compact(left_raw)
    right_chain  = compact(right_raw)
    center_chain = compact(center_raw)

    return left_chain, right_chain, center_chain, left_raw, right_raw


# ═════════════════════════════════════════════════════════════════════════════
#  MESSAGE BUILDING
# ═════════════════════════════════════════════════════════════════════════════

def build_json(left_chain, right_chain, center_chain,
               corridor_area, stamp):
    """
    Build the primary JSON payload.

    Coordinate convention:
        Origin = top-left of BEV canvas.
        +x = right,  +y = down (image convention).
        Index 0 of every chain = nearest strip (bottom of BEV = front of car).
    """
    return json.dumps({
        "stamp":          stamp,
        "bev_w":          BEV_W,
        "bev_h":          BEV_H,
        "n_strips":       N_STRIPS,
        "dp_epsilon":     DP_EPSILON,
        "left":           left_chain,
        "right":          right_chain,
        "center":         center_chain,
        "valid": {
            "left":  len(left_chain)   > 1,
            "right": len(right_chain)  > 1,
        },
        "corridor_area":  corridor_area,
    })


def _chain_to_path(chain, stamp_sec, frame_id):
    """Convert a list of (x,y) BEV pixels to a nav_msgs/Path."""
    msg = Path()
    msg.header.frame_id = frame_id
    # stamp set by caller
    for x, y in chain:
        ps = PoseStamped()
        ps.header.frame_id = frame_id
        # Store BEV pixels in the x,y fields of position; z=0
        ps.pose.position.x = float(x)
        ps.pose.position.y = float(y)
        ps.pose.position.z = 0.0
        ps.pose.orientation.w = 1.0
        msg.poses.append(ps)
    return msg


# ═════════════════════════════════════════════════════════════════════════════
#  DEBUG VISUALISATION
# ═════════════════════════════════════════════════════════════════════════════

def draw_debug(bev_bgr, line_mask, left_raw, right_raw,
               left_chain, right_chain, center_chain,
               corridor_mask, vectors_json, fps):
    vis = bev_bgr.copy()

    # Tint segmented line pixels
    tint = np.zeros_like(vis)
    tint[line_mask > 0] = (0, 220, 80)          # green = detected line pixels
    vis = cv2.addWeighted(vis, 1.0, tint, 0.35, 0)

    # Corridor fill
    if corridor_mask is not None:
        fill = np.zeros_like(vis)
        fill[corridor_mask > 0] = (255, 180, 0) # cyan-ish corridor
        vis = cv2.addWeighted(vis, 1.0, fill, 0.20, 0)

    # Raw per-strip dots (small, semi-transparent intention markers)
    for pt in left_raw:
        if pt:
            cv2.circle(vis, pt, 3, (120, 120, 255), -1)
    for pt in right_raw:
        if pt:
            cv2.circle(vis, pt, 3, (255, 120, 120), -1)

    # Simplified chains
    def draw_chain(chain, colour, thickness=2):
        for i in range(1, len(chain)):
            cv2.line(vis, chain[i - 1], chain[i], colour, thickness, cv2.LINE_AA)
        for pt in chain:
            cv2.circle(vis, pt, 4, colour, -1)

    draw_chain(left_chain,   (60,  60,  255), 2)   # blue  = left
    draw_chain(right_chain,  (255, 60,  60),  2)   # red   = right
    draw_chain(center_chain, (60,  255, 200), 2)   # green = centre

    # Cross-links between paired strips (shows "linked" structure)
    for lp, rp in zip(left_raw, right_raw):
        if lp and rp:
            cv2.line(vis, lp, rp, (200, 200, 80), 1)

    # HUD
    d = json.loads(vectors_json) if vectors_json else {}
    valid_l = d.get("valid", {}).get("left",  False)
    valid_r = d.get("valid", {}).get("right", False)
    area    = d.get("corridor_area", 0)
    nl      = len(d.get("left",   []))
    nr      = len(d.get("right",  []))
    nc      = len(d.get("center", []))

    lines = [
        f"FPS {fps:.1f}",
        f"L={nl}pts {'OK' if valid_l else '--'}  "
        f"R={nr}pts {'OK' if valid_r else '--'}  "
        f"C={nc}pts",
        f"Corridor area: {int(area)} px",
    ]
    for i, txt in enumerate(lines):
        cv2.putText(vis, txt, (8, 18 + i * 19),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.50,
                    (240, 240, 240), 1, cv2.LINE_AA)
    return vis


def draw_raw_overlay(raw_frame, src_pts, fps):
    """Draw the BEV source trapezoid on the raw camera frame."""
    pts = src_pts.astype(np.int32).reshape((-1, 1, 2))
    cv2.polylines(raw_frame, [pts], True, (0, 220, 220), 2)
    labels = ["BL", "BR", "TR", "TL"]
    for i, (x, y) in enumerate(src_pts.astype(int)):
        cv2.circle(raw_frame, (x, y), 5, (0, 100, 255), -1)
        cv2.putText(raw_frame, labels[i], (x + 5, y - 5),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 220, 220), 1)
    cv2.putText(raw_frame, f"RAW  FPS {fps:.1f}", (8, 18),
                cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 0), 1, cv2.LINE_AA)


# ═════════════════════════════════════════════════════════════════════════════
#  MJPEG HTTP SERVER
# ═════════════════════════════════════════════════════════════════════════════

class _LatestFrame:
    def __init__(self):
        self._lock  = threading.Lock()
        self._data  = b""
        self._seq   = 0

    def push(self, jpg_bytes):
        with self._lock:
            self._data = jpg_bytes
            self._seq += 1

    def get_if_newer(self, last_seq):
        with self._lock:
            if self._seq != last_seq:
                return self._seq, self._data
        return last_seq, None


_latest_frame = _LatestFrame()

_PAGE = (
    "HTTP/1.0 200 OK\r\nContent-Type: text/html\r\n\r\n"
    "<!doctype html><html><head><title>NXP Track Vision</title>"
    "<style>body{margin:0;background:#0a0a0a;display:flex;"
    "flex-direction:column;align-items:center;font-family:monospace;color:#0f0;}"
    "h2{margin:8px;}img{max-width:100%;border:1px solid #0f0;}"
    "p{font-size:0.8em;color:#888;}</style></head><body>"
    "<h2>NavQPlus Track Vision</h2>"
    "<img src='/stream'/>"
    "<p>LEFT (blue) | RIGHT (red) | CENTRE (green) | CORRIDOR (cyan tint)"
    " | CROSS-LINKS (yellow)</p>"
    "</body></html>"
).encode()


class _MJPEGHandler(BaseHTTPRequestHandler):
    def log_message(self, *args):
        pass

    def do_GET(self):
        if self.path == "/":
            self.wfile.write(_PAGE)
            return
        if self.path == "/stream":
            self.send_response(200)
            self.send_header("Content-Type",
                             "multipart/x-mixed-replace; boundary=frame")
            self.send_header("Cache-Control", "no-cache")
            self.end_headers()
            seq = -1
            try:
                while True:
                    seq, jpg = _latest_frame.get_if_newer(seq)
                    if jpg:
                        hdr = (
                            b"--frame\r\nContent-Type: image/jpeg\r\nContent-Length: "
                            + str(len(jpg)).encode()
                            + b"\r\n\r\n"
                        )
                        self.wfile.write(hdr + jpg + b"\r\n")
                        self.wfile.flush()
                    else:
                        time.sleep(0.01)
            except (BrokenPipeError, ConnectionResetError):
                pass
            return
        self.send_error(404)


def start_mjpeg(port):
    srv = HTTPServer(("0.0.0.0", port), _MJPEGHandler)
    threading.Thread(target=srv.serve_forever, daemon=True).start()
    print(f"[MJPEG]   http://<navqplus-ip>:{port}")
    print(f"          ip addr show | grep 'inet '")


# ═════════════════════════════════════════════════════════════════════════════
#  BEV CALIBRATION  (terminal-based, no GUI needed)
# ═════════════════════════════════════════════════════════════════════════════

def calibrate(frame):
    path = "calib_frame.jpg"
    cv2.imwrite(path, frame)
    print(f"\n[Calibration] Snapshot saved → {path}")
    print("  Open it in VS Code Explorer (or scp to your laptop).")
    print("  Click 4 points on the image.  Enter them below.\n")
    print("  Order: Bottom-Left, Bottom-Right, Top-Right, Top-Left\n")
    pts = []
    for i, label in enumerate(["Bottom-Left", "Bottom-Right",
                                "Top-Right",   "Top-Left"]):
        while True:
            raw = input(f"  Point {i} ({label})  x,y : ").strip()
            if raw == "":
                print("  Skipped – keeping current BEV_SRC.")
                return BEV_SRC.copy()
            try:
                x, y = [int(v.strip()) for v in raw.split(",")]
                pts.append([x, y])
                break
            except ValueError:
                print("  Format: x,y  e.g.  160,480")
    new = np.float32(pts)
    print(f"\n  New BEV_SRC:\n  {new.tolist()}")
    print("  Paste into BEV_SRC at the top of this script to persist.\n")
    return new


# ═════════════════════════════════════════════════════════════════════════════
#  ROS 2 NODE
# ═════════════════════════════════════════════════════════════════════════════

class TrackVisionNode(Node):
    def __init__(self):
        super().__init__(ROS_NODE_NAME)
        qos = 10
        self.pub_json   = self.create_publisher(String, "/nxp_cup/lane_chains",  qos)
        self.pub_left   = self.create_publisher(Path,   "/nxp_cup/left_path",    qos)
        self.pub_right  = self.create_publisher(Path,   "/nxp_cup/right_path",   qos)
        self.pub_center = self.create_publisher(Path,   "/nxp_cup/center_path",  qos)
        self.pub_img    = self.create_publisher(Image,  "/nxp_cup/debug_image",  qos)
        self.bridge     = CvBridge()
        self.get_logger().info("TrackVisionNode ready")

    def publish(self, json_str, left_chain, right_chain, center_chain, debug_bgr):
        # JSON (primary)
        msg = String()
        msg.data = json_str
        self.pub_json.publish(msg)

        # Paths
        now = self.get_clock().now().to_msg()
        for chain, pub in [
            (left_chain,   self.pub_left),
            (right_chain,  self.pub_right),
            (center_chain, self.pub_center),
        ]:
            if len(chain) > 0:
                path = _chain_to_path(chain, 0, ROS_FRAME_ID)
                path.header.stamp = now
                pub.publish(path)

        # Debug image
        img_msg = self.bridge.cv2_to_imgmsg(debug_bgr, encoding="bgr8")
        img_msg.header.stamp = now
        self.pub_img.publish(img_msg)


# ═════════════════════════════════════════════════════════════════════════════
#  MAIN LOOP
# ═════════════════════════════════════════════════════════════════════════════

def parse_args():
    import rclpy.utilities
    argv = rclpy.utilities.remove_ros_args(sys.argv[1:])
    p = argparse.ArgumentParser()
    p.add_argument("--device",     default=CAM_DEVICE)
    p.add_argument("--width",      type=int, default=CAM_W)
    p.add_argument("--height",     type=int, default=CAM_H)
    p.add_argument("--fps",        type=int, default=CAM_FPS)
    p.add_argument("--port",       type=int, default=MJPEG_PORT)
    p.add_argument("--no-flip",    action="store_true",
                   help="Disable the default 180 degree flip")
    p.add_argument("--calibrate",  action="store_true")
    p.add_argument("--no-ros",     action="store_true",
                   help="Disable ROS 2 publishing (MJPEG + stdout only)")
    p.add_argument("--sim",        action="store_true",
                   help="Sim mode: read from /camera/image_raw instead of CSI")
    p.add_argument("--jpeg-quality", type=int, default=JPEG_QUALITY)
    return p.parse_args(argv)   # ← replace p.parse_args() with this


def main():
    args = parse_args()

    use_ros = _ROS_AVAILABLE and not args.no_ros
    ros_node = None

    if use_ros:
        rclpy.init()
        ros_node = TrackVisionNode()
        ros_executor = rclpy.executors.SingleThreadedExecutor()
        ros_executor.add_node(ros_node)
        ros_thread = threading.Thread(target=ros_executor.spin, daemon=True)
        ros_thread.start()
        print("[ROS2] Node started")
    else:
        if not _ROS_AVAILABLE:
            print("[INFO] rclpy not found – running in MJPEG-only mode")
        else:
            print("[INFO] --no-ros flag set – ROS 2 publishing disabled")

    # ── Camera source: real CSI or sim ROS topic ─────────────────────────────
    _sim_frame      = None          # filled by ROS subscriber in sim mode
    _sim_frame_lock = threading.Lock()

    if args.sim:
        if not _ROS_AVAILABLE:
            print("[ERROR] --sim requires rclpy. Source your ROS 2 workspace.")
            sys.exit(1)

        from sensor_msgs.msg import Image as _ImageMsg
        from cv_bridge import CvBridge as _CvBridge
        _bridge = _CvBridge()

        class _SimCamNode(rclpy.node.Node):
            def __init__(self):
                super().__init__("nxp_track_vision_sim_cam")
                self.create_subscription(
                    _ImageMsg, "/camera/image_raw",
                    self._cb, 10)
            def _cb(self, msg):
                try:
                    bgr = _bridge.imgmsg_to_cv2(msg, "bgr8")
                    with _sim_frame_lock:
                        nonlocal _sim_frame
                        _sim_frame = bgr
                except Exception as e:
                    pass

        _sim_cam_node = _SimCamNode()
        if use_ros:
            ros_executor.add_node(_sim_cam_node)
        else:
            rclpy.init()
            _sim_cam_exec = rclpy.executors.SingleThreadedExecutor()
            _sim_cam_exec.add_node(_sim_cam_node)
            threading.Thread(target=_sim_cam_exec.spin, daemon=True).start()

        cap = None
        print("[Camera] Sim mode — subscribing to /camera/image_raw")
    else:
        cap = open_camera(args.device, args.width, args.height, args.fps)

    # ── Load camera_config.json if present (written by nxp_cam_init.py) ──────
    src_pts = BEV_SRC.copy()
    bright_thresh_override = None
    _cfg_path = "camera_config.json"
    if os.path.exists(_cfg_path):
        try:
            import json as _json
            with open(_cfg_path) as _f:
                _cfg = _json.load(_f)
            if "bev_src" in _cfg:
                src_pts = np.float32(_cfg["bev_src"])
            if "bright_thresh" in _cfg:
                bright_thresh_override = int(_cfg["bright_thresh"])
            if "flip" in _cfg and not args.no_flip:
                # config flip only applied if user hasn't passed --no-flip
                if not _cfg["flip"]:
                    args.no_flip = True   # config says no flip
            print(f"[Config] Loaded {_cfg_path}  "
                  f"thresh={bright_thresh_override}  flip={not args.no_flip}")
        except Exception as _e:
            print(f"[Config] Failed to load {_cfg_path}: {_e}")
    else:
        print(f"[Config] No camera_config.json found — using script defaults")

    global _bright_thresh_runtime
    _bright_thresh_runtime = bright_thresh_override
    M, M_inv = build_warp(src_pts, BEV_DST)

    if args.calibrate:
        ret, frame = cap.read()
        if ret:
            if not args.no_flip:
                frame = cv2.rotate(frame, cv2.ROTATE_180)
            src_pts  = calibrate(frame)
            M, M_inv = build_warp(src_pts, BEV_DST)

    start_mjpeg(args.port)

    enc_params  = [cv2.IMWRITE_JPEG_QUALITY, args.jpeg_quality]
    prev_t      = time.time()
    fps_disp    = 0.0
    frame_count = 0

    print("\n[Running]  Ctrl-C to stop\n")

    try:
        while True:
            if args.sim:
                with _sim_frame_lock:
                    frame = _sim_frame
                if frame is None:
                    time.sleep(0.02)
                    continue
            else:
                ret, frame = cap.read()
                if not ret:
                    time.sleep(0.02)
                    continue
                if not args.no_flip:
                    frame = cv2.rotate(frame, cv2.ROTATE_180)

            # ── FPS ───────────────────────────────────────────────────────────
            now      = time.time()
            fps_disp = 0.9 * fps_disp + 0.1 / max(now - prev_t, 1e-6)
            prev_t   = now
            frame_count += 1

            # ── BEV ───────────────────────────────────────────────────────────
            bev = warp(frame, M)

            # ── Segmentation on downscaled image ─────────────────────────────
            ph = max(1, int(BEV_H * PROC_SCALE))
            pw = max(1, int(BEV_W * PROC_SCALE))
            bev_small   = cv2.resize(bev, (pw, ph))
            line_mask_s = segment_lines(bev_small)

            # ── Chain extraction (in small-image coords) ──────────────────────
            left_raw_s, right_raw_s = extract_chains(line_mask_s, N_STRIPS)

            # ── Build chains (scales back to full BEV coords internally) ──────
            left_chain, right_chain, center_chain, left_raw_full, right_raw_full = \
                build_paired_chains(left_raw_s, right_raw_s, BEV_W, BEV_H)

            # ── Corridor mask ─────────────────────────────────────────────────
            corridor_mask = build_corridor_mask(left_chain, right_chain, BEV_H, BEV_W)
            corridor_area = float(np.count_nonzero(corridor_mask)) \
                            if corridor_mask is not None else 0.0

            # ── Full-res line mask for visualisation ──────────────────────────
            line_mask_full = cv2.resize(line_mask_s, (BEV_W, BEV_H),
                                        interpolation=cv2.INTER_NEAREST)

            # ── JSON payload ──────────────────────────────────────────────────
            stamp    = time.time()
            json_str = build_json(
                left_chain, right_chain, center_chain,
                corridor_area, stamp
            )

            # ── ROS 2 ─────────────────────────────────────────────────────────
            if use_ros and ros_node is not None:
                # Build a downscaled debug image for the ROS topic to save bandwidth
                debug_bev = draw_debug(
                    bev, line_mask_full,
                    left_raw_full, right_raw_full,
                    left_chain, right_chain, center_chain,
                    corridor_mask, json_str, fps_disp
                )
                ros_node.publish(json_str, left_chain, right_chain,
                                 center_chain, debug_bev)

            # ── Debug composite for MJPEG ─────────────────────────────────────
            raw_vis = frame.copy()
            draw_raw_overlay(raw_vis, src_pts, fps_disp)

            bev_vis = draw_debug(
                bev, line_mask_full,
                left_raw_full, right_raw_full,
                left_chain, right_chain, center_chain,
                corridor_mask, json_str, fps_disp
            )

            # Scale BEV panel to match raw frame height
            rh, rw = raw_vis.shape[:2]
            bev_vis = cv2.resize(bev_vis,
                                 (int(BEV_W * rh / BEV_H), rh))
            composite = np.hstack([raw_vis, bev_vis])

            ok, jpg = cv2.imencode(".jpg", composite, enc_params)
            if ok:
                _latest_frame.push(jpg.tobytes())

            # ── stdout summary every 60 frames ────────────────────────────────
            if frame_count % 60 == 0:
                d = json.loads(json_str)
                print(
                    f"fps={fps_disp:.1f}  "
                    f"L={len(d['left'])}pts  "
                    f"R={len(d['right'])}pts  "
                    f"C={len(d['center'])}pts  "
                    f"corridor={int(corridor_area)}px"
                )

    except KeyboardInterrupt:
        print("\n[Stopped]")
    finally:
        if cap is not None:
            cap.release()
        if use_ros:
            rclpy.shutdown()


if __name__ == "__main__":
    main()