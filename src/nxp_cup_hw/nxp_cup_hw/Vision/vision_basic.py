#!/usr/bin/env python3
"""
NXP Cup – NavQPlus CSI Camera Lane Vision
==========================================
Standalone (no ROS2) script that:
  1. Captures the CSI camera via GStreamer (NavQPlus / Google Coral cam)
  2. Applies a bird's-eye-view (BEV) perspective warp
  3. Detects white/bright track lines using adaptive threshold + sliding windows
  4. Extracts NXP-Cup-style Pixy vectors (m0 = left, m1 = right) in the
     official 78 × 51 coordinate space used by the CogniPilot follow_line()
  5. Streams a live side-by-side debug view via MJPEG HTTP server

Usage:
    python3 nxp_cup_vision.py [--device /dev/video3] [--width 640] [--height 480]
                              [--fps 30] [--port 8080] [--flip]
                              [--save-video out.mp4]

Then open in your laptop browser:
    http://<navqplus-ip>:8080

Find the NavQPlus IP with:  ip addr show | grep "inet "

Dependencies (already on NavQPlus Ubuntu):
    sudo apt install python3-opencv python3-numpy
"""

import cv2
import numpy as np
import argparse
import sys
import time
import threading
from http.server import BaseHTTPRequestHandler, HTTPServer

# ─────────────────────────────────────────────────────────────────────────────
#  CONFIGURATION  (edit here OR use CLI flags)
# ─────────────────────────────────────────────────────────────────────────────

# Camera device node.  NavQPlus Google Coral cam → typically /dev/video3
# Run:  v4l2-ctl --list-devices   to confirm yours.
DEFAULT_DEVICE   = "/dev/video3"
DEFAULT_WIDTH    = 640
DEFAULT_HEIGHT   = 480
DEFAULT_FPS      = 30

# ── Bird's-eye-view warp ──────────────────────────────────────────────────────
# These four SOURCE points (in raw-image pixel coords) define the trapezoidal
# region of the road ahead that gets warped into a top-down rectangle.
# Tune them with the interactive calibrator (press 'c' at runtime).
#
# Layout (bottom-left → bottom-right → top-right → top-left):
#
#          TL ─────── TR        ← horizon / far edge
#         /               \
#       BL ─────────────── BR   ← near edge (car front)
#
# For a 640×480 frame with the camera roughly 20 cm above the track:
BEV_SRC_POINTS = np.float32([
    [160, 480],   # Bottom-left
    [480, 480],   # Bottom-right
    [540, 280],   # Top-right
    [100, 280],   # Top-left
])

# DESTINATION rectangle in the warped (BEV) image.
BEV_W, BEV_H = 400, 300          # BEV canvas size (pixels)
BEV_DST_POINTS = np.float32([
    [0,       BEV_H],             # Bottom-left
    [BEV_W,   BEV_H],             # Bottom-right
    [BEV_W,   0    ],             # Top-right
    [0,       0    ],             # Top-left
])

# ── NXP Cup Pixy vector coordinate space ─────────────────────────────────────
# CogniPilot's follow_line() uses a 78 × 51 virtual frame.
# All detected vectors are reported in this normalised space.
FRAME_W = 78
FRAME_H = 51

# ── Lane detection ────────────────────────────────────────────────────────────
# Number of sliding windows stacked vertically
N_WINDOWS    = 9
WIN_MARGIN   = 40          # half-width of each window (pixels in BEV space)
MIN_PIX      = 30          # minimum pixels to re-centre a window

# Brightness threshold for white-line detection (0-255).
# Reduce if the track tape is grey rather than bright white.
BRIGHT_THRESH = 120


# ─────────────────────────────────────────────────────────────────────────────
#  HELPERS
# ─────────────────────────────────────────────────────────────────────────────

def build_gstreamer_pipeline(device: str, width: int, height: int, fps: int) -> str:
    """
    Build the GStreamer pipeline string for the NavQPlus CSI camera.
    Matches the pattern from official NXP NavQPlus docs.
    """
    return (
        f"v4l2src device={device} ! "
        f"video/x-raw,framerate={fps}/1,width={width},height={height} ! "
        f"videoconvert ! video/x-raw,format=BGR ! appsink drop=1"
    )


def get_perspective_transform(src: np.ndarray, dst: np.ndarray):
    """Return the forward (to BEV) and inverse perspective matrices."""
    M     = cv2.getPerspectiveTransform(src, dst)
    M_inv = cv2.getPerspectiveTransform(dst, src)
    return M, M_inv


def warp_to_bev(frame: np.ndarray, M: np.ndarray) -> np.ndarray:
    """Warp frame to bird's-eye view."""
    return cv2.warpPerspective(frame, M, (BEV_W, BEV_H))


def threshold_bev(bev_bgr: np.ndarray) -> np.ndarray:
    """
    Produce a binary image highlighting bright (white) pixels.
    Uses a combination of grayscale value threshold and Canny edges
    to work on both high-contrast and lower-contrast setups.
    """
    gray = cv2.cvtColor(bev_bgr, cv2.COLOR_BGR2GRAY)

    # Global brightness mask
    _, bright = cv2.threshold(gray, BRIGHT_THRESH, 255, cv2.THRESH_BINARY)

    # Adaptive threshold catches lines even under uneven lighting
    adaptive = cv2.adaptiveThreshold(
        gray, 255,
        cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY,
        blockSize=31, C=-10
    )

    # Edge map (catches the boundary of the tape even if it's grey)
    blur   = cv2.GaussianBlur(gray, (5, 5), 0)
    edges  = cv2.Canny(blur, 30, 100)
    dilated_edges = cv2.dilate(edges, np.ones((3, 3), np.uint8), iterations=1)

    # Combine all three cues
    combined = cv2.bitwise_or(bright, cv2.bitwise_or(adaptive, dilated_edges))

    # Morphological clean-up
    kernel   = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
    cleaned  = cv2.morphologyEx(combined, cv2.MORPH_CLOSE, kernel, iterations=2)
    return cleaned


def histogram_peaks(binary: np.ndarray):
    """
    Find the x positions of the left and right lane bases using a
    column-sum histogram of the bottom half of the binary BEV image.
    Returns (left_x, right_x) or None for each side if not found.
    """
    h, w  = binary.shape
    hist  = np.sum(binary[h // 2:, :], axis=0).astype(np.float32)

    mid   = w // 2
    left_x  = int(np.argmax(hist[:mid]))          if hist[:mid].max()  > 0 else None
    right_x = int(np.argmax(hist[mid:]) + mid)    if hist[mid:].max()  > 0 else None
    return left_x, right_x


def sliding_window_lane(binary: np.ndarray, base_x, side: str):
    """
    Run sliding-window search for one lane (left or right).
    Returns a list of (cx, cy) centroids per window level, or [] if not found.
    """
    if base_x is None:
        return []

    h, w    = binary.shape
    win_h   = h // N_WINDOWS
    cur_x   = base_x
    centroids = []

    for i in range(N_WINDOWS - 1, -1, -1):      # bottom → top
        y_lo = i       * win_h
        y_hi = (i + 1) * win_h
        x_lo = max(0, cur_x - WIN_MARGIN)
        x_hi = min(w, cur_x + WIN_MARGIN)

        roi  = binary[y_lo:y_hi, x_lo:x_hi]
        nz   = cv2.countNonZero(roi)

        if nz >= MIN_PIX:
            coords  = np.column_stack(np.where(roi > 0))      # (row, col)
            cx_local = int(np.mean(coords[:, 1]))
            cur_x    = x_lo + cx_local
            cy       = (y_lo + y_hi) // 2
            centroids.append((cur_x, cy))

    return centroids


def fit_line_to_centroids(centroids):
    """
    Fit a straight line through the centroids using np.polyfit (degree 1).
    Returns (slope, intercept) in y=f(x) form, or None if insufficient data.
    """
    if len(centroids) < 2:
        return None
    xs = np.array([c[0] for c in centroids], dtype=np.float32)
    ys = np.array([c[1] for c in centroids], dtype=np.float32)
    # fit x = f(y) for near-vertical lines
    try:
        coeffs = np.polyfit(ys, xs, 1)   # x = m*y + b
        return coeffs                     # [slope, intercept]
    except np.linalg.LinAlgError:
        return None


def line_endpoints(coeffs, img_h):
    """
    Given x = m*y + b coefficients and image height,
    return (x_bottom, y_bottom), (x_top, y_top) at y=bottom and y=top/3.
    """
    y_bot = img_h - 1
    y_top = img_h // 3
    x_bot = int(np.polyval(coeffs, y_bot))
    x_top = int(np.polyval(coeffs, y_top))
    return (x_bot, y_bot), (x_top, y_top)


def to_pixy_space(x_px, y_px, img_w, img_h):
    """
    Map pixel coordinates in the BEV image to the NXP Cup 78×51 Pixy space.
    """
    px = int(round(x_px / img_w * FRAME_W))
    py = int(round(y_px / img_h * FRAME_H))
    return (
        max(0, min(FRAME_W, px)),
        max(0, min(FRAME_H, py))
    )


def extract_vectors(left_coeffs, right_coeffs):
    """
    Convert lane line fits into two NXP Pixy-style vectors in 78×51 space.

    NXP convention:
        tail = bottom of vector (near the car)
        head = top  of vector (further ahead)
    Returns dict with keys: m0_x0, m0_y0, m0_x1, m0_y1,
                                    m1_x0, m1_y0, m1_x1, m1_y1
    All zero if a lane is not detected.
    """
    result = dict(m0_x0=0, m0_y0=0, m0_x1=0, m0_y1=0,
                  m1_x0=0, m1_y0=0, m1_x1=0, m1_y1=0)

    def fill(coeffs, prefix):
        if coeffs is None:
            return
        (x_bot, y_bot), (x_top, y_top) = line_endpoints(coeffs, BEV_H)
        tail = to_pixy_space(x_bot, y_bot, BEV_W, BEV_H)
        head = to_pixy_space(x_top, y_top, BEV_W, BEV_H)
        result[f"{prefix}_x0"], result[f"{prefix}_y0"] = tail
        result[f"{prefix}_x1"], result[f"{prefix}_y1"] = head

    fill(left_coeffs,  "m0")
    fill(right_coeffs, "m1")
    return result


def draw_debug_overlay(bev_bgr, binary, left_coeffs, right_coeffs, vectors):
    """
    Draw sliding windows, lane lines, centre path and Pixy vectors on a
    colour copy of the BEV image.  Returns the annotated image.
    """
    vis = bev_bgr.copy()

    # Tint binary detections green
    binary_colour = np.zeros_like(bev_bgr)
    binary_colour[:, :, 1] = binary // 2          # dim green tint
    vis = cv2.addWeighted(vis, 1.0, binary_colour, 0.4, 0)

    h, w = vis.shape[:2]

    def draw_lane(coeffs, colour):
        if coeffs is None:
            return
        pts = []
        for y in range(h):
            x = int(np.polyval(coeffs, y))
            if 0 <= x < w:
                pts.append((x, y))
        for i in range(1, len(pts)):
            cv2.line(vis, pts[i - 1], pts[i], colour, 2)

    draw_lane(left_coeffs,  (255, 100,  50))   # blue-ish  = left
    draw_lane(right_coeffs, ( 50, 200, 255))   # orange    = right

    # Centre corridor fill (between the two lanes)
    if left_coeffs is not None and right_coeffs is not None:
        poly_pts = []
        for y in range(h - 1, -1, -1):
            x = int(np.polyval(left_coeffs, y))
            poly_pts.append([np.clip(x, 0, w - 1), y])
        for y in range(h):
            x = int(np.polyval(right_coeffs, y))
            poly_pts.append([np.clip(x, 0, w - 1), y])
        overlay = vis.copy()
        cv2.fillPoly(overlay, [np.array(poly_pts)], (0, 255, 100))
        vis = cv2.addWeighted(vis, 0.7, overlay, 0.3, 0)

    # ── Draw Pixy vectors scaled back to BEV pixel space ──
    def pixy_to_px(px_norm, py_norm):
        return (
            int(px_norm / FRAME_W * w),
            int(py_norm / FRAME_H * h)
        )

    def draw_vector(x0, y0, x1, y1, colour, label):
        if x0 == 0 and y0 == 0 and x1 == 0 and y1 == 0:
            return
        tail = pixy_to_px(x0, y0)
        head = pixy_to_px(x1, y1)
        cv2.arrowedLine(vis, tail, head, colour, 3, tipLength=0.25)
        cv2.circle(vis, tail, 5, colour, -1)
        cv2.putText(vis, label, head, cv2.FONT_HERSHEY_SIMPLEX,
                    0.45, colour, 1, cv2.LINE_AA)

    draw_vector(vectors["m0_x0"], vectors["m0_y0"],
                vectors["m0_x1"], vectors["m0_y1"],
                (255,  60,  60), "m0")
    draw_vector(vectors["m1_x0"], vectors["m1_y0"],
                vectors["m1_x1"], vectors["m1_y1"],
                ( 60,  60, 255), "m1")

    # ── NXP 78×51 reference grid ──
    grid_col = (80, 80, 80)
    for gx in range(0, FRAME_W + 1, 13):     # ~6 cols
        px = int(gx / FRAME_W * w)
        cv2.line(vis, (px, 0), (px, h), grid_col, 1)
    for gy in range(0, FRAME_H + 1, 17):     # ~3 rows
        py = int(gy / FRAME_H * h)
        cv2.line(vis, (0, py), (w, py), grid_col, 1)

    return vis


def draw_src_polygon(frame, src_pts):
    """Draw the BEV source trapezoid on the raw camera frame."""
    pts = src_pts.astype(np.int32).reshape((-1, 1, 2))
    cv2.polylines(frame, [pts], isClosed=True, color=(0, 255, 255), thickness=2)
    for i, (x, y) in enumerate(src_pts.astype(int)):
        cv2.circle(frame, (x, y), 6, (0, 100, 255), -1)
        cv2.putText(frame, str(i), (x + 5, y - 5),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)


def hud_overlay(raw_frame, vectors, fps, num_vec):
    """Print key info on the raw frame."""
    lines = [
        f"FPS: {fps:.1f}",
        f"Vectors: {num_vec}",
        f"m0 tail({vectors['m0_x0']},{vectors['m0_y0']}) "
        f"head({vectors['m0_x1']},{vectors['m0_y1']})",
        f"m1 tail({vectors['m1_x0']},{vectors['m1_y0']}) "
        f"head({vectors['m1_x1']},{vectors['m1_y1']})",
    ]
    for i, txt in enumerate(lines):
        cv2.putText(raw_frame, txt, (10, 22 + i * 22),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 0), 1, cv2.LINE_AA)


# ─────────────────────────────────────────────────────────────────────────────
#  MJPEG HTTP STREAM SERVER
# ─────────────────────────────────────────────────────────────────────────────

class StreamState:
    """
    Thread-safe latest-frame store.
    push() overwrites immediately; the HTTP handler polls at ~30 fps max.
    This avoids the condition-variable miss that caused 1 frame / 5 s.
    """
    def __init__(self):
        self._lock  = threading.Lock()
        self._frame = b""
        self._seq   = 0   # increments on every push

    def push(self, jpeg_bytes: bytes):
        with self._lock:
            self._frame = jpeg_bytes
            self._seq  += 1

    def get_if_newer(self, last_seq: int):
        """Returns (seq, bytes) if a newer frame exists, else (last_seq, None)."""
        with self._lock:
            if self._seq != last_seq:
                return self._seq, self._frame
        return last_seq, None


_stream_state = StreamState()

# Minimal HTML page served at "/"
_INDEX_HTML = (
    "HTTP/1.0 200 OK\r\nContent-Type: text/html\r\n\r\n"
    "<!doctype html><html><head>"
    "<title>NXP Cup Vision</title>"
    "<style>body{margin:0;background:#111;display:flex;flex-direction:column;"
    "align-items:center;font-family:monospace;color:#0f0;}"
    "h2{margin:8px 0;}"
    "img{max-width:100%;border:1px solid #0f0;}</style>"
    "</head><body>"
    "<h2>NXP Cup Lane Vision - NavQPlus</h2>"
    "<img src='/stream' /><br>"
    "<small>RAW | BEV + Pixy Vectors</small>"
    "</body></html>"
).encode()

class MJPEGHandler(BaseHTTPRequestHandler):
    def log_message(self, *args):
        pass   # silence per-request log spam

    def do_GET(self):
        if self.path == "/":
            self.wfile.write(_INDEX_HTML)
            return

        if self.path == "/stream":
            self.send_response(200)
            self.send_header("Content-Type",
                             "multipart/x-mixed-replace; boundary=frame")
            self.send_header("Cache-Control", "no-cache")
            self.end_headers()
            last_seq = -1
            try:
                while True:
                    seq, jpg = _stream_state.get_if_newer(last_seq)
                    if jpg:
                        last_seq = seq
                        header = (
                            b"--frame\r\n"
                            b"Content-Type: image/jpeg\r\n"
                            b"Content-Length: " + str(len(jpg)).encode() + b"\r\n\r\n"
                        )
                        self.wfile.write(header + jpg + b"\r\n")
                        self.wfile.flush()
                    else:
                        time.sleep(0.01)   # no new frame yet, yield CPU
            except (BrokenPipeError, ConnectionResetError):
                pass   # client disconnected
            return

        # 404 for anything else
        self.send_error(404)


def start_mjpeg_server(port: int):
    server = HTTPServer(("0.0.0.0", port), MJPEGHandler)
    t = threading.Thread(target=server.serve_forever, daemon=True)
    t.start()
    return server


# ─────────────────────────────────────────────────────────────────────────────
#  BEV CALIBRATION  (text-based over SSH – no mouse needed)
# ─────────────────────────────────────────────────────────────────────────────

def calibrate_from_terminal(frame, port: int) -> np.ndarray:
    """
    Save the raw frame as 'calib_frame.jpg', then ask the user to open the
    MJPEG stream or the JPEG in their browser/VS Code and type in the 4 points.
    Returns new src_pts, or the original BEV_SRC_POINTS if the user skips.
    """
    snap_path = "calib_frame.jpg"
    cv2.imwrite(snap_path, frame)
    print(f"\n[Calibration] Saved snapshot → {snap_path}")
    print(f"  Open it locally (scp / VS Code Explorer) or view the stream at")
    print(f"  http://<navqplus-ip>:{port}")
    print()
    print("  Enter 4 point coordinates in order:")
    print("    0 = Bottom-Left  (near-left corner of track ROI)")
    print("    1 = Bottom-Right (near-right corner)")
    print("    2 = Top-Right    (far-right  corner)")
    print("    3 = Top-Left     (far-left   corner)")
    print("  Press ENTER on a blank line to skip and keep defaults.\n")

    pts = []
    for i, label in enumerate(["Bottom-Left", "Bottom-Right", "Top-Right", "Top-Left"]):
        while True:
            raw = input(f"  Point {i} ({label}) x,y  [or ENTER to skip]: ").strip()
            if raw == "":
                print("  Skipping – keeping current BEV_SRC_POINTS.")
                return BEV_SRC_POINTS.copy()
            try:
                x, y = [int(v.strip()) for v in raw.split(",")]
                pts.append([x, y])
                break
            except ValueError:
                print("  Bad format – enter as:  x,y   e.g.  160,480")

    new_pts = np.float32(pts)
    print(f"\n[Calibration] New BEV_SRC_POINTS:\n{new_pts.tolist()}")
    print("  Copy these into BEV_SRC_POINTS at the top of the script to persist.\n")
    return new_pts


# ─────────────────────────────────────────────────────────────────────────────
#  MAIN
# ─────────────────────────────────────────────────────────────────────────────

def parse_args():
    p = argparse.ArgumentParser(description="NXP Cup NavQPlus lane vision – MJPEG stream")
    p.add_argument("--device",      default=DEFAULT_DEVICE,
                   help="V4L2 device (default: /dev/video3)")
    p.add_argument("--width",       type=int, default=DEFAULT_WIDTH)
    p.add_argument("--height",      type=int, default=DEFAULT_HEIGHT)
    p.add_argument("--fps",         type=int, default=DEFAULT_FPS)
    p.add_argument("--port",        type=int, default=8080,
                   help="HTTP port for MJPEG stream (default: 8080)")
    p.add_argument("--save-video",  default="",
                   help="Also save output to this MP4 file, e.g. out.mp4")
    p.add_argument("--flip",        action="store_true",
                   help="Flip frame 180° (camera mounted upside-down)")
    p.add_argument("--calibrate",   action="store_true",
                   help="Run BEV point calibration on startup")
    p.add_argument("--jpeg-quality", type=int, default=75,
                   help="MJPEG compression quality 1-100 (default: 75)")
    return p.parse_args()


def main():
    args = parse_args()

    # ── Open camera ──────────────────────────────────────────────────────────
    pipeline = build_gstreamer_pipeline(
        args.device, args.width, args.height, args.fps
    )
    print(f"[Camera] Opening: {pipeline}")
    cap = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)

    if not cap.isOpened():
        print("[WARN] GStreamer pipeline failed – falling back to V4L2 direct open")
        cap = cv2.VideoCapture(args.device)
        cap.set(cv2.CAP_PROP_FRAME_WIDTH,  args.width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, args.height)
        cap.set(cv2.CAP_PROP_FPS,          args.fps)

    if not cap.isOpened():
        print(f"[ERROR] Cannot open camera {args.device}. "
              "Run: v4l2-ctl --list-devices")
        sys.exit(1)

    print("[Camera] Opened successfully.")

    # ── Start MJPEG server ────────────────────────────────────────────────────
    start_mjpeg_server(args.port)
    print(f"\n[Stream]  http://<navqplus-ip>:{args.port}")
    print( "          Find your IP with:  ip addr show | grep 'inet '")
    print( "\nPress Ctrl-C to stop.\n")

    # ── Perspective transform ─────────────────────────────────────────────────
    src_pts  = BEV_SRC_POINTS.copy()
    M, M_inv = get_perspective_transform(src_pts, BEV_DST_POINTS)

    # ── Optional startup calibration ──────────────────────────────────────────
    if args.calibrate:
        ret, frame = cap.read()
        if ret:
            if args.flip:
                frame = cv2.rotate(frame, cv2.ROTATE_180)
            src_pts  = calibrate_from_terminal(frame, args.port)
            M, M_inv = get_perspective_transform(src_pts, BEV_DST_POINTS)

    # ── Optional video writer ─────────────────────────────────────────────────
    writer = None
    if args.save_video:
        raw_h = args.height
        bev_resized_w = int(BEV_W * (raw_h / BEV_H))
        out_w = args.width + bev_resized_w
        fourcc = cv2.VideoWriter_fourcc(*"mp4v")
        writer = cv2.VideoWriter(args.save_video, fourcc, args.fps,
                                 (out_w, raw_h))
        print(f"[Video] Saving to {args.save_video} ({out_w}×{raw_h})")

    encode_params = [cv2.IMWRITE_JPEG_QUALITY, args.jpeg_quality]
    prev_time     = time.time()
    fps_display   = 0.0
    frame_count   = 0

    try:
        while True:
            ret, frame = cap.read()
            if not ret:
                time.sleep(0.05)
                continue

            if args.flip:
                frame = cv2.rotate(frame, cv2.ROTATE_180)

            # ── FPS ───────────────────────────────────────────────────────────
            now = time.time()
            fps_display = 0.9 * fps_display + 0.1 / max(now - prev_time, 1e-6)
            prev_time   = now
            frame_count += 1

            # ── BEV warp ──────────────────────────────────────────────────────
            bev = warp_to_bev(frame, M)

            # Process on a half-res BEV to save CPU; results scale back up.
            bev_small  = cv2.resize(bev, (BEV_W // 2, BEV_H // 2))
            binary_small = threshold_bev(bev_small)

            # ── Lane detection (on small image) ───────────────────────────────
            left_base_s, right_base_s = histogram_peaks(binary_small)
            left_centroids_s  = sliding_window_lane(binary_small, left_base_s,  "left")
            right_centroids_s = sliding_window_lane(binary_small, right_base_s, "right")

            # Scale centroids back to full BEV space
            def scale_up(pts):
                return [(x * 2, y * 2) for x, y in pts]

            left_centroids  = scale_up(left_centroids_s)
            right_centroids = scale_up(right_centroids_s)
            left_coeffs     = fit_line_to_centroids(left_centroids)
            right_coeffs    = fit_line_to_centroids(right_centroids)

            # Full-res binary for the overlay (cheap – just upscale)
            binary = cv2.resize(binary_small, (BEV_W, BEV_H),
                                interpolation=cv2.INTER_NEAREST)

            # ── Pixy vectors ──────────────────────────────────────────────────
            vectors = extract_vectors(left_coeffs, right_coeffs)
            num_vec = sum(
                1 for k in ["m0", "m1"]
                if not all(vectors[f"{k}_{c}"] == 0
                           for c in ("x0", "y0", "x1", "y1"))
            )

            # ── Build debug frame ─────────────────────────────────────────────
            raw_vis = frame.copy()
            draw_src_polygon(raw_vis, src_pts)
            hud_overlay(raw_vis, vectors, fps_display, num_vec)

            bev_vis    = draw_debug_overlay(bev, binary,
                                            left_coeffs, right_coeffs, vectors)
            raw_h, raw_w = raw_vis.shape[:2]
            bev_scale    = raw_h / BEV_H
            bev_resized  = cv2.resize(bev_vis, (int(BEV_W * bev_scale), raw_h))

            debug = np.hstack([raw_vis, bev_resized])
            cv2.putText(debug, "RAW + BEV ROI", (10, raw_h - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1)
            cv2.putText(debug, "BEV + Vectors", (raw_w + 10, raw_h - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1)

            # ── Push to MJPEG stream ──────────────────────────────────────────
            ok, jpg = cv2.imencode(".jpg", debug, encode_params)
            if ok:
                _stream_state.push(jpg.tobytes())

            if writer:
                writer.write(debug)

            # Print vector summary every 30 frames
            if frame_count % 30 == 0:
                print(
                    f"fps={fps_display:.1f}  vecs={num_vec}  "
                    f"m0 ({vectors['m0_x0']},{vectors['m0_y0']})"
                    f"→({vectors['m0_x1']},{vectors['m0_y1']})  "
                    f"m1 ({vectors['m1_x0']},{vectors['m1_y0']})"
                    f"→({vectors['m1_x1']},{vectors['m1_y1']})"
                )

    except KeyboardInterrupt:
        print("\n[Stopped by user]")
    finally:
        cap.release()
        if writer:
            writer.release()
        print("[Done]")


if __name__ == "__main__":
    main()