# Copyright 2024 NXP

# Copyright 2016 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import math

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage

from synapse_msgs.msg import EdgeVectors

QOS_PROFILE_DEFAULT = 10

PI = math.pi

RED_COLOR = (0, 0, 255)
BLUE_COLOR = (255, 0, 0)
GREEN_COLOR = (0, 255, 0)
YELLOW_COLOR = (0, 255, 255)
CYAN_COLOR = (255, 255, 0)
ORANGE_COLOR = (0, 165, 255)

# =========================
#   TEAMMATE-STYLE BEV
# =========================
# based on the source layout used in vision_basic.py / vision_chain.py
# source order: bottom-left, bottom-right, top-right, top-left
BASE_CAM_W = 640.0
BASE_CAM_H = 480.0

BEV_SRC_POINTS_BASE = np.float32([
    [160.0, 480.0],  # bottom-left
    [480.0, 480.0],  # bottom-right
    [540.0, 280.0],  # top-right
    [100.0, 280.0],  # top-left
])

BEV_W = 400
BEV_H = 300

BEV_DST_POINTS = np.float32([
    [0.0, BEV_H - 1.0],        # bottom-left
    [BEV_W - 1.0, BEV_H - 1.0],# bottom-right
    [BEV_W - 1.0, 0.0],        # top-right
    [0.0, 0.0],                # top-left
])

# ignore unstable far field after warp
IGNORE_TOP_ROWS_RATIO = 0.12

# =========================
#   TEAMMATE-STYLE THRESHOLDING
# =========================
BRIGHT_THRESH = 120
ADAPTIVE_BLOCK_SIZE = 31
ADAPTIVE_C = -10

CANNY_LOW = 30
CANNY_HIGH = 100

EDGE_DILATE_KERNEL = 3
OPEN_KERNEL = (3, 3)
CLOSE_KERNEL = (5, 3)
CLOSE_ITERATIONS = 2

# =========================
#   STRIP / WINDOW SEARCH
# =========================
N_STRIPS = 24
TRACK_SEARCH_W = 60
MIN_LINE_COLS = 3

# line vector extraction
LINE_TOP_Y_RATIO = 0.33

# debug / sanity
MIN_POINTS_FOR_FIT = 2


class EdgeVectorsPublisher(Node):
    """
    Publishes camera-derived edge-vector measurements.

    Input:
    - /camera/image_raw/compressed

    Output:
    - /edge_vectors
    - debug images

    Important:
    These are perception-layer measurements in BEV image space.
    """

    def __init__(self):
        super().__init__('edge_vectors_publisher')

        self.subscription_camera = self.create_subscription(
            CompressedImage,
            '/camera/image_raw/compressed',
            self.camera_image_callback,
            QOS_PROFILE_DEFAULT
        )

        self.publisher_edge_vectors = self.create_publisher(
            EdgeVectors,
            '/edge_vectors',
            QOS_PROFILE_DEFAULT
        )

        self.publisher_thresh_image = self.create_publisher(
            CompressedImage,
            '/debug_images/thresh_image',
            QOS_PROFILE_DEFAULT
        )

        self.publisher_vector_image = self.create_publisher(
            CompressedImage,
            '/debug_images/vector_image',
            QOS_PROFILE_DEFAULT
        )

        self.publisher_bev_image = self.create_publisher(
            CompressedImage,
            '/debug_images/bev_image',
            QOS_PROFILE_DEFAULT
        )

        self.publisher_source_image = self.create_publisher(
            CompressedImage,
            '/debug_images/bev_source_image',
            QOS_PROFILE_DEFAULT
        )

    def publish_debug_image(self, publisher, image):
        """Publishes images for debugging purposes."""
        message = CompressedImage()
        _, encoded_data = cv2.imencode('.jpg', image)
        message.format = "jpeg"
        message.data = encoded_data.tobytes()
        publisher.publish(message)

    def scale_src_points(self, width, height):
        """Scale teammate-style BEV source points to current camera size."""
        sx = width / BASE_CAM_W
        sy = height / BASE_CAM_H

        src = BEV_SRC_POINTS_BASE.copy()
        src[:, 0] *= sx
        src[:, 1] *= sy

        src[:, 0] = np.clip(src[:, 0], 0, width - 1)
        src[:, 1] = np.clip(src[:, 1], 0, height - 1)
        return src

    def compute_bev_image(self, image_bgr):
        """Applies the teammate-style bird's-eye-view transform."""
        height, width = image_bgr.shape[:2]

        src = self.scale_src_points(width, height)
        dst = BEV_DST_POINTS.copy()

        transform = cv2.getPerspectiveTransform(src, dst)

        source_mask = np.zeros((height, width), dtype=np.uint8)
        cv2.fillConvexPoly(source_mask, src.astype(np.int32), 255)

        masked_image = cv2.bitwise_and(image_bgr, image_bgr, mask=source_mask)

        bev_image = cv2.warpPerspective(masked_image, transform, (BEV_W, BEV_H))
        bev_mask = cv2.warpPerspective(source_mask, transform, (BEV_W, BEV_H))

        source_debug = image_bgr.copy()
        cv2.polylines(source_debug, [src.astype(np.int32)], True, YELLOW_COLOR, 2)

        for i, pt in enumerate(src.astype(int)):
            cv2.circle(source_debug, tuple(pt), 5, ORANGE_COLOR, -1)
            cv2.putText(
                source_debug,
                str(i),
                (pt[0] + 5, pt[1] - 5),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                YELLOW_COLOR,
                1,
                cv2.LINE_AA
            )

        return bev_image, bev_mask, source_debug

    def threshold_bev(self, bev_bgr, bev_mask):
        """
        Teammate-style combined threshold:
        - global brightness threshold
        - adaptive threshold
        - canny + dilated edges
        - morphology cleanup
        """
        gray = cv2.cvtColor(bev_bgr, cv2.COLOR_BGR2GRAY)

        _, bright = cv2.threshold(gray, BRIGHT_THRESH, 255, cv2.THRESH_BINARY)

        adaptive = cv2.adaptiveThreshold(
            gray,
            255,
            cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
            cv2.THRESH_BINARY,
            ADAPTIVE_BLOCK_SIZE,
            ADAPTIVE_C
        )

        blur = cv2.GaussianBlur(gray, (5, 5), 0)
        edges = cv2.Canny(blur, CANNY_LOW, CANNY_HIGH)

        dilated_edges = cv2.dilate(
            edges,
            np.ones((EDGE_DILATE_KERNEL, EDGE_DILATE_KERNEL), np.uint8),
            iterations=1
        )

        combined = cv2.bitwise_or(bright, cv2.bitwise_or(adaptive, dilated_edges))

        k_open = cv2.getStructuringElement(cv2.MORPH_RECT, OPEN_KERNEL)
        k_close = cv2.getStructuringElement(cv2.MORPH_RECT, CLOSE_KERNEL)

        combined = cv2.morphologyEx(combined, cv2.MORPH_OPEN, k_open, iterations=1)
        combined = cv2.morphologyEx(combined, cv2.MORPH_CLOSE, k_close, iterations=CLOSE_ITERATIONS)

        combined = cv2.bitwise_and(combined, bev_mask)

        ignore_top_rows = int(BEV_H * IGNORE_TOP_ROWS_RATIO)
        combined[:ignore_top_rows, :] = 0

        return combined

    def histogram_base(self, binary):
        """Find initial left/right x positions from bottom-half histogram."""
        h, w = binary.shape
        hist = np.sum(binary[h // 2:, :], axis=0).astype(np.float32)

        mid = w // 2

        left_hist = hist[:mid]
        right_hist = hist[mid:]

        left_x = int(np.argmax(left_hist)) if left_hist.max() > 0 else mid // 2
        right_x = int(np.argmax(right_hist) + mid) if right_hist.max() > 0 else mid + mid // 2

        return left_x, right_x

    def extract_strip_points(self, binary):
        """
        Strip-based left/right tracking, adapted from teammate's chain idea.
        Returns raw points ordered near -> far (bottom to top), with None if missing.
        """
        h, w = binary.shape
        strip_h = max(1, h // N_STRIPS)

        left_x, right_x = self.histogram_base(binary)

        left_raw = []
        right_raw = []

        for i in range(N_STRIPS - 1, -1, -1):
            y0 = i * strip_h
            y1 = min(y0 + strip_h, h)
            cy = (y0 + y1) // 2

            strip = binary[y0:y1, :]

            split = (left_x + right_x) // 2

            l_lo = max(0, left_x - TRACK_SEARCH_W)
            l_hi = min(split, left_x + TRACK_SEARCH_W)

            r_lo = max(split, right_x - TRACK_SEARCH_W)
            r_hi = min(w, right_x + TRACK_SEARCH_W)

            left_pt = None
            right_pt = None

            if l_hi > l_lo:
                left_cols = np.where(strip[:, l_lo:l_hi].max(axis=0) > 0)[0]
                if len(left_cols) >= MIN_LINE_COLS:
                    cx = l_lo + int(np.median(left_cols))
                    left_pt = (cx, cy)
                    left_x = cx

            if r_hi > r_lo:
                right_cols = np.where(strip[:, r_lo:r_hi].max(axis=0) > 0)[0]
                if len(right_cols) >= MIN_LINE_COLS:
                    cx = r_lo + int(np.median(right_cols))
                    right_pt = (cx, cy)
                    right_x = cx

            left_raw.append(left_pt)
            right_raw.append(right_pt)

        return left_raw, right_raw

    def fit_line_x_of_y(self, points):
        """
        Fit x = m*y + b.
        This is better than y=f(x) because lane edges are close to vertical in BEV.
        """
        pts = [p for p in points if p is not None]

        if len(pts) < MIN_POINTS_FOR_FIT:
            return None

        ys = np.array([p[1] for p in pts], dtype=np.float32)
        xs = np.array([p[0] for p in pts], dtype=np.float32)

        try:
            coeffs = np.polyfit(ys, xs, 1)
            return coeffs
        except np.linalg.LinAlgError:
            return None

    def coeffs_to_vector_points(self, coeffs):
        """
        Convert x = m*y + b into top and bottom points.
        IMPORTANT:
        return order matches your old node:
        [top_point, bottom_point]
        """
        if coeffs is None:
            return None

        y_bottom = BEV_H - 1
        y_top = int(BEV_H * LINE_TOP_Y_RATIO)

        x_bottom = int(np.polyval(coeffs, y_bottom))
        x_top = int(np.polyval(coeffs, y_top))

        x_bottom = int(np.clip(x_bottom, 0, BEV_W - 1))
        x_top = int(np.clip(x_top, 0, BEV_W - 1))

        top_point = [x_top, y_top]
        bottom_point = [x_bottom, y_bottom]

        return [top_point, bottom_point]

    def draw_lane_fit(self, image, coeffs, color):
        """Draw full fitted lane line on debug image."""
        if coeffs is None:
            return

        pts = []
        for y in range(BEV_H):
            x = int(np.polyval(coeffs, y))
            if 0 <= x < BEV_W:
                pts.append((x, y))

        for i in range(1, len(pts)):
            cv2.line(image, pts[i - 1], pts[i], color, 2)

    def draw_raw_points(self, image, points, color):
        """Draw raw strip centroids."""
        for pt in points:
            if pt is not None:
                cv2.circle(image, pt, 3, color, -1)

    def process_image_for_edge_vectors(self, image_bgr):
        """
        Full teammate-style measurement pipeline:

        camera -> BEV -> combined threshold -> strip tracking -> line fit -> vectors
        """
        bev_image, bev_mask, source_debug = self.compute_bev_image(image_bgr)
        thresh = self.threshold_bev(bev_image, bev_mask)

        left_raw, right_raw = self.extract_strip_points(thresh)

        left_coeffs = self.fit_line_x_of_y(left_raw)
        right_coeffs = self.fit_line_x_of_y(right_raw)

        left_vector = self.coeffs_to_vector_points(left_coeffs)
        right_vector = self.coeffs_to_vector_points(right_coeffs)

        debug_image = bev_image.copy()

        tint = np.zeros_like(debug_image)
        tint[:, :, 1] = thresh // 2
        debug_image = cv2.addWeighted(debug_image, 1.0, tint, 0.4, 0)

        self.draw_raw_points(debug_image, left_raw, CYAN_COLOR)
        self.draw_raw_points(debug_image, right_raw, ORANGE_COLOR)

        self.draw_lane_fit(debug_image, left_coeffs, BLUE_COLOR)
        self.draw_lane_fit(debug_image, right_coeffs, RED_COLOR)

        final_vectors = []

        if left_vector is not None:
            cv2.line(debug_image, tuple(left_vector[0]), tuple(left_vector[1]), GREEN_COLOR, 3)
            final_vectors.append(left_vector)

        if right_vector is not None:
            cv2.line(debug_image, tuple(right_vector[0]), tuple(right_vector[1]), GREEN_COLOR, 3)
            final_vectors.append(right_vector)

        self.publish_debug_image(self.publisher_source_image, source_debug)
        self.publish_debug_image(self.publisher_bev_image, bev_image)
        self.publish_debug_image(self.publisher_thresh_image, thresh)
        self.publish_debug_image(self.publisher_vector_image, debug_image)

        return final_vectors

    def camera_image_callback(self, message):
        """Analyze camera image and publish edge vectors."""
        np_arr = np.frombuffer(message.data, np.uint8)
        image_bgr = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

        if image_bgr is None:
            return

        final_vectors = self.process_image_for_edge_vectors(image_bgr)

        vectors_message = EdgeVectors()
        vectors_message.image_height = BEV_H
        vectors_message.image_width = BEV_W
        vectors_message.vector_count = 0

        if len(final_vectors) > 0:
            vectors_message.vector_1[0].x = float(final_vectors[0][0][0])  # top x
            vectors_message.vector_1[0].y = float(final_vectors[0][0][1])  # top y
            vectors_message.vector_1[1].x = float(final_vectors[0][1][0])  # bottom x
            vectors_message.vector_1[1].y = float(final_vectors[0][1][1])  # bottom y
            vectors_message.vector_count += 1

        if len(final_vectors) > 1:
            vectors_message.vector_2[0].x = float(final_vectors[1][0][0])  # top x
            vectors_message.vector_2[0].y = float(final_vectors[1][0][1])  # top y
            vectors_message.vector_2[1].x = float(final_vectors[1][1][0])  # bottom x
            vectors_message.vector_2[1].y = float(final_vectors[1][1][1])  # bottom y
            vectors_message.vector_count += 1

        self.publisher_edge_vectors.publish(vectors_message)

    def get_vector_angle_in_radians(self, vector):
        """Kept for compatibility / debugging if needed later."""
        if (vector[0][0] - vector[1][0]) == 0:
            theta = PI / 2
        else:
            slope = (vector[1][1] - vector[0][1]) / (vector[0][0] - vector[1][0])
            theta = math.atan(slope)
        return theta


def main(args=None):
    rclpy.init(args=args)
    edge_vectors_publisher = EdgeVectorsPublisher()
    rclpy.spin(edge_vectors_publisher)
    edge_vectors_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()