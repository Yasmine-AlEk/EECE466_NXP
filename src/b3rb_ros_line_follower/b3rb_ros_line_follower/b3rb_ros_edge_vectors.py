
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
ORANGE_COLOR = (0, 165, 255)

# ============================================================
# NOTE ABOUT DEBUG WINDOWS
# ============================================================
# This node publishes FOUR debug image topics:
#   /debug_images/bev_source_image
#   /debug_images/bev_image
#   /debug_images/thresh_image
#   /debug_images/vector_image
#
# So if your viewer subscribes to all debug image topics, you may
# see four image windows instead of only two.
# ============================================================

# =========================
#   TEAMMATE-STYLE BEV
# =========================
# source order: bottom-left, bottom-right, top-right, top-left
BASE_CAM_W = 640.0
BASE_CAM_H = 480.0

BEV_SRC_POINTS_BASE = np.float32([
    [80.0, 479.0],   # bottom-left
    [560.0, 479.0],  # bottom-right
    [610.0, 280.0],  # top-right
    [30.0, 280.0],   # top-left
])

BEV_W = 400
BEV_H = 300

BEV_DST_POINTS = np.float32([
    [0.0, BEV_H - 1.0],          # bottom-left
    [BEV_W - 1.0, BEV_H - 1.0],  # bottom-right
    [BEV_W - 1.0, 0.0],          # top-right
    [0.0, 0.0],                  # top-left
])

# ignore unstable far field after warp
IGNORE_TOP_ROWS_RATIO = 0.15

# =========================
#   HYBRID THRESHOLDING
# =========================
# BEV -> grayscale -> blur -> THRESH_BINARY_INV for black borders
THRESHOLD_BLACK = 90

# light morphology only
MORPH_CLOSE_KERNEL = (5, 5)
MORPH_OPEN_KERNEL = (3, 3)

# =========================
#   CONTOUR / VECTOR FILTERS
# =========================
VECTOR_MAGNITUDE_MINIMUM = 8.0
CONTOUR_AREA_MINIMUM = 20.0


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
        Hybrid thresholding:
        - grayscale
        - gaussian blur
        - THRESH_BINARY_INV to isolate black borders
        - light morphology
        """
        gray = cv2.cvtColor(bev_bgr, cv2.COLOR_BGR2GRAY)
        gray = cv2.GaussianBlur(gray, (5, 5), 0)

        thresh = cv2.threshold(
            gray,
            THRESHOLD_BLACK,
            255,
            cv2.THRESH_BINARY_INV
        )[1]

        thresh = cv2.bitwise_and(thresh, bev_mask)

        ignore_top_rows = int(BEV_H * IGNORE_TOP_ROWS_RATIO)
        thresh[:ignore_top_rows, :] = 0

        k_close = cv2.getStructuringElement(cv2.MORPH_RECT, MORPH_CLOSE_KERNEL)
        k_open = cv2.getStructuringElement(cv2.MORPH_RECT, MORPH_OPEN_KERNEL)

        thresh = cv2.morphologyEx(thresh, cv2.MORPH_CLOSE, k_close, iterations=1)
        thresh = cv2.morphologyEx(thresh, cv2.MORPH_OPEN, k_open, iterations=1)

        thresh = cv2.bitwise_and(thresh, bev_mask)
        thresh[:ignore_top_rows, :] = 0

        return thresh

    def get_vector_angle_in_radians(self, vector):
        """Calculates vector angle in radians."""
        if (vector[0][0] - vector[1][0]) == 0:
            theta = PI / 2
        else:
            slope = (vector[1][1] - vector[0][1]) / (vector[0][0] - vector[1][0])
            theta = math.atan(slope)
        return theta

    def compute_vectors_from_binary(self, debug_image, thresh):
        """
        Contour-based candidate vector extraction in BEV image space.
        Returns candidate vectors in the same order as the old node:
        [top_point, bottom_point, distance, magnitude]
        """
        contours = cv2.findContours(
            thresh,
            cv2.RETR_EXTERNAL,
            cv2.CHAIN_APPROX_NONE
        )[0]

        image_height, image_width = thresh.shape[:2]
        rover_point = np.array([image_width / 2.0, image_height - 1], dtype=np.float32)

        vectors = []

        for contour in contours:
            area = cv2.contourArea(contour)
            if area < CONTOUR_AREA_MINIMUM:
                continue

            coordinates = contour[:, 0, :]

            min_y_value = np.min(coordinates[:, 1])
            max_y_value = np.max(coordinates[:, 1])

            min_y_coords = np.array(coordinates[coordinates[:, 1] == min_y_value])
            max_y_coords = np.array(coordinates[coordinates[:, 1] == max_y_value])

            min_y_coord = min_y_coords[0].copy()
            max_y_coord = max_y_coords[0].copy()

            magnitude = np.linalg.norm(min_y_coord - max_y_coord)
            if magnitude <= VECTOR_MAGNITUDE_MINIMUM:
                continue

            middle_point = (min_y_coord + max_y_coord) / 2.0
            distance = np.linalg.norm(middle_point - rover_point)

            angle = self.get_vector_angle_in_radians([min_y_coord, max_y_coord])

            if angle > 0:
                min_y_coord[0] = np.max(min_y_coords[:, 0])
            else:
                max_y_coord[0] = np.max(max_y_coords[:, 0])

            vectors.append([
                list(min_y_coord),
                list(max_y_coord),
                distance,
                magnitude
            ])

            cv2.line(debug_image, tuple(min_y_coord), tuple(max_y_coord), BLUE_COLOR, 2)

        return vectors, debug_image

    @staticmethod
    def choose_best_vector(candidates):
        """Chooses the most useful vector on one side."""
        if len(candidates) == 0:
            return None

        return min(candidates, key=lambda v: (v[2] - 0.30 * v[3]))

    def process_image_for_edge_vectors(self, image_bgr):
        """
        Hybrid pipeline:

        camera -> BEV -> grayscale -> blur -> THRESH_BINARY_INV
               -> light morphology -> contour extraction -> left/right selection
        """
        bev_image, bev_mask, source_debug = self.compute_bev_image(image_bgr)
        thresh = self.threshold_bev(bev_image, bev_mask)

        debug_image = bev_image.copy()

        tint = np.zeros_like(debug_image)
        tint[:, :, 1] = thresh // 2
        debug_image = cv2.addWeighted(debug_image, 1.0, tint, 0.35, 0)

        vectors, debug_image = self.compute_vectors_from_binary(debug_image, thresh)

        half_width = BEV_W / 2.0

        vectors_left = [
            v for v in vectors
            if ((v[0][0] + v[1][0]) / 2.0) < half_width
        ]
        vectors_right = [
            v for v in vectors
            if ((v[0][0] + v[1][0]) / 2.0) >= half_width
        ]

        final_vectors = []

        left_best = self.choose_best_vector(vectors_left)
        right_best = self.choose_best_vector(vectors_right)

        for best_vector in [left_best, right_best]:
            if best_vector is not None:
                cv2.line(
                    debug_image,
                    tuple(best_vector[0]),
                    tuple(best_vector[1]),
                    GREEN_COLOR,
                    3
                )
                final_vectors.append(best_vector[:2])

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
            vectors_message.vector_1[0].x = float(final_vectors[0][0][0])
            vectors_message.vector_1[0].y = float(final_vectors[0][0][1])
            vectors_message.vector_1[1].x = float(final_vectors[0][1][0])
            vectors_message.vector_1[1].y = float(final_vectors[0][1][1])
            vectors_message.vector_count += 1

        if len(final_vectors) > 1:
            vectors_message.vector_2[0].x = float(final_vectors[1][0][0])
            vectors_message.vector_2[0].y = float(final_vectors[1][0][1])
            vectors_message.vector_2[1].x = float(final_vectors[1][1][0])
            vectors_message.vector_2[1].y = float(final_vectors[1][1][1])
            vectors_message.vector_count += 1

        self.publisher_edge_vectors.publish(vectors_message)


def main(args=None):
    rclpy.init(args=args)
    edge_vectors_publisher = EdgeVectorsPublisher()
    rclpy.spin(edge_vectors_publisher)
    edge_vectors_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
