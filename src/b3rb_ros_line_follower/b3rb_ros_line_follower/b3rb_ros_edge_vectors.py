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

#ignore tiny noisy contours
VECTOR_MAGNITUDE_MINIMUM = 4.0

#threshold / morphology
THRESHOLD_BLACK = 90
MORPH_KERNEL_SIZE = 5

# =========================
#   BIRD'S-EYE VIEW SETUP
# =========================

# vertical placement of the trapezoid in the ORIGINAL image
BEV_TOP_Y_RATIO = 0.48
BEV_BOTTOM_Y_RATIO = 0.98

# half-widths of the source trapezoid in the ORIGINAL image
# expressed as fractions of full image width
BEV_TOP_HALF_WIDTH_RATIO = 0.22
BEV_BOTTOM_HALF_WIDTH_RATIO = 0.52

# destination rectangle horizontal margins in the BEV image
BEV_DST_LEFT_RATIO = 0.12
BEV_DST_RIGHT_RATIO = 0.88

# after BEV, ignore the unstable top region
BEV_VALID_TOP_RATIO = 0.12


class EdgeVectorsPublisher(Node):
    """Initializes edge vector publisher node with the required publishers and subscriptions."""

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

        self.image_height = 0
        self.image_width = 0

    def publish_debug_image(self, publisher, image):
        """Publishes images for debugging purposes."""
        message = CompressedImage()
        _, encoded_data = cv2.imencode('.jpg', image)
        message.format = "jpeg"
        message.data = encoded_data.tobytes()
        publisher.publish(message)

    def get_vector_angle_in_radians(self, vector):
        """Calculates vector angle in radians."""
        if (vector[0][0] - vector[1][0]) == 0:
            theta = PI / 2
        else:
            slope = (vector[1][1] - vector[0][1]) / (vector[0][0] - vector[1][0])
            theta = math.atan(slope)
        return theta

    def get_bev_points(self, width, height):
        """Builds source trapezoid and destination rectangle for BEV."""
        cx = width / 2.0

        top_y = int(height * BEV_TOP_Y_RATIO)
        bottom_y = int(height * BEV_BOTTOM_Y_RATIO)

        top_half_width = int(width * BEV_TOP_HALF_WIDTH_RATIO)
        bottom_half_width = int(width * BEV_BOTTOM_HALF_WIDTH_RATIO)

        src = np.float32([
            [cx - top_half_width, top_y],         # top-left
            [cx + top_half_width, top_y],         # top-right
            [cx + bottom_half_width, bottom_y],   # bottom-right
            [cx - bottom_half_width, bottom_y],   # bottom-left
        ])

        dst_left = int(width * BEV_DST_LEFT_RATIO)
        dst_right = int(width * BEV_DST_RIGHT_RATIO)

        dst = np.float32([
            [dst_left, 0],            # top-left
            [dst_right, 0],           # top-right
            [dst_right, height - 1],  # bottom-right
            [dst_left, height - 1],   # bottom-left
        ])

        return src, dst

    def compute_bev_image(self, image):
        """Applies bird's-eye-view perspective transform."""
        height, width = image.shape[:2]

        src, dst = self.get_bev_points(width, height)

        transform = cv2.getPerspectiveTransform(src, dst)

        # mask only the source trapezoid before warp
        source_mask = np.zeros((height, width), dtype=np.uint8)
        cv2.fillConvexPoly(source_mask, src.astype(np.int32), 255)

        masked_image = cv2.bitwise_and(image, image, mask=source_mask)

        bev_image = cv2.warpPerspective(masked_image, transform, (width, height))
        bev_mask = cv2.warpPerspective(source_mask, transform, (width, height))

        # original image with trapezoid drawn for debug
        source_debug = image.copy()
        cv2.polylines(source_debug, [src.astype(np.int32)], True, YELLOW_COLOR, 2)

        return bev_image, bev_mask, source_debug

    def compute_vectors_from_image(self, image, thresh):
        """Creates candidate vectors from contours."""
        contours = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)[0]

        image_height, image_width = thresh.shape[:2]
        rover_point = np.array([image_width / 2.0, image_height - 1], dtype=np.float32)

        vectors = []
        for contour in contours:
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

            # make endpoint choice more stable
            if angle > 0:
                min_y_coord[0] = np.max(min_y_coords[:, 0])
            else:
                max_y_coord[0] = np.max(max_y_coords[:, 0])

            vectors.append([list(min_y_coord), list(max_y_coord), distance, magnitude])

            cv2.line(image, tuple(min_y_coord), tuple(max_y_coord), BLUE_COLOR, 2)

        return vectors, image

    def choose_best_vector(self, candidates):
        """Chooses the most useful vector on one side."""
        if len(candidates) == 0:
            return None

        # prefer close vectors, but give some credit to longer ones
        return min(candidates, key=lambda v: (v[2] - 0.30 * v[3]))

    def process_image_for_edge_vectors(self, image):
        """Processes the image and creates vectors on the road edges using BEV."""
        self.image_height, self.image_width, _ = image.shape

        bev_image, bev_mask, source_debug = self.compute_bev_image(image)

        gray = cv2.cvtColor(bev_image, cv2.COLOR_BGR2GRAY)
        gray = cv2.GaussianBlur(gray, (5, 5), 0)

        thresh = cv2.threshold(gray, THRESHOLD_BLACK, 255, cv2.THRESH_BINARY_INV)[1]

        # remove invalid warped area
        thresh = cv2.bitwise_and(thresh, bev_mask)

        kernel = np.ones((MORPH_KERNEL_SIZE, MORPH_KERNEL_SIZE), np.uint8)

        # close first to connect broken turn edges, then open to remove noise
        thresh = cv2.morphologyEx(thresh, cv2.MORPH_CLOSE, kernel)
        thresh = cv2.morphologyEx(thresh, cv2.MORPH_OPEN, kernel)

        # keep mask enforced after morphology too
        thresh = cv2.bitwise_and(thresh, bev_mask)

        # ignore unstable top warped region
        valid_top = int(self.image_height * BEV_VALID_TOP_RATIO)
        thresh[:valid_top, :] = 0

        vectors, debug_image = self.compute_vectors_from_image(bev_image.copy(), thresh)

        half_width = self.image_width / 2.0
        vectors_left = [v for v in vectors if ((v[0][0] + v[1][0]) / 2.0) < half_width]
        vectors_right = [v for v in vectors if ((v[0][0] + v[1][0]) / 2.0) >= half_width]

        final_vectors = []

        left_best = self.choose_best_vector(vectors_left)
        right_best = self.choose_best_vector(vectors_right)

        for best_vector in [left_best, right_best]:
            if best_vector is not None:
                cv2.line(debug_image, tuple(best_vector[0]), tuple(best_vector[1]), GREEN_COLOR, 2)
                final_vectors.append(best_vector[:2])

        self.publish_debug_image(self.publisher_source_image, source_debug)
        self.publish_debug_image(self.publisher_bev_image, bev_image)
        self.publish_debug_image(self.publisher_thresh_image, thresh)
        self.publish_debug_image(self.publisher_vector_image, debug_image)

        return final_vectors

    def camera_image_callback(self, message):
        """Analyzes the image received from /camera/image_raw/compressed to detect road edges."""
        np_arr = np.frombuffer(message.data, np.uint8)
        image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

        if image is None:
            return

        vectors = self.process_image_for_edge_vectors(image)

        vectors_message = EdgeVectors()
        vectors_message.image_height = image.shape[0]
        vectors_message.image_width = image.shape[1]
        vectors_message.vector_count = 0

        if len(vectors) > 0:
            vectors_message.vector_1[0].x = float(vectors[0][0][0])
            vectors_message.vector_1[0].y = float(vectors[0][0][1])
            vectors_message.vector_1[1].x = float(vectors[0][1][0])
            vectors_message.vector_1[1].y = float(vectors[0][1][1])
            vectors_message.vector_count += 1

        if len(vectors) > 1:
            vectors_message.vector_2[0].x = float(vectors[1][0][0])
            vectors_message.vector_2[0].y = float(vectors[1][0][1])
            vectors_message.vector_2[1].x = float(vectors[1][1][0])
            vectors_message.vector_2[1].y = float(vectors[1][1][1])
            vectors_message.vector_count += 1

        self.publisher_edge_vectors.publish(vectors_message)
        return


def main(args=None):
    rclpy.init(args=args)
    edge_vectors_publisher = EdgeVectorsPublisher()
    rclpy.spin(edge_vectors_publisher)
    edge_vectors_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()