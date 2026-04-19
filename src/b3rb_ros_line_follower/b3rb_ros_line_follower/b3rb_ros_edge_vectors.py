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

#slightly taller ROI so the car sees the turn after the ramp
VECTOR_IMAGE_HEIGHT_PERCENTAGE = 0.58

#ignore tiny noisy contours
VECTOR_MAGNITUDE_MINIMUM = 5.0


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

        self.image_height = 0
        self.image_width = 0
        self.lower_image_height = 0
        self.upper_image_height = 0

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

    def compute_vectors_from_image(self, image, thresh):
        """Creates candidate vectors from contours."""
        contours = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)[0]

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

            rover_point = [self.image_width / 2, self.lower_image_height]
            middle_point = (min_y_coord + max_y_coord) / 2
            distance = np.linalg.norm(middle_point - rover_point)

            angle = self.get_vector_angle_in_radians([min_y_coord, max_y_coord])

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

        #prefer close vectors, but give some credit to longer ones
        return min(candidates, key=lambda v: (v[2] - 0.30 * v[3]))

    def process_image_for_edge_vectors(self, image):
        """Processes the image and creates vectors on the road edges."""
        self.image_height, self.image_width, _ = image.shape
        self.lower_image_height = int(self.image_height * VECTOR_IMAGE_HEIGHT_PERCENTAGE)
        self.upper_image_height = int(self.image_height - self.lower_image_height)

        roi = image[self.image_height - self.lower_image_height:]
        gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
        gray = cv2.GaussianBlur(gray, (5, 5), 0)

        threshold_black = 90
        thresh = cv2.threshold(gray, threshold_black, 255, cv2.THRESH_BINARY_INV)[1]

        kernel = np.ones((3, 3), np.uint8)
        thresh = cv2.morphologyEx(thresh, cv2.MORPH_OPEN, kernel)
        thresh = cv2.morphologyEx(thresh, cv2.MORPH_CLOSE, kernel)

        vectors, debug_image = self.compute_vectors_from_image(roi.copy(), thresh)

        half_width = self.image_width / 2.0
        vectors_left = [v for v in vectors if ((v[0][0] + v[1][0]) / 2.0) < half_width]
        vectors_right = [v for v in vectors if ((v[0][0] + v[1][0]) / 2.0) >= half_width]

        final_vectors = []

        left_best = self.choose_best_vector(vectors_left)
        right_best = self.choose_best_vector(vectors_right)

        for best_vector in [left_best, right_best]:
            if best_vector is not None:
                cv2.line(debug_image, tuple(best_vector[0]), tuple(best_vector[1]), GREEN_COLOR, 2)
                best_vector[0][1] += self.upper_image_height
                best_vector[1][1] += self.upper_image_height
                final_vectors.append(best_vector[:2])

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
