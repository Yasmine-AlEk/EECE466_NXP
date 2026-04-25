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

import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy, LaserScan

from synapse_msgs.msg import EdgeVectors, TrafficStatus

QOS_PROFILE_DEFAULT = 10

LEFT_TURN = +1.0
RIGHT_TURN = -1.0

TURN_MIN = -1.0
TURN_MAX = +1.0
SPEED_MIN = 0.0
SPEED_MAX = 1.0

#speed tuning
SPEED_SEARCH = 0.22
SPEED_ONE_EDGE = 0.30
SPEED_CURVE = 0.40
SPEED_STRAIGHT = 0.56

#lane estimation when only one edge is visible
DEFAULT_SINGLE_EDGE_LANE_HALF_WIDTH_RATIO = 0.42

#PID gains
KP_CENTER = 0.95
KI_CENTER = 0.02
KD_CENTER = 0.12

#preview / heading term
KP_HEADING = 0.45

#command smoothing
TURN_SMOOTHING_OLD = 0.70
TURN_SMOOTHING_NEW = 0.30
SPEED_SMOOTHING_OLD = 0.60
SPEED_SMOOTHING_NEW = 0.40

#anti-windup
INTEGRAL_LIMIT = 0.80

#measurement filtering
MAX_CENTER_JUMP_RATIO = 0.16
CENTER_FILTER_ALPHA_TWO_EDGES = 0.30
CENTER_FILTER_ALPHA_ONE_EDGE = 0.20


class LineFollower(Node):
    """Initializes line follower node with the required publishers and subscriptions."""

    def __init__(self):
        super().__init__('line_follower')

        self.subscription_vectors = self.create_subscription(
            EdgeVectors,
            '/edge_vectors',
            self.edge_vectors_callback,
            QOS_PROFILE_DEFAULT
        )

        self.publisher_joy = self.create_publisher(
            Joy,
            '/cerebri/in/joy',
            QOS_PROFILE_DEFAULT
        )

        self.subscription_traffic = self.create_subscription(
            TrafficStatus,
            '/traffic_status',
            self.traffic_status_callback,
            QOS_PROFILE_DEFAULT
        )

        self.subscription_lidar = self.create_subscription(
            LaserScan,
            '/scan',
            self.lidar_callback,
            QOS_PROFILE_DEFAULT
        )

        self.traffic_status = TrafficStatus()
        self.obstacle_detected = False
        self.ramp_detected = False

        self.last_turn = 0.0
        self.last_speed = 0.0

        self.center_error_integral = 0.0
        self.prev_center_error = 0.0
        self.prev_time = time.monotonic()

        self.filtered_center_top = None
        self.filtered_center_bottom = None
        self.last_lane_width = None

    def clamp(self, value, min_value, max_value):
        """Clamps value into [min_value, max_value]."""
        return max(min_value, min(max_value, value))

    def rover_move_manual_mode(self, speed, turn):
        """Operates the rover in manual mode by publishing on /cerebri/in/joy."""
        msg = Joy()
        msg.buttons = [1, 0, 0, 0, 0, 0, 0, 1]
        msg.axes = [0.0, speed, 0.0, turn]
        self.publisher_joy.publish(msg)

    def get_dt(self):
        """Returns a safe controller timestep."""
        now = time.monotonic()
        dt = now - self.prev_time
        self.prev_time = now

        if dt <= 0.0 or dt > 0.2:
            dt = 0.05

        return dt

    def low_pass(self, previous_value, new_value, alpha):
        """Simple low-pass filter."""
        if previous_value is None:
            return new_value
        return (1.0 - alpha) * previous_value + alpha * new_value

    def estimate_center_from_one_edge(self, edge_top_x, edge_bottom_x, half_width):
        """Estimate lane center using one visible border."""
        if self.last_lane_width is not None and self.last_lane_width > 1.0:
            lane_half_width = 0.5 * self.last_lane_width
        else:
            lane_half_width = DEFAULT_SINGLE_EDGE_LANE_HALF_WIDTH_RATIO * half_width

        edge_mid_x = 0.5 * (edge_top_x + edge_bottom_x)

        if edge_mid_x < half_width:
            #left border visible
            center_top = edge_top_x + lane_half_width
            center_bottom = edge_bottom_x + lane_half_width
        else:
            #right border visible
            center_top = edge_top_x - lane_half_width
            center_bottom = edge_bottom_x - lane_half_width

        return center_top, center_bottom

    def edge_vectors_callback(self, message):
        """Uses edge vectors to compute speed and turn."""
        vectors = message
        half_width = vectors.image_width / 2.0 if vectors.image_width > 0 else 1.0
        dt = self.get_dt()

        raw_turn = self.last_turn
        target_speed = SPEED_SEARCH
        have_measurement = False
        use_integral = False

        if vectors.vector_count >= 2:
            #vector_1 = left edge, vector_2 = right edge
            left_top_x = vectors.vector_1[0].x
            left_bottom_x = vectors.vector_1[1].x
            right_top_x = vectors.vector_2[0].x
            right_bottom_x = vectors.vector_2[1].x

            measured_center_top = 0.5 * (left_top_x + right_top_x)
            measured_center_bottom = 0.5 * (left_bottom_x + right_bottom_x)
            measured_lane_width = right_bottom_x - left_bottom_x

            if measured_lane_width > 1.0:
                if self.last_lane_width is None:
                    self.last_lane_width = measured_lane_width
                else:
                    self.last_lane_width = 0.80 * self.last_lane_width + 0.20 * measured_lane_width

            max_jump = MAX_CENTER_JUMP_RATIO * half_width

            if self.filtered_center_top is not None:
                jump_top = measured_center_top - self.filtered_center_top
                measured_center_top = self.filtered_center_top + self.clamp(jump_top, -max_jump, max_jump)

            if self.filtered_center_bottom is not None:
                jump_bottom = measured_center_bottom - self.filtered_center_bottom
                measured_center_bottom = self.filtered_center_bottom + self.clamp(jump_bottom, -max_jump, max_jump)

            self.filtered_center_top = self.low_pass(
                self.filtered_center_top,
                measured_center_top,
                CENTER_FILTER_ALPHA_TWO_EDGES
            )
            self.filtered_center_bottom = self.low_pass(
                self.filtered_center_bottom,
                measured_center_bottom,
                CENTER_FILTER_ALPHA_TWO_EDGES
            )

            have_measurement = True
            use_integral = True

        elif vectors.vector_count == 1:
            edge_top_x = vectors.vector_1[0].x
            edge_bottom_x = vectors.vector_1[1].x

            measured_center_top, measured_center_bottom = self.estimate_center_from_one_edge(
                edge_top_x,
                edge_bottom_x,
                half_width
            )

            max_jump = MAX_CENTER_JUMP_RATIO * half_width

            if self.filtered_center_top is not None:
                jump_top = measured_center_top - self.filtered_center_top
                measured_center_top = self.filtered_center_top + self.clamp(jump_top, -max_jump, max_jump)

            if self.filtered_center_bottom is not None:
                jump_bottom = measured_center_bottom - self.filtered_center_bottom
                measured_center_bottom = self.filtered_center_bottom + self.clamp(jump_bottom, -max_jump, max_jump)

            self.filtered_center_top = self.low_pass(
                self.filtered_center_top,
                measured_center_top,
                CENTER_FILTER_ALPHA_ONE_EDGE
            )
            self.filtered_center_bottom = self.low_pass(
                self.filtered_center_bottom,
                measured_center_bottom,
                CENTER_FILTER_ALPHA_ONE_EDGE
            )

            have_measurement = True
            use_integral = False

        else:
            #no edges visible: hold last steering gently and move slowly
            self.center_error_integral *= 0.85
            self.prev_center_error *= 0.85
            raw_turn = self.last_turn * 0.96
            target_speed = SPEED_SEARCH

        if have_measurement:
            center_error = (half_width - self.filtered_center_bottom) / half_width
            heading_error = (self.filtered_center_bottom - self.filtered_center_top) / half_width

            if use_integral:
                self.center_error_integral += center_error * dt
                self.center_error_integral = self.clamp(
                    self.center_error_integral,
                    -INTEGRAL_LIMIT,
                    INTEGRAL_LIMIT
                )
            else:
                self.center_error_integral *= 0.90

            derivative = (center_error - self.prev_center_error) / dt
            self.prev_center_error = center_error

            raw_turn = (
                KP_CENTER * center_error +
                KI_CENTER * self.center_error_integral +
                KD_CENTER * derivative +
                KP_HEADING * heading_error
            )

            raw_turn = self.clamp(raw_turn, TURN_MIN, TURN_MAX)

            turn_strength = abs(raw_turn)

            if vectors.vector_count >= 2:
                if turn_strength < 0.10:
                    target_speed = SPEED_STRAIGHT
                elif turn_strength < 0.24:
                    target_speed = 0.48
                else:
                    target_speed = SPEED_CURVE
            else:
                target_speed = SPEED_ONE_EDGE

        #keep lidar disabled for Raceway_1
        if self.ramp_detected:
            target_speed = max(target_speed, SPEED_ONE_EDGE)

        if self.traffic_status.stop_sign:
            target_speed = SPEED_MIN
            self.center_error_integral = 0.0

        if self.obstacle_detected:
            target_speed = SPEED_MIN
            self.center_error_integral = 0.0

        turn = TURN_SMOOTHING_OLD * self.last_turn + TURN_SMOOTHING_NEW * raw_turn
        turn = self.clamp(turn, RIGHT_TURN, LEFT_TURN)

        if target_speed == SPEED_MIN:
            speed = SPEED_MIN
        else:
            speed = SPEED_SMOOTHING_OLD * self.last_speed + SPEED_SMOOTHING_NEW * target_speed
            speed = self.clamp(speed, SPEED_MIN, SPEED_MAX)

        self.last_turn = turn
        self.last_speed = speed

        self.rover_move_manual_mode(speed, turn)

    def traffic_status_callback(self, message):
        """Updates instance member with traffic status."""
        self.traffic_status = message

    def lidar_callback(self, message):
        """Keep lidar blocking disabled for Raceway_1."""
        self.obstacle_detected = False
        self.ramp_detected = False
        return


def main(args=None):
    rclpy.init(args=args)
    line_follower = LineFollower()
    rclpy.spin(line_follower)
    line_follower.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()