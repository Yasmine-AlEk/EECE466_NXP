
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

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy, LaserScan
from nav_msgs.msg import Odometry
from synapse_msgs.msg import EdgeVectors, TrafficStatus, Status

from .controllers.baseline_lane_controller import BaselineLaneController
from .debug.debug_snapshot import build_debug_snapshot
from .estimation.vehicle_state_estimator import VehicleStateEstimator
from .models.kinematic_bicycle import KinematicBicycleModel
from .mrac_config import (
    KINEMATIC_LF_M,
    KINEMATIC_LR_M,
    KINEMATIC_V_DEADZONE_MS,
    QOS_PROFILE_DEFAULT,
)
from .mrac_types import KinematicBicycleState
from .mrac_utils import turn_cmd_to_delta_f_est
from .perception.camera_measurements import CameraMeasurementExtractor


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

        self.subscription_odom = self.create_subscription(
            Odometry,
            '/cerebri/out/odometry',
            self.odometry_callback,
            QOS_PROFILE_DEFAULT
        )

        self.subscription_status = self.create_subscription(
            Status,
            '/cerebri/out/status',
            self.status_callback,
            QOS_PROFILE_DEFAULT
        )

        self.debug_timer = self.create_timer(
            0.5,
            self.debug_timer_callback
        )

        self.traffic_status = TrafficStatus()
        self.obstacle_detected = False
        self.ramp_detected = False

        self.estimator = VehicleStateEstimator()
        self.camera_extractor = CameraMeasurementExtractor()
        self.controller = BaselineLaneController()

        self.kinematic_model = KinematicBicycleModel(
            lf_m=KINEMATIC_LF_M,
            lr_m=KINEMATIC_LR_M
        )

        self.kinematic_state_est = None
        self.latest_kinematic_derivative = None
        self.latest_debug_snapshot = None

        self.delta_f_cmd_est = 0.0
        self.a_long_cmd_est = 0.0
        self.last_valid_delta_f_cmd_est = 0.0

    def rover_move_manual_mode(self, speed_cmd, turn_cmd):
        """Operates the rover in manual mode by publishing on /cerebri/in/joy."""
        msg = Joy()
        msg.buttons = [1, 0, 0, 0, 0, 0, 0, 1]
        msg.axes = [0.0, speed_cmd, 0.0, turn_cmd]
        self.publisher_joy.publish(msg)

    def update_debug_snapshot(self):
        self.latest_debug_snapshot = build_debug_snapshot(
            vehicle=self.estimator.vehicle,
            camera_measurement=self.camera_extractor.latest_camera_measurement,
            kinematic_state_est=self.kinematic_state_est,
            kinematic_derivative=self.latest_kinematic_derivative,
            turn_cmd=self.controller.last_turn_cmd,
            delta_f_cmd_est=self.delta_f_cmd_est,
        )

    def update_kinematic_bicycle_model(self, dt_s: float):
        """
        Runs the kinematic bicycle model in parallel with the controller.

        To stay compatible with the hardware bicycle-node conventions:
        - use the same wheelbase / steering-limit scale
        - hold last valid delta when |v| is below the deadzone
        - use reconstructed forward speed and filtered acceleration from odometry
        """
        vehicle = self.estimator.vehicle
        if not vehicle.odom_ready:
            return

        speed_for_model = vehicle.vx_recon
        delta_candidate = turn_cmd_to_delta_f_est(self.controller.last_turn_cmd)

        if abs(speed_for_model) < KINEMATIC_V_DEADZONE_MS:
            speed_for_model = 0.0
            self.delta_f_cmd_est = self.last_valid_delta_f_cmd_est
        else:
            self.delta_f_cmd_est = delta_candidate
            self.last_valid_delta_f_cmd_est = delta_candidate

        self.a_long_cmd_est = vehicle.a_long_filt

        measured_state = KinematicBicycleState(
            x=vehicle.x_meas,
            y=vehicle.y_meas,
            psi=vehicle.yaw_meas,
            v=max(0.0, speed_for_model)
        )

        self.latest_kinematic_derivative = self.kinematic_model.derivative(
            state=measured_state,
            delta_f_rad=self.delta_f_cmd_est,
            a_long_ms2=self.a_long_cmd_est
        )

        if self.kinematic_state_est is None:
            self.kinematic_state_est = measured_state
        else:
            self.kinematic_state_est, _ = self.kinematic_model.step_euler(
                state=self.kinematic_state_est,
                delta_f_rad=self.delta_f_cmd_est,
                a_long_ms2=self.a_long_cmd_est,
                dt_s=dt_s
            )

        self.update_debug_snapshot()

    def edge_vectors_callback(self, message):
        vectors = message
        half_width = vectors.image_width / 2.0 if vectors.image_width > 0 else 1.0

        camera = self.camera_extractor.extract(vectors, half_width)

        turn_cmd, speed_cmd, dt_s = self.controller.compute(
            camera=camera,
            stop_sign=getattr(self.traffic_status, 'stop_sign', False),
            obstacle_detected=self.obstacle_detected,
            ramp_detected=self.ramp_detected,
        )

        self.update_kinematic_bicycle_model(dt_s)
        self.rover_move_manual_mode(speed_cmd, turn_cmd)

    def odometry_callback(self, message):
        self.estimator.update_from_odometry(message)

    def status_callback(self, message):
        self.estimator.update_from_status(message)

    def debug_timer_callback(self):
        if not self.estimator.vehicle.odom_ready:
            self.get_logger().info("[MRAC scaffold] waiting for odometry...")
            return

        if self.latest_debug_snapshot is None:
            self.get_logger().info("[MRAC scaffold] waiting for first debug snapshot...")
            return

        if len(self.latest_debug_snapshot) == 0:
            self.get_logger().info("[MRAC scaffold] no debug fields enabled")
            return

        self.get_logger().info("[MRAC scaffold] " + ", ".join(self.latest_debug_snapshot))

    def traffic_status_callback(self, message):
        self.traffic_status = message

    def lidar_callback(self, message):
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
