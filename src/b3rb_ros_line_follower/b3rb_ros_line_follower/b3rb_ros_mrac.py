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
import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy, LaserScan

from dataclasses import dataclass
from typing import Optional

from nav_msgs.msg import Odometry
from synapse_msgs.msg import EdgeVectors, TrafficStatus, Status

QOS_PROFILE_DEFAULT = 10

LEFT_TURN = +1.0
RIGHT_TURN = -1.0

TURN_MIN = -1.0
TURN_MAX = +1.0
SPEED_MIN = 0.0
SPEED_MAX = 1.0

# speed tuning
SPEED_SEARCH = 0.22
SPEED_ONE_EDGE = 0.30
SPEED_CURVE = 0.40
SPEED_STRAIGHT = 0.56

# lane estimation when only one edge is visible
DEFAULT_SINGLE_EDGE_LANE_HALF_WIDTH_RATIO = 0.42

# PID gains
KP_CENTER = 0.95
KI_CENTER = 0.02
KD_CENTER = 0.12

# preview / heading term
KP_HEADING = 0.45

# command smoothing
TURN_SMOOTHING_OLD = 0.70
TURN_SMOOTHING_NEW = 0.30
SPEED_SMOOTHING_OLD = 0.60
SPEED_SMOOTHING_NEW = 0.40

# anti-windup
INTEGRAL_LIMIT = 0.80

# measurement filtering
MAX_CENTER_JUMP_RATIO = 0.16
CENTER_FILTER_ALPHA_TWO_EDGES = 0.30
CENTER_FILTER_ALPHA_ONE_EDGE = 0.20

# ============================================================
# KINEMATIC BICYCLE SETTINGS
# ------------------------------------------------------------
# These are based on the teammate hardware bicycle model node:
# wheelbase = 0.168 m
# max steer = 30 deg
# low-speed deadzone = 0.05 m/s
#
# For the report's IV-A beta-based kinematic model, we split
# the wheelbase equally because lf/lr are not yet identified.
# ============================================================
KINEMATIC_WHEELBASE_M = 0.168
KINEMATIC_LF_M = 0.5 * KINEMATIC_WHEELBASE_M
KINEMATIC_LR_M = 0.5 * KINEMATIC_WHEELBASE_M
KINEMATIC_MAX_STEER_RAD = math.radians(30.0)
KINEMATIC_V_DEADZONE_MS = 0.05

# ==========================================
# SIGNAL NAMING CONVENTION
# ------------------------------------------
# _meas  : directly measured from topic/sensor
# _recon : reconstructed from measured signals
# _filt  : filtered/smoothed version
# _est   : model/observer/actuator estimate
#
# Examples:
# x_meas, y_meas, yaw_meas
# vx_recon, vy_recon, r_recon, a_long_recon
# ye_cam_filt, psi_rel_cam_filt
# delta_f_cmd_est, beta_kin_est
# ==========================================


@dataclass
class VehicleSignals:
    # direct odometry pose measurements
    x_meas: float = 0.0
    y_meas: float = 0.0
    yaw_meas: float = 0.0

    # reconstructed body-frame states from odometry pose differences
    vx_recon: float = 0.0
    vy_recon: float = 0.0
    r_recon: float = 0.0
    a_long_recon: float = 0.0

    # direct status measurements
    battery_pct_meas: Optional[float] = None
    battery_power_meas: Optional[float] = None

    odom_ready: bool = False
    status_ready: bool = False


@dataclass
class CameraMeasurement:
    have_measurement: bool = False
    use_integral: bool = False
    vector_count: int = 0

    # camera-derived filtered measurement-layer outputs
    ye_cam_filt: float = 0.0
    psi_rel_cam_filt: float = 0.0


@dataclass
class KinematicBicycleState:
    x: float = 0.0
    y: float = 0.0
    psi: float = 0.0
    v: float = 0.0


@dataclass
class KinematicBicycleDerivative:
    beta: float = 0.0
    x_dot: float = 0.0
    y_dot: float = 0.0
    psi_dot: float = 0.0
    v_dot: float = 0.0


class KinematicBicycleModel:
    """
    Report IV-A style kinematic bicycle model:

        beta    = atan( lr / (lf + lr) * tan(delta_f) )
        x_dot   = v * cos(psi + beta)
        y_dot   = v * sin(psi + beta)
        psi_dot = (v / lr) * sin(beta)
        v_dot   = a_long
    """

    def __init__(self, lf_m: float, lr_m: float):
        self.lf_m = lf_m
        self.lr_m = lr_m

    @staticmethod
    def wrap_angle(angle_rad: float) -> float:
        return math.atan2(math.sin(angle_rad), math.cos(angle_rad))

    def compute_beta(self, delta_f_rad: float) -> float:
        wheelbase_m = self.lf_m + self.lr_m
        if wheelbase_m <= 1e-9:
            return 0.0
        return math.atan((self.lr_m / wheelbase_m) * math.tan(delta_f_rad))

    def derivative(
        self,
        state: KinematicBicycleState,
        delta_f_rad: float,
        a_long_ms2: float
    ) -> KinematicBicycleDerivative:
        beta_rad = self.compute_beta(delta_f_rad)

        x_dot = state.v * math.cos(state.psi + beta_rad)
        y_dot = state.v * math.sin(state.psi + beta_rad)

        if self.lr_m <= 1e-9:
            psi_dot = 0.0
        else:
            psi_dot = (state.v / self.lr_m) * math.sin(beta_rad)

        v_dot = a_long_ms2

        return KinematicBicycleDerivative(
            beta=beta_rad,
            x_dot=x_dot,
            y_dot=y_dot,
            psi_dot=psi_dot,
            v_dot=v_dot
        )

    def step_euler(
        self,
        state: KinematicBicycleState,
        delta_f_rad: float,
        a_long_ms2: float,
        dt_s: float
    ):
        deriv = self.derivative(state, delta_f_rad, a_long_ms2)

        next_state = KinematicBicycleState(
            x=state.x + deriv.x_dot * dt_s,
            y=state.y + deriv.y_dot * dt_s,
            psi=self.wrap_angle(state.psi + deriv.psi_dot * dt_s),
            v=max(0.0, state.v + deriv.v_dot * dt_s)
        )

        return next_state, deriv


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

        self.vehicle = VehicleSignals()

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

        self.last_turn_cmd = 0.0
        self.last_speed_cmd = 0.0

        self.center_error_integral = 0.0
        self.prev_center_error = 0.0
        self.prev_control_time = time.monotonic()

        # filtered camera-side lane-center quantities
        self.center_top_filt = None
        self.center_bottom_filt = None
        self.lane_width_filt = None

        self.latest_camera_measurement = CameraMeasurement()

        # previous direct odometry pose samples used for reconstruction
        self.prev_x_meas = None
        self.prev_y_meas = None
        self.prev_yaw_meas = None
        self.prev_odom_time = None
        self.prev_vx_recon = None

        # =========================
        # kinematic bicycle model
        # =========================
        self.kinematic_model = KinematicBicycleModel(
            lf_m=KINEMATIC_LF_M,
            lr_m=KINEMATIC_LR_M
        )

        self.kinematic_state_est = None
        self.latest_kinematic_derivative = None

        # model-aligned estimated inputs
        self.delta_f_cmd_est = 0.0
        self.a_long_cmd_est = 0.0
        self.last_valid_delta_f_cmd_est = 0.0

    def clamp(self, value, min_value, max_value):
        """Clamps value into [min_value, max_value]."""
        return max(min_value, min(max_value, value))

    def rover_move_manual_mode(self, speed_cmd, turn_cmd):
        """Operates the rover in manual mode by publishing on /cerebri/in/joy."""
        msg = Joy()
        msg.buttons = [1, 0, 0, 0, 0, 0, 0, 1]
        msg.axes = [0.0, speed_cmd, 0.0, turn_cmd]
        self.publisher_joy.publish(msg)

    def get_dt(self):
        """Returns a safe controller timestep."""
        now = time.monotonic()
        dt = now - self.prev_control_time
        self.prev_control_time = now

        if dt <= 0.0 or dt > 0.2:
            dt = 0.05

        return dt

    def low_pass(self, previous_value, new_value, alpha):
        """Simple low-pass filter."""
        if previous_value is None:
            return new_value
        return (1.0 - alpha) * previous_value + alpha * new_value

    def estimate_center_from_one_edge(self, edge_top_x_meas, edge_bottom_x_meas, half_width):
        """Estimate lane center using one visible border."""
        if self.lane_width_filt is not None and self.lane_width_filt > 1.0:
            lane_half_width = 0.5 * self.lane_width_filt
        else:
            lane_half_width = DEFAULT_SINGLE_EDGE_LANE_HALF_WIDTH_RATIO * half_width

        edge_mid_x = 0.5 * (edge_top_x_meas + edge_bottom_x_meas)

        if edge_mid_x < half_width:
            # left border visible
            center_top_est = edge_top_x_meas + lane_half_width
            center_bottom_est = edge_bottom_x_meas + lane_half_width
        else:
            # right border visible
            center_top_est = edge_top_x_meas - lane_half_width
            center_bottom_est = edge_bottom_x_meas - lane_half_width

        return center_top_est, center_bottom_est

    def quaternion_to_yaw(self, q):
        """Convert quaternion to planar yaw angle."""
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def wrap_angle(self, angle):
        """Wrap angle to [-pi, pi]."""
        return math.atan2(math.sin(angle), math.cos(angle))

    def turn_cmd_to_delta_f_est(self, turn_cmd: float) -> float:
        """
        Map normalized steering command [-1, 1] to approximate front-wheel
        steering angle [rad], aligned with the hardware bicycle node's
        normalized steering convention.
        """
        turn_cmd = self.clamp(turn_cmd, -1.0, 1.0)
        return turn_cmd * KINEMATIC_MAX_STEER_RAD

    def update_kinematic_bicycle_model(self, dt_s: float):
        """
        Runs the kinematic bicycle model in parallel with the controller.

        To stay compatible with the hardware bicycle-node conventions:
        - use the same wheelbase / steering-limit scale
        - hold last valid delta when |v| is below the deadzone
        - use reconstructed forward speed and acceleration from odometry
        """
        if not self.vehicle.odom_ready:
            return

        speed_for_model = self.vehicle.vx_recon
        delta_candidate = self.turn_cmd_to_delta_f_est(self.last_turn_cmd)

        # mimic teammate hardware behavior: when speed is too low,
        # hold last valid steering angle
        if abs(speed_for_model) < KINEMATIC_V_DEADZONE_MS:
            speed_for_model = 0.0
            self.delta_f_cmd_est = self.last_valid_delta_f_cmd_est
        else:
            self.delta_f_cmd_est = delta_candidate
            self.last_valid_delta_f_cmd_est = delta_candidate

        self.a_long_cmd_est = self.vehicle.a_long_recon

        measured_state = KinematicBicycleState(
            x=self.vehicle.x_meas,
            y=self.vehicle.y_meas,
            psi=self.vehicle.yaw_meas,
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

    def extract_camera_measurement(self, vectors, half_width):
        """
        Extract camera-side measurements only.

        Returns:
            ye_cam_filt       -> approximate lateral deviation from track center
            psi_rel_cam_filt  -> approximate heading error relative to track tangent

        This function does NOT compute commands.
        """
        measurement = CameraMeasurement(
            have_measurement=False,
            use_integral=False,
            vector_count=vectors.vector_count,
            ye_cam_filt=0.0,
            psi_rel_cam_filt=0.0
        )

        if vectors.vector_count >= 2:
            # vector_1 = left edge, vector_2 = right edge
            left_top_x_meas = vectors.vector_1[0].x
            left_bottom_x_meas = vectors.vector_1[1].x
            right_top_x_meas = vectors.vector_2[0].x
            right_bottom_x_meas = vectors.vector_2[1].x

            center_top_meas = 0.5 * (left_top_x_meas + right_top_x_meas)
            center_bottom_meas = 0.5 * (left_bottom_x_meas + right_bottom_x_meas)
            lane_width_meas = right_bottom_x_meas - left_bottom_x_meas

            if lane_width_meas > 1.0:
                if self.lane_width_filt is None:
                    self.lane_width_filt = lane_width_meas
                else:
                    self.lane_width_filt = 0.80 * self.lane_width_filt + 0.20 * lane_width_meas

            max_jump = MAX_CENTER_JUMP_RATIO * half_width

            if self.center_top_filt is not None:
                jump_top = center_top_meas - self.center_top_filt
                center_top_meas = self.center_top_filt + self.clamp(jump_top, -max_jump, max_jump)

            if self.center_bottom_filt is not None:
                jump_bottom = center_bottom_meas - self.center_bottom_filt
                center_bottom_meas = self.center_bottom_filt + self.clamp(jump_bottom, -max_jump, max_jump)

            self.center_top_filt = self.low_pass(
                self.center_top_filt,
                center_top_meas,
                CENTER_FILTER_ALPHA_TWO_EDGES
            )
            self.center_bottom_filt = self.low_pass(
                self.center_bottom_filt,
                center_bottom_meas,
                CENTER_FILTER_ALPHA_TWO_EDGES
            )

            measurement.have_measurement = True
            measurement.use_integral = True

        elif vectors.vector_count == 1:
            edge_top_x_meas = vectors.vector_1[0].x
            edge_bottom_x_meas = vectors.vector_1[1].x

            center_top_est, center_bottom_est = self.estimate_center_from_one_edge(
                edge_top_x_meas,
                edge_bottom_x_meas,
                half_width
            )

            max_jump = MAX_CENTER_JUMP_RATIO * half_width

            if self.center_top_filt is not None:
                jump_top = center_top_est - self.center_top_filt
                center_top_est = self.center_top_filt + self.clamp(jump_top, -max_jump, max_jump)

            if self.center_bottom_filt is not None:
                jump_bottom = center_bottom_est - self.center_bottom_filt
                center_bottom_est = self.center_bottom_filt + self.clamp(jump_bottom, -max_jump, max_jump)

            self.center_top_filt = self.low_pass(
                self.center_top_filt,
                center_top_est,
                CENTER_FILTER_ALPHA_ONE_EDGE
            )
            self.center_bottom_filt = self.low_pass(
                self.center_bottom_filt,
                center_bottom_est,
                CENTER_FILTER_ALPHA_ONE_EDGE
            )

            measurement.have_measurement = True
            measurement.use_integral = False

        if measurement.have_measurement:
            measurement.ye_cam_filt = (half_width - self.center_bottom_filt) / half_width
            measurement.psi_rel_cam_filt = (self.center_bottom_filt - self.center_top_filt) / half_width

        self.latest_camera_measurement = measurement
        return measurement

    def edge_vectors_callback(self, message):
        """Uses camera measurements to compute speed and turn."""
        vectors = message
        half_width = vectors.image_width / 2.0 if vectors.image_width > 0 else 1.0
        dt_s = self.get_dt()

        raw_turn_cmd = self.last_turn_cmd
        target_speed_cmd = SPEED_SEARCH

        camera = self.extract_camera_measurement(vectors, half_width)

        if camera.have_measurement:
            ye_cam_filt = camera.ye_cam_filt
            psi_rel_cam_filt = camera.psi_rel_cam_filt

            if camera.use_integral:
                self.center_error_integral += ye_cam_filt * dt_s
                self.center_error_integral = self.clamp(
                    self.center_error_integral,
                    -INTEGRAL_LIMIT,
                    INTEGRAL_LIMIT
                )
            else:
                self.center_error_integral *= 0.90

            center_error_derivative = (ye_cam_filt - self.prev_center_error) / dt_s
            self.prev_center_error = ye_cam_filt

            raw_turn_cmd = (
                KP_CENTER * ye_cam_filt +
                KI_CENTER * self.center_error_integral +
                KD_CENTER * center_error_derivative +
                KP_HEADING * psi_rel_cam_filt
            )

            raw_turn_cmd = self.clamp(raw_turn_cmd, TURN_MIN, TURN_MAX)

            turn_strength = abs(raw_turn_cmd)

            if camera.vector_count >= 2:
                if turn_strength < 0.10:
                    target_speed_cmd = SPEED_STRAIGHT
                elif turn_strength < 0.24:
                    target_speed_cmd = 0.48
                else:
                    target_speed_cmd = SPEED_CURVE
            else:
                target_speed_cmd = SPEED_ONE_EDGE

        else:
            # no edges visible: hold last steering gently and move slowly
            self.center_error_integral *= 0.85
            self.prev_center_error *= 0.85
            raw_turn_cmd = self.last_turn_cmd * 0.96
            target_speed_cmd = SPEED_SEARCH

        # keep lidar disabled for Raceway_1
        if self.ramp_detected:
            target_speed_cmd = max(target_speed_cmd, SPEED_ONE_EDGE)

        if self.traffic_status.stop_sign:
            target_speed_cmd = SPEED_MIN
            self.center_error_integral = 0.0

        if self.obstacle_detected:
            target_speed_cmd = SPEED_MIN
            self.center_error_integral = 0.0

        turn_cmd = TURN_SMOOTHING_OLD * self.last_turn_cmd + TURN_SMOOTHING_NEW * raw_turn_cmd
        turn_cmd = self.clamp(turn_cmd, RIGHT_TURN, LEFT_TURN)

        if target_speed_cmd == SPEED_MIN:
            speed_cmd = SPEED_MIN
        else:
            speed_cmd = SPEED_SMOOTHING_OLD * self.last_speed_cmd + SPEED_SMOOTHING_NEW * target_speed_cmd
            speed_cmd = self.clamp(speed_cmd, SPEED_MIN, SPEED_MAX)

        self.last_turn_cmd = turn_cmd
        self.last_speed_cmd = speed_cmd

        # run the kinematic bicycle model in parallel for validation
        self.update_kinematic_bicycle_model(dt_s)

        self.rover_move_manual_mode(speed_cmd, turn_cmd)

    def odometry_callback(self, message):
        """
        Stores odometry-related signals for later MRAC use.

        Directly measured from odometry pose:
        - x_meas, y_meas, yaw_meas

        Reconstructed from pose differences:
        - vx_recon, vy_recon, r_recon, a_long_recon
        """
        x_meas = float(message.pose.pose.position.x)
        y_meas = float(message.pose.pose.position.y)
        yaw_meas = self.quaternion_to_yaw(message.pose.pose.orientation)

        t_meas = float(message.header.stamp.sec) + 1e-9 * float(message.header.stamp.nanosec)

        self.vehicle.x_meas = x_meas
        self.vehicle.y_meas = y_meas
        self.vehicle.yaw_meas = yaw_meas

        if (
            self.prev_x_meas is not None and
            self.prev_y_meas is not None and
            self.prev_yaw_meas is not None and
            self.prev_odom_time is not None
        ):
            dt_s = t_meas - self.prev_odom_time

            if dt_s > 1e-4:
                vx_world = (x_meas - self.prev_x_meas) / dt_s
                vy_world = (y_meas - self.prev_y_meas) / dt_s

                dyaw = self.wrap_angle(yaw_meas - self.prev_yaw_meas)
                r_recon = dyaw / dt_s

                vx_recon = math.cos(yaw_meas) * vx_world + math.sin(yaw_meas) * vy_world
                vy_recon = -math.sin(yaw_meas) * vx_world + math.cos(yaw_meas) * vy_world

                if self.prev_vx_recon is None:
                    a_long_recon = 0.0
                else:
                    a_long_recon = (vx_recon - self.prev_vx_recon) / dt_s

                self.vehicle.vx_recon = vx_recon
                self.vehicle.vy_recon = vy_recon
                self.vehicle.r_recon = r_recon
                self.vehicle.a_long_recon = a_long_recon

                self.prev_vx_recon = vx_recon

        self.prev_x_meas = x_meas
        self.prev_y_meas = y_meas
        self.prev_yaw_meas = yaw_meas
        self.prev_odom_time = t_meas

        self.vehicle.odom_ready = True

    def status_callback(self, message):
        """Stores battery / vehicle status signals for later MRAC use."""
        fuel_percentage_meas = getattr(message, 'fuel_percentage', None)
        power_meas = getattr(message, 'power', None)

        self.vehicle.battery_pct_meas = float(fuel_percentage_meas) if fuel_percentage_meas is not None else None
        self.vehicle.battery_power_meas = float(power_meas) if power_meas is not None else None
        self.vehicle.status_ready = True

    def debug_timer_callback(self):
        """Prints the incoming MRAC-related measurements every 0.5 s."""
        if self.vehicle.odom_ready:
            if self.latest_camera_measurement.have_measurement:
                ye_text = f"{self.latest_camera_measurement.ye_cam_filt:.3f}"
                psi_text = f"{self.latest_camera_measurement.psi_rel_cam_filt:.3f}"
            else:
                ye_text = "None"
                psi_text = "None"

            if self.latest_kinematic_derivative is not None:
                beta_text = f"{self.latest_kinematic_derivative.beta:.3f}"
                xdot_text = f"{self.latest_kinematic_derivative.x_dot:.3f}"
                ydot_text = f"{self.latest_kinematic_derivative.y_dot:.3f}"
                psidot_text = f"{self.latest_kinematic_derivative.psi_dot:.3f}"
                vdot_text = f"{self.latest_kinematic_derivative.v_dot:.3f}"
            else:
                beta_text = "None"
                xdot_text = "None"
                ydot_text = "None"
                psidot_text = "None"
                vdot_text = "None"

            if self.kinematic_state_est is not None:
                kin_x_text = f"{self.kinematic_state_est.x:.3f}"
                kin_y_text = f"{self.kinematic_state_est.y:.3f}"
                kin_psi_text = f"{self.kinematic_state_est.psi:.3f}"
                kin_v_text = f"{self.kinematic_state_est.v:.3f}"
            else:
                kin_x_text = "None"
                kin_y_text = "None"
                kin_psi_text = "None"
                kin_v_text = "None"

            self.get_logger().info(
                f"[MRAC scaffold] "
                f"x_meas={self.vehicle.x_meas:.3f}, "
                f"y_meas={self.vehicle.y_meas:.3f}, "
                f"yaw_meas={self.vehicle.yaw_meas:.3f}, "
                f"vx_recon={self.vehicle.vx_recon:.3f}, "
                f"vy_recon={self.vehicle.vy_recon:.3f}, "
                f"r_recon={self.vehicle.r_recon:.3f}, "
                f"a_long_recon={self.vehicle.a_long_recon:.3f}, "
                f"ye_cam_filt={ye_text}, "
                f"psi_rel_cam_filt={psi_text}, "
                f"delta_f_cmd_est={self.delta_f_cmd_est:.3f}, "
                f"beta_kin={beta_text}, "
                f"x_dot_kin={xdot_text}, "
                f"y_dot_kin={ydot_text}, "
                f"psi_dot_kin={psidot_text}, "
                f"v_dot_kin={vdot_text}, "
                f"kin_x_est={kin_x_text}, "
                f"kin_y_est={kin_y_text}, "
                f"kin_psi_est={kin_psi_text}, "
                f"kin_v_est={kin_v_text}, "
                f"battery_pct_meas={self.vehicle.battery_pct_meas}, "
                f"battery_power_meas={self.vehicle.battery_power_meas}"
            )
        else:
            self.get_logger().info("[MRAC scaffold] waiting for odometry...")

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