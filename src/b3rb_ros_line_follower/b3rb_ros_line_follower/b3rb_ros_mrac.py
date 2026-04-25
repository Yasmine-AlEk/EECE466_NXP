# Copyright 2024 NXP

# Copyright 2016 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy, LaserScan
from nav_msgs.msg import Odometry
from synapse_msgs.msg import EdgeVectors, TrafficStatus, Status

from .controllers.baseline_lane_controller import BaselineLaneController
from .debug.debug_snapshot import build_debug_snapshot
from .estimation.vehicle_state_estimator import VehicleStateEstimator
from .models.actuator_models import PropulsionForceModel, SteeringActuatorModel
from .models.dynamic_bicycle import DynamicBicycleModel
from .models.kinematic_bicycle import KinematicBicycleModel
from .models.state_space_vehicle import StateSpaceVehicleModel
from .models.wheel_force_decomposition import WheelForceDecompositionModel
from .mrac_config import (
    BATTERY_MAX_V,
    BATTERY_MIN_V,
    BATTERY_NOMINAL_V,
    DYNAMIC_BICYCLE_IZ_KG_M2,
    DYNAMIC_BICYCLE_LF_M,
    DYNAMIC_BICYCLE_LR_M,
    DYNAMIC_BICYCLE_MASS_KG,
    DYNAMIC_DRAG_FORCE_N,
    DYNAMIC_MODEL_MAX_DT_S,
    DYNAMIC_RAMP_FORCE_N,
    DYNAMIC_TIRE_FORCE_PAIR_COUNT,
    DYNAMIC_TORQUE_VECTORING_YAW_MOMENT_N_M,
    FRONT_CORNERING_STIFFNESS_N_PER_RAD,
    KINEMATIC_LF_M,
    KINEMATIC_LR_M,
    KINEMATIC_V_DEADZONE_MS,
    PROPULSION_EFFICIENCY,
    PROPULSION_FORCE_GAIN_N_PER_V,
    PROPULSION_FORCE_MAX_ABS_N,
    PROPULSION_MOTOR_TAU_S,
    PROPULSION_VOLTAGE_SAG_AT_FULL_PWM,
    QOS_PROFILE_DEFAULT,
    REAR_CORNERING_STIFFNESS_N_PER_RAD,
    REAR_TRACK_WIDTH_M,
    SLIP_ANGLE_MAX_ABS_RAD,
    SLIP_ANGLE_MIN_VX_MS,
    STEERING_SERVO_MAX_STEER_RAD,
    STEERING_SERVO_TAU_S,
    STATE_SPACE_K_DRAG_COEFF,
    STATE_SPACE_MIN_VX_MS,
    TIRE_FORCE_MAX_ABS_N,
    TORQUE_VECTORING_DFX_CMD_N,
)
from .mrac_types import (
    DynamicBicycleInput,
    DynamicBicycleState,
    KinematicBicycleState,
)
from .mrac_utils import turn_cmd_to_delta_f_est
from .perception.camera_measurements import CameraMeasurementExtractor


class LineFollower(Node):
    """Line follower node with baseline control and MRAC/model scaffolding."""

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

        self.dynamic_model = DynamicBicycleModel(
            mass_kg=DYNAMIC_BICYCLE_MASS_KG,
            iz_kg_m2=DYNAMIC_BICYCLE_IZ_KG_M2,
            lf_m=DYNAMIC_BICYCLE_LF_M,
            lr_m=DYNAMIC_BICYCLE_LR_M,
            force_pair_count=DYNAMIC_TIRE_FORCE_PAIR_COUNT,
            max_dt_s=DYNAMIC_MODEL_MAX_DT_S,
            min_vx_for_slip_ms=SLIP_ANGLE_MIN_VX_MS,
            max_abs_slip_rad=SLIP_ANGLE_MAX_ABS_RAD,
            front_cornering_stiffness_n_per_rad=(
                FRONT_CORNERING_STIFFNESS_N_PER_RAD
            ),
            rear_cornering_stiffness_n_per_rad=(
                REAR_CORNERING_STIFFNESS_N_PER_RAD
            ),
            max_abs_tire_force_n=TIRE_FORCE_MAX_ABS_N,
        )

        self.steering_actuator_model = SteeringActuatorModel(
            tau_s=STEERING_SERVO_TAU_S,
            max_abs_delta_rad=STEERING_SERVO_MAX_STEER_RAD,
        )

        self.propulsion_model = PropulsionForceModel(
            tau_s=PROPULSION_MOTOR_TAU_S,
            efficiency=PROPULSION_EFFICIENCY,
            force_gain_n_per_v=PROPULSION_FORCE_GAIN_N_PER_V,
            voltage_sag_at_full_pwm_v=PROPULSION_VOLTAGE_SAG_AT_FULL_PWM,
            max_abs_force_n=PROPULSION_FORCE_MAX_ABS_N,
        )

        self.wheel_force_model = WheelForceDecompositionModel(
            rear_track_width_m=REAR_TRACK_WIDTH_M,
        )

        self.state_space_model = StateSpaceVehicleModel(
            mass_kg=DYNAMIC_BICYCLE_MASS_KG,
            iz_kg_m2=DYNAMIC_BICYCLE_IZ_KG_M2,
            lf_m=DYNAMIC_BICYCLE_LF_M,
            lr_m=DYNAMIC_BICYCLE_LR_M,
            rear_track_width_m=REAR_TRACK_WIDTH_M,
            c_alpha_f_n_per_rad=FRONT_CORNERING_STIFFNESS_N_PER_RAD,
            c_alpha_r_n_per_rad=REAR_CORNERING_STIFFNESS_N_PER_RAD,
            k_drag_coeff=STATE_SPACE_K_DRAG_COEFF,
            min_vx_ms=STATE_SPACE_MIN_VX_MS,
        )

        self.kinematic_state_est = None
        self.latest_kinematic_derivative = None

        self.dynamic_state_est = None
        self.latest_dynamic_derivative = None
        self.latest_dynamic_input = None
        self.latest_slip_angles = None
        self.latest_tire_forces = None

        self.latest_steering_actuator = None
        self.latest_propulsion_actuator = None
        self.latest_wheel_forces = None
        self.latest_state_space = None

        self.latest_debug_snapshot = None

        self.delta_ref_cmd_est = 0.0
        self.delta_f_cmd_est = 0.0
        self.a_long_cmd_est = 0.0

        self.last_speed_cmd = 0.0

    def rover_move_manual_mode(self, speed_cmd, turn_cmd):
        """Operates the rover in manual mode by publishing on /cerebri/in/joy."""
        msg = Joy()
        msg.buttons = [1, 0, 0, 0, 0, 0, 0, 1]
        msg.axes = [0.0, speed_cmd, 0.0, turn_cmd]
        self.publisher_joy.publish(msg)

    @staticmethod
    def clamp(value, min_value, max_value):
        return max(min_value, min(max_value, value))

    def estimate_battery_voltage(self):
        """
        Returns a simple battery-voltage estimate for the propulsion model.

        If fuel percentage exists:
            use it to interpolate between BATTERY_MIN_V and BATTERY_MAX_V.

        If not:
            use BATTERY_NOMINAL_V.
        """
        vehicle = self.estimator.vehicle

        if vehicle.battery_pct_meas is None:
            return BATTERY_NOMINAL_V

        pct = float(vehicle.battery_pct_meas)

        if pct > 1.0:
            pct = pct / 100.0

        pct = self.clamp(pct, 0.0, 1.0)

        return BATTERY_MIN_V + pct * (BATTERY_MAX_V - BATTERY_MIN_V)

    def update_actuator_models(self, speed_cmd: float, dt_s: float):
        """
        Task 2.5 / 2.6 actuator update.

        Steering:
            turn_cmd -> delta_ref -> first-order delta_f_est

        Propulsion:
            speed_cmd -> pwm_cmd -> first-order F_long_est
        """
        self.delta_ref_cmd_est = turn_cmd_to_delta_f_est(
            self.controller.last_turn_cmd
        )

        self.latest_steering_actuator = self.steering_actuator_model.step(
            delta_ref_rad=self.delta_ref_cmd_est,
            dt_s=dt_s,
        )

        self.delta_f_cmd_est = (
            self.latest_steering_actuator.delta_f_est_rad
        )

        battery_voltage_v = self.estimate_battery_voltage()

        self.latest_propulsion_actuator = self.propulsion_model.step(
            pwm_cmd=speed_cmd,
            battery_voltage_v=battery_voltage_v,
            dt_s=dt_s,
        )

    def update_debug_snapshot(self):
        self.latest_debug_snapshot = build_debug_snapshot(
            vehicle=self.estimator.vehicle,
            camera_measurement=self.camera_extractor.latest_camera_measurement,
            kinematic_state_est=self.kinematic_state_est,
            kinematic_derivative=self.latest_kinematic_derivative,
            dynamic_state_est=self.dynamic_state_est,
            dynamic_derivative=self.latest_dynamic_derivative,
            dynamic_input=self.latest_dynamic_input,
            slip_angles=self.latest_slip_angles,
            tire_forces=self.latest_tire_forces,
            steering_actuator=self.latest_steering_actuator,
            propulsion_actuator=self.latest_propulsion_actuator,
            wheel_forces=self.latest_wheel_forces,
            state_space=self.latest_state_space,
            speed_cmd=self.last_speed_cmd,
            turn_cmd=self.controller.last_turn_cmd,
            delta_f_cmd_est=self.delta_f_cmd_est,
        )

    def update_kinematic_bicycle_model(self, dt_s: float):
        """
        Runs the kinematic bicycle model in parallel.

        Now uses the lagged steering estimate delta_f_cmd_est instead of the
        instantaneous reference steering command.
        """
        vehicle = self.estimator.vehicle
        if not vehicle.odom_ready:
            return

        speed_for_model = vehicle.vx_recon

        if abs(speed_for_model) < KINEMATIC_V_DEADZONE_MS:
            speed_for_model = 0.0

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

    def update_dynamic_bicycle_model(self, dt_s: float):
        """
        Runs the dynamic bicycle model in parallel.

        Now uses:
        - lagged steering angle from Task 2.5
        - propulsion force estimate from Task 2.6
        - tire forces from Task 2.4
        """
        vehicle = self.estimator.vehicle
        if not vehicle.odom_ready:
            return

        measured_state = DynamicBicycleState(
            x=vehicle.x_meas,
            y=vehicle.y_meas,
            psi=vehicle.yaw_meas,
            v_x=vehicle.vx_recon,
            v_y=vehicle.vy_recon,
            r=vehicle.r_recon,
        )

        self.latest_slip_angles = self.dynamic_model.compute_slip_angles(
            state=measured_state,
            delta_f_rad=self.delta_f_cmd_est,
        )

        self.latest_tire_forces = self.dynamic_model.compute_linear_tire_forces(
            slip_angles=self.latest_slip_angles,
        )

        if self.latest_propulsion_actuator is None:
            propulsion_force_n = 0.0
        else:
            propulsion_force_n = (
                self.latest_propulsion_actuator.force_longitudinal_est_n
            )

        self.latest_wheel_forces = self.wheel_force_model.decompose(
            f_long_n=propulsion_force_n,
            dfx_n=TORQUE_VECTORING_DFX_CMD_N,
        )

        longitudinal_force_n = (
            self.latest_wheel_forces.reconstructed_f_long_n
        )

        torque_vectoring_yaw_moment_n_m = (
            DYNAMIC_TORQUE_VECTORING_YAW_MOMENT_N_M
            + self.latest_wheel_forces.torque_vectoring_yaw_moment_n_m
        )

        self.latest_state_space = self.state_space_model.build(
            vx_ms=vehicle.vx_recon,
            vy_ms=vehicle.vy_recon,
            r_rad_s=vehicle.r_recon,
            fx_left_n=self.latest_wheel_forces.fx_left_n,
            fx_right_n=self.latest_wheel_forces.fx_right_n,
            delta_f_rad=self.delta_f_cmd_est,
        )

        model_input = DynamicBicycleInput(
            delta_f_rad=self.delta_f_cmd_est,
            longitudinal_force_n=longitudinal_force_n,
            front_lateral_force_n=(
                self.latest_tire_forces.front_lateral_force_n
            ),
            rear_lateral_force_n=(
                self.latest_tire_forces.rear_lateral_force_n
            ),
            drag_force_n=DYNAMIC_DRAG_FORCE_N,
            ramp_force_n=DYNAMIC_RAMP_FORCE_N,
            torque_vectoring_yaw_moment_n_m=(
                torque_vectoring_yaw_moment_n_m
            ),
        )

        self.latest_dynamic_input = model_input

        self.latest_dynamic_derivative = self.dynamic_model.derivative(
            state=measured_state,
            model_input=model_input,
        )

        if self.dynamic_state_est is None:
            self.dynamic_state_est = measured_state
        else:
            self.dynamic_state_est, _ = self.dynamic_model.step_euler(
                state=self.dynamic_state_est,
                model_input=model_input,
                dt_s=dt_s,
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

        self.last_speed_cmd = speed_cmd

        self.update_actuator_models(speed_cmd, dt_s)
        self.update_kinematic_bicycle_model(dt_s)
        self.update_dynamic_bicycle_model(dt_s)

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
