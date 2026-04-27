from dataclasses import dataclass
from typing import Optional


@dataclass
class VehicleSignals:
    x_meas: float = 0.0
    y_meas: float = 0.0
    yaw_meas: float = 0.0

    vx_recon: float = 0.0
    vy_recon: float = 0.0
    r_recon: float = 0.0
    a_long_recon: float = 0.0
    a_long_filt: float = 0.0

    battery_pct_meas: Optional[float] = None
    battery_power_meas: Optional[float] = None

    odom_ready: bool = False
    status_ready: bool = False


@dataclass
class CameraMeasurement:
    have_measurement: bool = False
    use_integral: bool = False
    vector_count: int = 0

    ye_cam_filt: float = 0.0
    psi_rel_cam_filt: float = 0.0
    edge_balance_filt: float = 0.0

    center_far_filt: float = 0.0
    center_near_filt: float = 0.0
    lane_width_px: float = 0.0


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


@dataclass
class DynamicBicycleState:
    x: float = 0.0
    y: float = 0.0
    psi: float = 0.0

    v_x: float = 0.0
    v_y: float = 0.0
    r: float = 0.0


@dataclass
class DynamicBicycleInput:
    delta_f_rad: float = 0.0

    longitudinal_force_n: float = 0.0
    front_lateral_force_n: float = 0.0
    rear_lateral_force_n: float = 0.0

    drag_force_n: float = 0.0
    ramp_force_n: float = 0.0
    torque_vectoring_yaw_moment_n_m: float = 0.0


@dataclass
class DynamicBicycleDerivative:
    a_x_body: float = 0.0

    v_x_dot: float = 0.0
    v_y_dot: float = 0.0
    r_dot: float = 0.0

    x_dot: float = 0.0
    y_dot: float = 0.0
    psi_dot: float = 0.0


@dataclass
class SlipAngles:
    alpha_f: float = 0.0
    alpha_r: float = 0.0
    valid: bool = False


@dataclass
class LinearTireForces:
    front_lateral_force_n: float = 0.0
    rear_lateral_force_n: float = 0.0

    front_cornering_stiffness_n_per_rad: float = 0.0
    rear_cornering_stiffness_n_per_rad: float = 0.0

    valid: bool = False


@dataclass
class SteeringActuatorState:
    delta_ref_rad: float = 0.0
    delta_f_est_rad: float = 0.0
    delta_error_rad: float = 0.0
    tau_s: float = 0.0


@dataclass
class PropulsionActuatorState:
    pwm_cmd: float = 0.0
    battery_voltage_v: float = 0.0
    voltage_sag_v: float = 0.0

    target_force_n: float = 0.0
    force_longitudinal_est_n: float = 0.0

    tau_s: float = 0.0
    force_gain_n_per_v: float = 0.0

@dataclass
class WheelForceDecomposition:
    """
    Left/right rear wheel force decomposition.

    f_long_n:
        common-mode longitudinal force

    dfx_n:
        differential longitudinal force:
        dFx = F_x,R - F_x,L

    fx_left_n, fx_right_n:
        reconstructed left and right rear longitudinal forces

    torque_vectoring_yaw_moment_n_m:
        M_z_TV = (track_width / 2) * dFx
    """
    f_long_n: float = 0.0
    dfx_n: float = 0.0

    fx_left_n: float = 0.0
    fx_right_n: float = 0.0

    reconstructed_f_long_n: float = 0.0
    reconstructed_dfx_n: float = 0.0

    rear_track_width_m: float = 0.0
    torque_vectoring_yaw_moment_n_m: float = 0.0

    valid: bool = False

@dataclass
class StateSpacePlant:
    """
    Unified control-oriented state-space representation.

    x_state:
        [v_x, v_y, r]

    u_input:
        [F_x,L, F_x,R, delta_f]

    A_x, B_x:
        scheduled matrices from the report

    d_vector:
        optional disturbance / unmodeled dynamics vector

    x_dot_pred:
        A(x)x + B(x)u + d
    """
    x_state: list = None
    u_input: list = None

    A_x: list = None
    B_x: list = None
    d_vector: list = None
    x_dot_pred: list = None

    vx_safe: float = 0.0
    valid: bool = False

@dataclass
class OuterLongitudinalReducedOutput:
    """
    Outer-loop reduced longitudinal plant.

    Report form:

        v_x_dot = b_batt * PWM + d_x

    vx_ms:
        reconstructed/measured longitudinal speed

    pwm_cmd:
        normalized propulsion command

    battery_voltage_v:
        estimated battery voltage

    voltage_sag_v:
        estimated voltage sag under load

    effective_voltage_v:
        battery_voltage_v - voltage_sag_v

    b_batt_est:
        estimated uncertain battery/propulsion gain

    vx_dot_pred_ms2:
        predicted longitudinal acceleration from reduced model

    d_x_est_ms2:
        residual disturbance estimate:
        measured_vx_dot - vx_dot_pred
    """
    vx_ms: float = 0.0
    pwm_cmd: float = 0.0

    battery_voltage_v: float = 0.0
    voltage_sag_v: float = 0.0
    effective_voltage_v: float = 0.0

    b_batt_est: float = 0.0
    vx_dot_pred_ms2: float = 0.0
    d_x_est_ms2: float = 0.0

    measured_vx_dot_ms2: float = 0.0

    valid: bool = False

