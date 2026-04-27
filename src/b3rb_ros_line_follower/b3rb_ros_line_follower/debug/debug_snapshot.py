import math

from ..mrac_config import DEBUG_FIELDS, DYNAMIC_BICYCLE_MASS_KG


def _rad_to_deg_text(value_rad):
    if value_rad is None:
        return "None"
    return f"{math.degrees(value_rad):.2f}deg"


def build_debug_snapshot(
    vehicle,
    camera_measurement,
    kinematic_state_est,
    kinematic_derivative,
    dynamic_state_est,
    dynamic_derivative,
    dynamic_input,
    slip_angles,
    tire_forces,
    steering_actuator,
    propulsion_actuator,
    wheel_forces,
    state_space,
    inner_lateral_yaw,
    outer_longitudinal,
    outer_reference,
    inner_reference_command,
    inner_lateral_yaw_reference,
    speed_cmd,
    turn_cmd,
    delta_f_cmd_est,
):
    vx_recon_prev_value = getattr(vehicle, "vx_recon_prev", None)
    dt_odom_value = getattr(vehicle, "dt_odom", None)

    if vx_recon_prev_value is None:
        vx_recon_prev_text = "None"
    else:
        vx_recon_prev_text = f"{vx_recon_prev_value:.3f}"

    if dt_odom_value is None:
        dt_odom_text = "None"
    else:
        dt_odom_text = f"{dt_odom_value:.4f}"

    if camera_measurement.have_measurement:
        ye_text = f"{camera_measurement.ye_cam_filt:.3f}"
        psi_text = f"{camera_measurement.psi_rel_cam_filt:.3f}"
    else:
        ye_text = "None"
        psi_text = "None"

    if kinematic_derivative is not None:
        beta_text = f"{kinematic_derivative.beta:.3f}"
        xdot_text = f"{kinematic_derivative.x_dot:.3f}"
        ydot_text = f"{kinematic_derivative.y_dot:.3f}"
        psidot_text = f"{kinematic_derivative.psi_dot:.3f}"
        vdot_text = f"{kinematic_derivative.v_dot:.3f}"
    else:
        beta_text = "None"
        xdot_text = "None"
        ydot_text = "None"
        psidot_text = "None"
        vdot_text = "None"

    if kinematic_state_est is not None:
        kin_x_text = f"{kinematic_state_est.x:.3f}"
        kin_y_text = f"{kinematic_state_est.y:.3f}"
        kin_psi_text = f"{kinematic_state_est.psi:.3f}"
        kin_v_text = f"{kinematic_state_est.v:.3f}"
    else:
        kin_x_text = "None"
        kin_y_text = "None"
        kin_psi_text = "None"
        kin_v_text = "None"

    if dynamic_derivative is not None:
        dyn_ax_text = f"{dynamic_derivative.a_x_body:.3f}"
        dyn_vx_dot_text = f"{dynamic_derivative.v_x_dot:.3f}"
        dyn_vy_dot_text = f"{dynamic_derivative.v_y_dot:.3f}"
        dyn_r_dot_text = f"{dynamic_derivative.r_dot:.3f}"
        dyn_x_dot_text = f"{dynamic_derivative.x_dot:.3f}"
        dyn_y_dot_text = f"{dynamic_derivative.y_dot:.3f}"
        dyn_psi_dot_text = f"{dynamic_derivative.psi_dot:.3f}"
    else:
        dyn_ax_text = "None"
        dyn_vx_dot_text = "None"
        dyn_vy_dot_text = "None"
        dyn_r_dot_text = "None"
        dyn_x_dot_text = "None"
        dyn_y_dot_text = "None"
        dyn_psi_dot_text = "None"

    if dynamic_state_est is not None:
        dyn_x_text = f"{dynamic_state_est.x:.3f}"
        dyn_y_text = f"{dynamic_state_est.y:.3f}"
        dyn_psi_text = f"{dynamic_state_est.psi:.3f}"
        dyn_vx_text = f"{dynamic_state_est.v_x:.3f}"
        dyn_vy_text = f"{dynamic_state_est.v_y:.3f}"
        dyn_r_text = f"{dynamic_state_est.r:.3f}"
    else:
        dyn_x_text = "None"
        dyn_y_text = "None"
        dyn_psi_text = "None"
        dyn_vx_text = "None"
        dyn_vy_text = "None"
        dyn_r_text = "None"

    if dynamic_input is not None:
        dyn_delta_text = f"{dynamic_input.delta_f_rad:.3f}"
        dyn_fx_text = f"{dynamic_input.longitudinal_force_n:.3f}"
        dyn_fyf_text = f"{dynamic_input.front_lateral_force_n:.3f}"
        dyn_fyr_text = f"{dynamic_input.rear_lateral_force_n:.3f}"
        dyn_drag_text = f"{dynamic_input.drag_force_n:.3f}"
        dyn_ramp_text = f"{dynamic_input.ramp_force_n:.3f}"
        dyn_mz_text = f"{dynamic_input.torque_vectoring_yaw_moment_n_m:.3f}"
    else:
        dyn_delta_text = "None"
        dyn_fx_text = "None"
        dyn_fyf_text = "None"
        dyn_fyr_text = "None"
        dyn_drag_text = "None"
        dyn_ramp_text = "None"
        dyn_mz_text = "None"

    if slip_angles is not None:
        alpha_f_text = f"{slip_angles.alpha_f:.3f}"
        alpha_r_text = f"{slip_angles.alpha_r:.3f}"
        alpha_f_deg_text = _rad_to_deg_text(slip_angles.alpha_f)
        alpha_r_deg_text = _rad_to_deg_text(slip_angles.alpha_r)
        slip_valid_text = str(slip_angles.valid)
    else:
        alpha_f_text = "None"
        alpha_r_text = "None"
        alpha_f_deg_text = "None"
        alpha_r_deg_text = "None"
        slip_valid_text = "False"

    if tire_forces is not None:
        c_alpha_f_text = f"{tire_forces.front_cornering_stiffness_n_per_rad:.3f}"
        c_alpha_r_text = f"{tire_forces.rear_cornering_stiffness_n_per_rad:.3f}"
        tire_valid_text = str(tire_forces.valid)
    else:
        c_alpha_f_text = "None"
        c_alpha_r_text = "None"
        tire_valid_text = "False"

    if steering_actuator is not None:
        delta_ref_text = f"{steering_actuator.delta_ref_rad:.3f}"
        delta_f_est_text = f"{steering_actuator.delta_f_est_rad:.3f}"
        delta_error_text = f"{steering_actuator.delta_error_rad:.3f}"
        delta_ref_deg_text = _rad_to_deg_text(steering_actuator.delta_ref_rad)
        delta_f_est_deg_text = _rad_to_deg_text(steering_actuator.delta_f_est_rad)
        delta_error_deg_text = _rad_to_deg_text(steering_actuator.delta_error_rad)
        steer_tau_text = f"{steering_actuator.tau_s:.3f}"
    else:
        delta_ref_text = "None"
        delta_f_est_text = "None"
        delta_error_text = "None"
        delta_ref_deg_text = "None"
        delta_f_est_deg_text = "None"
        delta_error_deg_text = "None"
        steer_tau_text = "None"

    if propulsion_actuator is not None:
        pwm_text = f"{propulsion_actuator.pwm_cmd:.3f}"
        battery_voltage_text = f"{propulsion_actuator.battery_voltage_v:.3f}"
        v_sag_text = f"{propulsion_actuator.voltage_sag_v:.3f}"
        f_long_target_text = f"{propulsion_actuator.target_force_n:.3f}"
        f_long_est_text = f"{propulsion_actuator.force_longitudinal_est_n:.3f}"
        motor_tau_text = f"{propulsion_actuator.tau_s:.3f}"
    else:
        pwm_text = "None"
        battery_voltage_text = "None"
        v_sag_text = "None"
        f_long_target_text = "None"
        f_long_est_text = "None"
        motor_tau_text = "None"


    if propulsion_actuator is not None:
        outer_vx_dot_target_text = (
            f"{propulsion_actuator.target_force_n / DYNAMIC_BICYCLE_MASS_KG:.3f}"
        )
        outer_vx_dot_est_text = (
            f"{propulsion_actuator.force_longitudinal_est_n / DYNAMIC_BICYCLE_MASS_KG:.3f}"
        )
    else:
        outer_vx_dot_target_text = "None"
        outer_vx_dot_est_text = "None"


    if wheel_forces is not None:
        dfx_text = f"{wheel_forces.dfx_n:.3f}"
        fx_left_text = f"{wheel_forces.fx_left_n:.3f}"
        fx_right_text = f"{wheel_forces.fx_right_n:.3f}"
        fx_sum_recon_text = f"{wheel_forces.reconstructed_f_long_n:.3f}"
        dfx_recon_text = f"{wheel_forces.reconstructed_dfx_n:.3f}"
        mz_tv_text = f"{wheel_forces.torque_vectoring_yaw_moment_n_m:.3f}"
        rear_track_width_text = f"{wheel_forces.rear_track_width_m:.3f}"
        wheel_force_valid_text = str(wheel_forces.valid)
    else:
        dfx_text = "None"
        fx_left_text = "None"
        fx_right_text = "None"
        fx_sum_recon_text = "None"
        dfx_recon_text = "None"
        mz_tv_text = "None"
        rear_track_width_text = "None"
        wheel_force_valid_text = "False"


    if state_space is not None and state_space.valid:
        ss_x_dim_text = str(len(state_space.x_state))
        ss_u_dim_text = str(len(state_space.u_input))
        ss_A_dim_text = f"{len(state_space.A_x)}x{len(state_space.A_x[0])}"
        ss_B_dim_text = f"{len(state_space.B_x)}x{len(state_space.B_x[0])}"

        ss_vx_safe_text = f"{state_space.vx_safe:.3f}"

        ss_xdot_vx_text = f"{state_space.x_dot_pred[0]:.3f}"
        ss_xdot_vy_text = f"{state_space.x_dot_pred[1]:.3f}"
        ss_xdot_r_text = f"{state_space.x_dot_pred[2]:.3f}"

        ss_A_11_text = f"{state_space.A_x[1][1]:.3f}"
        ss_A_12_text = f"{state_space.A_x[1][2]:.3f}"
        ss_A_21_text = f"{state_space.A_x[2][1]:.3f}"
        ss_A_22_text = f"{state_space.A_x[2][2]:.3f}"

        ss_B_00_text = f"{state_space.B_x[0][0]:.3f}"
        ss_B_01_text = f"{state_space.B_x[0][1]:.3f}"
        ss_B_20_text = f"{state_space.B_x[2][0]:.3f}"
        ss_B_21_text = f"{state_space.B_x[2][1]:.3f}"
        ss_B_22_text = f"{state_space.B_x[2][2]:.3f}"

        ss_valid_text = "True"
    else:
        ss_x_dim_text = "None"
        ss_u_dim_text = "None"
        ss_A_dim_text = "None"
        ss_B_dim_text = "None"

        ss_vx_safe_text = "None"

        ss_xdot_vx_text = "None"
        ss_xdot_vy_text = "None"
        ss_xdot_r_text = "None"

        ss_A_11_text = "None"
        ss_A_12_text = "None"
        ss_A_21_text = "None"
        ss_A_22_text = "None"

        ss_B_00_text = "None"
        ss_B_01_text = "None"
        ss_B_20_text = "None"
        ss_B_21_text = "None"
        ss_B_22_text = "None"

        ss_valid_text = "False"


    if inner_lateral_yaw is not None:
        inner_valid_text = str(inner_lateral_yaw.valid)
        inner_vx0_text = f"{inner_lateral_yaw.vx0_ms:.3f}"
        inner_vx0_safe_text = f"{inner_lateral_yaw.vx0_safe_ms:.3f}"

        inner_x_lat_vy_text = f"{inner_lateral_yaw.x_lat[0]:.3f}"
        inner_x_lat_r_text = f"{inner_lateral_yaw.x_lat[1]:.3f}"

        inner_A_00_text = f"{inner_lateral_yaw.A_theta[0][0]:.3f}"
        inner_A_01_text = f"{inner_lateral_yaw.A_theta[0][1]:.3f}"
        inner_A_10_text = f"{inner_lateral_yaw.A_theta[1][0]:.3f}"
        inner_A_11_text = f"{inner_lateral_yaw.A_theta[1][1]:.3f}"

        inner_B_delta_0_text = f"{inner_lateral_yaw.B_delta_theta[0]:.3f}"
        inner_B_delta_1_text = f"{inner_lateral_yaw.B_delta_theta[1]:.3f}"

        inner_B_TV_0_text = f"{inner_lateral_yaw.B_TV[0]:.3f}"
        inner_B_TV_1_text = f"{inner_lateral_yaw.B_TV[1]:.3f}"

        inner_xdot_vy_text = f"{inner_lateral_yaw.x_dot_pred[0]:.3f}"
        inner_xdot_r_text = f"{inner_lateral_yaw.x_dot_pred[1]:.3f}"
    else:
        inner_valid_text = "False"
        inner_vx0_text = "None"
        inner_vx0_safe_text = "None"

        inner_x_lat_vy_text = "None"
        inner_x_lat_r_text = "None"

        inner_A_00_text = "None"
        inner_A_01_text = "None"
        inner_A_10_text = "None"
        inner_A_11_text = "None"

        inner_B_delta_0_text = "None"
        inner_B_delta_1_text = "None"

        inner_B_TV_0_text = "None"
        inner_B_TV_1_text = "None"

        inner_xdot_vy_text = "None"
        inner_xdot_r_text = "None"


    if outer_longitudinal is not None and outer_longitudinal.valid:
        outer_valid_text = "True"
        outer_vx_text = f"{outer_longitudinal.vx_ms:.3f}"
        outer_pwm_text = f"{outer_longitudinal.pwm_cmd:.3f}"
        outer_battery_voltage_text = f"{outer_longitudinal.battery_voltage_v:.3f}"
        outer_v_sag_text = f"{outer_longitudinal.voltage_sag_v:.3f}"
        outer_effective_voltage_text = f"{outer_longitudinal.effective_voltage_v:.3f}"
        outer_b_batt_est_text = f"{outer_longitudinal.b_batt_est:.3f}"
        outer_vx_dot_propulsion_text = (
            f"{outer_longitudinal.vx_dot_propulsion_ms2:.3f}"
        )
        outer_vx_loss_text = f"{outer_longitudinal.vx_loss_ms2:.3f}"
        outer_vx_dot_pred_text = f"{outer_longitudinal.vx_dot_pred_ms2:.3f}"
        outer_d_x_est_text = f"{outer_longitudinal.d_x_est_ms2:.3f}"
    else:
        outer_valid_text = "False"
        outer_vx_text = "None"
        outer_pwm_text = "None"
        outer_battery_voltage_text = "None"
        outer_v_sag_text = "None"
        outer_effective_voltage_text = "None"
        outer_b_batt_est_text = "None"
        outer_vx_dot_propulsion_text = "None"
        outer_vx_loss_text = "None"
        outer_vx_dot_pred_text = "None"
        outer_d_x_est_text = "None"

    if inner_reference_command is not None:
        inner_ref_valid_text = str(inner_reference_command.valid)
        inner_kappa_ref_text = (
            f"{inner_reference_command.kappa_ref_1pm:.3f}"
        )
        inner_r_ref_text = (
            f"{inner_reference_command.r_ref_rad_s:.3f}"
        )
        inner_uc_text = (
            f"{inner_reference_command.uc_rad_s:.3f}"
        )
    else:
        inner_ref_valid_text = "False"
        inner_kappa_ref_text = "None"
        inner_r_ref_text = "None"
        inner_uc_text = "None"


    if outer_reference is not None and outer_reference.valid:
        outer_ref_valid_text = "True"
        outer_vx_ref_text = f"{outer_reference.vx_ref_ms:.3f}"
        outer_vx_m_text = f"{outer_reference.vx_m_ms:.3f}"
        outer_vx_m_dot_text = f"{outer_reference.vx_m_dot_ms2:.3f}"
        outer_ref_error_text = f"{outer_reference.tracking_error_ms:.3f}"
        outer_ref_a_m_text = f"{outer_reference.a_m_s_inv:.3f}"
        outer_ref_dt_text = f"{outer_reference.dt_s:.3f}"
    else:
        outer_ref_valid_text = "False"
        outer_vx_ref_text = "None"
        outer_vx_m_text = "None"
        outer_vx_m_dot_text = "None"
        outer_ref_error_text = "None"
        outer_ref_a_m_text = "None"
        outer_ref_dt_text = "None"


    if inner_lateral_yaw_reference is not None:
        inner_model_valid_text = str(inner_lateral_yaw_reference.valid)
        inner_xm_vy_text = f"{inner_lateral_yaw_reference.vy_m_ms:.3f}"
        inner_xm_r_text = f"{inner_lateral_yaw_reference.r_m_rad_s:.3f}"
        inner_xm_dot_vy_text = f"{inner_lateral_yaw_reference.vy_m_dot_ms2:.3f}"
        inner_xm_dot_r_text = f"{inner_lateral_yaw_reference.r_m_dot_rad_s2:.3f}"

        inner_Am_00_text = f"{inner_lateral_yaw_reference.A_m[0][0]:.3f}"
        inner_Am_01_text = f"{inner_lateral_yaw_reference.A_m[0][1]:.3f}"
        inner_Am_10_text = f"{inner_lateral_yaw_reference.A_m[1][0]:.3f}"
        inner_Am_11_text = f"{inner_lateral_yaw_reference.A_m[1][1]:.3f}"

        inner_Bm_0_text = f"{inner_lateral_yaw_reference.B_m[0][0]:.3f}"
        inner_Bm_1_text = f"{inner_lateral_yaw_reference.B_m[1][0]:.3f}"
    else:
        inner_model_valid_text = "False"
        inner_xm_vy_text = "None"
        inner_xm_r_text = "None"
        inner_xm_dot_vy_text = "None"
        inner_xm_dot_r_text = "None"

        inner_Am_00_text = "None"
        inner_Am_01_text = "None"
        inner_Am_10_text = "None"
        inner_Am_11_text = "None"

        inner_Bm_0_text = "None"
        inner_Bm_1_text = "None"

    debug_map = {
        "turn_cmd": f"turn_cmd={turn_cmd:.3f}",
        "speed_cmd": f"speed_cmd={speed_cmd:.3f}",

        "x_meas": f"x_meas={vehicle.x_meas:.3f}",
        "y_meas": f"y_meas={vehicle.y_meas:.3f}",
        "yaw_meas": f"yaw_meas={vehicle.yaw_meas:.3f}",
        "vx_recon": f"vx_recon={vehicle.vx_recon:.3f}",
        "vx_recon_prev": f"vx_recon_prev={vx_recon_prev_text}",
        "dt_odom": f"dt_odom={dt_odom_text}",
        "vy_recon": f"vy_recon={vehicle.vy_recon:.3f}",
        "r_recon": f"r_recon={vehicle.r_recon:.3f}",
        "a_long_recon": f"a_long_recon={vehicle.a_long_recon:.3f}",
        "a_long_filt": f"a_long_filt={vehicle.a_long_filt:.3f}",

        "ye_cam_filt": f"ye_cam_filt={ye_text}",
        "psi_rel_cam_filt": f"psi_rel_cam_filt={psi_text}",

        "delta_f_cmd_est": f"delta_f_cmd_est={delta_f_cmd_est:.3f}",
        "delta_ref": f"delta_ref={delta_ref_text}",
        "delta_f_est": f"delta_f_est={delta_f_est_text}",
        "delta_error": f"delta_error={delta_error_text}",
        "delta_ref_deg": f"delta_ref_deg={delta_ref_deg_text}",
        "delta_f_est_deg": f"delta_f_est_deg={delta_f_est_deg_text}",
        "delta_error_deg": f"delta_error_deg={delta_error_deg_text}",
        "steer_tau_s": f"steer_tau_s={steer_tau_text}",

        "pwm_cmd": f"pwm_cmd={pwm_text}",
        "battery_voltage": f"battery_voltage={battery_voltage_text}",
        "v_sag": f"v_sag={v_sag_text}",
        "f_long_target": f"f_long_target={f_long_target_text}",
        "f_long_est": f"f_long_est={f_long_est_text}",
        "outer_vx_dot_target": f"outer_vx_dot_target={outer_vx_dot_target_text}",
        "outer_vx_dot_est": f"outer_vx_dot_est={outer_vx_dot_est_text}",
        "motor_tau_s": f"motor_tau_s={motor_tau_text}",

        "dfx_cmd": f"dfx_cmd={dfx_text}",
        "fx_left": f"fx_left={fx_left_text}",
        "fx_right": f"fx_right={fx_right_text}",
        "fx_sum_recon": f"fx_sum_recon={fx_sum_recon_text}",
        "dfx_recon": f"dfx_recon={dfx_recon_text}",
        "mz_tv": f"mz_tv={mz_tv_text}",
        "rear_track_width": f"rear_track_width={rear_track_width_text}",
        "wheel_force_valid": f"wheel_force_valid={wheel_force_valid_text}",

        "ss_valid": f"ss_valid={ss_valid_text}",
        "ss_x_dim": f"ss_x_dim={ss_x_dim_text}",
        "ss_u_dim": f"ss_u_dim={ss_u_dim_text}",
        "ss_A_dim": f"ss_A_dim={ss_A_dim_text}",
        "ss_B_dim": f"ss_B_dim={ss_B_dim_text}",

        "ss_vx_safe": f"ss_vx_safe={ss_vx_safe_text}",
        "ss_xdot_vx": f"ss_xdot_vx={ss_xdot_vx_text}",
        "ss_xdot_vy": f"ss_xdot_vy={ss_xdot_vy_text}",
        "ss_xdot_r": f"ss_xdot_r={ss_xdot_r_text}",

        "ss_A_11": f"ss_A_11={ss_A_11_text}",
        "ss_A_12": f"ss_A_12={ss_A_12_text}",
        "ss_A_21": f"ss_A_21={ss_A_21_text}",
        "ss_A_22": f"ss_A_22={ss_A_22_text}",

        "ss_B_00": f"ss_B_00={ss_B_00_text}",
        "ss_B_01": f"ss_B_01={ss_B_01_text}",
        "ss_B_20": f"ss_B_20={ss_B_20_text}",
        "ss_B_21": f"ss_B_21={ss_B_21_text}",
        "ss_B_22": f"ss_B_22={ss_B_22_text}",

        "inner_valid": f"inner_valid={inner_valid_text}",
        "inner_vx0": f"inner_vx0={inner_vx0_text}",
        "inner_vx0_safe": f"inner_vx0_safe={inner_vx0_safe_text}",
        "inner_x_lat_vy": f"inner_x_lat_vy={inner_x_lat_vy_text}",
        "inner_x_lat_r": f"inner_x_lat_r={inner_x_lat_r_text}",
        "inner_A_00": f"inner_A_00={inner_A_00_text}",
        "inner_A_01": f"inner_A_01={inner_A_01_text}",
        "inner_A_10": f"inner_A_10={inner_A_10_text}",
        "inner_A_11": f"inner_A_11={inner_A_11_text}",
        "inner_B_delta_0": f"inner_B_delta_0={inner_B_delta_0_text}",
        "inner_B_delta_1": f"inner_B_delta_1={inner_B_delta_1_text}",
        "inner_B_TV_0": f"inner_B_TV_0={inner_B_TV_0_text}",
        "inner_B_TV_1": f"inner_B_TV_1={inner_B_TV_1_text}",
        "inner_xdot_vy": f"inner_xdot_vy={inner_xdot_vy_text}",
        "inner_xdot_r": f"inner_xdot_r={inner_xdot_r_text}",

        "inner_ref_valid": f"inner_ref_valid={inner_ref_valid_text}",
        "inner_kappa_ref": f"inner_kappa_ref={inner_kappa_ref_text}",
        "inner_r_ref": f"inner_r_ref={inner_r_ref_text}",
        "inner_uc": f"inner_uc={inner_uc_text}",

        "outer_valid": f"outer_valid={outer_valid_text}",
        "outer_vx": f"outer_vx={outer_vx_text}",
        "outer_pwm": f"outer_pwm={outer_pwm_text}",
        "outer_battery_voltage": f"outer_battery_voltage={outer_battery_voltage_text}",
        "outer_v_sag": f"outer_v_sag={outer_v_sag_text}",
        "outer_effective_voltage": f"outer_effective_voltage={outer_effective_voltage_text}",
        "outer_b_batt_est": f"outer_b_batt_est={outer_b_batt_est_text}",
        "outer_vx_dot_propulsion": (
            f"outer_vx_dot_propulsion={outer_vx_dot_propulsion_text}"
        ),
        "outer_vx_loss": f"outer_vx_loss={outer_vx_loss_text}",
        "outer_vx_dot_pred": f"outer_vx_dot_pred={outer_vx_dot_pred_text}",
        "outer_d_x_est": f"outer_d_x_est={outer_d_x_est_text}",

        "outer_ref_valid": f"outer_ref_valid={outer_ref_valid_text}",
        "outer_vx_ref": f"outer_vx_ref={outer_vx_ref_text}",
        "outer_vx_m": f"outer_vx_m={outer_vx_m_text}",
        "outer_vx_m_dot": f"outer_vx_m_dot={outer_vx_m_dot_text}",
        "outer_ref_error": f"outer_ref_error={outer_ref_error_text}",
        "outer_ref_a_m": f"outer_ref_a_m={outer_ref_a_m_text}",
        "outer_ref_dt": f"outer_ref_dt={outer_ref_dt_text}",

        "inner_model_valid": f"inner_model_valid={inner_model_valid_text}",
        "inner_xm_vy": f"inner_xm_vy={inner_xm_vy_text}",
        "inner_xm_r": f"inner_xm_r={inner_xm_r_text}",
        "inner_xm_dot_vy": f"inner_xm_dot_vy={inner_xm_dot_vy_text}",
        "inner_xm_dot_r": f"inner_xm_dot_r={inner_xm_dot_r_text}",

        "inner_Am_00": f"inner_Am_00={inner_Am_00_text}",
        "inner_Am_01": f"inner_Am_01={inner_Am_01_text}",
        "inner_Am_10": f"inner_Am_10={inner_Am_10_text}",
        "inner_Am_11": f"inner_Am_11={inner_Am_11_text}",
        "inner_Bm_0": f"inner_Bm_0={inner_Bm_0_text}",
        "inner_Bm_1": f"inner_Bm_1={inner_Bm_1_text}",

        "beta_kin": f"beta_kin={beta_text}",
        "x_dot_kin": f"x_dot_kin={xdot_text}",
        "y_dot_kin": f"y_dot_kin={ydot_text}",
        "psi_dot_kin": f"psi_dot_kin={psidot_text}",
        "v_dot_kin": f"v_dot_kin={vdot_text}",
        "kin_x_est": f"kin_x_est={kin_x_text}",
        "kin_y_est": f"kin_y_est={kin_y_text}",
        "kin_psi_est": f"kin_psi_est={kin_psi_text}",
        "kin_v_est": f"kin_v_est={kin_v_text}",

        "alpha_f": f"alpha_f={alpha_f_text}",
        "alpha_r": f"alpha_r={alpha_r_text}",
        "alpha_f_deg": f"alpha_f_deg={alpha_f_deg_text}",
        "alpha_r_deg": f"alpha_r_deg={alpha_r_deg_text}",
        "slip_valid": f"slip_valid={slip_valid_text}",

        "c_alpha_f": f"c_alpha_f={c_alpha_f_text}",
        "c_alpha_r": f"c_alpha_r={c_alpha_r_text}",
        "tire_valid": f"tire_valid={tire_valid_text}",

        "dyn_delta_f": f"dyn_delta_f={dyn_delta_text}",
        "dyn_f_x": f"dyn_f_x={dyn_fx_text}",
        "dyn_f_y_front": f"dyn_f_y_front={dyn_fyf_text}",
        "dyn_f_y_rear": f"dyn_f_y_rear={dyn_fyr_text}",
        "dyn_drag": f"dyn_drag={dyn_drag_text}",
        "dyn_ramp": f"dyn_ramp={dyn_ramp_text}",
        "dyn_m_z_tv": f"dyn_m_z_tv={dyn_mz_text}",

        "dyn_a_x": f"dyn_a_x={dyn_ax_text}",
        "dyn_v_x_dot": f"dyn_v_x_dot={dyn_vx_dot_text}",
        "dyn_v_y_dot": f"dyn_v_y_dot={dyn_vy_dot_text}",
        "dyn_r_dot": f"dyn_r_dot={dyn_r_dot_text}",
        "dyn_x_dot": f"dyn_x_dot={dyn_x_dot_text}",
        "dyn_y_dot": f"dyn_y_dot={dyn_y_dot_text}",
        "dyn_psi_dot": f"dyn_psi_dot={dyn_psi_dot_text}",

        "dyn_x_est": f"dyn_x_est={dyn_x_text}",
        "dyn_y_est": f"dyn_y_est={dyn_y_text}",
        "dyn_psi_est": f"dyn_psi_est={dyn_psi_text}",
        "dyn_v_x_est": f"dyn_v_x_est={dyn_vx_text}",
        "dyn_v_y_est": f"dyn_v_y_est={dyn_vy_text}",
        "dyn_r_est": f"dyn_r_est={dyn_r_text}",

        "battery_pct_meas": f"battery_pct_meas={vehicle.battery_pct_meas}",
        "battery_power_meas": f"battery_power_meas={vehicle.battery_power_meas}",
    }

    snapshot = []
    for field_name in DEBUG_FIELDS:
        if field_name in debug_map:
            snapshot.append(debug_map[field_name])

    return snapshot
