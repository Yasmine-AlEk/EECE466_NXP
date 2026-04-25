
from ..mrac_config import DEBUG_FIELDS


def build_debug_snapshot(
    vehicle,
    camera_measurement,
    kinematic_state_est,
    kinematic_derivative,
    turn_cmd,
    delta_f_cmd_est,
):
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

    debug_map = {
        "turn_cmd": f"turn_cmd={turn_cmd:.3f}",
        "x_meas": f"x_meas={vehicle.x_meas:.3f}",
        "y_meas": f"y_meas={vehicle.y_meas:.3f}",
        "yaw_meas": f"yaw_meas={vehicle.yaw_meas:.3f}",
        "vx_recon": f"vx_recon={vehicle.vx_recon:.3f}",
        "vy_recon": f"vy_recon={vehicle.vy_recon:.3f}",
        "r_recon": f"r_recon={vehicle.r_recon:.3f}",
        "a_long_recon": f"a_long_recon={vehicle.a_long_recon:.3f}",
        "a_long_filt": f"a_long_filt={vehicle.a_long_filt:.3f}",
        "ye_cam_filt": f"ye_cam_filt={ye_text}",
        "psi_rel_cam_filt": f"psi_rel_cam_filt={psi_text}",
        "delta_f_cmd_est": f"delta_f_cmd_est={delta_f_cmd_est:.3f}",
        "beta_kin": f"beta_kin={beta_text}",
        "x_dot_kin": f"x_dot_kin={xdot_text}",
        "y_dot_kin": f"y_dot_kin={ydot_text}",
        "psi_dot_kin": f"psi_dot_kin={psidot_text}",
        "v_dot_kin": f"v_dot_kin={vdot_text}",
        "kin_x_est": f"kin_x_est={kin_x_text}",
        "kin_y_est": f"kin_y_est={kin_y_text}",
        "kin_psi_est": f"kin_psi_est={kin_psi_text}",
        "kin_v_est": f"kin_v_est={kin_v_text}",
        "battery_pct_meas": f"battery_pct_meas={vehicle.battery_pct_meas}",
        "battery_power_meas": f"battery_power_meas={vehicle.battery_power_meas}",
    }

    snapshot = []
    for field_name in DEBUG_FIELDS:
        if field_name in debug_map:
            snapshot.append(debug_map[field_name])

    return snapshot
