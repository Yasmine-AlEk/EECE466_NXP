import math

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
SPEED_TIGHT_TURN = 0.30

# continuous speed reduction in turns
CURVE_SPEED_GAIN = 0.65

# lane estimation when only one edge is visible
DEFAULT_SINGLE_EDGE_LANE_HALF_WIDTH_RATIO = 0.42

# preview rows in BEV / vector image
LOOKAHEAD_FAR_Y_RATIO = 0.52
LOOKAHEAD_NEAR_Y_RATIO = 0.82

# PID gains
KP_CENTER = 0.95
KI_CENTER = 0.02
KD_CENTER = 0.12

# preview / heading term
KP_HEADING = 0.45

# edge-repulsion / lane-balance term
KP_EDGE_BALANCE = 0.35

# command smoothing
TURN_SMOOTHING_OLD = 0.70
TURN_SMOOTHING_NEW = 0.30
SPEED_SMOOTHING_OLD = 0.60
SPEED_SMOOTHING_NEW = 0.40

# anti-windup
INTEGRAL_LIMIT = 0.80

# measurement filtering
MAX_CENTER_JUMP_RATIO = 0.22
CENTER_FILTER_ALPHA_TWO_EDGES = 0.40
CENTER_FILTER_ALPHA_ONE_EDGE = 0.25

# light filtering for reconstructed longitudinal acceleration
A_LONG_FILTER_ALPHA = 0.20

# ============================================================
# KINEMATIC BICYCLE SETTINGS
# ============================================================
KINEMATIC_WHEELBASE_M = 0.168
KINEMATIC_LF_M = 0.5 * KINEMATIC_WHEELBASE_M
KINEMATIC_LR_M = 0.5 * KINEMATIC_WHEELBASE_M
KINEMATIC_MAX_STEER_RAD = math.radians(30.0)
KINEMATIC_V_DEADZONE_MS = 0.05

# ============================================================
# DYNAMIC BICYCLE SETTINGS
# ============================================================
DYNAMIC_BICYCLE_WHEELBASE_M = KINEMATIC_WHEELBASE_M
DYNAMIC_BICYCLE_LF_M = 0.5 * DYNAMIC_BICYCLE_WHEELBASE_M
DYNAMIC_BICYCLE_LR_M = 0.5 * DYNAMIC_BICYCLE_WHEELBASE_M

DYNAMIC_BICYCLE_MASS_KG = 3.0
DYNAMIC_BICYCLE_IZ_KG_M2 = 0.05

DYNAMIC_TIRE_FORCE_PAIR_COUNT = 2.0
DYNAMIC_MAX_STEER_RAD = KINEMATIC_MAX_STEER_RAD
DYNAMIC_V_DEADZONE_MS = KINEMATIC_V_DEADZONE_MS
DYNAMIC_MODEL_MAX_DT_S = 0.10

DYNAMIC_DRAG_FORCE_N = 0.0
DYNAMIC_RAMP_FORCE_N = 0.0
DYNAMIC_TORQUE_VECTORING_YAW_MOMENT_N_M = 0.0

# ============================================================
# SLIP-ANGLE SETTINGS
# ============================================================
SLIP_ANGLE_MIN_VX_MS = 0.10
SLIP_ANGLE_MAX_ABS_RAD = math.radians(60.0)

# ============================================================
# LINEAR TIRE-FORCE SETTINGS
# ============================================================
FRONT_CORNERING_STIFFNESS_N_PER_RAD = 8.0
REAR_CORNERING_STIFFNESS_N_PER_RAD = 8.0
TIRE_FORCE_MAX_ABS_N = 5.0

# ============================================================
# STEERING ACTUATOR SETTINGS
# ------------------------------------------------------------
# Task 2.5:
# first-order steering servo lag:
#
#   delta_f_dot = (1 / tau_s) * (delta_ref - delta_f)
#
# Larger tau_s = slower steering response.
# ============================================================
STEERING_SERVO_TAU_S = 0.18
STEERING_SERVO_MAX_STEER_RAD = KINEMATIC_MAX_STEER_RAD

# ============================================================
# PROPULSION / MOTOR FORCE SETTINGS
# ------------------------------------------------------------
# Task 2.6:
# simple first-order longitudinal force model:
#
#   F_long_dot = (1 / tau_e) * (F_target - F_long)
#
#   F_target = gain * eta * (V_batt - V_sag) * pwm_cmd
#
# This is a simplified ESC/BLDC scaffold.
# ============================================================
PROPULSION_MOTOR_TAU_S = 0.25
PROPULSION_EFFICIENCY = 1.0
PROPULSION_FORCE_GAIN_N_PER_V = 0.25
PROPULSION_VOLTAGE_SAG_AT_FULL_PWM = 0.80
PROPULSION_FORCE_MAX_ABS_N = 5.0

BATTERY_NOMINAL_V = 13.8
BATTERY_MIN_V = 10.5
BATTERY_MAX_V = 13.8


# ============================================================
# WHEEL-FORCE DECOMPOSITION SETTINGS
# ------------------------------------------------------------
# Task 2.7:
#
#   F_long = F_x,L + F_x,R
#   dFx    = F_x,R - F_x,L
#
#   F_x,L = 0.5 * (F_long - dFx)
#   F_x,R = 0.5 * (F_long + dFx)
#
# The torque-vectoring yaw moment is:
#
#   M_z_TV = (rear_track_width / 2) * dFx
#
# dFx is kept zero for now so behavior remains identical.
# ============================================================
REAR_TRACK_WIDTH_M = 0.13
TORQUE_VECTORING_DFX_CMD_N = 0.0


# ============================================================
# UNIFIED STATE-SPACE SETTINGS
# ------------------------------------------------------------
# Task 2.8:
#
#   x = [v_x, v_y, r]^T
#   u = [F_x,L, F_x,R, delta_f]^T
#
#   x_dot = A(x)x + B(x)u + d
#
# The min vx protects scheduled matrix terms that divide by v_x.
# ============================================================
STATE_SPACE_MIN_VX_MS = 0.20
STATE_SPACE_K_DRAG_COEFF = 0.0


# ============================================================
# OUTER LONGITUDINAL REDUCED MODEL SETTINGS
# ------------------------------------------------------------
# Task 3.1:
#
#   v_x_dot = b_batt * PWM + d_x
#
# Current scaffold:
#
#   b_batt_est = efficiency * force_gain * (V_batt - V_sag) / m
#
# This uses the propulsion model from Task 2.6 and packages it
# in the report's outer-loop reduced form.
# ============================================================
# ============================================================
# INNER LATERAL-YAW REDUCED MODEL
# ------------------------------------------------------------
# Task 3.2. Protects the reduced model from division by tiny vx.
# Task 3.3 will later decide better scheduling/filtering.
# ============================================================
INNER_LATERAL_YAW_MIN_VX_MS = 0.20


# Task 3.3 scheduling variable handling.
# vx0 is a filtered version of vx_recon used by the inner lateral-yaw model.
INNER_SPEED_SCHEDULER_TAU_S = 0.40
INNER_SPEED_SCHEDULER_MAX_DT_S = 0.20

OUTER_LONGITUDINAL_DX_LIMIT_MS2 = 5.0

# Simple longitudinal resistance term for Task 3.1 refinement.
# Unit: 1/s because it is used as k_v * vx.
# First rough calibration from logs:
# steady vx ≈ 0.55 m/s and propulsion prediction ≈ 0.59 m/s^2
# so k_v ≈ 0.59 / 0.55 ≈ 1.1.
OUTER_LONGITUDINAL_K_V_S = 1.0

# ============================================================
# TASK 4.1: OUTER LONGITUDINAL REFERENCE MODEL
# ------------------------------------------------------------
# vx_m_dot = -a_m * vx_m + a_m * vx_ref
# Larger a_m -> faster reference response.
# ============================================================
OUTER_REFERENCE_A_M_S_INV = 2.0
OUTER_REFERENCE_SPEED_CMD_TO_VX_GAIN = 1.0
OUTER_REFERENCE_MIN_VX_REF_MS = 0.0
OUTER_REFERENCE_MAX_VX_REF_MS = 1.0
OUTER_REFERENCE_MAX_DT_S = 0.20




# ============================================================
# INNER LATERAL-YAW REFERENCE MODEL
# ------------------------------------------------------------
# Task 4.2: x_m_dot = A_m x_m + B_m u_c
# x_m = [vy_m, r_m]^T
# ============================================================
INNER_REFERENCE_MODEL_A_VY_S_INV = 3.0
INNER_REFERENCE_MODEL_A_R_S_INV = 4.0
INNER_REFERENCE_MODEL_MAX_ABS_UC_RAD_S = 2.0
INNER_REFERENCE_MODEL_MAX_ABS_VY_M_MS = 1.0
INNER_REFERENCE_MODEL_MAX_ABS_R_M_RAD_S = 2.0
INNER_REFERENCE_MODEL_MAX_DT_S = 0.10


# ============================================================
# TASK 5.1 BATTERY-SIDE GAIN ESTIMATOR SCAFFOLD
# ------------------------------------------------------------
# We estimate:
#   vx_dot = b_batt * PWM - k_v * vx
#
# Rearranged:
#   b_batt_inst = (a_long_filt + k_v * vx) / PWM
#
# This estimator is intentionally slow and protected.
# It is only for validation/debug for now.
# ============================================================
BATTERY_GAIN_EST_NOMINAL_MS2_PER_PWM = (
    PROPULSION_EFFICIENCY
    * PROPULSION_FORCE_GAIN_N_PER_V
    * BATTERY_NOMINAL_V
    / DYNAMIC_BICYCLE_MASS_KG
)

BATTERY_GAIN_EST_TAU_S = 8.0
BATTERY_GAIN_EST_MIN_PWM = 0.25
BATTERY_GAIN_EST_MIN_VX_MS = 0.20
BATTERY_GAIN_EST_MAX_ABS_R_RAD_S = 0.08
BATTERY_GAIN_EST_MAX_ABS_DELTA_RAD = 0.0873
BATTERY_GAIN_EST_MIN_DT_S = 0.0001
BATTERY_GAIN_EST_MAX_DT_S = 0.20
BATTERY_GAIN_EST_MIN_SCALE = 0.50
BATTERY_GAIN_EST_MAX_SCALE = 1.50

# ============================================================
# DEBUG FIELD SELECTION
# ------------------------------------------------------------
# Task 2.5 / 2.6 validation only.
# ============================================================


# ============================================================
# Task 3.4: inner-loop yaw-rate reference command
# ------------------------------------------------------------
# Temporary curvature proxy from camera outputs:
# kappa_ref = sign * (K_y * ye_cam_filt + K_psi * psi_rel_cam_filt)
# r_ref = vx0 * kappa_ref
# ============================================================
INNER_REF_KAPPA_FROM_YE_GAIN_1PM = 0.8
INNER_REF_KAPPA_FROM_PSI_GAIN_1PM = 1.6
INNER_REF_KAPPA_SIGN = 1.0

INNER_REF_MAX_ABS_KAPPA_1PM = 2.0
INNER_REF_MAX_ABS_R_REF_RAD_S = 1.2
INNER_REF_MIN_VX_MS = 0.20

DEBUG_FIELDS = [
    "speed_cmd",
    "vx_recon",
    "a_long_filt",
    "pwm_cmd",
    "outer_vx_loss",
    "outer_vx_dot_propulsion",
    "outer_vx_dot_pred",
    "outer_d_x_est",
    "outer_b_batt_est",
    "batt_gain_valid",
    "batt_gain_update",
    "batt_b_nom",
    "batt_b_voltage",
    "batt_b_inst",
    "batt_b_hat",
    "batt_b_error",
    "batt_alpha",
    "batt_pred_vx_dot_hat",
    "batt_reason",
    "r_recon",
    "delta_f_est_deg",
]
