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
# DEBUG FIELD SELECTION
# ------------------------------------------------------------
# Task 2.5 / 2.6 validation only.
# ============================================================
DEBUG_FIELDS = [
    # Task 2.7 + 2.8 validation
    "vx_recon",
    "vy_recon",
    "r_recon",

    "fx_left",
    "fx_right",
    "fx_sum_recon",
    "dfx_recon",

    "delta_f_est_deg",

    "ss_valid",
    "ss_x_dim",
    "ss_u_dim",
    "ss_A_dim",
    "ss_B_dim",

    "ss_vx_safe",
    "ss_xdot_vx",
    "ss_xdot_vy",
    "ss_xdot_r",

    "ss_A_11",
    "ss_A_12",
    "ss_A_21",
    "ss_A_22",
    "ss_B_00",
    "ss_B_01",
    "ss_B_20",
    "ss_B_21",
    "ss_B_22",
]
