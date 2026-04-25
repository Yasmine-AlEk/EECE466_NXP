
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
# y increases downward:
# - far row -> smaller y
# - near row -> larger y
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
# smaller alpha = more smoothing, more lag
# larger alpha = less smoothing, less lag
A_LONG_FILTER_ALPHA = 0.20

# ============================================================
# KINEMATIC BICYCLE SETTINGS
# ------------------------------------------------------------
# Based on teammate hardware bicycle model conventions:
# wheelbase = 0.168 m
# max steer = 30 deg
# low-speed deadzone = 0.05 m/s
#
# For the report's IV-A beta-based kinematic model, the
# wheelbase is split equally because lf/lr are not yet identified.
# ============================================================
KINEMATIC_WHEELBASE_M = 0.168
KINEMATIC_LF_M = 0.5 * KINEMATIC_WHEELBASE_M
KINEMATIC_LR_M = 0.5 * KINEMATIC_WHEELBASE_M
KINEMATIC_MAX_STEER_RAD = math.radians(30.0)
KINEMATIC_V_DEADZONE_MS = 0.05

# ============================================================
# DEBUG FIELD SELECTION
# ============================================================
DEBUG_FIELDS = [
    "turn_cmd",
    "vx_recon",
    "a_long_filt",
    "delta_f_cmd_est",
    "beta_kin",
    "x_dot_kin",
    "y_dot_kin",
    "psi_dot_kin",
    "v_dot_kin",
    "a_long_recon",
    "yaw_meas",
]
