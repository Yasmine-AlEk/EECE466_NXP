
import math

from .mrac_config import KINEMATIC_MAX_STEER_RAD


def clamp(value, min_value, max_value):
    return max(min_value, min(max_value, value))


def low_pass(previous_value, new_value, alpha):
    if previous_value is None:
        return new_value
    return (1.0 - alpha) * previous_value + alpha * new_value


def quaternion_to_yaw(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def wrap_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def turn_cmd_to_delta_f_est(turn_cmd: float) -> float:
    turn_cmd = clamp(turn_cmd, -1.0, 1.0)
    return turn_cmd * KINEMATIC_MAX_STEER_RAD
