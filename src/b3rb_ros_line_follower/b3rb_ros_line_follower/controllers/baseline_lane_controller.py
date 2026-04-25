
import time

from ..mrac_config import (
    CURVE_SPEED_GAIN,
    INTEGRAL_LIMIT,
    KP_CENTER,
    KP_EDGE_BALANCE,
    KP_HEADING,
    KD_CENTER,
    KI_CENTER,
    LEFT_TURN,
    RIGHT_TURN,
    SPEED_MAX,
    SPEED_MIN,
    SPEED_ONE_EDGE,
    SPEED_SEARCH,
    SPEED_SMOOTHING_NEW,
    SPEED_SMOOTHING_OLD,
    SPEED_STRAIGHT,
    SPEED_TIGHT_TURN,
    TURN_MAX,
    TURN_MIN,
    TURN_SMOOTHING_NEW,
    TURN_SMOOTHING_OLD,
)
from ..mrac_utils import clamp


class BaselineLaneController:
    """Current PID-like lane-following baseline."""

    def __init__(self):
        self.last_turn_cmd = 0.0
        self.last_speed_cmd = 0.0

        self.center_error_integral = 0.0
        self.prev_center_error = 0.0
        self.prev_control_time = time.monotonic()

    def get_dt(self):
        now = time.monotonic()
        dt = now - self.prev_control_time
        self.prev_control_time = now

        if dt <= 0.0 or dt > 0.2:
            dt = 0.05

        return dt

    def compute(self, camera, stop_sign: bool, obstacle_detected: bool, ramp_detected: bool):
        dt_s = self.get_dt()

        raw_turn_cmd = self.last_turn_cmd
        target_speed_cmd = SPEED_SEARCH

        if camera.have_measurement:
            ye_cam_filt = camera.ye_cam_filt
            psi_rel_cam_filt = camera.psi_rel_cam_filt
            edge_balance_filt = camera.edge_balance_filt

            if camera.use_integral:
                self.center_error_integral += ye_cam_filt * dt_s
                self.center_error_integral = clamp(
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
                KP_HEADING * psi_rel_cam_filt +
                KP_EDGE_BALANCE * edge_balance_filt
            )

            raw_turn_cmd = clamp(raw_turn_cmd, TURN_MIN, TURN_MAX)

            if camera.vector_count >= 2:
                curve_proxy = max(
                    abs(psi_rel_cam_filt),
                    0.60 * abs(ye_cam_filt),
                    0.80 * abs(edge_balance_filt)
                )

                target_speed_cmd = SPEED_STRAIGHT - CURVE_SPEED_GAIN * curve_proxy
                target_speed_cmd = clamp(
                    target_speed_cmd,
                    SPEED_TIGHT_TURN,
                    SPEED_STRAIGHT
                )
            else:
                target_speed_cmd = SPEED_ONE_EDGE

        else:
            self.center_error_integral *= 0.85
            self.prev_center_error *= 0.85
            raw_turn_cmd = self.last_turn_cmd * 0.96
            target_speed_cmd = SPEED_SEARCH

        if ramp_detected:
            target_speed_cmd = max(target_speed_cmd, SPEED_ONE_EDGE)

        if stop_sign:
            target_speed_cmd = SPEED_MIN
            self.center_error_integral = 0.0

        if obstacle_detected:
            target_speed_cmd = SPEED_MIN
            self.center_error_integral = 0.0

        turn_cmd = TURN_SMOOTHING_OLD * self.last_turn_cmd + TURN_SMOOTHING_NEW * raw_turn_cmd
        turn_cmd = clamp(turn_cmd, RIGHT_TURN, LEFT_TURN)

        if target_speed_cmd == SPEED_MIN:
            speed_cmd = SPEED_MIN
        else:
            speed_cmd = SPEED_SMOOTHING_OLD * self.last_speed_cmd + SPEED_SMOOTHING_NEW * target_speed_cmd
            speed_cmd = clamp(speed_cmd, SPEED_MIN, SPEED_MAX)

        self.last_turn_cmd = turn_cmd
        self.last_speed_cmd = speed_cmd

        return turn_cmd, speed_cmd, dt_s
