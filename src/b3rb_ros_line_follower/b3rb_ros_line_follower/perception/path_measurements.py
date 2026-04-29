import json

from ..mrac_config import (
    CENTER_FILTER_ALPHA_ONE_EDGE,
    CENTER_FILTER_ALPHA_TWO_EDGES,
    MAX_CENTER_JUMP_RATIO,
)
from ..mrac_types import CameraMeasurement
from ..mrac_utils import clamp, low_pass

# Must match vision_chain.py BEV_W = 400
_BEV_HALF_WIDTH = 200.0


class PathMeasurementExtractor:
    """
    Extracts controller-side measurements from the /nxp_cup/lane_chains JSON message.

    Produces the same CameraMeasurement interface as CameraMeasurementExtractor so
    that BaselineLaneController and all MRAC code are format-agnostic.

    Chain index convention (from vision_chain.py):
        index 0  = nearest strip  (bottom of BEV, front of car)
        index -1 = farthest strip (top of BEV, lookahead point)
    """

    def __init__(self):
        self.center_far_filt  = None
        self.center_near_filt = None
        self.lane_width_filt  = None
        self.edge_balance_filt = None
        self.latest_camera_measurement = CameraMeasurement()

    def extract_from_json(self, json_str: str) -> CameraMeasurement:
        try:
            data = json.loads(json_str)
        except (ValueError, TypeError):
            return self.latest_camera_measurement

        center = data.get("center", [])
        left   = data.get("left",   [])
        right  = data.get("right",  [])
        valid  = data.get("valid",  {})
        have_left  = bool(valid.get("left",  False))
        have_right = bool(valid.get("right", False))

        measurement = CameraMeasurement()

        if len(center) < 1:
            self.latest_camera_measurement = measurement
            return measurement

        center_near_x = float(center[0][0])
        center_far_x  = float(center[-1][0]) if len(center) >= 2 else center_near_x

        if have_left and have_right and len(left) >= 1 and len(right) >= 1:
            lane_width_meas = float(right[0][0]) - float(left[0][0])
            if lane_width_meas > 1.0:
                if self.lane_width_filt is None:
                    self.lane_width_filt = lane_width_meas
                else:
                    self.lane_width_filt = 0.80 * self.lane_width_filt + 0.20 * lane_width_meas

        alpha = (
            CENTER_FILTER_ALPHA_TWO_EDGES
            if (have_left and have_right)
            else CENTER_FILTER_ALPHA_ONE_EDGE
        )

        max_jump = MAX_CENTER_JUMP_RATIO * _BEV_HALF_WIDTH

        if self.center_far_filt is not None:
            jump = center_far_x - self.center_far_filt
            center_far_x = self.center_far_filt + clamp(jump, -max_jump, max_jump)

        if self.center_near_filt is not None:
            jump = center_near_x - self.center_near_filt
            center_near_x = self.center_near_filt + clamp(jump, -max_jump, max_jump)

        self.center_far_filt  = low_pass(self.center_far_filt,  center_far_x,  alpha)
        self.center_near_filt = low_pass(self.center_near_filt, center_near_x, alpha)

        edge_balance_meas = 0.0
        if (have_left and have_right and
                len(left) >= 1 and len(right) >= 1 and
                self.lane_width_filt is not None and self.lane_width_filt > 1.0):
            left_near_x  = float(left[0][0])
            right_near_x = float(right[0][0])
            left_margin  = self.center_near_filt - left_near_x
            right_margin = right_near_x - self.center_near_filt
            edge_balance_meas = (left_margin - right_margin) / self.lane_width_filt

        self.edge_balance_filt = low_pass(self.edge_balance_filt, edge_balance_meas, alpha)

        measurement.have_measurement  = True
        measurement.use_integral      = have_left and have_right
        measurement.vector_count      = (2 if (have_left and have_right) else
                                         1 if (have_left or have_right) else 0)
        measurement.center_far_filt   = self.center_far_filt
        measurement.center_near_filt  = self.center_near_filt
        measurement.lane_width_px     = self.lane_width_filt or 0.0
        measurement.edge_balance_filt = self.edge_balance_filt or 0.0
        measurement.ye_cam_filt       = ((_BEV_HALF_WIDTH - self.center_far_filt)
                                         / _BEV_HALF_WIDTH)
        measurement.psi_rel_cam_filt  = ((self.center_near_filt - self.center_far_filt)
                                         / _BEV_HALF_WIDTH)

        self.latest_camera_measurement = measurement
        return measurement
