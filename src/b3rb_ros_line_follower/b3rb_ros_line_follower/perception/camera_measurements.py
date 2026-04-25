
from ..mrac_config import (
    CENTER_FILTER_ALPHA_ONE_EDGE,
    CENTER_FILTER_ALPHA_TWO_EDGES,
    DEFAULT_SINGLE_EDGE_LANE_HALF_WIDTH_RATIO,
    LOOKAHEAD_FAR_Y_RATIO,
    LOOKAHEAD_NEAR_Y_RATIO,
    MAX_CENTER_JUMP_RATIO,
)
from ..mrac_types import CameraMeasurement
from ..mrac_utils import clamp, low_pass


class CameraMeasurementExtractor:
    """Extracts filtered controller-side measurements from /edge_vectors."""

    def __init__(self):
        self.center_far_filt = None
        self.center_near_filt = None
        self.lane_width_filt = None
        self.edge_balance_filt = None
        self.latest_camera_measurement = CameraMeasurement()

    def estimate_center_from_one_edge(self, edge_top_x_meas, edge_bottom_x_meas, half_width):
        if self.lane_width_filt is not None and self.lane_width_filt > 1.0:
            lane_half_width = 0.5 * self.lane_width_filt
        else:
            lane_half_width = DEFAULT_SINGLE_EDGE_LANE_HALF_WIDTH_RATIO * half_width

        edge_mid_x = 0.5 * (edge_top_x_meas + edge_bottom_x_meas)

        if edge_mid_x < half_width:
            center_top_est = edge_top_x_meas + lane_half_width
            center_bottom_est = edge_bottom_x_meas + lane_half_width
        else:
            center_top_est = edge_top_x_meas - lane_half_width
            center_bottom_est = edge_bottom_x_meas - lane_half_width

        return center_top_est, center_bottom_est

    def get_preview_rows(self, image_height: int):
        if image_height <= 1:
            return 0.0, 0.0

        y_max = float(image_height - 1)

        y_far = LOOKAHEAD_FAR_Y_RATIO * y_max
        y_near = LOOKAHEAD_NEAR_Y_RATIO * y_max

        y_far = clamp(y_far, 0.0, y_max)
        y_near = clamp(y_near, 0.0, y_max)

        if y_near < y_far:
            y_far, y_near = y_near, y_far

        return y_far, y_near

    @staticmethod
    def ordered_vector_points(p0, p1):
        top = (float(p0.x), float(p0.y))
        bottom = (float(p1.x), float(p1.y))

        if top[1] > bottom[1]:
            top, bottom = bottom, top

        return top, bottom

    @staticmethod
    def interpolate_x_at_y(top_pt, bottom_pt, y_query: float) -> float:
        x1, y1 = top_pt
        x2, y2 = bottom_pt

        dy = y2 - y1
        if abs(dy) < 1e-6:
            return x2

        t = (y_query - y1) / dy
        t = clamp(t, 0.0, 1.0)

        return x1 + t * (x2 - x1)

    def extract(self, vectors, half_width):
        measurement = CameraMeasurement(
            have_measurement=False,
            use_integral=False,
            vector_count=vectors.vector_count,
            ye_cam_filt=0.0,
            psi_rel_cam_filt=0.0,
            edge_balance_filt=0.0,
            center_far_filt=0.0,
            center_near_filt=0.0,
            lane_width_px=0.0
        )

        image_height = vectors.image_height if vectors.image_height > 0 else 1
        y_far, y_near = self.get_preview_rows(image_height)

        if vectors.vector_count >= 2:
            vec1_top, vec1_bottom = self.ordered_vector_points(vectors.vector_1[0], vectors.vector_1[1])
            vec2_top, vec2_bottom = self.ordered_vector_points(vectors.vector_2[0], vectors.vector_2[1])

            if vec1_bottom[0] <= vec2_bottom[0]:
                left_top, left_bottom = vec1_top, vec1_bottom
                right_top, right_bottom = vec2_top, vec2_bottom
            else:
                left_top, left_bottom = vec2_top, vec2_bottom
                right_top, right_bottom = vec1_top, vec1_bottom

            left_far_x = self.interpolate_x_at_y(left_top, left_bottom, y_far)
            left_near_x = self.interpolate_x_at_y(left_top, left_bottom, y_near)
            right_far_x = self.interpolate_x_at_y(right_top, right_bottom, y_far)
            right_near_x = self.interpolate_x_at_y(right_top, right_bottom, y_near)

            lane_width_far = right_far_x - left_far_x
            lane_width_near = right_near_x - left_near_x
            lane_width_meas = 0.5 * (lane_width_far + lane_width_near)

            if lane_width_meas > 1.0:
                if self.lane_width_filt is None:
                    self.lane_width_filt = lane_width_meas
                else:
                    self.lane_width_filt = 0.80 * self.lane_width_filt + 0.20 * lane_width_meas

            center_far_meas = 0.5 * (left_far_x + right_far_x)
            center_near_meas = 0.5 * (left_near_x + right_near_x)

            if lane_width_near > 1.0:
                left_margin = center_near_meas - left_near_x
                right_margin = right_near_x - center_near_meas
                edge_balance_meas = (left_margin - right_margin) / lane_width_near
            else:
                edge_balance_meas = 0.0

            max_jump = MAX_CENTER_JUMP_RATIO * half_width

            if self.center_far_filt is not None:
                jump_far = center_far_meas - self.center_far_filt
                center_far_meas = self.center_far_filt + clamp(jump_far, -max_jump, max_jump)

            if self.center_near_filt is not None:
                jump_near = center_near_meas - self.center_near_filt
                center_near_meas = self.center_near_filt + clamp(jump_near, -max_jump, max_jump)

            self.center_far_filt = low_pass(
                self.center_far_filt,
                center_far_meas,
                CENTER_FILTER_ALPHA_TWO_EDGES
            )
            self.center_near_filt = low_pass(
                self.center_near_filt,
                center_near_meas,
                CENTER_FILTER_ALPHA_TWO_EDGES
            )
            self.edge_balance_filt = low_pass(
                self.edge_balance_filt,
                edge_balance_meas,
                CENTER_FILTER_ALPHA_TWO_EDGES
            )

            measurement.have_measurement = True
            measurement.use_integral = True

        elif vectors.vector_count == 1:
            edge_top, edge_bottom = self.ordered_vector_points(vectors.vector_1[0], vectors.vector_1[1])

            center_top_est, center_bottom_est = self.estimate_center_from_one_edge(
                edge_top[0],
                edge_bottom[0],
                half_width
            )

            center_far_est = self.interpolate_x_at_y(
                (center_top_est, edge_top[1]),
                (center_bottom_est, edge_bottom[1]),
                y_far
            )
            center_near_est = self.interpolate_x_at_y(
                (center_top_est, edge_top[1]),
                (center_bottom_est, edge_bottom[1]),
                y_near
            )

            max_jump = MAX_CENTER_JUMP_RATIO * half_width

            if self.center_far_filt is not None:
                jump_far = center_far_est - self.center_far_filt
                center_far_est = self.center_far_filt + clamp(jump_far, -max_jump, max_jump)

            if self.center_near_filt is not None:
                jump_near = center_near_est - self.center_near_filt
                center_near_est = self.center_near_filt + clamp(jump_near, -max_jump, max_jump)

            self.center_far_filt = low_pass(
                self.center_far_filt,
                center_far_est,
                CENTER_FILTER_ALPHA_ONE_EDGE
            )
            self.center_near_filt = low_pass(
                self.center_near_filt,
                center_near_est,
                CENTER_FILTER_ALPHA_ONE_EDGE
            )
            self.edge_balance_filt = low_pass(
                self.edge_balance_filt,
                0.0,
                CENTER_FILTER_ALPHA_ONE_EDGE
            )

            measurement.have_measurement = True
            measurement.use_integral = False

        if measurement.have_measurement:
            measurement.center_far_filt = self.center_far_filt
            measurement.center_near_filt = self.center_near_filt
            measurement.edge_balance_filt = 0.0 if self.edge_balance_filt is None else self.edge_balance_filt
            measurement.lane_width_px = 0.0 if self.lane_width_filt is None else self.lane_width_filt

            measurement.ye_cam_filt = (half_width - self.center_far_filt) / half_width
            measurement.psi_rel_cam_filt = (self.center_near_filt - self.center_far_filt) / half_width

        self.latest_camera_measurement = measurement
        return measurement
