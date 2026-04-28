import math
from dataclasses import dataclass


@dataclass
class CorneringStiffnessRegressionOutputs:
    """
    Task 5.2 output scaffold for later cornering-stiffness RLS.

    Report outputs:
        y1 = a_y,meas = v_y_dot + v_x * r
        y2 = r_dot
        y2_corr = r_dot - (t_r / (2 * I_z)) * Delta_Fx

    In the current Gazebo scaffold, true IMU a_y is not wired in yet,
    so y1 is reconstructed from odometry-based v_y and yaw rate.
    """

    valid: bool
    reason: str

    vx_ms: float
    vy_ms: float
    r_rad_s: float
    dt_s: float

    vy_dot_raw_ms2: float
    r_dot_raw_rad_s2: float

    ay_raw_ms2: float
    ay_filt_ms2: float

    r_dot_filt_rad_s2: float
    dfx_n: float
    tv_correction_rad_s2: float
    y2_corr_rad_s2: float

    filter_alpha: float


class CorneringStiffnessRegressionOutputModel:
    """
    Computes the measurable outputs needed before building the RLS regressor.

    This class intentionally does not estimate C_alpha yet.
    It only prepares smooth, finite y1/y2 signals for Task 5.3 and 5.4.
    """

    def __init__(
        self,
        min_dt_s: float,
        max_dt_s: float,
        filter_alpha: float,
        min_vx_ms: float,
        max_abs_ay_ms2: float,
        max_abs_r_dot_rad_s2: float,
        rear_track_width_m: float,
        iz_kg_m2: float,
    ):
        self.min_dt_s = float(min_dt_s)
        self.max_dt_s = float(max_dt_s)
        self.filter_alpha = self._clamp(float(filter_alpha), 0.0, 1.0)
        self.min_vx_ms = float(min_vx_ms)
        self.max_abs_ay_ms2 = abs(float(max_abs_ay_ms2))
        self.max_abs_r_dot_rad_s2 = abs(float(max_abs_r_dot_rad_s2))
        self.rear_track_width_m = float(rear_track_width_m)
        self.iz_kg_m2 = float(iz_kg_m2)

        self.prev_vy_ms = None
        self.prev_r_rad_s = None

        self.ay_filt_ms2 = 0.0
        self.r_dot_filt_rad_s2 = 0.0

    @staticmethod
    def _finite(*values) -> bool:
        return all(math.isfinite(float(v)) for v in values)

    @staticmethod
    def _clamp(value: float, min_value: float, max_value: float) -> float:
        return max(min_value, min(max_value, value))

    def _make_output(
        self,
        valid: bool,
        reason: str,
        vx_ms: float,
        vy_ms: float,
        r_rad_s: float,
        dt_s: float,
        vy_dot_raw_ms2: float,
        r_dot_raw_rad_s2: float,
        ay_raw_ms2: float,
        dfx_n: float,
    ) -> CorneringStiffnessRegressionOutputs:
        if self.iz_kg_m2 > 1e-9:
            tv_correction = (
                self.rear_track_width_m / (2.0 * self.iz_kg_m2)
            ) * dfx_n
        else:
            tv_correction = 0.0

        y2_corr = self.r_dot_filt_rad_s2 - tv_correction

        return CorneringStiffnessRegressionOutputs(
            valid=valid,
            reason=reason,
            vx_ms=vx_ms,
            vy_ms=vy_ms,
            r_rad_s=r_rad_s,
            dt_s=dt_s,
            vy_dot_raw_ms2=vy_dot_raw_ms2,
            r_dot_raw_rad_s2=r_dot_raw_rad_s2,
            ay_raw_ms2=ay_raw_ms2,
            ay_filt_ms2=self.ay_filt_ms2,
            r_dot_filt_rad_s2=self.r_dot_filt_rad_s2,
            dfx_n=dfx_n,
            tv_correction_rad_s2=tv_correction,
            y2_corr_rad_s2=y2_corr,
            filter_alpha=self.filter_alpha,
        )

    def update(
        self,
        vx_ms: float,
        vy_ms: float,
        r_rad_s: float,
        dfx_n: float,
        dt_s: float,
    ) -> CorneringStiffnessRegressionOutputs:
        vx_ms = float(vx_ms)
        vy_ms = float(vy_ms)
        r_rad_s = float(r_rad_s)
        dfx_n = float(dfx_n)
        dt_s = float(dt_s)

        if not self._finite(vx_ms, vy_ms, r_rad_s, dfx_n, dt_s):
            return self._make_output(
                valid=False,
                reason="nonfinite_input",
                vx_ms=vx_ms,
                vy_ms=vy_ms,
                r_rad_s=r_rad_s,
                dt_s=dt_s,
                vy_dot_raw_ms2=0.0,
                r_dot_raw_rad_s2=0.0,
                ay_raw_ms2=0.0,
                dfx_n=dfx_n,
            )

        if dt_s < self.min_dt_s or dt_s > self.max_dt_s:
            # Store the current sample so the next valid dt is not using old data.
            self.prev_vy_ms = vy_ms
            self.prev_r_rad_s = r_rad_s

            return self._make_output(
                valid=False,
                reason="bad_dt",
                vx_ms=vx_ms,
                vy_ms=vy_ms,
                r_rad_s=r_rad_s,
                dt_s=dt_s,
                vy_dot_raw_ms2=0.0,
                r_dot_raw_rad_s2=0.0,
                ay_raw_ms2=0.0,
                dfx_n=dfx_n,
            )

        if self.prev_vy_ms is None or self.prev_r_rad_s is None:
            self.prev_vy_ms = vy_ms
            self.prev_r_rad_s = r_rad_s

            return self._make_output(
                valid=False,
                reason="first_sample",
                vx_ms=vx_ms,
                vy_ms=vy_ms,
                r_rad_s=r_rad_s,
                dt_s=dt_s,
                vy_dot_raw_ms2=0.0,
                r_dot_raw_rad_s2=0.0,
                ay_raw_ms2=0.0,
                dfx_n=dfx_n,
            )

        vy_dot_raw = (vy_ms - self.prev_vy_ms) / dt_s
        r_dot_raw = (r_rad_s - self.prev_r_rad_s) / dt_s

        # Report relation: ay_meas = vy_dot + vx * r.
        ay_raw = vy_dot_raw + vx_ms * r_rad_s

        ay_for_filter = self._clamp(
            ay_raw,
            -self.max_abs_ay_ms2,
            self.max_abs_ay_ms2,
        )
        r_dot_for_filter = self._clamp(
            r_dot_raw,
            -self.max_abs_r_dot_rad_s2,
            self.max_abs_r_dot_rad_s2,
        )

        self.ay_filt_ms2 = (
            (1.0 - self.filter_alpha) * self.ay_filt_ms2
            + self.filter_alpha * ay_for_filter
        )
        self.r_dot_filt_rad_s2 = (
            (1.0 - self.filter_alpha) * self.r_dot_filt_rad_s2
            + self.filter_alpha * r_dot_for_filter
        )

        self.prev_vy_ms = vy_ms
        self.prev_r_rad_s = r_rad_s

        valid = True
        reason = "ok"

        if abs(vx_ms) < self.min_vx_ms:
            valid = False
            reason = "low_vx"
        elif abs(ay_raw) > self.max_abs_ay_ms2:
            valid = False
            reason = "ay_clamped"
        elif abs(r_dot_raw) > self.max_abs_r_dot_rad_s2:
            valid = False
            reason = "rdot_clamped"

        return self._make_output(
            valid=valid,
            reason=reason,
            vx_ms=vx_ms,
            vy_ms=vy_ms,
            r_rad_s=r_rad_s,
            dt_s=dt_s,
            vy_dot_raw_ms2=vy_dot_raw,
            r_dot_raw_rad_s2=r_dot_raw,
            ay_raw_ms2=ay_raw,
            dfx_n=dfx_n,
        )
