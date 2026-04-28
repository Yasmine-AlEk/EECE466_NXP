import math
from dataclasses import dataclass


@dataclass
class CorneringStiffnessRLSEstimate:
    """
    Task 5.4/5.5 RLS estimate for front/rear cornering stiffness.

    theta_hat = [C_alpha_f_hat, C_alpha_r_hat]^T

    This estimator is still a scaffold/debug estimator.
    It does not modify the controller yet.
    """

    valid: bool
    updated: bool
    reason: str

    c_alpha_f_hat: float
    c_alpha_r_hat: float
    c_alpha_f_prev: float
    c_alpha_r_prev: float

    error_0: float
    error_1: float
    y_hat_0: float
    y_hat_1: float

    gain_00: float
    gain_01: float
    gain_10: float
    gain_11: float

    p_00: float
    p_01: float
    p_10: float
    p_11: float
    p_trace: float

    lambda_forgetting: float
    update_count: int
    skip_count: int
    ready_sample_count: int


class CorneringStiffnessRLSModel:
    """
    Protected recursive least squares estimator for:

        y_corr = Phi * theta

    where:
        theta = [C_alpha_f, C_alpha_r]^T

    Task 5.5 additions:
        - low-speed blocking
        - steering/yaw excitation gates
        - stronger Phi-norm gate
        - filtered output magnitude checks
        - update decimation
        - parameter-step limiting
        - projection bounds
        - sigma-style pullback during low excitation
    """

    def __init__(
        self,
        initial_c_alpha_f: float,
        initial_c_alpha_r: float,
        p0: float,
        lambda_forgetting: float,
        min_c_alpha: float,
        max_c_alpha: float,
        min_det: float,
        max_covariance: float,
        min_update_vx_ms: float,
        min_abs_delta_rad: float,
        min_abs_r_rad_s: float,
        min_phi_norm: float,
        max_abs_ycorr_0: float,
        max_abs_ycorr_1: float,
        update_every_n_ready: int,
        max_abs_parameter_step: float,
        max_relative_parameter_step: float,
        sigma_pullback_alpha: float,
    ):
        self.theta_f = float(initial_c_alpha_f)
        self.theta_r = float(initial_c_alpha_r)

        self.nominal_theta_f = float(initial_c_alpha_f)
        self.nominal_theta_r = float(initial_c_alpha_r)

        p0 = abs(float(p0))
        self.p00 = p0
        self.p01 = 0.0
        self.p10 = 0.0
        self.p11 = p0

        self.lambda_forgetting = float(lambda_forgetting)
        self.min_c_alpha = float(min_c_alpha)
        self.max_c_alpha = float(max_c_alpha)
        self.min_det = abs(float(min_det))
        self.max_covariance = abs(float(max_covariance))

        self.min_update_vx_ms = abs(float(min_update_vx_ms))
        self.min_abs_delta_rad = abs(float(min_abs_delta_rad))
        self.min_abs_r_rad_s = abs(float(min_abs_r_rad_s))
        self.min_phi_norm = abs(float(min_phi_norm))
        self.max_abs_ycorr_0 = abs(float(max_abs_ycorr_0))
        self.max_abs_ycorr_1 = abs(float(max_abs_ycorr_1))
        self.update_every_n_ready = max(1, int(update_every_n_ready))
        self.max_abs_parameter_step = abs(float(max_abs_parameter_step))
        self.max_relative_parameter_step = abs(float(max_relative_parameter_step))
        self.sigma_pullback_alpha = self._clamp(
            float(sigma_pullback_alpha),
            0.0,
            1.0,
        )

        self.update_count = 0
        self.skip_count = 0
        self.ready_sample_count = 0

        if self.lambda_forgetting <= 0.0 or self.lambda_forgetting > 1.0:
            raise ValueError("lambda_forgetting must be in (0, 1].")

        if self.min_c_alpha <= 0.0:
            raise ValueError("min_c_alpha must be positive.")

        if self.max_c_alpha <= self.min_c_alpha:
            raise ValueError("max_c_alpha must be greater than min_c_alpha.")

    @staticmethod
    def _finite(*values) -> bool:
        return all(math.isfinite(float(v)) for v in values)

    @staticmethod
    def _clamp(value: float, lower: float, upper: float) -> float:
        return max(lower, min(upper, value))

    def _apply_projection(self):
        self.theta_f = self._clamp(
            self.theta_f,
            self.min_c_alpha,
            self.max_c_alpha,
        )
        self.theta_r = self._clamp(
            self.theta_r,
            self.min_c_alpha,
            self.max_c_alpha,
        )

    def _apply_sigma_pullback(self):
        """
        Small sigma-style regularization.

        When excitation is weak, pull the estimates very slightly back toward
        the nominal values. This prevents slow drift during long straight or
        weakly excited sections.
        """
        if self.sigma_pullback_alpha <= 0.0:
            return

        self.theta_f += self.sigma_pullback_alpha * (
            self.nominal_theta_f - self.theta_f
        )
        self.theta_r += self.sigma_pullback_alpha * (
            self.nominal_theta_r - self.theta_r
        )
        self._apply_projection()

    def _make_estimate(
        self,
        valid: bool,
        updated: bool,
        reason: str,
        prev_f: float,
        prev_r: float,
        error_0: float,
        error_1: float,
        y_hat_0: float,
        y_hat_1: float,
        gain_00: float,
        gain_01: float,
        gain_10: float,
        gain_11: float,
    ) -> CorneringStiffnessRLSEstimate:
        return CorneringStiffnessRLSEstimate(
            valid=valid,
            updated=updated,
            reason=reason,
            c_alpha_f_hat=self.theta_f,
            c_alpha_r_hat=self.theta_r,
            c_alpha_f_prev=prev_f,
            c_alpha_r_prev=prev_r,
            error_0=error_0,
            error_1=error_1,
            y_hat_0=y_hat_0,
            y_hat_1=y_hat_1,
            gain_00=gain_00,
            gain_01=gain_01,
            gain_10=gain_10,
            gain_11=gain_11,
            p_00=self.p00,
            p_01=self.p01,
            p_10=self.p10,
            p_11=self.p11,
            p_trace=self.p00 + self.p11,
            lambda_forgetting=self.lambda_forgetting,
            update_count=self.update_count,
            skip_count=self.skip_count,
            ready_sample_count=self.ready_sample_count,
        )

    def _skip(
        self,
        reason: str,
        prev_f: float,
        prev_r: float,
        valid: bool = False,
    ) -> CorneringStiffnessRLSEstimate:
        self.skip_count += 1

        if valid and (
            "low_excitation" in reason
            or "low_phi" in reason
            or "turn_too_small" in reason
        ):
            self._apply_sigma_pullback()

        return self._make_estimate(
            valid=valid,
            updated=False,
            reason=reason,
            prev_f=prev_f,
            prev_r=prev_r,
            error_0=0.0,
            error_1=0.0,
            y_hat_0=0.0,
            y_hat_1=0.0,
            gain_00=0.0,
            gain_01=0.0,
            gain_10=0.0,
            gain_11=0.0,
        )

    def _protect_sample(self, matrix):
        if abs(matrix.vx_ms) < self.min_update_vx_ms:
            return False, "protect_low_vx"

        if (
            abs(matrix.delta_f_rad) < self.min_abs_delta_rad
            or abs(matrix.r_rad_s) < self.min_abs_r_rad_s
        ):
            return False, "protect_turn_too_small"

        if matrix.phi_norm < self.min_phi_norm:
            return False, "protect_low_phi_norm"

        if abs(matrix.y_corr_0) > self.max_abs_ycorr_0:
            return False, "protect_ycorr0_spike"

        if abs(matrix.y_corr_1) > self.max_abs_ycorr_1:
            return False, "protect_ycorr1_spike"

        return True, "ok"

    def _limit_parameter_step(self, delta: float, current_value: float):
        rel_limit = self.max_relative_parameter_step * max(
            abs(current_value),
            1.0,
        )
        step_limit = min(self.max_abs_parameter_step, rel_limit)

        limited_delta = self._clamp(delta, -step_limit, step_limit)
        was_limited = abs(limited_delta - delta) > 1e-12

        return limited_delta, was_limited

    def update(self, matrix) -> CorneringStiffnessRLSEstimate:
        prev_f = self.theta_f
        prev_r = self.theta_r

        if matrix is None:
            return self._skip("missing_matrix", prev_f, prev_r, valid=False)

        if not getattr(matrix, "valid", False):
            return self._skip(
                f"matrix_{getattr(matrix, 'reason', 'invalid')}",
                prev_f,
                prev_r,
                valid=False,
            )

        if not getattr(matrix, "ready_for_update", False):
            return self._skip(
                f"matrix_{getattr(matrix, 'reason', 'not_ready')}",
                prev_f,
                prev_r,
                valid=True,
            )

        protect_ok, protect_reason = self._protect_sample(matrix)

        if not protect_ok:
            return self._skip(
                protect_reason,
                prev_f,
                prev_r,
                valid=True,
            )

        self.ready_sample_count += 1

        if (self.ready_sample_count % self.update_every_n_ready) != 0:
            return self._skip(
                "protect_update_decimation",
                prev_f,
                prev_r,
                valid=True,
            )

        phi00 = float(matrix.phi_00)
        phi01 = float(matrix.phi_01)
        phi10 = float(matrix.phi_10)
        phi11 = float(matrix.phi_11)
        y0 = float(matrix.y_corr_0)
        y1 = float(matrix.y_corr_1)

        if not self._finite(
            phi00,
            phi01,
            phi10,
            phi11,
            y0,
            y1,
            self.theta_f,
            self.theta_r,
            self.p00,
            self.p01,
            self.p10,
            self.p11,
        ):
            return self._skip("nonfinite_input", prev_f, prev_r, valid=False)

        # Prediction: y_hat = Phi * theta_hat.
        y_hat_0 = phi00 * self.theta_f + phi01 * self.theta_r
        y_hat_1 = phi10 * self.theta_f + phi11 * self.theta_r

        error_0 = y0 - y_hat_0
        error_1 = y1 - y_hat_1

        # A = Phi * P.
        a00 = phi00 * self.p00 + phi01 * self.p10
        a01 = phi00 * self.p01 + phi01 * self.p11
        a10 = phi10 * self.p00 + phi11 * self.p10
        a11 = phi10 * self.p01 + phi11 * self.p11

        # S = lambda * I + Phi * P * Phi^T.
        s00 = self.lambda_forgetting + a00 * phi00 + a01 * phi01
        s01 = a00 * phi10 + a01 * phi11
        s10 = a10 * phi00 + a11 * phi01
        s11 = self.lambda_forgetting + a10 * phi10 + a11 * phi11

        det_s = s00 * s11 - s01 * s10

        if abs(det_s) < self.min_det:
            return self._skip("singular_s", prev_f, prev_r, valid=False)

        inv_s00 = s11 / det_s
        inv_s01 = -s01 / det_s
        inv_s10 = -s10 / det_s
        inv_s11 = s00 / det_s

        # M = P * Phi^T.
        m00 = self.p00 * phi00 + self.p01 * phi01
        m01 = self.p00 * phi10 + self.p01 * phi11
        m10 = self.p10 * phi00 + self.p11 * phi01
        m11 = self.p10 * phi10 + self.p11 * phi11

        # K = P * Phi^T * S^-1.
        k00 = m00 * inv_s00 + m01 * inv_s10
        k01 = m00 * inv_s01 + m01 * inv_s11
        k10 = m10 * inv_s00 + m11 * inv_s10
        k11 = m10 * inv_s01 + m11 * inv_s11

        raw_delta_f = k00 * error_0 + k01 * error_1
        raw_delta_r = k10 * error_0 + k11 * error_1

        limited_delta_f, limited_f = self._limit_parameter_step(
            raw_delta_f,
            self.theta_f,
        )
        limited_delta_r, limited_r = self._limit_parameter_step(
            raw_delta_r,
            self.theta_r,
        )

        self.theta_f += limited_delta_f
        self.theta_r += limited_delta_r

        before_projection_f = self.theta_f
        before_projection_r = self.theta_r

        self._apply_projection()

        projected = (
            abs(before_projection_f - self.theta_f) > 1e-12
            or abs(before_projection_r - self.theta_r) > 1e-12
        )

        # P_new = (P - K * Phi * P) / lambda.
        # We already computed A = Phi * P.
        kp00 = k00 * a00 + k01 * a10
        kp01 = k00 * a01 + k01 * a11
        kp10 = k10 * a00 + k11 * a10
        kp11 = k10 * a01 + k11 * a11

        p00_new = (self.p00 - kp00) / self.lambda_forgetting
        p01_new = (self.p01 - kp01) / self.lambda_forgetting
        p10_new = (self.p10 - kp10) / self.lambda_forgetting
        p11_new = (self.p11 - kp11) / self.lambda_forgetting

        # Keep P symmetric. This is numerically helpful for repeated updates.
        p01_sym = 0.5 * (p01_new + p10_new)

        self.p00 = self._clamp(p00_new, 1e-9, self.max_covariance)
        self.p01 = self._clamp(
            p01_sym,
            -self.max_covariance,
            self.max_covariance,
        )
        self.p10 = self.p01
        self.p11 = self._clamp(p11_new, 1e-9, self.max_covariance)

        self.update_count += 1

        step_limited = limited_f or limited_r

        if projected and step_limited:
            reason = "projected_step_limited"
        elif projected:
            reason = "projected"
        elif step_limited:
            reason = "step_limited"
        else:
            reason = "ok"

        return self._make_estimate(
            valid=True,
            updated=True,
            reason=reason,
            prev_f=prev_f,
            prev_r=prev_r,
            error_0=error_0,
            error_1=error_1,
            y_hat_0=y_hat_0,
            y_hat_1=y_hat_1,
            gain_00=k00,
            gain_01=k01,
            gain_10=k10,
            gain_11=k11,
        )
