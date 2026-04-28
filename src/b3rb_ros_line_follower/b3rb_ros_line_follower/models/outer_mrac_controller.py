from dataclasses import dataclass
import math


@dataclass
class OuterMRACOutput:
    valid: bool
    adaptation_enabled: bool
    update_enabled: bool
    applied: bool
    reason: str

    dt_s: float
    vx_ms: float
    vx_m_ms: float
    vx_ref_ms: float
    ex_ms: float

    phi_ref: float
    phi_vx: float
    phi_norm: float

    k_r_hat: float
    k_x_hat: float
    k_r_dot: float
    k_x_dot: float
    theta_norm: float

    pwm_baseline: float
    pwm_adaptive_raw: float
    pwm_adaptive_sat: float
    pwm_final: float

    b_hat_ms2_per_pwm: float


class OuterMRACController:
    """
    Task 6.1 outer-loop MRAC controller.

    Reduced plant used in the report:
        vx_dot = b_batt * PWM - k_v * vx + uncertainty

    Control law:
        PWM = k_r_hat * vx_ref - k_x_hat * vx

    This class can compute and adapt the MRAC law, but by default the
    integrated node will keep it in shadow mode so the baseline PID speed
    command remains the actual command sent to the car.
    """

    def __init__(
        self,
        k_r_initial: float,
        k_x_initial: float,
        k_r_nominal: float,
        k_x_nominal: float,
        gamma_k_r: float,
        gamma_k_x: float,
        sigma: float,
        min_k_r: float,
        max_k_r: float,
        min_k_x: float,
        max_k_x: float,
        min_vx_ms: float,
        min_abs_pwm: float,
        min_phi_norm: float,
        max_abs_ex_ms: float,
        max_abs_r_rad_s: float,
        max_abs_delta_rad: float,
        min_b_hat_ms2_per_pwm: float,
        max_abs_theta_dot: float,
        enable_adaptation: bool,
        apply_to_speed_cmd: bool,
        apply_blend_alpha: float,
        max_applied_pwm_delta: float,
        pwm_min: float,
        pwm_max: float,
    ):
        self.k_r_hat = float(k_r_initial)
        self.k_x_hat = float(k_x_initial)

        self.k_r_nominal = float(k_r_nominal)
        self.k_x_nominal = float(k_x_nominal)

        self.gamma_k_r = max(0.0, float(gamma_k_r))
        self.gamma_k_x = max(0.0, float(gamma_k_x))
        self.sigma = max(0.0, float(sigma))

        self.min_k_r = float(min_k_r)
        self.max_k_r = float(max_k_r)
        self.min_k_x = float(min_k_x)
        self.max_k_x = float(max_k_x)

        self.min_vx_ms = max(0.0, float(min_vx_ms))
        self.min_abs_pwm = max(0.0, float(min_abs_pwm))
        self.min_phi_norm = max(0.0, float(min_phi_norm))
        self.max_abs_ex_ms = max(0.0, float(max_abs_ex_ms))
        self.max_abs_r_rad_s = max(0.0, float(max_abs_r_rad_s))
        self.max_abs_delta_rad = max(0.0, float(max_abs_delta_rad))
        self.min_b_hat_ms2_per_pwm = max(1.0e-9, float(min_b_hat_ms2_per_pwm))
        self.max_abs_theta_dot = max(0.0, float(max_abs_theta_dot))

        self.enable_adaptation = bool(enable_adaptation)
        self.apply_to_speed_cmd = bool(apply_to_speed_cmd)
        self.apply_blend_alpha = self._clamp(float(apply_blend_alpha), 0.0, 1.0)
        self.max_applied_pwm_delta = abs(float(max_applied_pwm_delta))

        self.pwm_min = float(pwm_min)
        self.pwm_max = float(pwm_max)

        self.k_r_hat = self._clamp(self.k_r_hat, self.min_k_r, self.max_k_r)
        self.k_x_hat = self._clamp(self.k_x_hat, self.min_k_x, self.max_k_x)

    @staticmethod
    def _clamp(value: float, lower: float, upper: float) -> float:
        return max(lower, min(upper, value))

    def _make_output(
        self,
        valid: bool,
        adaptation_enabled: bool,
        update_enabled: bool,
        applied: bool,
        reason: str,
        dt_s: float,
        vx_ms: float,
        vx_m_ms: float,
        vx_ref_ms: float,
        ex_ms: float,
        phi_ref: float,
        phi_vx: float,
        phi_norm: float,
        k_r_dot: float,
        k_x_dot: float,
        pwm_baseline: float,
        pwm_adaptive_raw: float,
        pwm_adaptive_sat: float,
        pwm_final: float,
        b_hat_ms2_per_pwm: float,
    ) -> OuterMRACOutput:
        theta_norm = math.sqrt(self.k_r_hat * self.k_r_hat + self.k_x_hat * self.k_x_hat)

        return OuterMRACOutput(
            valid=valid,
            adaptation_enabled=adaptation_enabled,
            update_enabled=update_enabled,
            applied=applied,
            reason=reason,
            dt_s=dt_s,
            vx_ms=vx_ms,
            vx_m_ms=vx_m_ms,
            vx_ref_ms=vx_ref_ms,
            ex_ms=ex_ms,
            phi_ref=phi_ref,
            phi_vx=phi_vx,
            phi_norm=phi_norm,
            k_r_hat=self.k_r_hat,
            k_x_hat=self.k_x_hat,
            k_r_dot=k_r_dot,
            k_x_dot=k_x_dot,
            theta_norm=theta_norm,
            pwm_baseline=pwm_baseline,
            pwm_adaptive_raw=pwm_adaptive_raw,
            pwm_adaptive_sat=pwm_adaptive_sat,
            pwm_final=pwm_final,
            b_hat_ms2_per_pwm=b_hat_ms2_per_pwm,
        )

    def update(
        self,
        vx_ms: float,
        reference_output,
        pwm_baseline: float,
        r_rad_s: float,
        delta_f_rad: float,
        b_hat_ms2_per_pwm: float,
        dt_s: float,
    ) -> OuterMRACOutput:
        dt_s = float(dt_s)
        vx_ms = float(vx_ms)
        pwm_baseline = float(pwm_baseline)
        r_rad_s = float(r_rad_s)
        delta_f_rad = float(delta_f_rad)
        b_hat_ms2_per_pwm = float(b_hat_ms2_per_pwm)

        if reference_output is None:
            return self._make_output(
                valid=False,
                adaptation_enabled=self.enable_adaptation,
                update_enabled=False,
                applied=False,
                reason="no_outer_reference",
                dt_s=dt_s,
                vx_ms=vx_ms,
                vx_m_ms=0.0,
                vx_ref_ms=0.0,
                ex_ms=0.0,
                phi_ref=0.0,
                phi_vx=0.0,
                phi_norm=0.0,
                k_r_dot=0.0,
                k_x_dot=0.0,
                pwm_baseline=pwm_baseline,
                pwm_adaptive_raw=pwm_baseline,
                pwm_adaptive_sat=pwm_baseline,
                pwm_final=pwm_baseline,
                b_hat_ms2_per_pwm=b_hat_ms2_per_pwm,
            )

        vx_ref_ms = float(reference_output.vx_ref_ms)
        vx_m_ms = float(reference_output.vx_m_ms)
        ex_ms = vx_ms - vx_m_ms

        phi_ref = vx_ref_ms
        phi_vx = -vx_ms
        phi_norm = math.sqrt(phi_ref * phi_ref + phi_vx * phi_vx)

        valid = bool(getattr(reference_output, "valid", True))
        reason = "ok"

        if not valid:
            reason = "reference_invalid"
        elif dt_s <= 0.0:
            valid = False
            reason = "bad_dt"
        elif b_hat_ms2_per_pwm < self.min_b_hat_ms2_per_pwm:
            valid = False
            reason = "bad_b_hat"

        pwm_adaptive_raw = self.k_r_hat * phi_ref + self.k_x_hat * phi_vx
        pwm_adaptive_sat = self._clamp(
            pwm_adaptive_raw,
            self.pwm_min,
            self.pwm_max,
        )

        update_enabled = valid
        if update_enabled and not self.enable_adaptation:
            update_enabled = False
            reason = "adaptation_disabled"
        elif update_enabled and abs(vx_ms) < self.min_vx_ms:
            update_enabled = False
            reason = "protect_low_vx"
        elif update_enabled and abs(pwm_baseline) < self.min_abs_pwm:
            update_enabled = False
            reason = "protect_low_pwm"
        elif update_enabled and abs(r_rad_s) > self.max_abs_r_rad_s:
            update_enabled = False
            reason = "protect_turning_r"
        elif update_enabled and abs(delta_f_rad) > self.max_abs_delta_rad:
            update_enabled = False
            reason = "protect_turning_delta"
        elif update_enabled and phi_norm < self.min_phi_norm:
            update_enabled = False
            reason = "protect_low_phi"
        elif update_enabled and abs(ex_ms) > self.max_abs_ex_ms:
            update_enabled = False
            reason = "protect_large_ex"

        k_r_dot = 0.0
        k_x_dot = 0.0

        if update_enabled:
            b_sign = 1.0 if b_hat_ms2_per_pwm >= 0.0 else -1.0

            k_r_dot = (
                -self.gamma_k_r * phi_ref * ex_ms * b_sign
                -self.sigma * (self.k_r_hat - self.k_r_nominal)
            )
            k_x_dot = (
                -self.gamma_k_x * phi_vx * ex_ms * b_sign
                -self.sigma * (self.k_x_hat - self.k_x_nominal)
            )

            k_r_dot = self._clamp(
                k_r_dot,
                -self.max_abs_theta_dot,
                self.max_abs_theta_dot,
            )
            k_x_dot = self._clamp(
                k_x_dot,
                -self.max_abs_theta_dot,
                self.max_abs_theta_dot,
            )

            self.k_r_hat = self._clamp(
                self.k_r_hat + dt_s * k_r_dot,
                self.min_k_r,
                self.max_k_r,
            )
            self.k_x_hat = self._clamp(
                self.k_x_hat + dt_s * k_x_dot,
                self.min_k_x,
                self.max_k_x,
            )

            pwm_adaptive_raw = self.k_r_hat * phi_ref + self.k_x_hat * phi_vx
            pwm_adaptive_sat = self._clamp(
                pwm_adaptive_raw,
                self.pwm_min,
                self.pwm_max,
            )

            reason = "ok"

        applied = bool(self.apply_to_speed_cmd and valid and reason == "ok")

        if applied:
            pwm_delta_full = pwm_adaptive_sat - pwm_baseline
            pwm_delta_blended = self.apply_blend_alpha * pwm_delta_full
            pwm_delta_blended = self._clamp(
                pwm_delta_blended,
                -self.max_applied_pwm_delta,
                self.max_applied_pwm_delta,
            )
            pwm_final = pwm_baseline + pwm_delta_blended
            pwm_final = self._clamp(pwm_final, self.pwm_min, self.pwm_max)
        else:
            pwm_final = pwm_baseline

        return self._make_output(
            valid=valid,
            adaptation_enabled=self.enable_adaptation,
            update_enabled=update_enabled,
            applied=applied,
            reason=reason,
            dt_s=dt_s,
            vx_ms=vx_ms,
            vx_m_ms=vx_m_ms,
            vx_ref_ms=vx_ref_ms,
            ex_ms=ex_ms,
            phi_ref=phi_ref,
            phi_vx=phi_vx,
            phi_norm=phi_norm,
            k_r_dot=k_r_dot,
            k_x_dot=k_x_dot,
            pwm_baseline=pwm_baseline,
            pwm_adaptive_raw=pwm_adaptive_raw,
            pwm_adaptive_sat=pwm_adaptive_sat,
            pwm_final=pwm_final,
            b_hat_ms2_per_pwm=b_hat_ms2_per_pwm,
        )
