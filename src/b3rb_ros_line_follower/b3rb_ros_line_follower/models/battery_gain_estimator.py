import math
from dataclasses import dataclass


@dataclass
class BatteryGainEstimate:
    valid: bool
    update_enabled: bool

    b_nominal_ms2_per_pwm: float
    b_voltage_ms2_per_pwm: float
    b_hat_ms2_per_pwm: float
    b_inst_ms2_per_pwm: float
    b_error_ms2_per_pwm: float

    alpha: float

    vx_ms: float
    pwm_cmd: float
    measured_vx_dot_ms2: float
    estimated_loss_ms2: float
    predicted_vx_dot_from_hat_ms2: float

    reason: str


class BatteryGainEstimator:
    """
    Task 5.1 scaffold.

    Estimates the slow battery-side longitudinal gain:

        vx_dot ≈ b_batt * pwm - k_v * vx

    Rearranged instantaneous estimate:

        b_inst ≈ (vx_dot_measured + k_v * vx) / pwm

    Then b_hat is updated slowly using a low-pass estimator.
    """

    def __init__(
        self,
        b_nominal_ms2_per_pwm: float,
        tau_s: float,
        min_pwm: float,
        min_vx_ms: float,
        max_abs_r_rad_s: float,
        max_abs_delta_rad: float,
        min_dt_s: float,
        max_dt_s: float,
        min_scale: float,
        max_scale: float,
    ):
        self.b_nominal = float(b_nominal_ms2_per_pwm)
        self.tau_s = max(1e-6, float(tau_s))

        self.min_pwm = abs(float(min_pwm))
        self.min_vx_ms = abs(float(min_vx_ms))
        self.max_abs_r_rad_s = abs(float(max_abs_r_rad_s))
        self.max_abs_delta_rad = abs(float(max_abs_delta_rad))

        self.min_dt_s = max(0.0, float(min_dt_s))
        self.max_dt_s = max(self.min_dt_s, float(max_dt_s))

        self.b_min = self.b_nominal * float(min_scale)
        self.b_max = self.b_nominal * float(max_scale)

        self.b_hat = self.b_nominal
        self.initialized = False

    def _project(self, value: float) -> float:
        return min(max(value, self.b_min), self.b_max)

    def update(
        self,
        vx_ms: float,
        pwm_cmd: float,
        measured_vx_dot_ms2: float,
        k_v_loss_s: float,
        b_voltage_ms2_per_pwm: float,
        r_rad_s: float,
        delta_f_rad: float,
        dt_s: float,
    ) -> BatteryGainEstimate:
        vx_ms = float(vx_ms)
        pwm_cmd = float(pwm_cmd)
        measured_vx_dot_ms2 = float(measured_vx_dot_ms2)
        k_v_loss_s = float(k_v_loss_s)
        b_voltage_ms2_per_pwm = float(b_voltage_ms2_per_pwm)
        r_rad_s = float(r_rad_s)
        delta_f_rad = float(delta_f_rad)
        dt_s = float(dt_s)

        if not self.initialized:
            if math.isfinite(b_voltage_ms2_per_pwm) and b_voltage_ms2_per_pwm > 0.0:
                self.b_hat = self._project(b_voltage_ms2_per_pwm)
            else:
                self.b_hat = self._project(self.b_nominal)
            self.initialized = True

        estimated_loss_ms2 = k_v_loss_s * vx_ms

        update_enabled = True
        reason = "ok"

        if dt_s < self.min_dt_s or dt_s > self.max_dt_s:
            update_enabled = False
            reason = "bad_dt"
        elif abs(pwm_cmd) < self.min_pwm:
            update_enabled = False
            reason = "low_pwm"
        elif abs(vx_ms) < self.min_vx_ms:
            update_enabled = False
            reason = "low_vx"
        elif abs(r_rad_s) > self.max_abs_r_rad_s:
            update_enabled = False
            reason = "turning_r"
        elif abs(delta_f_rad) > self.max_abs_delta_rad:
            update_enabled = False
            reason = "turning_delta"

        if abs(pwm_cmd) > 1e-9:
            b_inst = (measured_vx_dot_ms2 + estimated_loss_ms2) / pwm_cmd
        else:
            b_inst = self.b_hat

        if not math.isfinite(b_inst):
            update_enabled = False
            reason = "bad_b_inst"
            b_inst = self.b_hat

        b_inst_projected = self._project(b_inst)

        if update_enabled:
            alpha = min(1.0, max(0.0, dt_s / self.tau_s))
            self.b_hat = self.b_hat + alpha * (b_inst_projected - self.b_hat)
            self.b_hat = self._project(self.b_hat)
        else:
            alpha = 0.0

        b_error = b_inst_projected - self.b_hat

        predicted_vx_dot_from_hat_ms2 = self.b_hat * pwm_cmd - estimated_loss_ms2

        return BatteryGainEstimate(
            valid=True,
            update_enabled=update_enabled,
            b_nominal_ms2_per_pwm=self.b_nominal,
            b_voltage_ms2_per_pwm=b_voltage_ms2_per_pwm,
            b_hat_ms2_per_pwm=self.b_hat,
            b_inst_ms2_per_pwm=b_inst_projected,
            b_error_ms2_per_pwm=b_error,
            alpha=alpha,
            vx_ms=vx_ms,
            pwm_cmd=pwm_cmd,
            measured_vx_dot_ms2=measured_vx_dot_ms2,
            estimated_loss_ms2=estimated_loss_ms2,
            predicted_vx_dot_from_hat_ms2=predicted_vx_dot_from_hat_ms2,
            reason=reason,
        )
