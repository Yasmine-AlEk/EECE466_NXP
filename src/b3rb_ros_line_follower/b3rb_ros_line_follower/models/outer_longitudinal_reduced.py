from dataclasses import dataclass


@dataclass
class OuterLongitudinalReducedOutput:
    vx_ms: float
    pwm_cmd: float
    battery_voltage_v: float
    voltage_sag_v: float
    effective_voltage_v: float

    b_batt_est: float

    # Gross propulsion-only acceleration prediction.
    vx_dot_propulsion_ms2: float

    # Simple speed-dependent resistance acceleration.
    vx_loss_ms2: float

    # Net reduced-model acceleration prediction.
    vx_dot_pred_ms2: float

    # Residual/disturbance estimate:
    # measured_vx_dot - predicted_vx_dot.
    d_x_est_ms2: float

    k_v_loss_s: float
    valid: bool


class OuterLongitudinalReducedModel:
    """
    Task 3.1 outer longitudinal reduced model.

    Original simplified scaffold:
        vx_dot = b_batt * PWM + d_x

    Refined validation scaffold:
        vx_dot = b_batt * PWM - k_v * vx + d_x

    The -k_v * vx term is not the final physics model. It is a simple
    resistance/load term so steady cruising does not look like constant
    positive acceleration.
    """

    def __init__(
        self,
        mass_kg: float,
        force_gain_n_per_v: float,
        efficiency: float,
        d_x_limit_ms2: float,
        k_v_loss_s: float = 0.0,
    ):
        self.mass_kg = max(float(mass_kg), 1e-6)
        self.force_gain_n_per_v = float(force_gain_n_per_v)
        self.efficiency = float(efficiency)
        self.d_x_limit_ms2 = abs(float(d_x_limit_ms2))
        self.k_v_loss_s = max(0.0, float(k_v_loss_s))

    def _clamp(self, value: float, limit: float) -> float:
        if limit <= 0.0:
            return value
        return max(-limit, min(limit, value))

    def build(
        self,
        vx_ms: float,
        pwm_cmd: float,
        battery_voltage_v: float,
        voltage_sag_v: float,
        measured_vx_dot_ms2: float,
    ) -> OuterLongitudinalReducedOutput:
        vx_ms = float(vx_ms)
        pwm_cmd = float(pwm_cmd)
        battery_voltage_v = float(battery_voltage_v)
        voltage_sag_v = max(0.0, float(voltage_sag_v))
        measured_vx_dot_ms2 = float(measured_vx_dot_ms2)

        effective_voltage_v = max(0.0, battery_voltage_v - voltage_sag_v)

        b_batt_est = (
            self.efficiency
            * self.force_gain_n_per_v
            * effective_voltage_v
            / self.mass_kg
        )

        vx_dot_propulsion_ms2 = b_batt_est * pwm_cmd
        vx_loss_ms2 = self.k_v_loss_s * vx_ms

        vx_dot_pred_ms2 = vx_dot_propulsion_ms2 - vx_loss_ms2

        d_x_raw_ms2 = measured_vx_dot_ms2 - vx_dot_pred_ms2
        d_x_est_ms2 = self._clamp(d_x_raw_ms2, self.d_x_limit_ms2)

        return OuterLongitudinalReducedOutput(
            vx_ms=vx_ms,
            pwm_cmd=pwm_cmd,
            battery_voltage_v=battery_voltage_v,
            voltage_sag_v=voltage_sag_v,
            effective_voltage_v=effective_voltage_v,
            b_batt_est=b_batt_est,
            vx_dot_propulsion_ms2=vx_dot_propulsion_ms2,
            vx_loss_ms2=vx_loss_ms2,
            vx_dot_pred_ms2=vx_dot_pred_ms2,
            d_x_est_ms2=d_x_est_ms2,
            k_v_loss_s=self.k_v_loss_s,
            valid=True,
        )
