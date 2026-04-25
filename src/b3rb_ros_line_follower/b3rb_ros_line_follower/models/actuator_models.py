from ..mrac_types import PropulsionActuatorState, SteeringActuatorState


class SteeringActuatorModel:
    """
    First-order steering servo model.

        delta_f_dot = (1 / tau_s) * (delta_ref - delta_f)

    This converts the commanded steering angle delta_ref into a smoother
    estimated actual front steering angle delta_f_est.
    """

    def __init__(self, tau_s: float, max_abs_delta_rad: float):
        self.tau_s = float(tau_s)
        self.max_abs_delta_rad = float(max_abs_delta_rad)
        self.delta_f_est_rad = 0.0

    @staticmethod
    def clamp(value: float, min_value: float, max_value: float) -> float:
        return max(min_value, min(max_value, value))

    def step(self, delta_ref_rad: float, dt_s: float) -> SteeringActuatorState:
        safe_dt_s = max(0.0, float(dt_s))

        delta_ref_rad = self.clamp(
            float(delta_ref_rad),
            -self.max_abs_delta_rad,
            self.max_abs_delta_rad,
        )

        if self.tau_s <= 1e-9:
            self.delta_f_est_rad = delta_ref_rad
        else:
            alpha = self.clamp(safe_dt_s / self.tau_s, 0.0, 1.0)
            self.delta_f_est_rad += alpha * (
                delta_ref_rad - self.delta_f_est_rad
            )

        self.delta_f_est_rad = self.clamp(
            self.delta_f_est_rad,
            -self.max_abs_delta_rad,
            self.max_abs_delta_rad,
        )

        return SteeringActuatorState(
            delta_ref_rad=delta_ref_rad,
            delta_f_est_rad=self.delta_f_est_rad,
            delta_error_rad=delta_ref_rad - self.delta_f_est_rad,
            tau_s=self.tau_s,
        )


class PropulsionForceModel:
    """
    Simplified first-order BLDC / ESC longitudinal force model.

        F_long_dot = (1 / tau_e) * (F_target - F_long)

        F_target = gain * eta * (V_batt - V_sag) * pwm_cmd

    This is intentionally simple for Task 2.6. It gives us:
    - a PWM command
    - a battery voltage dependency
    - a non-instantaneous tractive-force response
    """

    def __init__(
        self,
        tau_s: float,
        efficiency: float,
        force_gain_n_per_v: float,
        voltage_sag_at_full_pwm_v: float,
        max_abs_force_n: float,
    ):
        self.tau_s = float(tau_s)
        self.efficiency = float(efficiency)
        self.force_gain_n_per_v = float(force_gain_n_per_v)
        self.voltage_sag_at_full_pwm_v = float(voltage_sag_at_full_pwm_v)
        self.max_abs_force_n = float(max_abs_force_n)

        self.force_longitudinal_est_n = 0.0

    @staticmethod
    def clamp(value: float, min_value: float, max_value: float) -> float:
        return max(min_value, min(max_value, value))

    def step(
        self,
        pwm_cmd: float,
        battery_voltage_v: float,
        dt_s: float,
    ) -> PropulsionActuatorState:
        safe_dt_s = max(0.0, float(dt_s))

        pwm_cmd = self.clamp(float(pwm_cmd), -1.0, 1.0)
        battery_voltage_v = max(0.0, float(battery_voltage_v))

        voltage_sag_v = self.voltage_sag_at_full_pwm_v * abs(pwm_cmd)
        effective_voltage_v = max(0.0, battery_voltage_v - voltage_sag_v)

        target_force_n = (
            self.force_gain_n_per_v
            * self.efficiency
            * effective_voltage_v
            * pwm_cmd
        )

        target_force_n = self.clamp(
            target_force_n,
            -self.max_abs_force_n,
            self.max_abs_force_n,
        )

        if self.tau_s <= 1e-9:
            self.force_longitudinal_est_n = target_force_n
        else:
            alpha = self.clamp(safe_dt_s / self.tau_s, 0.0, 1.0)
            self.force_longitudinal_est_n += alpha * (
                target_force_n - self.force_longitudinal_est_n
            )

        self.force_longitudinal_est_n = self.clamp(
            self.force_longitudinal_est_n,
            -self.max_abs_force_n,
            self.max_abs_force_n,
        )

        return PropulsionActuatorState(
            pwm_cmd=pwm_cmd,
            battery_voltage_v=battery_voltage_v,
            voltage_sag_v=voltage_sag_v,
            target_force_n=target_force_n,
            force_longitudinal_est_n=self.force_longitudinal_est_n,
            tau_s=self.tau_s,
            force_gain_n_per_v=self.force_gain_n_per_v,
        )
