from dataclasses import dataclass


@dataclass
class WheelActuationCommand:
    valid: bool
    reason: str

    f_long_cmd_n: float
    dfx_cmd_n: float

    fx_left_n: float
    fx_right_n: float

    reconstructed_f_long_n: float
    reconstructed_dfx_n: float

    f_long_recomposition_error_n: float
    dfx_recomposition_error_n: float

    pwm_left_cmd: float
    pwm_right_cmd: float
    pwm_common_cmd: float
    pwm_diff_cmd: float

    battery_voltage_v: float
    force_saturated: bool
    dfx_saturated: bool

    outer_pwm_cmd: float
    outer_pwm_source: str
    dfx_source: str
    actuation_mode: str


class WheelForceAllocator:
    """
    Phase 7 wheel-force allocator.

    Converts:
        F_long, Delta_Fx

    Into:
        F_x,L = (F_long - Delta_Fx) / 2
        F_x,R = (F_long + Delta_Fx) / 2

    Then converts the allocated left/right forces into equivalent
    left/right PWM estimates. Since the current ROS interface only
    accepts common throttle + steering, the final sent throttle is the
    recomposed common PWM from the wheel allocation.
    """

    def __init__(
        self,
        rear_track_width_m: float,
        propulsion_efficiency: float,
        total_force_gain_n_per_v: float,
        voltage_sag_at_full_pwm_v: float,
        max_abs_total_force_n: float,
        max_abs_wheel_force_n: float,
        max_abs_dfx_n: float,
        pwm_min: float,
        pwm_max: float,
    ):
        self.rear_track_width_m = float(rear_track_width_m)
        self.propulsion_efficiency = float(propulsion_efficiency)
        self.total_force_gain_n_per_v = float(total_force_gain_n_per_v)
        self.wheel_force_gain_n_per_v = 0.5 * self.total_force_gain_n_per_v
        self.voltage_sag_at_full_pwm_v = float(voltage_sag_at_full_pwm_v)
        self.max_abs_total_force_n = abs(float(max_abs_total_force_n))
        self.max_abs_wheel_force_n = abs(float(max_abs_wheel_force_n))
        self.max_abs_dfx_n = abs(float(max_abs_dfx_n))
        self.pwm_min = float(pwm_min)
        self.pwm_max = float(pwm_max)

    @staticmethod
    def _clamp(value: float, lower: float, upper: float) -> float:
        return max(lower, min(upper, value))

    def total_force_from_pwm(self, pwm_cmd: float, battery_voltage_v: float) -> float:
        pwm_cmd = self._clamp(float(pwm_cmd), -1.0, 1.0)
        battery_voltage_v = max(0.0, float(battery_voltage_v))

        voltage_sag_v = self.voltage_sag_at_full_pwm_v * abs(pwm_cmd)
        effective_voltage_v = max(0.0, battery_voltage_v - voltage_sag_v)

        force_n = (
            self.total_force_gain_n_per_v
            * self.propulsion_efficiency
            * effective_voltage_v
            * pwm_cmd
        )

        return self._clamp(
            force_n,
            -self.max_abs_total_force_n,
            self.max_abs_total_force_n,
        )

    def _wheel_force_from_pwm(self, pwm_cmd: float, battery_voltage_v: float) -> float:
        pwm_cmd = self._clamp(float(pwm_cmd), -1.0, 1.0)
        battery_voltage_v = max(0.0, float(battery_voltage_v))

        voltage_sag_v = self.voltage_sag_at_full_pwm_v * abs(pwm_cmd)
        effective_voltage_v = max(0.0, battery_voltage_v - voltage_sag_v)

        force_n = (
            self.wheel_force_gain_n_per_v
            * self.propulsion_efficiency
            * effective_voltage_v
            * pwm_cmd
        )

        return self._clamp(
            force_n,
            -self.max_abs_wheel_force_n,
            self.max_abs_wheel_force_n,
        )

    def _wheel_pwm_from_force(self, force_n: float, battery_voltage_v: float) -> float:
        force_n = self._clamp(
            float(force_n),
            -self.max_abs_wheel_force_n,
            self.max_abs_wheel_force_n,
        )

        if abs(force_n) < 1.0e-9:
            return 0.0

        sign = 1.0 if force_n >= 0.0 else -1.0
        target_force_n = abs(force_n)

        max_force_n = abs(self._wheel_force_from_pwm(sign * 1.0, battery_voltage_v))

        if max_force_n <= 1.0e-9:
            return 0.0

        if target_force_n >= max_force_n:
            return self._clamp(sign * 1.0, self.pwm_min, self.pwm_max)

        lo = 0.0
        hi = 1.0

        for _ in range(40):
            mid = 0.5 * (lo + hi)
            mid_force_n = abs(self._wheel_force_from_pwm(sign * mid, battery_voltage_v))

            if mid_force_n < target_force_n:
                lo = mid
            else:
                hi = mid

        return self._clamp(sign * 0.5 * (lo + hi), self.pwm_min, self.pwm_max)

    def allocate(
        self,
        f_long_cmd_n: float,
        dfx_cmd_n: float,
        battery_voltage_v: float,
        outer_pwm_cmd: float,
        outer_pwm_source: str,
        dfx_source: str,
    ) -> WheelActuationCommand:
        raw_f_long_cmd_n = float(f_long_cmd_n)
        raw_dfx_cmd_n = float(dfx_cmd_n)

        f_long_cmd_n = self._clamp(
            raw_f_long_cmd_n,
            -self.max_abs_total_force_n,
            self.max_abs_total_force_n,
        )

        dfx_cmd_n = self._clamp(
            raw_dfx_cmd_n,
            -self.max_abs_dfx_n,
            self.max_abs_dfx_n,
        )

        force_saturated = abs(f_long_cmd_n - raw_f_long_cmd_n) > 1.0e-9
        dfx_saturated = abs(dfx_cmd_n - raw_dfx_cmd_n) > 1.0e-9

        fx_left_n = 0.5 * (f_long_cmd_n - dfx_cmd_n)
        fx_right_n = 0.5 * (f_long_cmd_n + dfx_cmd_n)

        fx_left_n_sat = self._clamp(
            fx_left_n,
            -self.max_abs_wheel_force_n,
            self.max_abs_wheel_force_n,
        )

        fx_right_n_sat = self._clamp(
            fx_right_n,
            -self.max_abs_wheel_force_n,
            self.max_abs_wheel_force_n,
        )

        if abs(fx_left_n_sat - fx_left_n) > 1.0e-9:
            force_saturated = True

        if abs(fx_right_n_sat - fx_right_n) > 1.0e-9:
            force_saturated = True

        fx_left_n = fx_left_n_sat
        fx_right_n = fx_right_n_sat

        reconstructed_f_long_n = fx_left_n + fx_right_n
        reconstructed_dfx_n = fx_right_n - fx_left_n

        f_long_recomposition_error_n = reconstructed_f_long_n - f_long_cmd_n
        dfx_recomposition_error_n = reconstructed_dfx_n - dfx_cmd_n

        pwm_left_cmd = self._wheel_pwm_from_force(fx_left_n, battery_voltage_v)
        pwm_right_cmd = self._wheel_pwm_from_force(fx_right_n, battery_voltage_v)

        pwm_common_cmd = 0.5 * (pwm_left_cmd + pwm_right_cmd)
        pwm_diff_cmd = pwm_right_cmd - pwm_left_cmd

        pwm_common_cmd = self._clamp(pwm_common_cmd, self.pwm_min, self.pwm_max)

        return WheelActuationCommand(
            valid=True,
            reason="ok",
            f_long_cmd_n=f_long_cmd_n,
            dfx_cmd_n=dfx_cmd_n,
            fx_left_n=fx_left_n,
            fx_right_n=fx_right_n,
            reconstructed_f_long_n=reconstructed_f_long_n,
            reconstructed_dfx_n=reconstructed_dfx_n,
            f_long_recomposition_error_n=f_long_recomposition_error_n,
            dfx_recomposition_error_n=dfx_recomposition_error_n,
            pwm_left_cmd=pwm_left_cmd,
            pwm_right_cmd=pwm_right_cmd,
            pwm_common_cmd=pwm_common_cmd,
            pwm_diff_cmd=pwm_diff_cmd,
            battery_voltage_v=float(battery_voltage_v),
            force_saturated=force_saturated,
            dfx_saturated=dfx_saturated,
            outer_pwm_cmd=float(outer_pwm_cmd),
            outer_pwm_source=str(outer_pwm_source),
            dfx_source=str(dfx_source),
            actuation_mode="phase7_wheel_force_allocation",
        )
