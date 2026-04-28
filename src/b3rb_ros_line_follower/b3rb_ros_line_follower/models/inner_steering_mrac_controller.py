from dataclasses import dataclass
import math


@dataclass
class InnerSteeringMRACOutput:
    valid: bool
    adaptation_enabled: bool
    update_enabled: bool
    applied: bool
    reason: str

    dt_s: float

    vx_ms: float
    vy_ms: float
    r_rad_s: float
    uc_rad_s: float

    vy_m_ms: float
    r_m_rad_s: float
    e_vy_ms: float
    e_r_rad_s: float
    e_track: float

    phi_vy: float
    phi_r: float
    phi_uc: float
    phi_norm: float

    theta_vy_hat: float
    theta_r_hat: float
    theta_uc_hat: float

    theta_vy_dot: float
    theta_r_dot: float
    theta_uc_dot: float
    theta_norm: float

    delta_baseline_rad: float
    delta_raw_rad: float
    delta_sat_rad: float
    delta_final_rad: float

    turn_baseline_cmd: float
    turn_raw_cmd: float
    turn_sat_cmd: float
    turn_final_cmd: float


class InnerSteeringMRACController:
    """
    Task 6.2 inner-loop steering MRAC channel.

    Report control law:

        delta_f = theta_delta_hat^T phi

    with:

        phi = [-vy, -r, uc]

    This controller is safe by default:
      - it computes adaptive steering in shadow mode
      - it logs the result
      - it does not affect the vehicle unless apply_to_steering_cmd=True
    """

    def __init__(
        self,
        theta_vy_initial: float,
        theta_r_initial: float,
        theta_uc_initial: float,
        gamma_vy: float,
        gamma_r: float,
        gamma_uc: float,
        sigma: float,
        theta_min: float,
        theta_max: float,
        max_abs_theta_dot: float,
        min_vx_ms: float,
        min_phi_norm: float,
        max_abs_tracking_error: float,
        max_dt_s: float,
        max_delta_rad: float,
        max_applied_delta_rad: float,
        max_delta_disagreement_rad: float,
        steering_rad_per_turn_cmd: float,
        max_turn_cmd: float,
        error_vy_weight: float,
        enable_adaptation: bool,
        apply_to_steering_cmd: bool,
        blend: float,
    ):
        self.theta_vy_initial = float(theta_vy_initial)
        self.theta_r_initial = float(theta_r_initial)
        self.theta_uc_initial = float(theta_uc_initial)

        self.theta_vy_hat = float(theta_vy_initial)
        self.theta_r_hat = float(theta_r_initial)
        self.theta_uc_hat = float(theta_uc_initial)

        self.gamma_vy = float(gamma_vy)
        self.gamma_r = float(gamma_r)
        self.gamma_uc = float(gamma_uc)
        self.sigma = float(sigma)

        self.theta_min = float(theta_min)
        self.theta_max = float(theta_max)
        self.max_abs_theta_dot = float(max_abs_theta_dot)

        self.min_vx_ms = float(min_vx_ms)
        self.min_phi_norm = float(min_phi_norm)
        self.max_abs_tracking_error = float(max_abs_tracking_error)
        self.max_dt_s = float(max_dt_s)

        self.max_delta_rad = abs(float(max_delta_rad))
        self.max_applied_delta_rad = abs(float(max_applied_delta_rad))
        self.max_delta_disagreement_rad = abs(float(max_delta_disagreement_rad))

        self.steering_rad_per_turn_cmd = max(
            abs(float(steering_rad_per_turn_cmd)),
            1.0e-6,
        )
        self.max_turn_cmd = abs(float(max_turn_cmd))

        self.error_vy_weight = float(error_vy_weight)

        self.enable_adaptation = bool(enable_adaptation)
        self.apply_to_steering_cmd = bool(apply_to_steering_cmd)
        self.blend = max(0.0, min(1.0, float(blend)))

    @staticmethod
    def _clamp(value: float, lower: float, upper: float) -> float:
        return max(lower, min(upper, value))

    @staticmethod
    def _get_float(obj, names, default: float = 0.0) -> float:
        if obj is None:
            return float(default)

        for name in names:
            if hasattr(obj, name):
                return float(getattr(obj, name))

        return float(default)

    @staticmethod
    def _get_bool(obj, names, default: bool = False) -> bool:
        if obj is None:
            return bool(default)

        for name in names:
            if hasattr(obj, name):
                return bool(getattr(obj, name))

        return bool(default)

    def _delta_to_turn_cmd(self, delta_rad: float) -> float:
        return self._clamp(
            float(delta_rad) / self.steering_rad_per_turn_cmd,
            -self.max_turn_cmd,
            self.max_turn_cmd,
        )

    def _make_output(
        self,
        valid: bool,
        update_enabled: bool,
        applied: bool,
        reason: str,
        dt_s: float,
        vx_ms: float,
        vy_ms: float,
        r_rad_s: float,
        uc_rad_s: float,
        vy_m_ms: float,
        r_m_rad_s: float,
        e_vy_ms: float,
        e_r_rad_s: float,
        e_track: float,
        phi_vy: float,
        phi_r: float,
        phi_uc: float,
        phi_norm: float,
        theta_vy_dot: float,
        theta_r_dot: float,
        theta_uc_dot: float,
        delta_baseline_rad: float,
        delta_raw_rad: float,
        delta_sat_rad: float,
        delta_final_rad: float,
        turn_baseline_cmd: float,
    ) -> InnerSteeringMRACOutput:
        theta_norm = math.sqrt(
            self.theta_vy_hat * self.theta_vy_hat
            + self.theta_r_hat * self.theta_r_hat
            + self.theta_uc_hat * self.theta_uc_hat
        )

        return InnerSteeringMRACOutput(
            valid=valid,
            adaptation_enabled=self.enable_adaptation,
            update_enabled=update_enabled,
            applied=applied,
            reason=reason,
            dt_s=dt_s,
            vx_ms=vx_ms,
            vy_ms=vy_ms,
            r_rad_s=r_rad_s,
            uc_rad_s=uc_rad_s,
            vy_m_ms=vy_m_ms,
            r_m_rad_s=r_m_rad_s,
            e_vy_ms=e_vy_ms,
            e_r_rad_s=e_r_rad_s,
            e_track=e_track,
            phi_vy=phi_vy,
            phi_r=phi_r,
            phi_uc=phi_uc,
            phi_norm=phi_norm,
            theta_vy_hat=self.theta_vy_hat,
            theta_r_hat=self.theta_r_hat,
            theta_uc_hat=self.theta_uc_hat,
            theta_vy_dot=theta_vy_dot,
            theta_r_dot=theta_r_dot,
            theta_uc_dot=theta_uc_dot,
            theta_norm=theta_norm,
            delta_baseline_rad=delta_baseline_rad,
            delta_raw_rad=delta_raw_rad,
            delta_sat_rad=delta_sat_rad,
            delta_final_rad=delta_final_rad,
            turn_baseline_cmd=turn_baseline_cmd,
            turn_raw_cmd=self._delta_to_turn_cmd(delta_raw_rad),
            turn_sat_cmd=self._delta_to_turn_cmd(delta_sat_rad),
            turn_final_cmd=self._delta_to_turn_cmd(delta_final_rad),
        )

    def update(
        self,
        vx_ms: float,
        vy_ms: float,
        r_rad_s: float,
        reference_command,
        reference_model,
        baseline_delta_rad: float,
        baseline_turn_cmd: float,
        dt_s: float,
    ) -> InnerSteeringMRACOutput:
        vx_ms = float(vx_ms)
        vy_ms = float(vy_ms)
        r_rad_s = float(r_rad_s)
        baseline_delta_rad = float(baseline_delta_rad)
        baseline_turn_cmd = float(baseline_turn_cmd)
        dt_s = float(dt_s)

        uc_rad_s = self._get_float(
            reference_command,
            ["uc_rad_s", "u_c_rad_s", "uc"],
            0.0,
        )

        command_valid = self._get_bool(
            reference_command,
            ["valid", "command_valid"],
            False,
        )

        reference_valid = self._get_bool(
            reference_model,
            ["valid", "reference_valid"],
            True,
        )

        vy_m_ms = self._get_float(
            reference_model,
            ["vy_m_ms", "v_y_m_ms", "vy_ref_ms"],
            0.0,
        )

        r_m_rad_s = self._get_float(
            reference_model,
            ["r_m_rad_s", "r_ref_rad_s"],
            0.0,
        )

        e_vy_ms = vy_ms - vy_m_ms
        e_r_rad_s = r_rad_s - r_m_rad_s
        e_track = e_r_rad_s + self.error_vy_weight * e_vy_ms

        phi_vy = -vy_ms
        phi_r = -r_rad_s
        phi_uc = uc_rad_s
        phi_norm = math.sqrt(
            phi_vy * phi_vy
            + phi_r * phi_r
            + phi_uc * phi_uc
        )

        valid = True
        reason = "ok"

        if reference_command is None:
            valid = False
            reason = "no_inner_reference_command"
        elif not command_valid:
            valid = False
            reason = "inner_reference_command_invalid"
        elif reference_model is None:
            valid = False
            reason = "no_inner_reference_model"
        elif not reference_valid:
            valid = False
            reason = "inner_reference_model_invalid"
        elif dt_s <= 0.0 or dt_s > self.max_dt_s:
            valid = False
            reason = "bad_dt"
        elif abs(vx_ms) < self.min_vx_ms:
            valid = False
            reason = "protect_low_vx"

        update_enabled = False
        theta_vy_dot = 0.0
        theta_r_dot = 0.0
        theta_uc_dot = 0.0

        if valid:
            if not self.enable_adaptation:
                reason = "adaptation_disabled"
            elif phi_norm < self.min_phi_norm:
                reason = "low_excitation"
            elif abs(e_track) > self.max_abs_tracking_error:
                reason = "protect_large_tracking_error"
            else:
                update_enabled = True
                reason = "ok"

        if update_enabled:
            theta_vy_dot = (
                -self.gamma_vy * phi_vy * e_track
                - self.sigma * (self.theta_vy_hat - self.theta_vy_initial)
            )
            theta_r_dot = (
                -self.gamma_r * phi_r * e_track
                - self.sigma * (self.theta_r_hat - self.theta_r_initial)
            )
            theta_uc_dot = (
                -self.gamma_uc * phi_uc * e_track
                - self.sigma * (self.theta_uc_hat - self.theta_uc_initial)
            )

            theta_vy_dot = self._clamp(
                theta_vy_dot,
                -self.max_abs_theta_dot,
                self.max_abs_theta_dot,
            )
            theta_r_dot = self._clamp(
                theta_r_dot,
                -self.max_abs_theta_dot,
                self.max_abs_theta_dot,
            )
            theta_uc_dot = self._clamp(
                theta_uc_dot,
                -self.max_abs_theta_dot,
                self.max_abs_theta_dot,
            )

            self.theta_vy_hat = self._clamp(
                self.theta_vy_hat + theta_vy_dot * dt_s,
                self.theta_min,
                self.theta_max,
            )
            self.theta_r_hat = self._clamp(
                self.theta_r_hat + theta_r_dot * dt_s,
                self.theta_min,
                self.theta_max,
            )
            self.theta_uc_hat = self._clamp(
                self.theta_uc_hat + theta_uc_dot * dt_s,
                self.theta_min,
                self.theta_max,
            )

        delta_raw_rad = (
            self.theta_vy_hat * phi_vy
            + self.theta_r_hat * phi_r
            + self.theta_uc_hat * phi_uc
        )

        delta_sat_rad = self._clamp(
            delta_raw_rad,
            -self.max_delta_rad,
            self.max_delta_rad,
        )

        delta_disagreement_rad = abs(delta_sat_rad - baseline_delta_rad)

        can_apply = (
            valid
            and delta_disagreement_rad <= self.max_delta_disagreement_rad
        )

        applied = bool(self.apply_to_steering_cmd and can_apply)

        if self.apply_to_steering_cmd and valid and not can_apply:
            reason = "protect_delta_disagreement"

        if applied:
            blended_delta_rad = (
                baseline_delta_rad
                + self.blend * (delta_sat_rad - baseline_delta_rad)
            )

            delta_final_rad = self._clamp(
                blended_delta_rad,
                baseline_delta_rad - self.max_applied_delta_rad,
                baseline_delta_rad + self.max_applied_delta_rad,
            )

            delta_final_rad = self._clamp(
                delta_final_rad,
                -self.max_delta_rad,
                self.max_delta_rad,
            )
        else:
            delta_final_rad = baseline_delta_rad

        return self._make_output(
            valid=valid,
            update_enabled=update_enabled,
            applied=applied,
            reason=reason,
            dt_s=dt_s,
            vx_ms=vx_ms,
            vy_ms=vy_ms,
            r_rad_s=r_rad_s,
            uc_rad_s=uc_rad_s,
            vy_m_ms=vy_m_ms,
            r_m_rad_s=r_m_rad_s,
            e_vy_ms=e_vy_ms,
            e_r_rad_s=e_r_rad_s,
            e_track=e_track,
            phi_vy=phi_vy,
            phi_r=phi_r,
            phi_uc=phi_uc,
            phi_norm=phi_norm,
            theta_vy_dot=theta_vy_dot,
            theta_r_dot=theta_r_dot,
            theta_uc_dot=theta_uc_dot,
            delta_baseline_rad=baseline_delta_rad,
            delta_raw_rad=delta_raw_rad,
            delta_sat_rad=delta_sat_rad,
            delta_final_rad=delta_final_rad,
            turn_baseline_cmd=baseline_turn_cmd,
        )
