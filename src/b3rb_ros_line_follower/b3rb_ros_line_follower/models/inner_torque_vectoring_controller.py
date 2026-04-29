from dataclasses import dataclass
import math


@dataclass
class InnerTorqueVectoringOutput:
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

    q_vy: float
    q_r: float
    p_vy: float
    p_r: float

    b_tv_vy: float
    b_tv_r: float
    s_tv: float

    theta_vy_hat: float
    theta_r_hat: float
    theta_uc_hat: float

    theta_vy_dot: float
    theta_r_dot: float
    theta_uc_dot: float
    theta_norm: float

    dfx_raw_n: float
    dfx_sat_n: float
    dfx_final_n: float
    yaw_moment_final_nm: float

    c_alpha_f_used: float
    c_alpha_r_used: float


class InnerTorqueVectoringController:
    """
    Task 6.3 + Task 6.4 inner-loop torque-vectoring channel.

    Report law:
        Delta_Fx = theta_TV_hat^T phi

    Adaptation law:
        theta_TV_hat_dot =
            -Gamma_TV * phi * s_TV
            - sigma * (theta_TV_hat - theta_TV_0)

    where:
        s_TV = B_TV^T P e
        A_m^T P + P A_m = -Q

    Current implementation:
        - computes Delta_Fx
        - sends Delta_Fx to model-side wheel-force allocation
        - does not command real independent wheels
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
        max_abs_dfx_n: float,
        rear_track_width_m: float,
        iz_kg_m2: float,
        q_vy: float,
        q_r: float,
        reference_a_vy_s_inv: float,
        reference_a_r_s_inv: float,
        enable_adaptation: bool,
        apply_to_model: bool,
        nominal_c_alpha_f: float,
        nominal_c_alpha_r: float,
    ):
        self.theta_vy_initial = float(theta_vy_initial)
        self.theta_r_initial = float(theta_r_initial)
        self.theta_uc_initial = float(theta_uc_initial)

        self.theta_vy_hat = float(theta_vy_initial)
        self.theta_r_hat = float(theta_r_initial)
        self.theta_uc_hat = float(theta_uc_initial)

        self.gamma_vy = max(0.0, float(gamma_vy))
        self.gamma_r = max(0.0, float(gamma_r))
        self.gamma_uc = max(0.0, float(gamma_uc))
        self.Gamma_TV = [self.gamma_vy, self.gamma_r, self.gamma_uc]

        self.sigma = max(0.0, float(sigma))

        self.theta_min = float(theta_min)
        self.theta_max = float(theta_max)
        self.max_abs_theta_dot = abs(float(max_abs_theta_dot))

        self.min_vx_ms = max(0.0, float(min_vx_ms))
        self.min_phi_norm = max(0.0, float(min_phi_norm))
        self.max_abs_tracking_error = abs(float(max_abs_tracking_error))
        self.max_dt_s = max(1.0e-9, float(max_dt_s))

        self.max_abs_dfx_n = abs(float(max_abs_dfx_n))

        self.rear_track_width_m = float(rear_track_width_m)
        self.iz_kg_m2 = max(1.0e-9, float(iz_kg_m2))

        self.b_tv_vy_default = 0.0
        self.b_tv_r_default = (0.5 * self.rear_track_width_m) / self.iz_kg_m2

        self.q_vy = max(1.0e-9, float(q_vy))
        self.q_r = max(1.0e-9, float(q_r))

        self.a_vy = max(1.0e-9, float(reference_a_vy_s_inv))
        self.a_r = max(1.0e-9, float(reference_a_r_s_inv))

        self.p_vy = self.q_vy / (2.0 * self.a_vy)
        self.p_r = self.q_r / (2.0 * self.a_r)

        self.Q_TV = [
            [self.q_vy, 0.0],
            [0.0, self.q_r],
        ]
        self.P_TV = [
            [self.p_vy, 0.0],
            [0.0, self.p_r],
        ]

        self.enable_adaptation = bool(enable_adaptation)
        self.apply_to_model = bool(apply_to_model)

        self.nominal_c_alpha_f = max(1.0e-9, float(nominal_c_alpha_f))
        self.nominal_c_alpha_r = max(1.0e-9, float(nominal_c_alpha_r))

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

    @staticmethod
    def _get_list_value(obj, name: str, index: int, default: float) -> float:
        if obj is None or not hasattr(obj, name):
            return float(default)
        values = getattr(obj, name)
        try:
            return float(values[index])
        except Exception:
            return float(default)

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
        b_tv_vy: float,
        b_tv_r: float,
        s_tv: float,
        theta_vy_dot: float,
        theta_r_dot: float,
        theta_uc_dot: float,
        dfx_raw_n: float,
        dfx_sat_n: float,
        dfx_final_n: float,
        c_alpha_f_used: float,
        c_alpha_r_used: float,
    ) -> InnerTorqueVectoringOutput:
        theta_norm = math.sqrt(
            self.theta_vy_hat * self.theta_vy_hat
            + self.theta_r_hat * self.theta_r_hat
            + self.theta_uc_hat * self.theta_uc_hat
        )

        yaw_moment_final_nm = 0.5 * self.rear_track_width_m * dfx_final_n

        return InnerTorqueVectoringOutput(
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
            q_vy=self.q_vy,
            q_r=self.q_r,
            p_vy=self.p_vy,
            p_r=self.p_r,
            b_tv_vy=b_tv_vy,
            b_tv_r=b_tv_r,
            s_tv=s_tv,
            theta_vy_hat=self.theta_vy_hat,
            theta_r_hat=self.theta_r_hat,
            theta_uc_hat=self.theta_uc_hat,
            theta_vy_dot=theta_vy_dot,
            theta_r_dot=theta_r_dot,
            theta_uc_dot=theta_uc_dot,
            theta_norm=theta_norm,
            dfx_raw_n=dfx_raw_n,
            dfx_sat_n=dfx_sat_n,
            dfx_final_n=dfx_final_n,
            yaw_moment_final_nm=yaw_moment_final_nm,
            c_alpha_f_used=c_alpha_f_used,
            c_alpha_r_used=c_alpha_r_used,
        )

    def update(
        self,
        vx_ms: float,
        vy_ms: float,
        r_rad_s: float,
        reference_command,
        reference_model,
        inner_lateral_yaw,
        cornering_stiffness_used,
        baseline_dfx_n: float,
        dt_s: float,
    ) -> InnerTorqueVectoringOutput:
        vx_ms = float(vx_ms)
        vy_ms = float(vy_ms)
        r_rad_s = float(r_rad_s)
        baseline_dfx_n = float(baseline_dfx_n)
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
        e_track = e_r_rad_s + 0.5 * e_vy_ms

        phi_vy = -vy_ms
        phi_r = -r_rad_s
        phi_uc = uc_rad_s
        phi_norm = math.sqrt(
            phi_vy * phi_vy
            + phi_r * phi_r
            + phi_uc * phi_uc
        )

        b_tv_vy = self._get_list_value(
            inner_lateral_yaw,
            "B_TV",
            0,
            self.b_tv_vy_default,
        )
        b_tv_r = self._get_list_value(
            inner_lateral_yaw,
            "B_TV",
            1,
            self.b_tv_r_default,
        )

        # Lyapunov scalar for the TV channel:
        # s_TV = B_TV^T P e
        s_tv = b_tv_vy * self.p_vy * e_vy_ms + b_tv_r * self.p_r * e_r_rad_s

        c_alpha_f_used = self._get_float(
            cornering_stiffness_used,
            ["c_alpha_f_used", "c_alpha_f_n_per_rad", "c_alpha_f_hat"],
            self.nominal_c_alpha_f,
        )
        c_alpha_r_used = self._get_float(
            cornering_stiffness_used,
            ["c_alpha_r_used", "c_alpha_r_n_per_rad", "c_alpha_r_hat"],
            self.nominal_c_alpha_r,
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
        elif inner_lateral_yaw is None:
            valid = False
            reason = "no_inner_lateral_yaw_model"
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
                -self.gamma_vy * phi_vy * s_tv
                - self.sigma * (self.theta_vy_hat - self.theta_vy_initial)
            )
            theta_r_dot = (
                -self.gamma_r * phi_r * s_tv
                - self.sigma * (self.theta_r_hat - self.theta_r_initial)
            )
            theta_uc_dot = (
                -self.gamma_uc * phi_uc * s_tv
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

        dfx_raw_n = (
            self.theta_vy_hat * phi_vy
            + self.theta_r_hat * phi_r
            + self.theta_uc_hat * phi_uc
        )

        dfx_sat_n = self._clamp(
            dfx_raw_n,
            -self.max_abs_dfx_n,
            self.max_abs_dfx_n,
        )

        applied = bool(self.apply_to_model and valid)

        if applied:
            dfx_final_n = dfx_sat_n
        else:
            dfx_final_n = baseline_dfx_n

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
            b_tv_vy=b_tv_vy,
            b_tv_r=b_tv_r,
            s_tv=s_tv,
            theta_vy_dot=theta_vy_dot,
            theta_r_dot=theta_r_dot,
            theta_uc_dot=theta_uc_dot,
            dfx_raw_n=dfx_raw_n,
            dfx_sat_n=dfx_sat_n,
            dfx_final_n=dfx_final_n,
            c_alpha_f_used=c_alpha_f_used,
            c_alpha_r_used=c_alpha_r_used,
        )
