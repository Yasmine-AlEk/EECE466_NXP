import math
from dataclasses import dataclass


@dataclass
class CorneringStiffnessRegressionMatrix:
    """
    Task 5.3 regression matrix scaffold for later cornering-stiffness RLS.

    Regression form:
        y_corr = Phi * theta

    where:
        theta = [C_alpha_f, C_alpha_r]^T
        y_corr = [y1, y2_corr]^T

    This class only builds Phi and y_corr.
    It does not update C_alpha estimates yet.
    """

    valid: bool
    excited: bool
    ready_for_update: bool
    reason: str

    vx_ms: float
    vy_ms: float
    r_rad_s: float
    delta_f_rad: float
    vx_safe_ms: float

    y_corr_0: float
    y_corr_1: float

    phi_00: float
    phi_01: float
    phi_10: float
    phi_11: float

    phi_norm: float
    min_excitation_norm: float


class CorneringStiffnessRegressionMatrixModel:
    """
    Builds the RLS regression matrix for estimating C_alpha_f and C_alpha_r.

    Report regressors:

        phi_1^T = [
            delta_f / m - (v_y + l_f r) / (m v_x),
            -(v_y - l_r r) / (m v_x)
        ]

        phi_2^T = [
            l_f delta_f / I_z - l_f (v_y + l_f r) / (I_z v_x),
            l_r (v_y - l_r r) / (I_z v_x)
        ]

    The output vector is:
        y_corr = [a_y, y2_corr]^T
    """

    def __init__(
        self,
        mass_kg: float,
        iz_kg_m2: float,
        lf_m: float,
        lr_m: float,
        min_vx_ms: float,
        min_excitation_norm: float,
    ):
        self.mass_kg = float(mass_kg)
        self.iz_kg_m2 = float(iz_kg_m2)
        self.lf_m = float(lf_m)
        self.lr_m = float(lr_m)
        self.min_vx_ms = abs(float(min_vx_ms))
        self.min_excitation_norm = abs(float(min_excitation_norm))

    @staticmethod
    def _finite(*values) -> bool:
        return all(math.isfinite(float(v)) for v in values)

    def _zero_output(
        self,
        valid: bool,
        excited: bool,
        reason: str,
        vx_ms: float,
        vy_ms: float,
        r_rad_s: float,
        delta_f_rad: float,
        vx_safe_ms: float,
        y_corr_0: float,
        y_corr_1: float,
    ) -> CorneringStiffnessRegressionMatrix:
        return CorneringStiffnessRegressionMatrix(
            valid=valid,
            excited=excited,
            ready_for_update=valid and excited,
            reason=reason,
            vx_ms=vx_ms,
            vy_ms=vy_ms,
            r_rad_s=r_rad_s,
            delta_f_rad=delta_f_rad,
            vx_safe_ms=vx_safe_ms,
            y_corr_0=y_corr_0,
            y_corr_1=y_corr_1,
            phi_00=0.0,
            phi_01=0.0,
            phi_10=0.0,
            phi_11=0.0,
            phi_norm=0.0,
            min_excitation_norm=self.min_excitation_norm,
        )

    def build(
        self,
        vx_ms: float,
        vy_ms: float,
        r_rad_s: float,
        delta_f_rad: float,
        outputs,
    ) -> CorneringStiffnessRegressionMatrix:
        vx_ms = float(vx_ms)
        vy_ms = float(vy_ms)
        r_rad_s = float(r_rad_s)
        delta_f_rad = float(delta_f_rad)

        if outputs is None:
            return self._zero_output(
                valid=False,
                excited=False,
                reason="missing_outputs",
                vx_ms=vx_ms,
                vy_ms=vy_ms,
                r_rad_s=r_rad_s,
                delta_f_rad=delta_f_rad,
                vx_safe_ms=0.0,
                y_corr_0=0.0,
                y_corr_1=0.0,
            )

        y_corr_0 = float(outputs.ay_filt_ms2)
        y_corr_1 = float(outputs.y2_corr_rad_s2)

        if not outputs.valid:
            return self._zero_output(
                valid=False,
                excited=False,
                reason=f"outputs_{outputs.reason}",
                vx_ms=vx_ms,
                vy_ms=vy_ms,
                r_rad_s=r_rad_s,
                delta_f_rad=delta_f_rad,
                vx_safe_ms=vx_ms,
                y_corr_0=y_corr_0,
                y_corr_1=y_corr_1,
            )

        if not self._finite(
            vx_ms,
            vy_ms,
            r_rad_s,
            delta_f_rad,
            y_corr_0,
            y_corr_1,
            self.mass_kg,
            self.iz_kg_m2,
            self.lf_m,
            self.lr_m,
        ):
            return self._zero_output(
                valid=False,
                excited=False,
                reason="nonfinite_input",
                vx_ms=vx_ms,
                vy_ms=vy_ms,
                r_rad_s=r_rad_s,
                delta_f_rad=delta_f_rad,
                vx_safe_ms=vx_ms,
                y_corr_0=y_corr_0,
                y_corr_1=y_corr_1,
            )

        if self.mass_kg <= 1e-9 or self.iz_kg_m2 <= 1e-9:
            return self._zero_output(
                valid=False,
                excited=False,
                reason="bad_params",
                vx_ms=vx_ms,
                vy_ms=vy_ms,
                r_rad_s=r_rad_s,
                delta_f_rad=delta_f_rad,
                vx_safe_ms=vx_ms,
                y_corr_0=y_corr_0,
                y_corr_1=y_corr_1,
            )

        if abs(vx_ms) < self.min_vx_ms:
            vx_safe_ms = self.min_vx_ms if vx_ms >= 0.0 else -self.min_vx_ms
            return self._zero_output(
                valid=False,
                excited=False,
                reason="low_vx",
                vx_ms=vx_ms,
                vy_ms=vy_ms,
                r_rad_s=r_rad_s,
                delta_f_rad=delta_f_rad,
                vx_safe_ms=vx_safe_ms,
                y_corr_0=y_corr_0,
                y_corr_1=y_corr_1,
            )

        vx_safe_ms = vx_ms

        front_velocity_term = vy_ms + self.lf_m * r_rad_s
        rear_velocity_term = vy_ms - self.lr_m * r_rad_s

        phi_00 = (
            delta_f_rad / self.mass_kg
            - front_velocity_term / (self.mass_kg * vx_safe_ms)
        )
        phi_01 = -rear_velocity_term / (self.mass_kg * vx_safe_ms)

        phi_10 = (
            self.lf_m * delta_f_rad / self.iz_kg_m2
            - self.lf_m * front_velocity_term / (self.iz_kg_m2 * vx_safe_ms)
        )
        phi_11 = (
            self.lr_m * rear_velocity_term
            / (self.iz_kg_m2 * vx_safe_ms)
        )

        phi_norm = math.sqrt(
            phi_00 * phi_00
            + phi_01 * phi_01
            + phi_10 * phi_10
            + phi_11 * phi_11
        )

        excited = phi_norm >= self.min_excitation_norm
        reason = "ok" if excited else "low_excitation"

        return CorneringStiffnessRegressionMatrix(
            valid=True,
            excited=excited,
            ready_for_update=excited,
            reason=reason,
            vx_ms=vx_ms,
            vy_ms=vy_ms,
            r_rad_s=r_rad_s,
            delta_f_rad=delta_f_rad,
            vx_safe_ms=vx_safe_ms,
            y_corr_0=y_corr_0,
            y_corr_1=y_corr_1,
            phi_00=phi_00,
            phi_01=phi_01,
            phi_10=phi_10,
            phi_11=phi_11,
            phi_norm=phi_norm,
            min_excitation_norm=self.min_excitation_norm,
        )
