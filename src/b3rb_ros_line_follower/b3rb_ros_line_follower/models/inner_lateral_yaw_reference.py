from dataclasses import dataclass


def _clamp(value, min_value, max_value):
    return max(min_value, min(max_value, value))


@dataclass
class InnerLateralYawReferenceOutput:
    valid: bool
    uc_rad_s: float
    dt_s: float

    # x_m = [vy_m, r_m]
    vy_m_ms: float
    r_m_rad_s: float
    x_m: list

    # x_m_dot = A_m x_m + B_m u_c
    vy_m_dot_ms2: float
    r_m_dot_rad_s2: float
    x_m_dot: list

    # Reference model matrices
    A_m: list
    B_m: list

    a_vy_s_inv: float
    a_r_s_inv: float


class InnerLateralYawReferenceModel:
    """
    Desired inner-loop reference model.

    State:
        x_m = [vy_m, r_m]^T

    Input:
        u_c = r_ref

    Model:
        vy_m_dot = -a_vy * vy_m
        r_m_dot  = -a_r * r_m + a_r * u_c

    This gives a stable yaw-rate target for the future MRAC controller.
    """

    def __init__(
        self,
        a_vy_s_inv: float,
        a_r_s_inv: float,
        max_abs_uc_rad_s: float,
        max_abs_vy_m_ms: float,
        max_abs_r_m_rad_s: float,
        max_dt_s: float,
    ):
        self.a_vy_s_inv = max(0.0, float(a_vy_s_inv))
        self.a_r_s_inv = max(0.0, float(a_r_s_inv))
        self.max_abs_uc_rad_s = max(0.0, float(max_abs_uc_rad_s))
        self.max_abs_vy_m_ms = max(0.0, float(max_abs_vy_m_ms))
        self.max_abs_r_m_rad_s = max(0.0, float(max_abs_r_m_rad_s))
        self.max_dt_s = max(1e-6, float(max_dt_s))

        self.vy_m_ms = 0.0
        self.r_m_rad_s = 0.0

    def reset(self):
        self.vy_m_ms = 0.0
        self.r_m_rad_s = 0.0

    def step(
        self,
        uc_rad_s: float,
        dt_s: float,
        command_valid: bool = True,
    ) -> InnerLateralYawReferenceOutput:
        if command_valid:
            uc_rad_s = _clamp(
                float(uc_rad_s),
                -self.max_abs_uc_rad_s,
                self.max_abs_uc_rad_s,
            )
            valid = True
        else:
            uc_rad_s = 0.0
            valid = False

        dt_s = _clamp(float(dt_s), 0.0, self.max_dt_s)

        A_m = [
            [-self.a_vy_s_inv, 0.0],
            [0.0, -self.a_r_s_inv],
        ]

        B_m = [
            [0.0],
            [self.a_r_s_inv],
        ]

        vy_m_dot_ms2 = -self.a_vy_s_inv * self.vy_m_ms
        r_m_dot_rad_s2 = (
            -self.a_r_s_inv * self.r_m_rad_s
            + self.a_r_s_inv * uc_rad_s
        )

        self.vy_m_ms += vy_m_dot_ms2 * dt_s
        self.r_m_rad_s += r_m_dot_rad_s2 * dt_s

        self.vy_m_ms = _clamp(
            self.vy_m_ms,
            -self.max_abs_vy_m_ms,
            self.max_abs_vy_m_ms,
        )

        self.r_m_rad_s = _clamp(
            self.r_m_rad_s,
            -self.max_abs_r_m_rad_s,
            self.max_abs_r_m_rad_s,
        )

        return InnerLateralYawReferenceOutput(
            valid=valid,
            uc_rad_s=uc_rad_s,
            dt_s=dt_s,
            vy_m_ms=self.vy_m_ms,
            r_m_rad_s=self.r_m_rad_s,
            x_m=[self.vy_m_ms, self.r_m_rad_s],
            vy_m_dot_ms2=vy_m_dot_ms2,
            r_m_dot_rad_s2=r_m_dot_rad_s2,
            x_m_dot=[vy_m_dot_ms2, r_m_dot_rad_s2],
            A_m=A_m,
            B_m=B_m,
            a_vy_s_inv=self.a_vy_s_inv,
            a_r_s_inv=self.a_r_s_inv,
        )
