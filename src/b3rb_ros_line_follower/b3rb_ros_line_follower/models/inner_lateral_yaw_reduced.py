from dataclasses import dataclass


@dataclass
class InnerLateralYawReducedOutput:
    valid: bool
    vx0_ms: float
    vx0_safe_ms: float
    x_lat: list
    u_input: list
    A_theta: list
    B_delta_theta: list
    B_TV: list
    x_dot_pred: list


class InnerLateralYawReducedModel:
    """
    Reduced inner-loop lateral-yaw model.

    State:
        x_lat = [v_y, r]^T

    Inputs:
        delta_f = front steering angle
        dFx = Fx_right - Fx_left

    Scheduling variable:
        vx0 = frozen longitudinal speed
    """

    def __init__(
        self,
        mass_kg: float,
        iz_kg_m2: float,
        lf_m: float,
        lr_m: float,
        rear_track_width_m: float,
        c_alpha_f_n_per_rad: float,
        c_alpha_r_n_per_rad: float,
        min_vx_ms: float,
    ):
        self.mass_kg = float(mass_kg)
        self.iz_kg_m2 = float(iz_kg_m2)
        self.lf_m = float(lf_m)
        self.lr_m = float(lr_m)
        self.rear_track_width_m = float(rear_track_width_m)
        self.c_alpha_f = float(c_alpha_f_n_per_rad)
        self.c_alpha_r = float(c_alpha_r_n_per_rad)
        self.min_vx_ms = float(min_vx_ms)

    def build(
        self,
        vy_ms: float,
        r_rad_s: float,
        vx0_ms: float,
        delta_f_rad: float,
        dfx_n: float,
    ) -> InnerLateralYawReducedOutput:

        vx0_abs = abs(float(vx0_ms))
        vx0_safe = max(vx0_abs, self.min_vx_ms)
        valid = vx0_abs >= self.min_vx_ms

        m = self.mass_kg
        iz = self.iz_kg_m2
        lf = self.lf_m
        lr = self.lr_m
        cf = self.c_alpha_f
        cr = self.c_alpha_r
        tw = self.rear_track_width_m

        # x_lat = [vy, r]^T
        a00 = -(cf + cr) / (m * vx0_safe)
        a01 = ((cr * lr - cf * lf) / (m * vx0_safe)) - vx0_safe

        a10 = (cr * lr - cf * lf) / (iz * vx0_safe)
        a11 = -((cf * lf * lf) + (cr * lr * lr)) / (iz * vx0_safe)

        A_theta = [
            [a00, a01],
            [a10, a11],
        ]

        # steering channel
        B_delta_theta = [
            cf / m,
            (lf * cf) / iz,
        ]

        # torque-vectoring channel
        # Mz_TV = (track_width / 2) * dFx
        B_TV = [
            0.0,
            (tw / 2.0) / iz,
        ]

        x_lat = [
            float(vy_ms),
            float(r_rad_s),
        ]

        u_input = [
            float(delta_f_rad),
            float(dfx_n),
        ]

        vy_dot = (
            A_theta[0][0] * x_lat[0]
            + A_theta[0][1] * x_lat[1]
            + B_delta_theta[0] * u_input[0]
            + B_TV[0] * u_input[1]
        )

        r_dot = (
            A_theta[1][0] * x_lat[0]
            + A_theta[1][1] * x_lat[1]
            + B_delta_theta[1] * u_input[0]
            + B_TV[1] * u_input[1]
        )

        x_dot_pred = [
            vy_dot,
            r_dot,
        ]

        return InnerLateralYawReducedOutput(
            valid=valid,
            vx0_ms=float(vx0_ms),
            vx0_safe_ms=vx0_safe,
            x_lat=x_lat,
            u_input=u_input,
            A_theta=A_theta,
            B_delta_theta=B_delta_theta,
            B_TV=B_TV,
            x_dot_pred=x_dot_pred,
        )
