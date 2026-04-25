
import math

from ..mrac_types import KinematicBicycleDerivative, KinematicBicycleState


class KinematicBicycleModel:
    """
    Report IV-A style kinematic bicycle model:

        beta    = atan( lr / (lf + lr) * tan(delta_f) )
        x_dot   = v * cos(psi + beta)
        y_dot   = v * sin(psi + beta)
        psi_dot = (v / lr) * sin(beta)
        v_dot   = a_long
    """

    def __init__(self, lf_m: float, lr_m: float):
        self.lf_m = lf_m
        self.lr_m = lr_m

    @staticmethod
    def wrap_angle(angle_rad: float) -> float:
        return math.atan2(math.sin(angle_rad), math.cos(angle_rad))

    def compute_beta(self, delta_f_rad: float) -> float:
        wheelbase_m = self.lf_m + self.lr_m
        if wheelbase_m <= 1e-9:
            return 0.0
        return math.atan((self.lr_m / wheelbase_m) * math.tan(delta_f_rad))

    def derivative(
        self,
        state: KinematicBicycleState,
        delta_f_rad: float,
        a_long_ms2: float
    ) -> KinematicBicycleDerivative:
        beta_rad = self.compute_beta(delta_f_rad)

        x_dot = state.v * math.cos(state.psi + beta_rad)
        y_dot = state.v * math.sin(state.psi + beta_rad)

        if self.lr_m <= 1e-9:
            psi_dot = 0.0
        else:
            psi_dot = (state.v / self.lr_m) * math.sin(beta_rad)

        v_dot = a_long_ms2

        return KinematicBicycleDerivative(
            beta=beta_rad,
            x_dot=x_dot,
            y_dot=y_dot,
            psi_dot=psi_dot,
            v_dot=v_dot
        )

    def step_euler(
        self,
        state: KinematicBicycleState,
        delta_f_rad: float,
        a_long_ms2: float,
        dt_s: float
    ):
        deriv = self.derivative(state, delta_f_rad, a_long_ms2)

        next_state = KinematicBicycleState(
            x=state.x + deriv.x_dot * dt_s,
            y=state.y + deriv.y_dot * dt_s,
            psi=self.wrap_angle(state.psi + deriv.psi_dot * dt_s),
            v=max(0.0, state.v + deriv.v_dot * dt_s)
        )

        return next_state, deriv
