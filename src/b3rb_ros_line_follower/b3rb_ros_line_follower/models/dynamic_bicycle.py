import math

from ..mrac_types import (
    DynamicBicycleDerivative,
    DynamicBicycleInput,
    DynamicBicycleState,
    LinearTireForces,
    SlipAngles,
)


class DynamicBicycleModel:
    """
    Control-oriented 3-DOF dynamic bicycle model.

    Dynamic equations:

        v_x_dot = r v_y + a_x

        v_y_dot = -r v_x + 2/m * (F_cf cos(delta_f) + F_cr)

        r_dot = 2/Iz * (lf F_cf - lr F_cr) + M_z_tv / Iz

        X_dot = v_x cos(psi) - v_y sin(psi)

        Y_dot = v_x sin(psi) + v_y cos(psi)

        psi_dot = r

    Slip-angle approximations:

        alpha_f = atan2(v_y + lf r, v_x) - delta_f

        alpha_r = atan2(v_y - lr r, v_x)

    Linear tire-force model:

        F_c,f = -C_alpha_f * alpha_f

        F_c,r = -C_alpha_r * alpha_r
    """

    def __init__(
        self,
        mass_kg: float,
        iz_kg_m2: float,
        lf_m: float,
        lr_m: float,
        force_pair_count: float = 2.0,
        max_dt_s: float = 0.10,
        min_vx_for_slip_ms: float = 0.10,
        max_abs_slip_rad: float = math.radians(60.0),
        front_cornering_stiffness_n_per_rad: float = 8.0,
        rear_cornering_stiffness_n_per_rad: float = 8.0,
        max_abs_tire_force_n: float = 5.0,
    ):
        self.mass_kg = float(mass_kg)
        self.iz_kg_m2 = float(iz_kg_m2)
        self.lf_m = float(lf_m)
        self.lr_m = float(lr_m)
        self.force_pair_count = float(force_pair_count)
        self.max_dt_s = float(max_dt_s)

        self.min_vx_for_slip_ms = float(min_vx_for_slip_ms)
        self.max_abs_slip_rad = float(max_abs_slip_rad)

        self.front_cornering_stiffness_n_per_rad = float(
            front_cornering_stiffness_n_per_rad
        )
        self.rear_cornering_stiffness_n_per_rad = float(
            rear_cornering_stiffness_n_per_rad
        )
        self.max_abs_tire_force_n = float(max_abs_tire_force_n)

    @staticmethod
    def wrap_angle(angle_rad: float) -> float:
        return math.atan2(math.sin(angle_rad), math.cos(angle_rad))

    @staticmethod
    def clamp(value: float, min_value: float, max_value: float) -> float:
        return max(min_value, min(max_value, value))

    def compute_slip_angles(
        self,
        state: DynamicBicycleState,
        delta_f_rad: float,
    ) -> SlipAngles:
        v_x = float(state.v_x)

        if abs(v_x) < self.min_vx_for_slip_ms:
            return SlipAngles(
                alpha_f=0.0,
                alpha_r=0.0,
                valid=False,
            )

        front_lateral_velocity = float(state.v_y) + self.lf_m * float(state.r)
        rear_lateral_velocity = float(state.v_y) - self.lr_m * float(state.r)

        alpha_f = math.atan2(front_lateral_velocity, v_x) - float(delta_f_rad)
        alpha_r = math.atan2(rear_lateral_velocity, v_x)

        alpha_f = self.wrap_angle(alpha_f)
        alpha_r = self.wrap_angle(alpha_r)

        alpha_f = self.clamp(
            alpha_f,
            -self.max_abs_slip_rad,
            self.max_abs_slip_rad,
        )
        alpha_r = self.clamp(
            alpha_r,
            -self.max_abs_slip_rad,
            self.max_abs_slip_rad,
        )

        return SlipAngles(
            alpha_f=alpha_f,
            alpha_r=alpha_r,
            valid=True,
        )

    def compute_linear_tire_forces(
        self,
        slip_angles: SlipAngles,
    ) -> LinearTireForces:
        """
        Task 2.4 linear tire-force model.

        Report equation:

            F_c,i = -C_alpha_i * alpha_i

        If slip_angles.valid is False, tire forces are set to zero.
        """
        if slip_angles is None or not slip_angles.valid:
            return LinearTireForces(
                front_lateral_force_n=0.0,
                rear_lateral_force_n=0.0,
                front_cornering_stiffness_n_per_rad=(
                    self.front_cornering_stiffness_n_per_rad
                ),
                rear_cornering_stiffness_n_per_rad=(
                    self.rear_cornering_stiffness_n_per_rad
                ),
                valid=False,
            )

        front_lateral_force_n = (
            -self.front_cornering_stiffness_n_per_rad * slip_angles.alpha_f
        )
        rear_lateral_force_n = (
            -self.rear_cornering_stiffness_n_per_rad * slip_angles.alpha_r
        )

        front_lateral_force_n = self.clamp(
            front_lateral_force_n,
            -self.max_abs_tire_force_n,
            self.max_abs_tire_force_n,
        )
        rear_lateral_force_n = self.clamp(
            rear_lateral_force_n,
            -self.max_abs_tire_force_n,
            self.max_abs_tire_force_n,
        )

        return LinearTireForces(
            front_lateral_force_n=front_lateral_force_n,
            rear_lateral_force_n=rear_lateral_force_n,
            front_cornering_stiffness_n_per_rad=(
                self.front_cornering_stiffness_n_per_rad
            ),
            rear_cornering_stiffness_n_per_rad=(
                self.rear_cornering_stiffness_n_per_rad
            ),
            valid=True,
        )

    def derivative(
        self,
        state: DynamicBicycleState,
        model_input: DynamicBicycleInput,
    ) -> DynamicBicycleDerivative:
        if self.mass_kg <= 1e-9:
            raise ValueError("DynamicBicycleModel mass_kg must be positive.")

        if self.iz_kg_m2 <= 1e-9:
            raise ValueError("DynamicBicycleModel iz_kg_m2 must be positive.")

        delta_f = float(model_input.delta_f_rad)

        front_lateral_force_n = float(model_input.front_lateral_force_n)
        rear_lateral_force_n = float(model_input.rear_lateral_force_n)

        front_lateral_body_n = front_lateral_force_n * math.cos(delta_f)
        front_longitudinal_body_n = -front_lateral_force_n * math.sin(delta_f)

        effective_longitudinal_force_n = (
            float(model_input.longitudinal_force_n)
            + self.force_pair_count * front_longitudinal_body_n
            - float(model_input.drag_force_n)
            - float(model_input.ramp_force_n)
        )

        a_x_body = effective_longitudinal_force_n / self.mass_kg

        v_x_dot = state.r * state.v_y + a_x_body

        v_y_dot = (
            -state.r * state.v_x
            + (self.force_pair_count / self.mass_kg)
            * (front_lateral_body_n + rear_lateral_force_n)
        )

        r_dot = (
            (self.force_pair_count / self.iz_kg_m2)
            * (
                self.lf_m * front_lateral_force_n
                - self.lr_m * rear_lateral_force_n
            )
            + float(model_input.torque_vectoring_yaw_moment_n_m) / self.iz_kg_m2
        )

        x_dot = (
            state.v_x * math.cos(state.psi)
            - state.v_y * math.sin(state.psi)
        )

        y_dot = (
            state.v_x * math.sin(state.psi)
            + state.v_y * math.cos(state.psi)
        )

        psi_dot = state.r

        return DynamicBicycleDerivative(
            a_x_body=a_x_body,
            v_x_dot=v_x_dot,
            v_y_dot=v_y_dot,
            r_dot=r_dot,
            x_dot=x_dot,
            y_dot=y_dot,
            psi_dot=psi_dot,
        )

    def step_euler(
        self,
        state: DynamicBicycleState,
        model_input: DynamicBicycleInput,
        dt_s: float,
    ):
        safe_dt_s = max(0.0, min(float(dt_s), self.max_dt_s))

        deriv = self.derivative(
            state=state,
            model_input=model_input,
        )

        next_state = DynamicBicycleState(
            x=state.x + deriv.x_dot * safe_dt_s,
            y=state.y + deriv.y_dot * safe_dt_s,
            psi=self.wrap_angle(state.psi + deriv.psi_dot * safe_dt_s),
            v_x=state.v_x + deriv.v_x_dot * safe_dt_s,
            v_y=state.v_y + deriv.v_y_dot * safe_dt_s,
            r=state.r + deriv.r_dot * safe_dt_s,
        )

        return next_state, deriv
