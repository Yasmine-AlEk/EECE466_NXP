from ..mrac_types import StateSpacePlant


class StateSpaceVehicleModel:
    """
    Unified scheduled state-space model.

    State:
        x = [v_x, v_y, r]^T

    Input:
        u = [F_x,L, F_x,R, delta_f]^T

    Model:
        x_dot = A(x)x + B(x)u + d

    This is the bridge between the physical vehicle model and the later
    reduced MRAC / RLS control-oriented model.
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
        k_drag_coeff: float = 0.0,
        min_vx_ms: float = 0.20,
    ):
        self.mass_kg = float(mass_kg)
        self.iz_kg_m2 = float(iz_kg_m2)

        self.lf_m = float(lf_m)
        self.lr_m = float(lr_m)
        self.rear_track_width_m = float(rear_track_width_m)

        self.c_alpha_f_n_per_rad = float(c_alpha_f_n_per_rad)
        self.c_alpha_r_n_per_rad = float(c_alpha_r_n_per_rad)

        self.k_drag_coeff = float(k_drag_coeff)
        self.min_vx_ms = float(min_vx_ms)

    @staticmethod
    def mat_vec_mul(matrix, vector):
        out = []
        for row in matrix:
            total = 0.0
            for a, b in zip(row, vector):
                total += a * b
            out.append(total)
        return out

    @staticmethod
    def vec_add(a, b, c):
        return [ai + bi + ci for ai, bi, ci in zip(a, b, c)]

    def build(
        self,
        vx_ms: float,
        vy_ms: float,
        r_rad_s: float,
        fx_left_n: float,
        fx_right_n: float,
        delta_f_rad: float,
        disturbance_vector=None,
    ) -> StateSpacePlant:
        if self.mass_kg <= 1e-9:
            raise ValueError("StateSpaceVehicleModel mass_kg must be positive.")

        if self.iz_kg_m2 <= 1e-9:
            raise ValueError("StateSpaceVehicleModel iz_kg_m2 must be positive.")

        vx = float(vx_ms)
        vy = float(vy_ms)
        r = float(r_rad_s)

        fx_left = float(fx_left_n)
        fx_right = float(fx_right_n)
        delta_f = float(delta_f_rad)

        if abs(vx) < self.min_vx_ms:
            if vx >= 0.0:
                vx_safe = self.min_vx_ms
            else:
                vx_safe = -self.min_vx_ms
        else:
            vx_safe = vx

        m = self.mass_kg
        iz = self.iz_kg_m2
        lf = self.lf_m
        lr = self.lr_m
        tr = self.rear_track_width_m
        caf = self.c_alpha_f_n_per_rad
        car = self.c_alpha_r_n_per_rad
        kdrag = self.k_drag_coeff

        # A(x) matrix from the report
        a00 = -kdrag / m
        a01 = r
        a02 = 0.0

        a10 = 0.0
        a11 = -(caf + car) / (m * vx_safe)
        a12 = ((lr * car - lf * caf) / (m * vx_safe)) - vx

        a20 = 0.0
        a21 = (lr * car - lf * caf) / (iz * vx_safe)
        a22 = -((lf * lf * caf) + (lr * lr * car)) / (iz * vx_safe)

        A_x = [
            [a00, a01, a02],
            [a10, a11, a12],
            [a20, a21, a22],
        ]

        # B(x) matrix from the report
        B_x = [
            [1.0 / m, 1.0 / m, 0.0],
            [0.0, 0.0, caf / m],
            [-tr / (2.0 * iz), tr / (2.0 * iz), lf * caf / iz],
        ]

        x_state = [vx, vy, r]
        u_input = [fx_left, fx_right, delta_f]

        if disturbance_vector is None:
            d_vector = [0.0, 0.0, 0.0]
        else:
            d_vector = [float(v) for v in disturbance_vector]

        ax = self.mat_vec_mul(A_x, x_state)
        bu = self.mat_vec_mul(B_x, u_input)
        x_dot_pred = self.vec_add(ax, bu, d_vector)

        return StateSpacePlant(
            x_state=x_state,
            u_input=u_input,
            A_x=A_x,
            B_x=B_x,
            d_vector=d_vector,
            x_dot_pred=x_dot_pred,
            vx_safe=vx_safe,
            valid=True,
        )
