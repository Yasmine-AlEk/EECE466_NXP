from ..mrac_types import WheelForceDecomposition


class WheelForceDecompositionModel:
    """
    Common/differential rear wheel force decomposition.

    Report relationships:

        F_long = F_x,L + F_x,R

        dFx = F_x,R - F_x,L

    Solving for left/right forces:

        F_x,L = 0.5 * (F_long - dFx)

        F_x,R = 0.5 * (F_long + dFx)

    Torque-vectoring yaw moment:

        M_z_TV = (rear_track_width / 2) * dFx
    """

    def __init__(self, rear_track_width_m: float):
        self.rear_track_width_m = float(rear_track_width_m)

    def decompose(
        self,
        f_long_n: float,
        dfx_n: float,
    ) -> WheelForceDecomposition:
        f_long_n = float(f_long_n)
        dfx_n = float(dfx_n)

        fx_left_n = 0.5 * (f_long_n - dfx_n)
        fx_right_n = 0.5 * (f_long_n + dfx_n)

        reconstructed_f_long_n = fx_left_n + fx_right_n
        reconstructed_dfx_n = fx_right_n - fx_left_n

        torque_vectoring_yaw_moment_n_m = (
            0.5 * self.rear_track_width_m * dfx_n
        )

        return WheelForceDecomposition(
            f_long_n=f_long_n,
            dfx_n=dfx_n,
            fx_left_n=fx_left_n,
            fx_right_n=fx_right_n,
            reconstructed_f_long_n=reconstructed_f_long_n,
            reconstructed_dfx_n=reconstructed_dfx_n,
            rear_track_width_m=self.rear_track_width_m,
            torque_vectoring_yaw_moment_n_m=(
                torque_vectoring_yaw_moment_n_m
            ),
            valid=True,
        )
