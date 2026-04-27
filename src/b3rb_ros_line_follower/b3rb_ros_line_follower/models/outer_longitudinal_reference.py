from dataclasses import dataclass


@dataclass
class OuterLongitudinalReferenceOutput:
    valid: bool

    # Desired commanded speed
    vx_ref_ms: float

    # Reference-model state
    vx_m_ms: float

    # Reference-model derivative
    vx_m_dot_ms2: float

    # vx_ref - vx_m after update
    tracking_error_ms: float

    # Reference model pole / response rate
    a_m_s_inv: float

    # Time step used in update
    dt_s: float


class OuterLongitudinalReferenceModel:
    """
    First-order outer-loop reference model:

        vx_m_dot = -a_m * vx_m + a_m * vx_ref

    equivalently:

        vx_m_dot = a_m * (vx_ref - vx_m)

    This model does NOT control the vehicle yet.
    It only generates the desired smooth longitudinal response
    needed later by the outer-loop MRAC law.
    """

    def __init__(
        self,
        a_m_s_inv: float,
        speed_cmd_to_vx_ref_gain: float,
        min_vx_ref_ms: float,
        max_vx_ref_ms: float,
        max_dt_s: float,
    ):
        self.a_m_s_inv = max(0.0, float(a_m_s_inv))
        self.speed_cmd_to_vx_ref_gain = float(speed_cmd_to_vx_ref_gain)
        self.min_vx_ref_ms = float(min_vx_ref_ms)
        self.max_vx_ref_ms = float(max_vx_ref_ms)
        self.max_dt_s = max(1e-6, float(max_dt_s))

        self.vx_m_ms = 0.0
        self.initialized = False

    @staticmethod
    def _clamp(value: float, lower: float, upper: float) -> float:
        return max(lower, min(upper, value))

    def speed_cmd_to_vx_ref(self, speed_cmd: float) -> float:
        """
        Temporary mapping for the current scaffold.

        Right now speed_cmd is the baseline controller's speed command.
        Since logs show speed_cmd roughly tracks vx in m/s, we use a
        simple gain mapping for vx_ref.
        """
        raw_vx_ref = self.speed_cmd_to_vx_ref_gain * float(speed_cmd)

        return self._clamp(
            raw_vx_ref,
            self.min_vx_ref_ms,
            self.max_vx_ref_ms,
        )

    def reset(self, vx_m_ms: float = 0.0):
        self.vx_m_ms = self._clamp(
            float(vx_m_ms),
            self.min_vx_ref_ms,
            self.max_vx_ref_ms,
        )
        self.initialized = True

    def step_from_speed_cmd(
        self,
        speed_cmd: float,
        dt_s: float,
    ) -> OuterLongitudinalReferenceOutput:
        vx_ref_ms = self.speed_cmd_to_vx_ref(speed_cmd)
        return self.step(vx_ref_ms=vx_ref_ms, dt_s=dt_s)

    def step(
        self,
        vx_ref_ms: float,
        dt_s: float,
    ) -> OuterLongitudinalReferenceOutput:
        vx_ref_ms = self._clamp(
            float(vx_ref_ms),
            self.min_vx_ref_ms,
            self.max_vx_ref_ms,
        )

        dt_s = max(0.0, min(float(dt_s), self.max_dt_s))

        if not self.initialized:
            self.vx_m_ms = 0.0
            self.initialized = True

        error_before_update = vx_ref_ms - self.vx_m_ms

        vx_m_dot_ms2 = self.a_m_s_inv * error_before_update

        # Stable first-order discrete update.
        alpha = max(0.0, min(1.0, self.a_m_s_inv * dt_s))
        self.vx_m_ms = self.vx_m_ms + alpha * error_before_update

        self.vx_m_ms = self._clamp(
            self.vx_m_ms,
            self.min_vx_ref_ms,
            self.max_vx_ref_ms,
        )

        error_after_update = vx_ref_ms - self.vx_m_ms

        return OuterLongitudinalReferenceOutput(
            valid=True,
            vx_ref_ms=vx_ref_ms,
            vx_m_ms=self.vx_m_ms,
            vx_m_dot_ms2=vx_m_dot_ms2,
            tracking_error_ms=error_after_update,
            a_m_s_inv=self.a_m_s_inv,
            dt_s=dt_s,
        )
