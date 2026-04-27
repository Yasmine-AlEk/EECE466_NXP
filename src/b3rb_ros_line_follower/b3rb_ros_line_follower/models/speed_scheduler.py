from dataclasses import dataclass


@dataclass
class ScheduledSpeedOutput:
    raw_vx_ms: float
    vx0_ms: float
    previous_vx0_ms: float
    alpha: float
    valid: bool


class FilteredSpeedScheduler:
    """
    Task 3.3 scheduler for the inner lateral-yaw model.

    It converts the raw reconstructed speed vx_recon into a smoother
    scheduling variable vx0. The inner model should use vx0 instead of
    raw vx_recon so that A_theta and B_theta do not jump sharply.
    """

    def __init__(
        self,
        tau_s: float,
        min_valid_vx_ms: float,
        max_dt_s: float,
    ):
        self.tau_s = max(1e-6, float(tau_s))
        self.min_valid_vx_ms = max(0.0, float(min_valid_vx_ms))
        self.max_dt_s = max(1e-6, float(max_dt_s))

        self.initialized = False
        self.vx0_ms = 0.0

    def update(self, raw_vx_ms: float, dt_s: float) -> ScheduledSpeedOutput:
        raw_vx_ms = float(raw_vx_ms)
        dt_s = max(0.0, min(float(dt_s), self.max_dt_s))

        previous_vx0_ms = self.vx0_ms

        raw_valid = abs(raw_vx_ms) >= self.min_valid_vx_ms

        # If the vehicle is basically stopped, do not fake a valid scheduled speed.
        # This preserves inner_valid=False in the reduced model.
        if not raw_valid:
            self.vx0_ms = raw_vx_ms
            self.initialized = False
            return ScheduledSpeedOutput(
                raw_vx_ms=raw_vx_ms,
                vx0_ms=self.vx0_ms,
                previous_vx0_ms=previous_vx0_ms,
                alpha=0.0,
                valid=False,
            )

        # First valid sample initializes the scheduler directly.
        if not self.initialized:
            self.vx0_ms = raw_vx_ms
            self.initialized = True
            return ScheduledSpeedOutput(
                raw_vx_ms=raw_vx_ms,
                vx0_ms=self.vx0_ms,
                previous_vx0_ms=previous_vx0_ms,
                alpha=1.0,
                valid=True,
            )

        alpha = dt_s / (self.tau_s + dt_s)

        self.vx0_ms = self.vx0_ms + alpha * (raw_vx_ms - self.vx0_ms)

        return ScheduledSpeedOutput(
            raw_vx_ms=raw_vx_ms,
            vx0_ms=self.vx0_ms,
            previous_vx0_ms=previous_vx0_ms,
            alpha=alpha,
            valid=True,
        )
