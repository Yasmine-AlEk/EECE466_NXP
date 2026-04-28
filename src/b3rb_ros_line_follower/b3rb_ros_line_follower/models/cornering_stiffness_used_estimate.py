import math
from dataclasses import dataclass


@dataclass
class CorneringStiffnessUsedEstimate:
    """
    Task 5.6 safe output for future adaptive control.

    Raw RLS estimates are allowed to move internally, but the controller-facing
    "used" estimate only updates when the estimator is ready and a real valid
    update occurred.

    This prevents:
      - using startup transients
      - using low-speed turns
      - using low-excitation straight segments
      - using small noisy sigma/projection motion as if it were a real update
    """

    ready: bool
    frozen: bool
    reason: str

    c_alpha_f_used: float
    c_alpha_r_used: float

    raw_c_alpha_f_hat: float
    raw_c_alpha_r_hat: float

    raw_valid: bool
    raw_updated: bool
    raw_update_count: int
    raw_p_trace: float


class CorneringStiffnessUsedEstimateModel:
    """
    Filters and freezes the controller-facing cornering stiffness estimates.

    Rule:
      - before enough real RLS updates: use nominal values
      - after ready: update used estimates only on new real RLS updates
      - otherwise: freeze the last used estimates
    """

    def __init__(
        self,
        nominal_c_alpha_f_n_per_rad: float,
        nominal_c_alpha_r_n_per_rad: float,
        min_c_alpha_n_per_rad: float,
        max_c_alpha_n_per_rad: float,
        ready_min_updates: int,
        max_p_trace: float,
        filter_alpha: float,
    ):
        self.nominal_c_alpha_f_n_per_rad = float(nominal_c_alpha_f_n_per_rad)
        self.nominal_c_alpha_r_n_per_rad = float(nominal_c_alpha_r_n_per_rad)

        self.min_c_alpha_n_per_rad = float(min_c_alpha_n_per_rad)
        self.max_c_alpha_n_per_rad = float(max_c_alpha_n_per_rad)

        self.ready_min_updates = int(ready_min_updates)
        self.max_p_trace = float(max_p_trace)
        self.filter_alpha = self._clamp(float(filter_alpha), 0.0, 1.0)

        self.c_alpha_f_used = self._project(self.nominal_c_alpha_f_n_per_rad)
        self.c_alpha_r_used = self._project(self.nominal_c_alpha_r_n_per_rad)

        self.ready = False
        self.last_update_count_seen = -1

    @staticmethod
    def _is_finite(value: float) -> bool:
        return math.isfinite(float(value))

    @staticmethod
    def _clamp(value: float, lo: float, hi: float) -> float:
        return min(max(value, lo), hi)

    def _project(self, value: float) -> float:
        if not self._is_finite(value):
            return self.nominal_c_alpha_f_n_per_rad
        return self._clamp(
            float(value),
            self.min_c_alpha_n_per_rad,
            self.max_c_alpha_n_per_rad,
        )

    def current(self) -> CorneringStiffnessUsedEstimate:
        return CorneringStiffnessUsedEstimate(
            ready=self.ready,
            frozen=True,
            reason="initial_nominal",
            c_alpha_f_used=self.c_alpha_f_used,
            c_alpha_r_used=self.c_alpha_r_used,
            raw_c_alpha_f_hat=self.c_alpha_f_used,
            raw_c_alpha_r_hat=self.c_alpha_r_used,
            raw_valid=False,
            raw_updated=False,
            raw_update_count=0,
            raw_p_trace=0.0,
        )

    def update(self, estimate) -> CorneringStiffnessUsedEstimate:
        if estimate is None:
            return self._make_output(
                reason="no_rls_estimate",
                frozen=True,
                raw_valid=False,
                raw_updated=False,
                raw_update_count=0,
                raw_p_trace=0.0,
                raw_c_alpha_f_hat=self.c_alpha_f_used,
                raw_c_alpha_r_hat=self.c_alpha_r_used,
            )

        raw_valid = bool(getattr(estimate, "valid", False))
        raw_updated = bool(getattr(estimate, "updated", False))

        raw_update_count = int(getattr(estimate, "update_count", 0))
        raw_p_trace = float(getattr(estimate, "p_trace", float("inf")))

        raw_c_alpha_f_hat = float(
            getattr(estimate, "c_alpha_f_hat", self.c_alpha_f_used)
        )
        raw_c_alpha_r_hat = float(
            getattr(estimate, "c_alpha_r_hat", self.c_alpha_r_used)
        )

        raw_reason = str(getattr(estimate, "reason", "unknown"))

        raw_finite = (
            self._is_finite(raw_c_alpha_f_hat)
            and self._is_finite(raw_c_alpha_r_hat)
            and self._is_finite(raw_p_trace)
        )

        raw_in_bounds = (
            self.min_c_alpha_n_per_rad
            <= raw_c_alpha_f_hat
            <= self.max_c_alpha_n_per_rad
            and self.min_c_alpha_n_per_rad
            <= raw_c_alpha_r_hat
            <= self.max_c_alpha_n_per_rad
        )

        ready_now = (
            raw_valid
            and raw_finite
            and raw_in_bounds
            and raw_update_count >= self.ready_min_updates
            and raw_p_trace <= self.max_p_trace
        )

        if ready_now:
            self.ready = True

        if not self.ready:
            return self._make_output(
                reason=f"not_ready_{raw_reason}",
                frozen=True,
                raw_valid=raw_valid,
                raw_updated=raw_updated,
                raw_update_count=raw_update_count,
                raw_p_trace=raw_p_trace,
                raw_c_alpha_f_hat=raw_c_alpha_f_hat,
                raw_c_alpha_r_hat=raw_c_alpha_r_hat,
            )

        if not raw_valid:
            return self._make_output(
                reason=f"frozen_invalid_{raw_reason}",
                frozen=True,
                raw_valid=raw_valid,
                raw_updated=raw_updated,
                raw_update_count=raw_update_count,
                raw_p_trace=raw_p_trace,
                raw_c_alpha_f_hat=raw_c_alpha_f_hat,
                raw_c_alpha_r_hat=raw_c_alpha_r_hat,
            )

        if not raw_updated:
            return self._make_output(
                reason=f"frozen_no_update_{raw_reason}",
                frozen=True,
                raw_valid=raw_valid,
                raw_updated=raw_updated,
                raw_update_count=raw_update_count,
                raw_p_trace=raw_p_trace,
                raw_c_alpha_f_hat=raw_c_alpha_f_hat,
                raw_c_alpha_r_hat=raw_c_alpha_r_hat,
            )

        if raw_update_count <= self.last_update_count_seen:
            return self._make_output(
                reason="frozen_repeated_update_count",
                frozen=True,
                raw_valid=raw_valid,
                raw_updated=raw_updated,
                raw_update_count=raw_update_count,
                raw_p_trace=raw_p_trace,
                raw_c_alpha_f_hat=raw_c_alpha_f_hat,
                raw_c_alpha_r_hat=raw_c_alpha_r_hat,
            )

        self.last_update_count_seen = raw_update_count

        projected_f = self._project(raw_c_alpha_f_hat)
        projected_r = self._project(raw_c_alpha_r_hat)

        self.c_alpha_f_used = self._project(
            self.c_alpha_f_used
            + self.filter_alpha * (projected_f - self.c_alpha_f_used)
        )
        self.c_alpha_r_used = self._project(
            self.c_alpha_r_used
            + self.filter_alpha * (projected_r - self.c_alpha_r_used)
        )

        return self._make_output(
            reason="filtered_update",
            frozen=False,
            raw_valid=raw_valid,
            raw_updated=raw_updated,
            raw_update_count=raw_update_count,
            raw_p_trace=raw_p_trace,
            raw_c_alpha_f_hat=raw_c_alpha_f_hat,
            raw_c_alpha_r_hat=raw_c_alpha_r_hat,
        )

    def _make_output(
        self,
        reason: str,
        frozen: bool,
        raw_valid: bool,
        raw_updated: bool,
        raw_update_count: int,
        raw_p_trace: float,
        raw_c_alpha_f_hat: float,
        raw_c_alpha_r_hat: float,
    ) -> CorneringStiffnessUsedEstimate:
        return CorneringStiffnessUsedEstimate(
            ready=self.ready,
            frozen=bool(frozen),
            reason=reason,
            c_alpha_f_used=self.c_alpha_f_used,
            c_alpha_r_used=self.c_alpha_r_used,
            raw_c_alpha_f_hat=raw_c_alpha_f_hat,
            raw_c_alpha_r_hat=raw_c_alpha_r_hat,
            raw_valid=bool(raw_valid),
            raw_updated=bool(raw_updated),
            raw_update_count=int(raw_update_count),
            raw_p_trace=float(raw_p_trace),
        )
