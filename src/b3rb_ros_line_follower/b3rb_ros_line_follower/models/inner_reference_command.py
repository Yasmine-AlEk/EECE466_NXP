from dataclasses import dataclass


@dataclass
class InnerReferenceCommandOutput:
    valid: bool
    have_camera_measurement: bool

    vx0_ms: float
    ye_cam_filt: float
    psi_rel_cam_filt: float

    kappa_ref_1pm: float
    r_ref_rad_s: float
    uc_rad_s: float

    max_abs_kappa_1pm: float
    max_abs_r_ref_rad_s: float


class InnerReferenceCommandModel:
    """
    Temporary inner-loop reference command generator.

    Report relation:
        u_c = r_ref ≈ vx0 * kappa_ref

    Since we do not yet have racing-line curvature, kappa_ref is estimated
    from filtered camera path proxies:
        kappa_ref = sign * (K_y * ye_cam_filt + K_psi * psi_rel_cam_filt)

    Later, this can be replaced by true racing-line curvature.
    """

    def __init__(
        self,
        kappa_from_ye_gain_1pm: float,
        kappa_from_psi_gain_1pm: float,
        kappa_sign: float,
        max_abs_kappa_1pm: float,
        max_abs_r_ref_rad_s: float,
        min_vx_ms: float,
    ):
        self.kappa_from_ye_gain_1pm = float(kappa_from_ye_gain_1pm)
        self.kappa_from_psi_gain_1pm = float(kappa_from_psi_gain_1pm)
        self.kappa_sign = 1.0 if kappa_sign >= 0.0 else -1.0
        self.max_abs_kappa_1pm = abs(float(max_abs_kappa_1pm))
        self.max_abs_r_ref_rad_s = abs(float(max_abs_r_ref_rad_s))
        self.min_vx_ms = abs(float(min_vx_ms))

    @staticmethod
    def _clamp(value: float, limit: float) -> float:
        if value > limit:
            return limit
        if value < -limit:
            return -limit
        return value

    def build(
        self,
        vx0_ms: float,
        have_camera_measurement: bool,
        ye_cam_filt: float,
        psi_rel_cam_filt: float,
    ) -> InnerReferenceCommandOutput:
        vx0_ms = float(vx0_ms)

        if not have_camera_measurement:
            return InnerReferenceCommandOutput(
                valid=False,
                have_camera_measurement=False,
                vx0_ms=vx0_ms,
                ye_cam_filt=0.0,
                psi_rel_cam_filt=0.0,
                kappa_ref_1pm=0.0,
                r_ref_rad_s=0.0,
                uc_rad_s=0.0,
                max_abs_kappa_1pm=self.max_abs_kappa_1pm,
                max_abs_r_ref_rad_s=self.max_abs_r_ref_rad_s,
            )

        ye_cam_filt = float(ye_cam_filt)
        psi_rel_cam_filt = float(psi_rel_cam_filt)

        kappa_raw = self.kappa_sign * (
            self.kappa_from_ye_gain_1pm * ye_cam_filt
            + self.kappa_from_psi_gain_1pm * psi_rel_cam_filt
        )

        kappa_ref = self._clamp(
            kappa_raw,
            self.max_abs_kappa_1pm,
        )

        r_ref = vx0_ms * kappa_ref
        r_ref = self._clamp(
            r_ref,
            self.max_abs_r_ref_rad_s,
        )

        valid = abs(vx0_ms) >= self.min_vx_ms

        return InnerReferenceCommandOutput(
            valid=valid,
            have_camera_measurement=True,
            vx0_ms=vx0_ms,
            ye_cam_filt=ye_cam_filt,
            psi_rel_cam_filt=psi_rel_cam_filt,
            kappa_ref_1pm=kappa_ref,
            r_ref_rad_s=r_ref,
            uc_rad_s=r_ref,
            max_abs_kappa_1pm=self.max_abs_kappa_1pm,
            max_abs_r_ref_rad_s=self.max_abs_r_ref_rad_s,
        )
