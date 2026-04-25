
from dataclasses import dataclass
from typing import Optional


@dataclass
class VehicleSignals:
    x_meas: float = 0.0
    y_meas: float = 0.0
    yaw_meas: float = 0.0

    vx_recon: float = 0.0
    vy_recon: float = 0.0
    r_recon: float = 0.0
    a_long_recon: float = 0.0
    a_long_filt: float = 0.0

    battery_pct_meas: Optional[float] = None
    battery_power_meas: Optional[float] = None

    odom_ready: bool = False
    status_ready: bool = False


@dataclass
class CameraMeasurement:
    have_measurement: bool = False
    use_integral: bool = False
    vector_count: int = 0

    ye_cam_filt: float = 0.0
    psi_rel_cam_filt: float = 0.0
    edge_balance_filt: float = 0.0

    center_far_filt: float = 0.0
    center_near_filt: float = 0.0
    lane_width_px: float = 0.0


@dataclass
class KinematicBicycleState:
    x: float = 0.0
    y: float = 0.0
    psi: float = 0.0
    v: float = 0.0


@dataclass
class KinematicBicycleDerivative:
    beta: float = 0.0
    x_dot: float = 0.0
    y_dot: float = 0.0
    psi_dot: float = 0.0
    v_dot: float = 0.0
