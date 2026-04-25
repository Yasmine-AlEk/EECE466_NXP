
import math

from ..mrac_config import A_LONG_FILTER_ALPHA
from ..mrac_types import VehicleSignals
from ..mrac_utils import low_pass, quaternion_to_yaw, wrap_angle


class VehicleStateEstimator:
    """Stores measured and reconstructed vehicle signals."""

    def __init__(self):
        self.vehicle = VehicleSignals()

        self.prev_x_meas = None
        self.prev_y_meas = None
        self.prev_yaw_meas = None
        self.prev_odom_time = None
        self.prev_vx_recon = None

        self.a_long_filter_initialized = False

    def update_from_odometry(self, message):
        x_meas = float(message.pose.pose.position.x)
        y_meas = float(message.pose.pose.position.y)
        yaw_meas = quaternion_to_yaw(message.pose.pose.orientation)

        t_meas = float(message.header.stamp.sec) + 1e-9 * float(message.header.stamp.nanosec)

        self.vehicle.x_meas = x_meas
        self.vehicle.y_meas = y_meas
        self.vehicle.yaw_meas = yaw_meas

        if (
            self.prev_x_meas is not None and
            self.prev_y_meas is not None and
            self.prev_yaw_meas is not None and
            self.prev_odom_time is not None
        ):
            dt_s = t_meas - self.prev_odom_time

            if dt_s > 1e-4:
                vx_world = (x_meas - self.prev_x_meas) / dt_s
                vy_world = (y_meas - self.prev_y_meas) / dt_s

                dyaw = wrap_angle(yaw_meas - self.prev_yaw_meas)
                r_recon = dyaw / dt_s

                vx_recon = math.cos(yaw_meas) * vx_world + math.sin(yaw_meas) * vy_world
                vy_recon = -math.sin(yaw_meas) * vx_world + math.cos(yaw_meas) * vy_world

                if self.prev_vx_recon is None:
                    a_long_recon = 0.0
                else:
                    a_long_recon = (vx_recon - self.prev_vx_recon) / dt_s

                self.vehicle.vx_recon = vx_recon
                self.vehicle.vy_recon = vy_recon
                self.vehicle.r_recon = r_recon
                self.vehicle.a_long_recon = a_long_recon

                if not self.a_long_filter_initialized:
                    self.vehicle.a_long_filt = a_long_recon
                    self.a_long_filter_initialized = True
                else:
                    self.vehicle.a_long_filt = low_pass(
                        self.vehicle.a_long_filt,
                        a_long_recon,
                        A_LONG_FILTER_ALPHA
                    )

                self.prev_vx_recon = vx_recon

        self.prev_x_meas = x_meas
        self.prev_y_meas = y_meas
        self.prev_yaw_meas = yaw_meas
        self.prev_odom_time = t_meas

        self.vehicle.odom_ready = True
        return self.vehicle

    def update_from_status(self, message):
        fuel_percentage_meas = getattr(message, 'fuel_percentage', None)
        power_meas = getattr(message, 'power', None)

        self.vehicle.battery_pct_meas = float(fuel_percentage_meas) if fuel_percentage_meas is not None else None
        self.vehicle.battery_power_meas = float(power_meas) if power_meas is not None else None
        self.vehicle.status_ready = True
        return self.vehicle
