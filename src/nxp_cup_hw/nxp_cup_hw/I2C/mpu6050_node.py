#!/usr/bin/env python3
"""
mpu6050_node.py
---------------
Reads MPU6050 accelerometer + gyroscope over I2C and publishes
sensor_msgs/Imu to /nxp_cup/imu.

Mounting: flat, X forward, Y left, Z up
  ax = forward acceleration  [m/s²]
  ay = lateral acceleration  [m/s²]  (positive = left)
  gz = yaw rate              [rad/s] (positive = turning left / CCW)
"""

import struct
import time
import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import smbus2

# ── MPU6050 registers ─────────────────────────────────────────────────────────
PWR_MGMT_1   = 0x6B
ACCEL_CONFIG = 0x1C
GYRO_CONFIG  = 0x1B
ACCEL_XOUT_H = 0x3B   # 14 bytes: AX AY AZ TEMP GX GY GZ (high/low each)

# Scale factors
ACCEL_SCALE  = 16384.0   # ±2g  → LSB/g
GYRO_SCALE   = 131.0     # ±250°/s → LSB/°/s
G            = 9.80665   # m/s²

I2C_BUS      = 5
IMU_ADDR     = 0x68
PUBLISH_HZ   = 100.0


class MPU6050Node(Node):

    def __init__(self):
        super().__init__("mpu6050_node")

        self._bus = smbus2.SMBus(I2C_BUS)

        # Wake up (clear sleep bit)
        self._bus.write_byte_data(IMU_ADDR, PWR_MGMT_1, 0x00)
        time.sleep(0.1)

        # ±2g accelerometer, ±250°/s gyro (default after reset, explicit here)
        self._bus.write_byte_data(IMU_ADDR, ACCEL_CONFIG, 0x00)
        self._bus.write_byte_data(IMU_ADDR, GYRO_CONFIG,  0x00)

        self._pub = self.create_publisher(Imu, "/nxp_cup/imu", 10)
        self.create_timer(1.0 / PUBLISH_HZ, self._poll)

        self._consecutive_errors = 0

        # Bias calibration accumulators (first 100 samples at rest)
        self._cal_samples = 0
        self._cal_max     = 100
        self._ax_bias = self._ay_bias = self._az_bias = 0.0
        self._gx_bias = self._gy_bias = self._gz_bias = 0.0
        self._calibrated = False

        self.get_logger().info(
            f"MPU6050 ready on i2c-{I2C_BUS} addr=0x{IMU_ADDR:02X} — "
            f"calibrating biases ({self._cal_max} samples)…"
        )

    def _read_raw(self):
        """Read 14 bytes starting at ACCEL_XOUT_H, return 7 signed int16s."""
        data = self._bus.read_i2c_block_data(IMU_ADDR, ACCEL_XOUT_H, 14)
        return struct.unpack(">7h", bytes(data))   # big-endian signed shorts

    def _recover_i2c(self):
        """Attempt to unstick a locked I2C bus by toggling SCL 9 times."""
        self.get_logger().error('I2C bus lock detected — attempting recovery')
        try:
            # Close and reopen the bus
            self._bus.close()
            import time as _t
            _t.sleep(0.05)
            self._bus = smbus2.SMBus(I2C_BUS)
            # Re-wake MPU6050
            self._bus.write_byte_data(IMU_ADDR, PWR_MGMT_1, 0x00)
            _t.sleep(0.01)
            self._calibrated = False   # force recalibration
            self._cal_samples = 0
            self._ax_bias = self._ay_bias = self._az_bias = 0.0
            self._gx_bias = self._gy_bias = self._gz_bias = 0.0
            self.get_logger().warn('I2C bus recovered — recalibrating IMU')
        except Exception as e:
            self.get_logger().error(f'I2C recovery failed: {e}')

    def _poll(self):
        try:
            ax_r, ay_r, az_r, _, gx_r, gy_r, gz_r = self._read_raw()
            self._consecutive_errors = 0   # reset error counter on success
        except OSError as e:
            self._consecutive_errors += 1
            self.get_logger().warn(
                f"I2C read error #{self._consecutive_errors}: {e}",
                throttle_duration_sec=1.0
            )
            if self._consecutive_errors >= 5:
                self._consecutive_errors = 0
                self._recover_i2c()
            return

        # Convert to SI
        ax = ax_r / ACCEL_SCALE * G
        ay = ay_r / ACCEL_SCALE * G
        az = az_r / ACCEL_SCALE * G
        gx = math.radians(gx_r / GYRO_SCALE)
        gy = math.radians(gy_r / GYRO_SCALE)
        gz = math.radians(gz_r / GYRO_SCALE)

        # Bias calibration during first N samples (assume stationary at startup)
        if not self._calibrated:
            self._ax_bias += ax
            self._ay_bias += ay
            self._az_bias += az - G    # remove gravity on Z
            self._gx_bias += gx
            self._gy_bias += gy
            self._gz_bias += gz
            self._cal_samples += 1

            if self._cal_samples >= self._cal_max:
                n = float(self._cal_max)
                self._ax_bias /= n
                self._ay_bias /= n
                self._az_bias /= n
                self._gx_bias /= n
                self._gy_bias /= n
                self._gz_bias /= n
                self._calibrated = True
                self.get_logger().info(
                    f"IMU calibrated  gz_bias={math.degrees(self._gz_bias):.3f}°/s"
                )
            return

        # Apply bias
        ax -= self._ax_bias
        ay -= self._ay_bias
        az -= self._az_bias
        gx -= self._gx_bias
        gy -= self._gy_bias
        gz -= self._gz_bias

        # Guard against NaN/inf from raw sensor glitches
        if not all(math.isfinite(v) for v in (ax, ay, az, gx, gy, gz)):
            return

        msg = Imu()
        msg.header.stamp    = self.get_clock().now().to_msg()
        msg.header.frame_id = "imu_link"

        msg.linear_acceleration.x = ax
        msg.linear_acceleration.y = ay
        msg.linear_acceleration.z = az

        msg.angular_velocity.x = gx
        msg.angular_velocity.y = gy
        msg.angular_velocity.z = gz

        # Explicit full covariance arrays (required by rclpy Humble pybind11)
        # orientation unknown (no magnetometer) → first element = -1
        msg.orientation_covariance        = [-1.0, 0.0, 0.0,
                                              0.0, 0.0, 0.0,
                                              0.0, 0.0, 0.0]
        msg.angular_velocity_covariance   = [1e-4, 0.0, 0.0,
                                              0.0, 1e-4, 0.0,
                                              0.0, 0.0, 1e-4]
        msg.linear_acceleration_covariance= [1e-2, 0.0, 0.0,
                                              0.0, 1e-2, 0.0,
                                              0.0, 0.0, 1e-2]

        self._pub.publish(msg)

    def destroy_node(self):
        self._bus.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = MPU6050Node()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()