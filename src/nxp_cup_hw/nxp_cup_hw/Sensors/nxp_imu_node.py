#!/usr/bin/env python3
"""
NXP Cup – MPU6050 IMU Node
============================
Reads linear acceleration (and optionally gyro) from the MPU6050 over I2C
and publishes a standard sensor_msgs/Imu message.

Hardware
--------
  MPU6050  I2C address 0x68 (AD0 pin = GND) or 0x69 (AD0 = VCC)
  Connected to NavQPlus I2C bus (default /dev/i2c-1)

Publishes
---------
  /nxp_cup/imu    sensor_msgs/Imu

  Fields populated:
    linear_acceleration   x, y, z   [m/s²]   — used by MRAC state estimator
    angular_velocity      x, y, z   [rad/s]  — bonus; useful for yaw rate
    orientation           not estimated (covariance set to -1 per REP-145)

Parameters
----------
  i2c_bus        int    5       /dev/i2c-N
  i2c_addr       int    0x68    0x68 or 0x69
  accel_range    int    2       ±2g / ±4g / ±8g / ±16g
  gyro_range     int    250     ±250 / ±500 / ±1000 / ±2000 °/s
  publish_hz     int    100     Output rate
  calibrate      bool   true    Average first N samples to zero biases

Install
-------
  pip install smbus2 --break-system-packages
"""

import time
import struct
import threading
import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import Header

try:
    import smbus2
except ImportError:
    raise SystemExit("[ERROR] smbus2 not found.\n"
                     "  pip install smbus2 --break-system-packages")

# ─────────────────────────────────────────────────────────────────────────────
#  MPU6050 REGISTER MAP
# ─────────────────────────────────────────────────────────────────────────────

MPU_ADDR         = 0x68
REG_PWR_MGMT_1   = 0x6B
REG_SMPLRT_DIV   = 0x19
REG_CONFIG       = 0x1A
REG_ACCEL_CONFIG = 0x1C
REG_GYRO_CONFIG  = 0x1B
REG_ACCEL_XOUT_H = 0x3B   # 6 bytes: AX_H AX_L AY_H AY_L AZ_H AZ_L
REG_GYRO_XOUT_H  = 0x43   # 6 bytes: GX_H GX_L GY_H GY_L GZ_H GZ_L
REG_TEMP_OUT_H   = 0x41

ACCEL_SCALE = {2: 16384.0, 4: 8192.0, 8: 4096.0, 16: 2048.0}  # LSB/g
GYRO_SCALE  = {250: 131.0, 500: 65.5, 1000: 32.8, 2000: 16.4} # LSB/(°/s)
ACCEL_CFG   = {2: 0x00,    4: 0x08,   8: 0x10,    16: 0x18}
GYRO_CFG    = {250: 0x00,  500: 0x08, 1000: 0x10, 2000: 0x18}
G           = 9.80665      # m/s²
DEG2RAD     = math.pi / 180.0

CALIB_SAMPLES = 200        # samples averaged for bias estimation


# ─────────────────────────────────────────────────────────────────────────────
#  MPU6050 DRIVER
# ─────────────────────────────────────────────────────────────────────────────

class MPU6050:

    def __init__(self, bus_num, addr, accel_range, gyro_range):
        self._bus  = smbus2.SMBus(bus_num)
        self._addr = addr
        self._ascale = ACCEL_SCALE[accel_range]
        self._gscale = GYRO_SCALE[gyro_range]

        # Wake up (clear sleep bit)
        self._write(REG_PWR_MGMT_1, 0x00)
        time.sleep(0.1)

        # Use PLL with X-gyro for better clock stability
        self._write(REG_PWR_MGMT_1, 0x01)

        # 1kHz sample rate divider → 1kHz / (1+0) = 1kHz
        self._write(REG_SMPLRT_DIV, 0x00)

        # DLPF config: 94Hz BW accel / 98Hz gyro (reduces noise at the cost
        # of a small phase lag — fine for 100Hz control loop)
        self._write(REG_CONFIG, 0x02)

        # Accelerometer range
        self._write(REG_ACCEL_CONFIG, ACCEL_CFG[accel_range])

        # Gyroscope range
        self._write(REG_GYRO_CONFIG, GYRO_CFG[gyro_range])

        time.sleep(0.05)

    def read_raw(self):
        """
        Returns (ax, ay, az, gx, gy, gz) in physical units:
            acceleration in m/s²,  angular velocity in rad/s.
        """
        # Read 14 bytes starting at ACCEL_XOUT_H:
        #   [ax_h, ax_l, ay_h, ay_l, az_h, az_l,
        #    temp_h, temp_l,
        #    gx_h, gx_l, gy_h, gy_l, gz_h, gz_l]
        raw = self._bus.read_i2c_block_data(self._addr, REG_ACCEL_XOUT_H, 14)
        vals = struct.unpack(">7h", bytes(raw))    # 7 signed 16-bit big-endian
        ax, ay, az = [v / self._ascale * G  for v in vals[0:3]]
        gx, gy, gz = [v / self._gscale * DEG2RAD for v in vals[4:7]]
        return ax, ay, az, gx, gy, gz

    def close(self):
        self._bus.close()

    def _write(self, reg, val):
        self._bus.write_byte_data(self._addr, reg, val)


# ─────────────────────────────────────────────────────────────────────────────
#  ROS 2 NODE
# ─────────────────────────────────────────────────────────────────────────────

class ImuNode(Node):

    def __init__(self):
        super().__init__("nxp_imu_node")

        self.declare_parameter("i2c_bus",    1)
        self.declare_parameter("i2c_addr",   MPU_ADDR)
        self.declare_parameter("accel_range", 2)
        self.declare_parameter("gyro_range",  250)
        self.declare_parameter("publish_hz",  100)
        self.declare_parameter("calibrate",   True)

        bus         = self.get_parameter("i2c_bus").value
        addr        = self.get_parameter("i2c_addr").value
        accel_range = self.get_parameter("accel_range").value
        gyro_range  = self.get_parameter("gyro_range").value
        hz          = self.get_parameter("publish_hz").value
        do_calib    = self.get_parameter("calibrate").value

        self.get_logger().info(
            f"Opening MPU6050  bus={bus}  addr=0x{addr:02X}  "
            f"accel=±{accel_range}g  gyro=±{gyro_range}dps  rate={hz}Hz"
        )

        self._mpu = MPU6050(bus, addr, accel_range, gyro_range)

        # Bias offsets (filled by calibration)
        self._ab = [0.0, 0.0, 0.0]   # accel bias [x, y, z]
        self._gb = [0.0, 0.0, 0.0]   # gyro  bias

        if do_calib:
            self._calibrate()

        # Covariance values — diagonal only, empirically set for MPU6050
        # Accel noise density ≈ 400 µg/√Hz → at 100Hz ≈ 0.04 m/s² RMS
        # Gyro  noise density ≈ 0.005 °/s/√Hz
        accel_var = (0.04) ** 2
        gyro_var  = (0.005 * DEG2RAD) ** 2

        self._accel_cov = [0.0] * 9
        self._accel_cov[0] = self._accel_cov[4] = self._accel_cov[8] = accel_var

        self._gyro_cov  = [0.0] * 9
        self._gyro_cov[0]  = self._gyro_cov[4]  = self._gyro_cov[8]  = gyro_var

        # orientation_covariance[0] = -1 → not estimated (REP-145)
        self._orient_cov = [-1.0] + [0.0] * 8

        self._pub = self.create_publisher(Imu, "/nxp_cup/imu", 10)
        self.create_timer(1.0 / hz, self._publish)
        self.get_logger().info("IMU node ready.")

    # ── Calibration ───────────────────────────────────────────────────────────

    def _calibrate(self):
        self.get_logger().info(
            f"Calibrating IMU — keep the car still ({CALIB_SAMPLES} samples)…"
        )
        sums = [0.0] * 6
        for _ in range(CALIB_SAMPLES):
            vals = self._mpu.read_raw()
            for i in range(6):
                sums[i] += vals[i]
            time.sleep(0.005)

        self._ab = [sums[i] / CALIB_SAMPLES for i in range(3)]
        self._gb = [sums[i] / CALIB_SAMPLES for i in range(3, 6)]

        # Z-axis accelerometer should read +g when flat, not 0.
        # Correct only the bias relative to expected gravity vector.
        # If IMU is flat, az should be +9.81; remove everything above that.
        self._ab[2] -= G   # preserve gravity direction

        self.get_logger().info(
            f"Calibration done.  "
            f"Accel bias: {[f'{v:.4f}' for v in self._ab]} m/s²  "
            f"Gyro  bias: {[f'{v:.5f}' for v in self._gb]} rad/s"
        )

    # ── Timer callback ────────────────────────────────────────────────────────

    def _publish(self):
        try:
            ax, ay, az, gx, gy, gz = self._mpu.read_raw()
        except OSError as e:
            self.get_logger().warn(f"I2C read error: {e}", throttle_duration_sec=2.0)
            return

        # Apply bias correction
        ax -= self._ab[0]
        ay -= self._ab[1]
        az -= self._ab[2]
        gx -= self._gb[0]
        gy -= self._gb[1]
        gz -= self._gb[2]

        msg = Imu()
        msg.header.stamp    = self.get_clock().now().to_msg()
        msg.header.frame_id = "imu_link"

        msg.linear_acceleration.x = ax
        msg.linear_acceleration.y = ay
        msg.linear_acceleration.z = az
        msg.linear_acceleration_covariance = self._accel_cov

        msg.angular_velocity.x = gx
        msg.angular_velocity.y = gy
        msg.angular_velocity.z = gz
        msg.angular_velocity_covariance = self._gyro_cov

        msg.orientation_covariance = self._orient_cov

        self._pub.publish(msg)

    def destroy_node(self):
        self._mpu.close()
        super().destroy_node()


# ─────────────────────────────────────────────────────────────────────────────

def main():
    rclpy.init()
    node = ImuNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()