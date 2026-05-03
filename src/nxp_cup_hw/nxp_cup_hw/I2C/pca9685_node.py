#!/usr/bin/env python3
"""
pca9685_node.py
---------------
Drives two BLDC ESCs + one steering servo via PCA9685.
Motor PWM is mapped from a calibration CSV (pwm_us, left_rad_s, right_rad_s)
so that cmd speed [0..1] maps linearly to rad/s, not to raw PWM.

Calibration CSV must be in the same folder as this script.

Parameters
----------
min_speed_only : bool (default False)
    When True any non-zero speed command is clamped to the minimum calibrated
    rad/s instead of the requested value.  Useful for slow walk-through testing.
    Pass via launch file:  parameters=[{'min_speed_only': True}]
"""

import os
import csv
import math
import numpy as np

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import Joy
import smbus2

# ── Hardware config ───────────────────────────────────────────────────────────
I2C_BUS      = 5
I2C_ADR     = 0x40
PWM_FREQ_HZ  = 50

SERVO_CH     = 1
SERVO_MIN_US = 1000
SERVO_MID_US = 1500
SERVO_MAX_US = 2000
STEER_GAIN   = 1.0

BLDC_CH_L    = 2
BLDC_CH_R    = 3
BLDC_MIN_US  = 1000   # stop / armed

CAL_FILE     = os.path.join(os.path.dirname(os.path.abspath(__file__)), "motor_calibration.csv")
# ─────────────────────────────────────────────────────────────────────────────


def load_calibration(path: str):
    pwm_col, l_col, r_col = [], [], []

    with open(path, newline="") as f:
        for row in csv.DictReader(f):
            pwm_col.append(float(row["pwm_us"]))
            l_col.append(float(row["left_rad_s"]))
            r_col.append(float(row["right_rad_s"]))

    def monotone_slice(rad_s_vals):
        for i in range(1, len(rad_s_vals)):
            if rad_s_vals[i] <= rad_s_vals[i - 1]:
                return i
        return len(rad_s_vals)

    l_end = monotone_slice(l_col)
    r_end = monotone_slice(r_col)

    l_rads = np.array(l_col[:l_end])
    l_pwm  = np.array(pwm_col[:l_end])
    r_rads = np.array(r_col[:r_end])
    r_pwm  = np.array(pwm_col[:r_end])

    max_rad_s = min(l_rads[-1], r_rads[-1])
    min_rad_s = max(l_rads[0],  r_rads[0])   # slowest speed both wheels can do

    return l_rads, l_pwm, r_rads, r_pwm, max_rad_s, min_rad_s


def set_channel(bus, ch, pulse_us):
    off = int((pulse_us / (1_000_000 / PWM_FREQ_HZ)) * 4096)
    off = max(0, min(4095, off))
    reg = 0x06 + 4 * ch
    bus.write_byte_data(I2C_ADR, reg + 0, 0x00)
    bus.write_byte_data(I2C_ADR, reg + 1, 0x00)
    bus.write_byte_data(I2C_ADR, reg + 2, off & 0xFF)
    bus.write_byte_data(I2C_ADR, reg + 3, (off >> 8) & 0x0F)


class PCA9685Node(Node):

    def __init__(self):
        super().__init__('pca9685_node')
        self._bus = smbus2.SMBus(I2C_BUS)

        # ── Parameter ─────────────────────────────────────────────────────────
        self.declare_parameter('min_speed_only', False)
        self._min_speed_only = self.get_parameter('min_speed_only').get_parameter_value().bool_value

        # ── Calibration ───────────────────────────────────────────────────────
        l_rads, l_pwm, r_rads, r_pwm, self._max_rad_s, self._min_rad_s = load_calibration(CAL_FILE)
        self._l_rads, self._l_pwm = l_rads, l_pwm
        self._r_rads, self._r_pwm = r_rads, r_pwm

        self.get_logger().info(
            f'Calibration loaded: {len(l_rads)} L-points, {len(r_rads)} R-points, '
            f'max usable rad/s = {self._max_rad_s:.2f}'
        )
        self.get_logger().info(
            f'min_speed_only = {self._min_speed_only}  '
            f'(min rad/s = {self._min_rad_s:.2f})'
        )

        self._last_l_us = float(BLDC_MIN_US)
        self._last_r_us = float(BLDC_MIN_US)

        self.create_subscription(TwistStamped, '/nxp_cup/cmd_safe', self._cb, 10)
        self.create_subscription(Joy, '/cerebri/in/joy', self._joy_cb, 10)

        self.get_logger().info('PCA9685 node ready')

    def _rad_s_to_pwm(self, desired_rad_s: float) -> tuple[float, float]:
        desired_rad_s = max(0.0, min(desired_rad_s, self._max_rad_s))
        l_us = float(np.interp(desired_rad_s, self._l_rads, self._l_pwm))
        r_us = float(np.interp(desired_rad_s, self._r_rads, self._r_pwm))
        return l_us, r_us

    def _joy_cb(self, msg: Joy):
        if len(msg.axes) < 4:
            return
        twist = TwistStamped()
        twist.twist.linear.x  = float(msg.axes[1])
        twist.twist.angular.z = float(msg.axes[3])
        self._cb(twist)

    def _cb(self, msg: TwistStamped):
        steer = float(msg.twist.angular.z)
        speed = max(0.0, min(1.0, float(msg.twist.linear.x)))

        steer_gained = max(-1.0, min(1.0, steer * STEER_GAIN))

        servo_us = (SERVO_MID_US
                    + steer_gained * (SERVO_MAX_US - SERVO_MID_US if steer_gained >= 0
                                      else SERVO_MID_US - SERVO_MIN_US))

        # When min_speed_only is set, clamp any non-zero speed to minimum rad/s
        if self._min_speed_only and speed > 0.0:
            desired_rad_s = self._min_rad_s
        else:
            desired_rad_s = speed * self._max_rad_s

        l_us, r_us = self._rad_s_to_pwm(desired_rad_s)

        if speed == 0.0:
            l_us = self._last_l_us
            r_us = self._last_r_us
        else:
            self._last_l_us = l_us
            self._last_r_us = r_us

        set_channel(self._bus, SERVO_CH,  servo_us)
        set_channel(self._bus, BLDC_CH_L, l_us)
        set_channel(self._bus, BLDC_CH_R, r_us)

        self.get_logger().info(
            f'steer_raw={steer:+.4f} gained={steer_gained:+.3f} servo={servo_us:.0f}µs  '
            f'speed={speed:.3f} ({desired_rad_s:.2f} rad/s)  '
            f'L={l_us:.0f}µs R={r_us:.0f}µs',
            throttle_duration_sec=0.5
        )

    def destroy_node(self) -> None:
        self.get_logger().info('Shutting down — sending stop PWM')
        try:
            set_channel(self._bus, SERVO_CH,  SERVO_MID_US)
            set_channel(self._bus, BLDC_CH_L, BLDC_MIN_US)
            set_channel(self._bus, BLDC_CH_R, BLDC_MIN_US)
        except Exception:
            pass
        self._bus.close()
        super().destroy_node()


def main():
    rclpy.init()
    node = PCA9685Node()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()