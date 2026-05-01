#!/usr/bin/env python3
"""
pca9685_node.py
---------------
Drives two BLDC ESCs + one steering servo via PCA9685.
Motor PWM is mapped from a calibration CSV (pwm_us, left_rad_s, right_rad_s)
so that cmd speed [0..1] maps linearly to rad/s, not to raw PWM.

Calibration CSV must be in the same folder as this script.
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
I2C_ADDR     = 0x40
PWM_FREQ_HZ  = 50

SERVO_CH     = 1
SERVO_MIN_US = 1000
SERVO_MID_US = 1500
SERVO_MAX_US = 2000
# MRAC inner_turn_final is typically 0.001–0.05 (designed for Cerebri which
# applies its own internal gain).  Multiply here to get real servo deflection.
# At STEER_GAIN=15: turn_final=0.05 → 1500 + 0.75*500 = 1875µs (large turn)
#                   turn_final=0.01 → 1500 + 0.15*500 = 1575µs (gentle turn)
# Tune this value on the actual track — start at 10, increase if car understeers.
STEER_GAIN   = 15.0

BLDC_CH_L    = 2
BLDC_CH_R    = 3
BLDC_MIN_US  = 1000   # stop / armed

CAL_FILE     = os.path.join(os.path.dirname(os.path.abspath(__file__)), "motor_calibration.csv")
# ─────────────────────────────────────────────────────────────────────────────


def load_calibration(path: str):
    """
    Load calibration CSV and return two interpolators:
      speed_to_pwm_l(rad_s) → pwm_us   for left  motor
      speed_to_pwm_r(rad_s) → pwm_us   for right motor

    Only the monotonically-increasing portion of each curve is used
    (drops the aliased tail automatically).
    """
    pwm_col, l_col, r_col = [], [], []

    with open(path, newline="") as f:
        for row in csv.DictReader(f):
            pwm_col.append(float(row["pwm_us"]))
            l_col.append(float(row["left_rad_s"]))
            r_col.append(float(row["right_rad_s"]))

    def monotone_slice(rad_s_vals):
        """Return index up to (not including) first non-increasing sample."""
        for i in range(1, len(rad_s_vals)):
            if rad_s_vals[i] <= rad_s_vals[i - 1]:
                return i
        return len(rad_s_vals)

    l_end = monotone_slice(l_col)
    r_end = monotone_slice(r_col)

    # Valid (rad/s → pwm) pairs for each wheel
    l_rads = np.array(l_col[:l_end])
    l_pwm  = np.array(pwm_col[:l_end])
    r_rads = np.array(r_col[:r_end])
    r_pwm  = np.array(pwm_col[:r_end])

    # Shared max rad/s: stay within range of both wheels
    max_rad_s = min(l_rads[-1], r_rads[-1])

    return l_rads, l_pwm, r_rads, r_pwm, max_rad_s


def set_channel(bus, ch, pulse_us):
    off = int((pulse_us / (1_000_000 / PWM_FREQ_HZ)) * 4096)
    off = max(0, min(4095, off))
    reg = 0x06 + 4 * ch
    bus.write_byte_data(I2C_ADDR, reg + 0, 0x00)
    bus.write_byte_data(I2C_ADDR, reg + 1, 0x00)
    bus.write_byte_data(I2C_ADDR, reg + 2, off & 0xFF)
    bus.write_byte_data(I2C_ADDR, reg + 3, (off >> 8) & 0x0F)


class PCA9685Node(Node):

    def __init__(self):
        super().__init__('pca9685_node')
        self._bus = smbus2.SMBus(I2C_BUS)

        # Load calibration
        l_rads, l_pwm, r_rads, r_pwm, self._max_rad_s = load_calibration(CAL_FILE)
        self._l_rads, self._l_pwm = l_rads, l_pwm
        self._r_rads, self._r_pwm = r_rads, r_pwm

        self.get_logger().info(
            f'Calibration loaded: {len(l_rads)} L-points, {len(r_rads)} R-points, '
            f'max usable rad/s = {self._max_rad_s:.2f}'
        )

        # Cache last non-zero PWM to hold between edge vector frames
        self._last_l_us = float(BLDC_MIN_US)
        self._last_r_us = float(BLDC_MIN_US)

        # /nxp_cup/cmd_safe  – from bicycle model / teleop (TwistStamped)
        self.create_subscription(TwistStamped, '/nxp_cup/cmd_safe', self._cb, 10)

        # /cerebri/in/joy    – from b3rb_ros_line_follower / b3rb_ros_mrac (Joy)
        # axes[1] = speed [-1..1],  axes[3] = steering [-1..1]
        self.create_subscription(Joy, '/cerebri/in/joy', self._joy_cb, 10)

        self.get_logger().info('PCA9685 node ready')

    def _rad_s_to_pwm(self, desired_rad_s: float) -> tuple[float, float]:
        """Invert calibration curve: rad/s → (left_pwm_us, right_pwm_us)."""
        desired_rad_s = max(0.0, min(desired_rad_s, self._max_rad_s))
        l_us = float(np.interp(desired_rad_s, self._l_rads, self._l_pwm))
        r_us = float(np.interp(desired_rad_s, self._r_rads, self._r_pwm))
        return l_us, r_us

    def _joy_cb(self, msg: Joy):
        """Handle /cerebri/in/joy from the line follower package."""
        if len(msg.axes) < 4:
            return
        # Build a synthetic TwistStamped and reuse the existing callback
        twist = TwistStamped()
        twist.twist.linear.x  = float(msg.axes[1])   # speed    [-1..1]
        twist.twist.angular.z = float(msg.axes[3])   # steering [-1..1]
        self._cb(twist)

    def _cb(self, msg: TwistStamped):
        steer = float(msg.twist.angular.z)   # raw from MRAC, not pre-clamped
        speed = max( 0.0, min(1.0,  float(msg.twist.linear.x)))

        # Apply gain then clamp to [-1, 1] so servo never exceeds limits
        steer_gained = max(-1.0, min(1.0, steer * STEER_GAIN))

        # Servo
        servo_us = (SERVO_MID_US
                    + steer_gained * (SERVO_MAX_US - SERVO_MID_US if steer_gained >= 0
                                      else SERVO_MID_US - SERVO_MIN_US))

        # BLDC: map [0..1] → [0..max_rad_s] → individual PWM per wheel
        desired_rad_s    = speed * self._max_rad_s
        l_us, r_us       = self._rad_s_to_pwm(desired_rad_s)

        # At zero speed hold last commanded PWM (prevents ESC reset between frames)
        # Only send 1000µs if we've never moved
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


    # ── Cleanup ───────────────────────────────────────────────────────────────
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