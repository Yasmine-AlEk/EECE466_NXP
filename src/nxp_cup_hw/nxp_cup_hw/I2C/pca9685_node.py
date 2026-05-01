#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
import smbus2

I2C_BUS      = 5
I2C_ADDR     = 0x40
PWM_FREQ_HZ  = 50

SERVO_CH     = 1
SERVO_MIN_US = 1000
SERVO_MID_US = 1500
SERVO_MAX_US = 2000

BLDC_CH_L    = 2
BLDC_CH_R    = 3
BLDC_MIN_US  = 1000   # stop / armed
BLDC_MAX_US  = 2000   # full throttle
# speed in [0..1] maps to [1000..2000us]
# negative speed = 0 (no reverse on ESC)


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
        self.get_logger().info('PCA9685 ready')
        self.create_subscription(TwistStamped, '/nxp_cup/cmd_safe', self._cb, 10)

    def _cb(self, msg):
        steer = max(-1.0, min(1.0, float(msg.twist.angular.z)))
        speed = max( 0.0, min(1.0, float(msg.twist.linear.x)))  # clamp to [0..1], no reverse

        servo_us = SERVO_MID_US + steer * (SERVO_MAX_US - SERVO_MID_US if steer >= 0 else SERVO_MID_US - SERVO_MIN_US)
        bldc_us  = BLDC_MIN_US  + speed * (BLDC_MAX_US - BLDC_MIN_US)

        set_channel(self._bus, SERVO_CH,  servo_us)
        set_channel(self._bus, BLDC_CH_L, bldc_us)
        set_channel(self._bus, BLDC_CH_R, bldc_us)

        self.get_logger().info(
            f'steer={steer:+.3f} servo={servo_us:.0f}us  '
            f'speed={speed:.3f} bldc={bldc_us:.0f}us',
            throttle_duration_sec=0.5)

def main():
    rclpy.init()
    rclpy.spin(PCA9685Node())
    rclpy.shutdown()

if __name__ == '__main__':
    main()