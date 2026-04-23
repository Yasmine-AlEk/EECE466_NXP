#!/usr/bin/env python3
"""
NXP Cup – Servo Node  (PCA9685)
================================
Subscribes to a steering angle command and drives a PWM servo via the
PCA9685 over I2C.  Handles angle→pulse-width mapping, rate limiting,
and a safety timeout that centres the servo if commands stop arriving.

Hardware
--------
  PCA9685  I2C address 0x40 (default, A0-A5 = GND)
  Channel  0  →  steering servo signal wire

Subscribes
----------
  /nxp_cup/cmd_safe   geometry_msgs/TwistStamped
      .twist.angular.z   steering command  [-1.0 … +1.0]
      (−1 = full left,  0 = centre,  +1 = full right)

Publishes
---------
  /nxp_cup/servo_state   std_msgs/Float32   current angle (normalised)

Parameters
----------
  i2c_bus         int    1       /dev/i2c-N
  i2c_addr        int    0x40    PCA9685 address
  channel         int    0       PWM channel on the board
  pwm_freq_hz     int    50      Servo PWM frequency (standard 50Hz)
  pulse_min_us    int    1000    Pulse width for full-left  (µs)
  pulse_mid_us    int    1500    Pulse width for centre     (µs)
  pulse_max_us    int    2000    Pulse width for full-right (µs)
  rate_limit      float  5.0     Max normalised units per second (slew rate)
  timeout_sec     float  0.5     Zero steering if no cmd received within this

Install
-------
  pip install smbus2 --break-system-packages
"""

import time
import threading

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from std_msgs.msg import Float32

try:
    import smbus2
except ImportError:
    raise SystemExit("[ERROR] smbus2 not found.\n"
                     "  pip install smbus2 --break-system-packages")

# ─────────────────────────────────────────────────────────────────────────────
#  PCA9685 DRIVER  (direct I2C, no Adafruit dependency)
# ─────────────────────────────────────────────────────────────────────────────

PCA_MODE1     = 0x00
PCA_PRESCALE  = 0xFE
PCA_LED0_ON_L = 0x06      # base register; each channel = base + 4*ch

class PCA9685:
    CLOCK_FREQ = 25_000_000   # internal oscillator Hz

    def __init__(self, bus_num, addr):
        self._bus  = smbus2.SMBus(bus_num)
        self._addr = addr
        self._reset()

    def set_pwm_freq(self, freq_hz: int):
        """Set PWM frequency for all channels."""
        prescale = round(self.CLOCK_FREQ / (4096 * freq_hz)) - 1
        old_mode = self._read(PCA_MODE1)
        # Go to sleep (bit 4) to change prescaler
        self._write(PCA_MODE1, (old_mode & 0x7F) | 0x10)
        self._write(PCA_PRESCALE, prescale)
        self._write(PCA_MODE1, old_mode)
        time.sleep(0.005)
        # Restart (bit 7)
        self._write(PCA_MODE1, old_mode | 0x80)

    def set_pulse_us(self, channel: int, pulse_us: float, freq_hz: int):
        """Set a channel's on-time in microseconds."""
        period_us = 1_000_000 / freq_hz
        ticks     = int(round(pulse_us / period_us * 4096))
        ticks     = max(0, min(4095, ticks))
        self._set_channel(channel, 0, ticks)

    def set_off(self, channel: int):
        """Force a channel fully off (pulse = 0)."""
        self._set_channel(channel, 0, 0)

    def close(self):
        self._bus.close()

    # ── Internal ──────────────────────────────────────────────────────────────

    def _reset(self):
        self._write(PCA_MODE1, 0x00)
        time.sleep(0.01)

    def _set_channel(self, ch, on, off):
        reg = PCA_LED0_ON_L + 4 * ch
        self._bus.write_i2c_block_data(
            self._addr, reg,
            [on & 0xFF, on >> 8, off & 0xFF, off >> 8]
        )

    def _write(self, reg, val):
        self._bus.write_byte_data(self._addr, reg, val)

    def _read(self, reg):
        return self._bus.read_byte_data(self._addr, reg)


# ─────────────────────────────────────────────────────────────────────────────
#  ROS 2 NODE
# ─────────────────────────────────────────────────────────────────────────────

class ServoNode(Node):

    def __init__(self):
        super().__init__("nxp_servo_node")

        self.declare_parameter("i2c_bus",      5)
        self.declare_parameter("i2c_addr",     0x40)
        self.declare_parameter("channel",      0)
        self.declare_parameter("pwm_freq_hz",  50)
        self.declare_parameter("pulse_min_us", 1000)
        self.declare_parameter("pulse_mid_us", 1500)
        self.declare_parameter("pulse_max_us", 2000)
        self.declare_parameter("rate_limit",   5.0)
        self.declare_parameter("timeout_sec",  0.5)

        bus      = self.get_parameter("i2c_bus").value
        addr     = self.get_parameter("i2c_addr").value
        self._ch      = self.get_parameter("channel").value
        self._freq    = self.get_parameter("pwm_freq_hz").value
        self._p_min   = self.get_parameter("pulse_min_us").value
        self._p_mid   = self.get_parameter("pulse_mid_us").value
        self._p_max   = self.get_parameter("pulse_max_us").value
        self._rate_lim = self.get_parameter("rate_limit").value
        self._timeout  = self.get_parameter("timeout_sec").value

        self.get_logger().info(
            f"Opening PCA9685  bus={bus}  addr=0x{addr:02X}  "
            f"ch={self._ch}  freq={self._freq}Hz  "
            f"pulse=[{self._p_min},{self._p_mid},{self._p_max}]µs"
        )

        self._pca = PCA9685(bus, addr)
        self._pca.set_pwm_freq(self._freq)

        # State
        self._target  = 0.0    # normalised [-1, 1]
        self._current = 0.0
        self._last_cmd_t = time.monotonic()
        self._lock    = threading.Lock()

        # Centre servo on startup
        self._apply(0.0)

        # Subscribers / publishers
        self.create_subscription(
            TwistStamped, "/nxp_cup/cmd_safe", self._on_cmd, 10)
        self._pub = self.create_publisher(Float32, "/nxp_cup/servo_state", 10)

        # Control loop at 50Hz (matches servo PWM for minimal latency)
        self.create_timer(1.0 / 50.0, self._loop)
        self.get_logger().info("Servo node ready — centred.")

    # ── Callback ──────────────────────────────────────────────────────────────

    def _on_cmd(self, msg: TwistStamped):
        steer = float(msg.twist.angular.z)
        steer = max(-1.0, min(1.0, steer))
        with self._lock:
            self._target      = steer
            self._last_cmd_t  = time.monotonic()

    # ── Control loop ──────────────────────────────────────────────────────────

    def _loop(self):
        dt = 1.0 / 50.0

        with self._lock:
            target   = self._target
            last_t   = self._last_cmd_t

        # Safety timeout — centre if commands go stale
        if time.monotonic() - last_t > self._timeout:
            target = 0.0

        # Slew rate limit
        max_step = self._rate_lim * dt
        delta    = target - self._current
        delta    = max(-max_step, min(max_step, delta))
        self._current += delta

        self._apply(self._current)

        state = Float32()
        state.data = float(self._current)
        self._pub.publish(state)

    # ── Hardware write ────────────────────────────────────────────────────────

    def _apply(self, normalised: float):
        """
        Convert normalised [-1…+1] to pulse width in µs and send to PCA9685.
        Negative → left (shorter pulse), positive → right (longer pulse).
        """
        if normalised >= 0:
            pulse = self._p_mid + normalised * (self._p_max - self._p_mid)
        else:
            pulse = self._p_mid + normalised * (self._p_mid - self._p_min)
        pulse = max(self._p_min, min(self._p_max, pulse))
        try:
            self._pca.set_pulse_us(self._ch, pulse, self._freq)
        except OSError as e:
            self.get_logger().warn(f"PCA9685 write error: {e}",
                                   throttle_duration_sec=2.0)

    def destroy_node(self):
        try:
            self._apply(0.0)         # centre on shutdown
            self._pca.set_off(self._ch)
            self._pca.close()
        except Exception:
            pass
        super().destroy_node()


# ─────────────────────────────────────────────────────────────────────────────

def main():
    rclpy.init()
    node = ServoNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()