#!/usr/bin/env python3
"""
NXP Cup – BLDC Motor Node  (PCA9685)
======================================
Drives 2 BLDC motors via their ESCs using standard RC PWM (1000–2000µs)
through the PCA9685.  Handles arming, speed limits, direction, safety timeout,
and a smooth ramp to prevent current spikes.

Hardware
--------
  PCA9685  I2C address 0x40  (same board as servo, different channels)
  Channel  1  →  Left  motor ESC signal wire
  Channel  2  →  Right motor ESC signal wire

  If the servo node uses channel 0, this node uses channels 1 and 2.
  All three can share the SAME PCA9685 board — just different channels.

Subscribes
----------
  /nxp_cup/cmd_safe   geometry_msgs/TwistStamped
      .twist.linear.x    forward speed  [-1.0 … +1.0]
                         (negative = reverse, if ESC supports it)
      .twist.angular.z   NOT used here — servo node handles steering

Publishes
---------
  /nxp_cup/motor_state   std_msgs/Float32MultiArray   [left_cmd, right_cmd]
      Normalised commanded values after limiting and ramping.

Parameters
----------
  i2c_bus          int    1       /dev/i2c-N
  i2c_addr         int    0x40    PCA9685 address (share with servo node)
  channel_left     int    1
  channel_right    int    2
  pwm_freq_hz      int    50
  pulse_stop_us    int    1500    ESC stop / neutral pulse
  pulse_min_us     int    1000    Full reverse (or min forward for unidirectional)
  pulse_max_us     int    2000    Full forward
  max_speed        float  0.7     Hard cap on commanded speed [0…1]
  ramp_rate        float  2.0     Max normalised speed change per second
  timeout_sec      float  0.5     Stop motors if no cmd received within this
  arm_on_startup   bool   true    Send neutral pulse sequence to arm ESCs

  differential_k   float  1.0     How much angular.z is mixed into L/R
                                  for skid-steer differential.
                                  Set to 0.0 if this is an Ackermann car
                                  (steering handled by servo node).

Notes on ESC arming
-------------------
  Most hobby ESCs need to see the neutral (stop) pulse for ~2 seconds
  before they accept throttle commands.  arm_on_startup handles this.
  If your ESCs are already armed from a previous session they will simply
  ignore the redundant arm sequence.

Install
-------
  pip install smbus2 --break-system-packages
"""

import time
import threading

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from std_msgs.msg import Float32MultiArray

try:
    import smbus2
except ImportError:
    raise SystemExit("[ERROR] smbus2 not found.\n"
                     "  pip install smbus2 --break-system-packages")

# ─────────────────────────────────────────────────────────────────────────────
#  PCA9685 DRIVER  (same as servo node — duplicated so each file is standalone)
# ─────────────────────────────────────────────────────────────────────────────

PCA_MODE1     = 0x00
PCA_PRESCALE  = 0xFE
PCA_LED0_ON_L = 0x06

class PCA9685:
    CLOCK_FREQ = 25_000_000

    def __init__(self, bus_num, addr):
        self._bus  = smbus2.SMBus(bus_num)
        self._addr = addr
        self._write(PCA_MODE1, 0x00)
        time.sleep(0.01)

    def set_pwm_freq(self, freq_hz: int):
        prescale = round(self.CLOCK_FREQ / (4096 * freq_hz)) - 1
        old      = self._read(PCA_MODE1)
        self._write(PCA_MODE1, (old & 0x7F) | 0x10)
        self._write(PCA_PRESCALE, prescale)
        self._write(PCA_MODE1, old)
        time.sleep(0.005)
        self._write(PCA_MODE1, old | 0x80)

    def set_pulse_us(self, channel: int, pulse_us: float, freq_hz: int):
        period_us = 1_000_000 / freq_hz
        ticks     = int(round(pulse_us / period_us * 4096))
        ticks     = max(0, min(4095, ticks))
        self._set_ch(channel, 0, ticks)

    def set_off(self, channel: int):
        self._set_ch(channel, 0, 0)

    def close(self):
        self._bus.close()

    def _set_ch(self, ch, on, off):
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

class BLDCNode(Node):

    def __init__(self):
        super().__init__("nxp_bldc_node")

        self.declare_parameter("i2c_bus",         5)
        self.declare_parameter("i2c_addr",        0x40)
        self.declare_parameter("channel_left",    1)
        self.declare_parameter("channel_right",   2)
        self.declare_parameter("pwm_freq_hz",     50)
        self.declare_parameter("pulse_stop_us",   1500)
        self.declare_parameter("pulse_min_us",    1000)
        self.declare_parameter("pulse_max_us",    2000)
        self.declare_parameter("max_speed",       0.7)
        self.declare_parameter("ramp_rate",       2.0)
        self.declare_parameter("timeout_sec",     0.5)
        self.declare_parameter("arm_on_startup",  True)
        self.declare_parameter("differential_k",  0.0)

        bus      = self.get_parameter("i2c_bus").value
        addr     = self.get_parameter("i2c_addr").value
        self._ch_l      = self.get_parameter("channel_left").value
        self._ch_r      = self.get_parameter("channel_right").value
        self._freq      = self.get_parameter("pwm_freq_hz").value
        self._p_stop    = self.get_parameter("pulse_stop_us").value
        self._p_min     = self.get_parameter("pulse_min_us").value
        self._p_max     = self.get_parameter("pulse_max_us").value
        self._max_spd   = self.get_parameter("max_speed").value
        self._ramp      = self.get_parameter("ramp_rate").value
        self._timeout   = self.get_parameter("timeout_sec").value
        self._diff_k    = self.get_parameter("differential_k").value
        do_arm          = self.get_parameter("arm_on_startup").value

        self.get_logger().info(
            f"Opening PCA9685  bus={bus}  addr=0x{addr:02X}  "
            f"L=ch{self._ch_l}  R=ch{self._ch_r}  "
            f"freq={self._freq}Hz  max_spd={self._max_spd}"
        )

        self._pca = PCA9685(bus, addr)
        self._pca.set_pwm_freq(self._freq)

        # State
        self._target_l  = 0.0
        self._target_r  = 0.0
        self._current_l = 0.0
        self._current_r = 0.0
        self._last_cmd_t = time.monotonic()
        self._lock = threading.Lock()
        self._armed = False

        if do_arm:
            self._arm()

        # Subscriber / publisher
        self.create_subscription(
            TwistStamped, "/nxp_cup/cmd_safe", self._on_cmd, 10)
        self._pub = self.create_publisher(
            Float32MultiArray, "/nxp_cup/motor_state", 10)

        # Control loop at 50Hz
        self.create_timer(1.0 / 50.0, self._loop)
        self.get_logger().info("BLDC node ready.")

    # ── ESC arming sequence ───────────────────────────────────────────────────

    def _arm(self):
        """
        Standard RC ESC arming: hold neutral pulse for 2 seconds.
        Logs clearly so the operator knows to wait.
        """
        self.get_logger().info("Arming ESCs — hold neutral for 2 s …")
        self._write_both(0.0)
        time.sleep(2.0)
        self._armed = True
        self.get_logger().info("ESCs armed.")

    # ── Callback ──────────────────────────────────────────────────────────────

    def _on_cmd(self, msg: TwistStamped):
        if not self._armed:
            return

        speed = float(msg.twist.linear.x)
        steer = float(msg.twist.angular.z)   # only used if differential_k > 0

        speed = max(-1.0, min(1.0, speed))
        speed = max(-self._max_spd, min(self._max_spd, speed))

        # Differential mixing (skid-steer).
        # For Ackermann cars leave differential_k = 0.
        left  = speed - self._diff_k * steer
        right = speed + self._diff_k * steer

        left  = max(-self._max_spd, min(self._max_spd, left))
        right = max(-self._max_spd, min(self._max_spd, right))

        with self._lock:
            self._target_l   = left
            self._target_r   = right
            self._last_cmd_t = time.monotonic()

    # ── Control loop ──────────────────────────────────────────────────────────

    def _loop(self):
        dt = 1.0 / 50.0

        with self._lock:
            tl, tr = self._target_l, self._target_r
            last_t = self._last_cmd_t

        # Safety timeout
        if time.monotonic() - last_t > self._timeout:
            tl = tr = 0.0

        # Ramp both motors
        max_step = self._ramp * dt

        self._current_l += max(-max_step,
                               min(max_step, tl - self._current_l))
        self._current_r += max(-max_step,
                               min(max_step, tr - self._current_r))

        self._write_both_lr(self._current_l, self._current_r)

        state = Float32MultiArray()
        state.data = [float(self._current_l), float(self._current_r)]
        self._pub.publish(state)

    # ── Hardware write ────────────────────────────────────────────────────────

    def _normalised_to_pulse(self, value: float) -> float:
        """
        Map normalised [-1…+1] to pulse µs.
        0   → stop (neutral)
        +1  → full forward
        -1  → full reverse
        """
        if value >= 0:
            pulse = self._p_stop + value * (self._p_max - self._p_stop)
        else:
            pulse = self._p_stop + value * (self._p_stop - self._p_min)
        return max(float(self._p_min), min(float(self._p_max), pulse))

    def _write_both(self, value: float):
        self._write_both_lr(value, value)

    def _write_both_lr(self, l: float, r: float):
        pl = self._normalised_to_pulse(l)
        pr = self._normalised_to_pulse(r)
        try:
            self._pca.set_pulse_us(self._ch_l, pl, self._freq)
            self._pca.set_pulse_us(self._ch_r, pr, self._freq)
        except OSError as e:
            self.get_logger().warn(f"PCA9685 write error: {e}",
                                   throttle_duration_sec=2.0)

    def destroy_node(self):
        try:
            self._write_both(0.0)
            self._pca.set_off(self._ch_l)
            self._pca.set_off(self._ch_r)
            self._pca.close()
        except Exception:
            pass
        super().destroy_node()


# ─────────────────────────────────────────────────────────────────────────────

def main():
    rclpy.init()
    node = BLDCNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()