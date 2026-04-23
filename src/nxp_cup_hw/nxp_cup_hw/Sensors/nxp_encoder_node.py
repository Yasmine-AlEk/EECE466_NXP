#!/usr/bin/env python3
"""
NXP Cup – Wheel Encoder Node
==============================
Reads 2 incremental encoders from a PCF8574 I/O expander over I2C.
Detects rising edges on 2 configurable pin bits, accumulates tick counts,
and publishes wheel odometry.

Hardware
--------
  PCF8574  I2C address 0x20 (default, A0-A2 = GND)
  Pin P0   → Left  wheel encoder optical sensor  (0/1)
  Pin P1   → Right wheel encoder optical sensor  (0/1)
  10 counts per full wheel revolution (optical disc, 10 holes)

Publishes
---------
  /nxp_cup/wheel_ticks    std_msgs/Int32MultiArray   [left_ticks, right_ticks]
  /nxp_cup/wheel_odom     nav_msgs/Odometry          dead-reckoning odometry

Parameters (set at top of file or via ROS2 params)
----------
  i2c_bus          int     1        I2C bus number (/dev/i2c-N)
  i2c_addr         int     0x20     PCF8574 I2C address
  left_pin         int     0        PCF8574 pin bit for left encoder  (0-7)
  right_pin        int     1        PCF8574 pin bit for right encoder (0-7)
  counts_per_rev   int     10       Encoder counts per wheel revolution
  wheel_radius_m   float   0.033    Wheel radius in metres
  wheel_base_m     float   0.165    Track width (axle to axle) in metres
  poll_hz          int     500      PCF8574 polling rate in Hz

Install
-------
  pip install smbus2 --break-system-packages
"""

import time
import threading
import math

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
import tf2_ros

try:
    import smbus2
except ImportError:
    raise SystemExit("[ERROR] smbus2 not found.\n"
                     "  pip install smbus2 --break-system-packages")

# ─────────────────────────────────────────────────────────────────────────────
#  CONFIG
# ─────────────────────────────────────────────────────────────────────────────

I2C_BUS        = 5
I2C_ADDR       = 0x20      # PCF8574 default (A0-A2 tied to GND)
LEFT_PIN       = 0         # bit index (0-7)
RIGHT_PIN      = 1
COUNTS_PER_REV = 10
WHEEL_RADIUS   = 0.033     # metres — measure your wheel
WHEEL_BASE     = 0.165     # metres — measure axle-to-axle
POLL_HZ        = 500       # Hz — PCF8574 is fast enough for this

PUBLISH_HZ     = 50        # odometry publish rate


# ─────────────────────────────────────────────────────────────────────────────
#  PCF8574 READER  (runs in its own thread at POLL_HZ)
# ─────────────────────────────────────────────────────────────────────────────

class PCF8574:
    def __init__(self, bus_num, addr):
        self._bus  = smbus2.SMBus(bus_num)
        self._addr = addr
        # Drive all pins HIGH (PCF8574 reads need pins set to 1 first)
        self._bus.write_byte(self._addr, 0xFF)
        time.sleep(0.01)

    def read(self) -> int:
        """Return the 8-bit port value."""
        return self._bus.read_byte(self._addr)

    def close(self):
        self._bus.close()


class EncoderReader:
    """
    Polls the PCF8574 at high rate and counts rising edges on two pins.
    Thread-safe: tick counts can be read from any thread.
    """

    def __init__(self, bus_num, addr, left_pin, right_pin, poll_hz):
        self._pcf   = PCF8574(bus_num, addr)
        self._lmask = 1 << left_pin
        self._rmask = 1 << right_pin
        self._dt    = 1.0 / poll_hz

        self._lock      = threading.Lock()
        self._left_ticks  = 0
        self._right_ticks = 0

        self._prev_byte = self._pcf.read()
        self._running   = False
        self._thread    = threading.Thread(target=self._loop, daemon=True)

    def start(self):
        self._running = True
        self._thread.start()

    def stop(self):
        self._running = False
        self._thread.join(timeout=1.0)
        self._pcf.close()

    def get_and_reset(self):
        """Return (left_delta, right_delta) since last call and reset counters."""
        with self._lock:
            l, r = self._left_ticks, self._right_ticks
            self._left_ticks  = 0
            self._right_ticks = 0
        return l, r

    def get_total(self):
        with self._lock:
            return self._left_ticks, self._right_ticks

    def _loop(self):
        period = self._dt
        next_t = time.monotonic()
        while self._running:
            now = time.monotonic()
            if now >= next_t:
                try:
                    byte = self._pcf.read()
                except OSError:
                    # I2C glitch — skip this sample
                    next_t += period
                    continue

                # Rising edge detection  (0→1 transition)
                rose = (~self._prev_byte) & byte
                with self._lock:
                    if rose & self._lmask:
                        self._left_ticks  += 1
                    if rose & self._rmask:
                        self._right_ticks += 1
                self._prev_byte = byte
                next_t += period
            else:
                time.sleep(max(0, next_t - now - 0.0001))


# ─────────────────────────────────────────────────────────────────────────────
#  ROS 2 NODE
# ─────────────────────────────────────────────────────────────────────────────

class EncoderNode(Node):

    def __init__(self):
        super().__init__("nxp_encoder_node")

        # ROS parameters (can override via ros2 param set)
        self.declare_parameter("i2c_bus",        I2C_BUS)
        self.declare_parameter("i2c_addr",        I2C_ADDR)
        self.declare_parameter("left_pin",        LEFT_PIN)
        self.declare_parameter("right_pin",       RIGHT_PIN)
        self.declare_parameter("counts_per_rev",  COUNTS_PER_REV)
        self.declare_parameter("wheel_radius_m",  WHEEL_RADIUS)
        self.declare_parameter("wheel_base_m",    WHEEL_BASE)
        self.declare_parameter("poll_hz",         POLL_HZ)

        bus      = self.get_parameter("i2c_bus").value
        addr     = self.get_parameter("i2c_addr").value
        lpin     = self.get_parameter("left_pin").value
        rpin     = self.get_parameter("right_pin").value
        self._cpr     = self.get_parameter("counts_per_rev").value
        self._radius  = self.get_parameter("wheel_radius_m").value
        self._base    = self.get_parameter("wheel_base_m").value
        poll_hz       = self.get_parameter("poll_hz").value

        # Derived
        self._dist_per_tick = (2 * math.pi * self._radius) / self._cpr

        # Odometry state
        self._x   = 0.0
        self._y   = 0.0
        self._yaw = 0.0
        self._prev_time = self.get_clock().now()

        # Hardware
        self.get_logger().info(
            f"Opening PCF8574  bus={bus}  addr=0x{addr:02X}  "
            f"L=P{lpin}  R=P{rpin}  poll={poll_hz}Hz"
        )
        self._enc = EncoderReader(bus, addr, lpin, rpin, poll_hz)
        self._enc.start()

        # Publishers
        self._pub_ticks = self.create_publisher(
            Int32MultiArray, "/nxp_cup/wheel_ticks", 10)
        self._pub_odom  = self.create_publisher(
            Odometry, "/nxp_cup/wheel_odom", 10)

        # TF broadcaster
        self._tf_br = tf2_ros.TransformBroadcaster(self)

        # Timer
        self.create_timer(1.0 / PUBLISH_HZ, self._publish)
        self.get_logger().info("Encoder node ready.")

    # ── Timer callback ────────────────────────────────────────────────────────

    def _publish(self):
        now = self.get_clock().now()
        dt  = (now - self._prev_time).nanoseconds * 1e-9
        self._prev_time = now
        if dt <= 0:
            return

        dl_ticks, dr_ticks = self._enc.get_and_reset()

        # Distances
        dl = dl_ticks * self._dist_per_tick
        dr = dr_ticks * self._dist_per_tick

        # Differential drive kinematics
        d_center = (dl + dr) / 2.0
        d_yaw    = (dr - dl) / self._base

        self._yaw += d_yaw
        self._x   += d_center * math.cos(self._yaw)
        self._y   += d_center * math.sin(self._yaw)

        v_linear  = d_center / dt
        v_angular = d_yaw    / dt

        # ── wheel_ticks ───────────────────────────────────────────────────────
        tick_msg = Int32MultiArray()
        tick_msg.data = [int(dl_ticks), int(dr_ticks)]
        self._pub_ticks.publish(tick_msg)

        # ── Odometry ──────────────────────────────────────────────────────────
        odom = Odometry()
        odom.header.stamp    = now.to_msg()
        odom.header.frame_id = "odom"
        odom.child_frame_id  = "base_link"

        odom.pose.pose.position.x = self._x
        odom.pose.pose.position.y = self._y
        odom.pose.pose.position.z = 0.0

        cy, sy = math.cos(self._yaw / 2), math.sin(self._yaw / 2)
        odom.pose.pose.orientation.z = sy
        odom.pose.pose.orientation.w = cy

        odom.twist.twist.linear.x  = v_linear
        odom.twist.twist.angular.z = v_angular

        # Diagonal covariance — tuned to encoder noise level
        p_cov = [0.01, 0, 0, 0, 0, 0,
                 0, 0.01, 0, 0, 0, 0,
                 0, 0, 1e9, 0, 0, 0,
                 0, 0, 0, 1e9, 0, 0,
                 0, 0, 0, 0, 1e9, 0,
                 0, 0, 0, 0, 0, 0.05]
        odom.pose.covariance  = p_cov
        odom.twist.covariance = p_cov
        self._pub_odom.publish(odom)

        # ── TF odom → base_link ───────────────────────────────────────────────
        tf = TransformStamped()
        tf.header.stamp    = now.to_msg()
        tf.header.frame_id = "odom"
        tf.child_frame_id  = "base_link"
        tf.transform.translation.x = self._x
        tf.transform.translation.y = self._y
        tf.transform.rotation.z    = sy
        tf.transform.rotation.w    = cy
        self._tf_br.sendTransform(tf)

    def destroy_node(self):
        self._enc.stop()
        super().destroy_node()


# ─────────────────────────────────────────────────────────────────────────────

def main():
    rclpy.init()
    node = EncoderNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()