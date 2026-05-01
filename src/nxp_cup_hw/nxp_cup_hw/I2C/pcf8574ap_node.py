#!/usr/bin/env python3
"""
encoder_odometry_node.py
------------------------
ROS 2 node that reads two IR encoder signals from a PCF8574AP I/O expander
over I2C and publishes nav_msgs/Odometry.

Hardware mapping
----------------
  PCF8574AP physical pin 11  →  P6  (bit 6)  →  LEFT  wheel encoder
  PCF8574AP physical pin 12  →  P7  (bit 7)  →  RIGHT wheel encoder

  PCF8574AP I2C address: 0x38 (A0=A1=A2=GND, AP variant)

ROS topics published
--------------------
  /odom  (nav_msgs/Odometry)

TF broadcast
------------
  odom → base_link

Dependencies
------------
  pip install smbus2
  ros-<distro>-tf2-ros  (usually included)
"""

import math
import time

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
import tf2_ros

try:
    import smbus2
except ImportError as exc:
    raise SystemExit("smbus2 not found – run:  pip install smbus2") from exc


# ── Pin / bit constants ────────────────────────────────────────────────────────
_LEFT_BIT  = 6   # P6  → physical pin 11
_RIGHT_BIT = 7   # P7  → physical pin 12


class EncoderOdometryNode(Node):

    def __init__(self) -> None:
        super().__init__("encoder_odometry")

        # ── Parameters ────────────────────────────────────────────────────────
        self.declare_parameter("i2c_bus",          5)
        self.declare_parameter("i2c_address",      0x3b)   # PCF8574AP, A0-A2 = GND
        self.declare_parameter("holes_per_rev",    10)     # encoder disc holes
        self.declare_parameter("wheel_radius_m",   0.03675)  # metres
        self.declare_parameter("wheel_base_m",     0.160)  # metres, axle-to-axle
        self.declare_parameter("poll_rate_hz",     500.0)  # I2C poll frequency
        self.declare_parameter("publish_rate_hz",   50.0)  # odometry publish rate
        self.declare_parameter("velocity_timeout_s", 0.35) # zero-vel after this

        bus_num      = self.get_parameter("i2c_bus").value
        self.address = self.get_parameter("i2c_address").value
        self.holes   = self.get_parameter("holes_per_rev").value
        self.r       = self.get_parameter("wheel_radius_m").value
        self.b       = self.get_parameter("wheel_base_m").value
        poll_hz      = self.get_parameter("poll_rate_hz").value
        pub_hz       = self.get_parameter("publish_rate_hz").value
        self.v_timeout = self.get_parameter("velocity_timeout_s").value

        # ── I2C setup ─────────────────────────────────────────────────────────
        self.bus = smbus2.SMBus(bus_num)
        # Write 0xFF to set all pins as quasi-bidirectional inputs
        self.bus.write_byte(self.address, 0xFF)
        self.get_logger().info(
            f"PCF8574AP opened on i2c-{bus_num}, address=0x{self.address:02X}"
        )

        # ── Encoder state [left=0, right=1] ───────────────────────────────────
        self._bits         = [_LEFT_BIT, _RIGHT_BIT]
        self._last_state   = [None, None]
        self._last_edge_t  = [None, None]   # monotonic seconds of last rising edge
        self._rad_s        = [0.0,  0.0]    # current angular velocity estimate

        # ── Odometry pose integrator ──────────────────────────────────────────
        self._x     = 0.0
        self._y     = 0.0
        self._theta = 0.0
        self._last_odom_ros_t = self.get_clock().now()

        # ── ROS publisher + TF broadcaster ───────────────────────────────────
        self._odom_pub = self.create_publisher(Odometry, "odom", 10)
        self._tf_br    = tf2_ros.TransformBroadcaster(self)

        # ── Timers ────────────────────────────────────────────────────────────
        self.create_timer(1.0 / poll_hz, self._poll_cb)
        self.create_timer(1.0 / pub_hz,  self._publish_cb)

    # ── I2C poll ──────────────────────────────────────────────────────────────
    def _poll_cb(self) -> None:
        try:
            byte_val = self.bus.read_byte(self.address)
        except OSError as exc:
            self.get_logger().warn(
                f"I2C read failed: {exc}", throttle_duration_sec=1.0
            )
            return

        now = time.monotonic()

        for idx, bit in enumerate(self._bits):
            state = (byte_val >> bit) & 1

            # Initialise on first sample
            if self._last_state[idx] is None:
                self._last_state[idx]  = state
                self._last_edge_t[idx] = now
                continue

            # Rising edge → one complete slot has passed
            if self._last_state[idx] == 0 and state == 1:
                prev_t = self._last_edge_t[idx]
                if prev_t is not None:
                    dt = now - prev_t
                    if dt > 1e-6:                       # guard against glitches
                        # rad per rising edge = 2π / holes_per_rev
                        self._rad_s[idx] = (2.0 * math.pi / self.holes) / dt
                self._last_edge_t[idx] = now

            self._last_state[idx] = state

    # ── Odometry publish ──────────────────────────────────────────────────────
    def _publish_cb(self) -> None:
        now_ros = self.get_clock().now()
        dt = (now_ros - self._last_odom_ros_t).nanoseconds * 1e-9
        self._last_odom_ros_t = now_ros

        # Zero velocity if no rising edge has arrived recently
        mono_now = time.monotonic()
        for idx in range(2):
            last_t = self._last_edge_t[idx]
            if last_t is None or (mono_now - last_t) > self.v_timeout:
                self._rad_s[idx] = 0.0

        omega_l, omega_r = self._rad_s          # rad/s
        print("omega_l = ", omega_l)
        print("omega_r = ", omega_r)
        v_l = omega_l * self.r                  # linear speed left  wheel [m/s]
        v_r = omega_r * self.r                  # linear speed right wheel [m/s]

        v     = (v_r + v_l) / 2.0              # forward velocity [m/s]
        omega = (v_r - v_l) / self.b           # yaw rate [rad/s]

        # Integrate pose
        self._x     += v * math.cos(self._theta) * dt
        self._y     += v * math.sin(self._theta) * dt
        self._theta += omega * dt

        half_th = self._theta / 2.0

        # ── nav_msgs/Odometry ─────────────────────────────────────────────────
        odom = Odometry()
        odom.header.stamp      = now_ros.to_msg()
        odom.header.frame_id   = "odom"
        odom.child_frame_id    = "base_link"

        odom.pose.pose.position.x    = self._x
        odom.pose.pose.position.y    = self._y
        odom.pose.pose.orientation.z = math.sin(half_th)
        odom.pose.pose.orientation.w = math.cos(half_th)

        # Twist (body frame): forward vel + yaw rate
        odom.twist.twist.linear.x  = v
        odom.twist.twist.angular.z = omega

        self._odom_pub.publish(odom)

        # ── TF: odom → base_link ──────────────────────────────────────────────
        tf_msg = TransformStamped()
        tf_msg.header.stamp    = now_ros.to_msg()
        tf_msg.header.frame_id = "odom"
        tf_msg.child_frame_id  = "base_link"
        tf_msg.transform.translation.x = self._x
        tf_msg.transform.translation.y = self._y
        tf_msg.transform.rotation.z    = math.sin(half_th)
        tf_msg.transform.rotation.w    = math.cos(half_th)
        self._tf_br.sendTransform(tf_msg)

    # ── Cleanup ───────────────────────────────────────────────────────────────
    def destroy_node(self) -> None:
        self.bus.close()
        super().destroy_node()


# ── Entry point ───────────────────────────────────────────────────────────────
def main(args=None) -> None:
    rclpy.init(args=args)
    node = EncoderOdometryNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()