#!/usr/bin/env python3
"""
NXP Cup – Bicycle Model Node
==============================
Receives a (v, w) velocity command — linear speed and angular rate — and
applies the kinematic bicycle model to produce:
  • A normalised steering angle  → servo node  (via /nxp_cup/cmd_safe .angular.z)
  • A normalised wheel speed     → BLDC node   (via /nxp_cup/cmd_safe .linear.x)

Kinematic bicycle model
-----------------------
  Steering angle:   δ = arctan( w · L / v )
  where
    v  = commanded forward speed   [m/s]
    w  = commanded angular rate    [rad/s]
    L  = wheelbase                 [m]   (front axle to rear axle)

  When v ≈ 0 the formula is undefined; in that case δ is held at the
  last valid value and speed is set to zero.

  The resulting δ is clamped to ±δ_max, then normalised to [-1, 1]
  for the servo node.  v is clamped to ±v_max and normalised to [-1, 1]
  for the BLDC node.

Subscribes
----------
  /nxp_cup/cmd           geometry_msgs/TwistStamped
      .twist.linear.x    v  [m/s]       forward speed command
      .twist.angular.z   w  [rad/s]     yaw rate command

Publishes
---------
  /nxp_cup/cmd_safe      geometry_msgs/TwistStamped
      .twist.linear.x    normalised speed   [-1 … +1]  → BLDC node
      .twist.angular.z   normalised steer   [-1 … +1]  → servo node

  /nxp_cup/bicycle_debug  std_msgs/String   (JSON)
      { "v_cmd": float,   "w_cmd": float,
        "delta_deg": float, "delta_norm": float,
        "speed_norm": float, "v_min_active": bool }

Parameters
----------
  wheelbase_m       float   0.165   Distance front-to-rear axle [m]
  max_steer_deg     float   30.0    Physical steering limit [degrees]
  max_speed_ms      float   2.0     Speed at normalised cmd = 1.0 [m/s]
  v_deadzone_ms     float   0.05    |v| below this → speed=0, hold last δ
  publish_hz        int     50      Output rate [Hz]
  timeout_sec       float   0.5     Zero output if no cmd received within this

  steer_sign        int     1       Flip to -1 if servo direction is reversed
  speed_sign        int     1       Flip to -1 if motor direction is reversed
"""

import math
import json
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from std_msgs.msg import String

# ─────────────────────────────────────────────────────────────────────────────
#  DEFAULTS  (all overridable via ROS 2 parameters)
# ─────────────────────────────────────────────────────────────────────────────

WHEELBASE_M    = 0.168   # metres — measure front-axle to rear-axle
MAX_STEER_DEG  = 30.0    # physical steering limit
MAX_SPEED_MS   = 2.0     # m/s at full normalised command
V_DEADZONE     = 0.05    # m/s — below this v is treated as zero
PUBLISH_HZ     = 50
TIMEOUT_SEC    = 0.5


# ─────────────────────────────────────────────────────────────────────────────
#  NODE
# ─────────────────────────────────────────────────────────────────────────────

class BicycleModelNode(Node):

    def __init__(self):
        super().__init__("nxp_bicycle_model_node")

        # ── Parameters ───────────────────────────────────────────────────────
        self.declare_parameter("wheelbase_m",    WHEELBASE_M)
        self.declare_parameter("max_steer_deg",  MAX_STEER_DEG)
        self.declare_parameter("max_speed_ms",   MAX_SPEED_MS)
        self.declare_parameter("v_deadzone_ms",  V_DEADZONE)
        self.declare_parameter("publish_hz",     PUBLISH_HZ)
        self.declare_parameter("timeout_sec",    TIMEOUT_SEC)
        self.declare_parameter("steer_sign",     1)
        self.declare_parameter("speed_sign",     1)

        self._L          = self.get_parameter("wheelbase_m").value
        self._delta_max  = math.radians(
                            self.get_parameter("max_steer_deg").value)
        self._v_max      = self.get_parameter("max_speed_ms").value
        self._v_dz       = self.get_parameter("v_deadzone_ms").value
        self._hz         = self.get_parameter("publish_hz").value
        self._timeout    = self.get_parameter("timeout_sec").value
        self._steer_sign = float(self.get_parameter("steer_sign").value)
        self._speed_sign = float(self.get_parameter("speed_sign").value)

        # ── State ─────────────────────────────────────────────────────────────
        self._v_cmd      = 0.0
        self._w_cmd      = 0.0
        self._last_delta = 0.0    # hold last valid steering angle
        self._last_cmd_t = time.monotonic()

        # ── ROS interfaces ────────────────────────────────────────────────────
        self.create_subscription(
            TwistStamped, "/nxp_cup/cmd", self._on_cmd, 10)

        self._pub_cmd   = self.create_publisher(
            TwistStamped, "/nxp_cup/cmd_safe", 10)
        self._pub_debug = self.create_publisher(
            String, "/nxp_cup/bicycle_debug", 10)

        self.create_timer(1.0 / self._hz, self._publish)

        self.get_logger().info(
            f"Bicycle model ready  "
            f"L={self._L}m  δ_max=±{math.degrees(self._delta_max):.1f}°  "
            f"v_max={self._v_max}m/s"
        )

    # ── Subscriber callback ───────────────────────────────────────────────────

    def _on_cmd(self, msg: TwistStamped):
        self._v_cmd      = float(msg.twist.linear.x)
        self._w_cmd      = float(msg.twist.angular.z)
        self._last_cmd_t = time.monotonic()

    # ── Timer: solve bicycle model and publish ────────────────────────────────

    def _publish(self):
        # Safety timeout
        stale = (time.monotonic() - self._last_cmd_t) > self._timeout
        if stale:
            v, w = 0.0, 0.0
        else:
            v, w = self._v_cmd, self._w_cmd

        # ── Bicycle model ─────────────────────────────────────────────────────
        #
        #   Rear-axle reference point convention:
        #     v   = longitudinal speed of the rear axle midpoint
        #     w   = yaw rate of the body
        #     δ   = front wheel steering angle
        #
        #   Relationship:  w = v · tan(δ) / L
        #   Inverse:        δ = arctan( w · L / v )
        #
        v_min_active = abs(v) < self._v_dz

        if v_min_active:
            # Speed too low for valid angle computation.
            # Keep the last valid δ; command zero speed.
            delta     = self._last_delta
            speed_out = 0.0
        else:
            # Clamp the argument before arctan to avoid unrealisable angles
            # that exceed the physical steering limit.
            tan_delta = (w * self._L) / v
            # arctan is always valid; just clamp the result to the physical limit
            delta = math.atan(tan_delta)
            delta = max(-self._delta_max, min(self._delta_max, delta))
            self._last_delta = delta

            # Speed: clamp to ±v_max, preserve sign (reverse is allowed)
            v_clamped = max(-self._v_max, min(self._v_max, v))
            speed_out = v_clamped / self._v_max   # normalise to [-1, 1]

        # Normalise steering angle to [-1, 1] for servo node
        steer_out = delta / self._delta_max       # [-1, 1]

        # Apply direction signs (in case hardware is wired inverted)
        steer_out *= self._steer_sign
        speed_out *= self._speed_sign

        # ── Publish cmd_safe ──────────────────────────────────────────────────
        now = self.get_clock().now()

        cmd = TwistStamped()
        cmd.header.stamp    = now.to_msg()
        cmd.header.frame_id = "base_link"
        cmd.twist.linear.x  = speed_out
        cmd.twist.angular.z = steer_out
        self._pub_cmd.publish(cmd)

        # ── Publish debug JSON ────────────────────────────────────────────────
        dbg = String()
        dbg.data = json.dumps({
            "v_cmd":        round(v,         4),
            "w_cmd":        round(w,         4),
            "delta_deg":    round(math.degrees(delta), 3),
            "delta_norm":   round(steer_out, 4),
            "speed_norm":   round(speed_out, 4),
            "v_min_active": v_min_active,
            "stale":        stale,
        })
        self._pub_debug.publish(dbg)


# ─────────────────────────────────────────────────────────────────────────────

def main():
    rclpy.init()
    node = BicycleModelNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()