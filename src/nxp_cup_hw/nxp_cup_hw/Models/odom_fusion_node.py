#!/usr/bin/env python3
"""
odom_fusion_node.py
-------------------
Fuses wheel encoder odometry with MPU6050 IMU using a complementary filter.

Subscriptions
-------------
  /nxp_cup/encoder_odom   nav_msgs/Odometry   (from pcf8574ap_node)
  /nxp_cup/imu            sensor_msgs/Imu     (from mpu6050_node)

Publications
------------
  /cerebri/out/odometry   nav_msgs/Odometry   → consumed by b3rb_ros_mrac
  /nxp_cup/wheel_odom     nav_msgs/Odometry   → consumed by vision_stream

Fusion strategy
---------------
Forward velocity (vx):
  Complementary filter:
    vx = ALPHA * vx_encoder + (1-ALPHA) * (vx_prev + ax * dt)
  Encoder is trusted for low-frequency absolute value;
  accelerometer fills in high-frequency transients between encoder updates.

Yaw rate (r):
  Taken directly from gyro gz — far superior to differential encoder estimate.
  This fixes r_recon=0.000 in the MRAC.

Pose:
  Integrated from fused vx and gyro gz.
"""

import math
import time

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from geometry_msgs.msg import TransformStamped
import tf2_ros

# ── Tuning ────────────────────────────────────────────────────────────────────
# Higher ALPHA → trust encoder more (smoother but slower response)
# Lower  ALPHA → trust accelerometer more (faster but noisier)
ALPHA = 0.85


class OdomFusionNode(Node):

    def __init__(self):
        super().__init__("odom_fusion")

        self.declare_parameter("use_imu", True)
        self._use_imu = self.get_parameter("use_imu").value

        # ── State ─────────────────────────────────────────────────────────────
        self._vx_fused   = 0.0
        self._gz_fused   = 0.0
        self._ax         = 0.0
        self._last_imu_t = None
        self._last_enc_t = None

        self._x     = 0.0
        self._y     = 0.0
        self._theta = 0.0
        self._last_pose_t = self.get_clock().now()

        # ── Subscriptions ─────────────────────────────────────────────────────
        self.create_subscription(
            Odometry, "/nxp_cup/encoder_odom", self._enc_cb, 10
        )
        if self._use_imu:
            self.create_subscription(
                Imu, "/nxp_cup/imu", self._imu_cb, 10
            )

        # ── Publishers ────────────────────────────────────────────────────────
        self._mrac_pub  = self.create_publisher(Odometry, "/cerebri/out/odometry", 10)
        self._dash_pub  = self.create_publisher(Odometry, "/nxp_cup/wheel_odom",   10)
        self._tf_br     = tf2_ros.TransformBroadcaster(self)

        # ── Publish timer at 50 Hz ────────────────────────────────────────────
        self.create_timer(0.02, self._publish_cb)

        self.get_logger().info(
            f"Odometry fusion node ready  use_imu={self._use_imu}"
        )

    # ── IMU callback — update ax and gz ──────────────────────────────────────
    def _imu_cb(self, msg: Imu):
        try:
            ax = float(msg.linear_acceleration.x)
            gz = float(msg.angular_velocity.z)
            if math.isfinite(ax) and math.isfinite(gz):
                self._ax = ax
                # Low-pass filter on yaw rate to suppress gyro noise
                # alpha=0.3: trust new sample 30%, history 70%
                self._gz_fused = 0.3 * gz + 0.7 * self._gz_fused
                self._last_imu_t = time.monotonic()
        except Exception as e:
            self.get_logger().warn(f'IMU msg error: {e}', throttle_duration_sec=1.0)

    # ── Encoder callback — complementary filter on vx ────────────────────────
    def _enc_cb(self, msg: Odometry):
        try:
            now = time.monotonic()
            vx_enc = float(msg.twist.twist.linear.x)

            if self._last_enc_t is None:
                self._vx_fused = vx_enc
            else:
                dt = now - self._last_enc_t
                if 0 < dt < 0.5:
                    vx_accel = self._vx_fused + self._ax * dt
                    self._vx_fused = ALPHA * vx_enc + (1.0 - ALPHA) * vx_accel

            self._last_enc_t = now
        except Exception as e:
            self.get_logger().warn(f'Enc msg error: {e}', throttle_duration_sec=1.0)

    # ── Publish fused odometry ────────────────────────────────────────────────
    def _publish_cb(self):
        now_ros = self.get_clock().now()
        dt = (now_ros - self._last_pose_t).nanoseconds * 1e-9
        self._last_pose_t = now_ros

        vx = self._vx_fused
        gz = self._gz_fused

        # Integrate pose
        self._x     += vx * math.cos(self._theta) * dt
        self._y     += vx * math.sin(self._theta) * dt
        self._theta += gz * dt

        half_th = self._theta / 2.0

        odom = Odometry()
        odom.header.stamp      = now_ros.to_msg()
        odom.header.frame_id   = "odom"
        odom.child_frame_id    = "base_link"

        odom.pose.pose.position.x    = self._x
        odom.pose.pose.position.y    = self._y
        odom.pose.pose.orientation.z = math.sin(half_th)
        odom.pose.pose.orientation.w = math.cos(half_th)

        odom.twist.twist.linear.x  = vx
        odom.twist.twist.angular.z = gz   # ← gyro gz, not encoder diff

        self._mrac_pub.publish(odom)
        self._dash_pub.publish(odom)

        # TF
        tf_msg = TransformStamped()
        tf_msg.header.stamp    = now_ros.to_msg()
        tf_msg.header.frame_id = "odom"
        tf_msg.child_frame_id  = "base_link"
        tf_msg.transform.translation.x = self._x
        tf_msg.transform.translation.y = self._y
        tf_msg.transform.rotation.z    = math.sin(half_th)
        tf_msg.transform.rotation.w    = math.cos(half_th)
        self._tf_br.sendTransform(tf_msg)


def main(args=None):
    rclpy.init(args=args)
    node = OdomFusionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()