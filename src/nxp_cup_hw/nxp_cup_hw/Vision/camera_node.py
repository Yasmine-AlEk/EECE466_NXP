#!/usr/bin/env python3
"""
camera_node.py
--------------
Publishes /camera/image_raw/compressed (sensor_msgs/CompressedImage)
from the NavQPlus CSI camera via GStreamer.

This is a drop-in replacement for v4l2_camera_node on hardware where
the v4l2_camera package fails (mxc-isi-cap driver).

Add to nxp_cup_hw/Vision/ and register in setup.py:
    "camera_node = nxp_cup_hw.Vision.camera_node:main"
"""

import sys
import threading
import time

import cv2
import numpy as np

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage

CAM_DEVICE   = "/dev/video3"
CAM_W        = 640
CAM_H        = 480
CAM_FPS      = 30
JPEG_QUALITY = 80
TOPIC        = "/camera/image_raw/compressed"


def _gst_pipeline(device, w, h, fps):
    return (
        f"v4l2src device={device} ! "
        f"video/x-raw,framerate={fps}/1,width={w},height={h} ! "
        f"videoconvert ! video/x-raw,format=BGR ! appsink drop=1"
    )


class CameraNode(Node):

    def __init__(self):
        super().__init__("camera_node")

        self.declare_parameter("device",       CAM_DEVICE)
        self.declare_parameter("width",        CAM_W)
        self.declare_parameter("height",       CAM_H)
        self.declare_parameter("fps",          CAM_FPS)
        self.declare_parameter("jpeg_quality", JPEG_QUALITY)

        device  = self.get_parameter("device").get_parameter_value().string_value
        width   = self.get_parameter("width").get_parameter_value().integer_value
        height  = self.get_parameter("height").get_parameter_value().integer_value
        fps     = self.get_parameter("fps").get_parameter_value().integer_value
        self._quality = self.get_parameter("jpeg_quality").get_parameter_value().integer_value
        self._enc_params = [cv2.IMWRITE_JPEG_QUALITY, self._quality]

        # Try GStreamer first, fall back to plain OpenCV
        pipe = _gst_pipeline(device, width, height, fps)
        self._cap = cv2.VideoCapture(pipe, cv2.CAP_GSTREAMER)
        if not self._cap.isOpened():
            self.get_logger().warn("GStreamer pipeline failed, trying plain OpenCV...")
            self._cap = cv2.VideoCapture(device)
            self._cap.set(cv2.CAP_PROP_FRAME_WIDTH,  width)
            self._cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
            self._cap.set(cv2.CAP_PROP_FPS,          fps)

        if not self._cap.isOpened():
            self.get_logger().fatal(f"Cannot open camera {device}")
            sys.exit(1)

        self._pub = self.create_publisher(CompressedImage, TOPIC, 10)
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()

        self.get_logger().info(
            f"CameraNode ready  {device} {width}x{height}@{fps}  → {TOPIC}"
        )

    def _loop(self):
        while rclpy.ok():
            ret, frame = self._cap.read()
            if not ret:
                time.sleep(0.02)
                continue

            ok, buf = cv2.imencode(".jpg", frame, self._enc_params)
            if not ok:
                continue

            msg = CompressedImage()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = "camera"
            msg.format = "jpeg"
            msg.data   = buf.tobytes()
            self._pub.publish(msg)

    def destroy_node(self):
        if self._cap.isOpened():
            self._cap.release()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CameraNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()