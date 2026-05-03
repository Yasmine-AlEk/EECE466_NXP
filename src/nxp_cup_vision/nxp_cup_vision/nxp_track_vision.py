#!/usr/bin/env python3
"""
nxp_track_vision.py  (patched for synapse_msgs without PixyVector)
-------------------------------------------------------------------
Original: NXP / Benjamin Perseghetti (rudislabs.com)
Patch   : removed PixyVector + Status imports (not in this synapse_msgs build).
          Publishes EdgeVectors directly on /edge_vectors — no bridge needed.

Detection pipeline (unchanged from original):
  /camera/image_raw/compressed
    → perspective warp (320×240)
    → THRESH_BINARY_INV @ 80
    → contour find → fitLine on 2 largest contours
    → clip to image bounds
    → remap to BEV space (400×300) for EdgeVectors

Publishes:
  /edge_vectors          synapse_msgs/EdgeVectors
  /nxp_cup/debug_image   sensor_msgs/CompressedImage  (debug=True only)
"""

import copy
import cv2
import numpy as np
import rclpy
from rclpy.clock import ROSClock
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rcl_interfaces.msg import ParameterDescriptor, ParameterType
from geometry_msgs.msg import Point
import sensor_msgs.msg
from cv_bridge import CvBridge
from synapse_msgs.msg import EdgeVectors

if cv2.__version__ < "4.0.0":
    raise ImportError(f"Requires opencv >= 4.0, found {cv2.__version__}")

BEV_W = 400
BEV_H = 300

# Calibrated source points (tune via bev_calibrator tool)
BEV_SRC_POINTS = np.float32([
    [4.0, 187.0],   # bottom-left
    [630.0, 187.0],   # bottom-right
    [484.0, 100.0],   # top-right
    [113.0, 100.0],   # top-left
])

BEV_DST_POINTS = np.float32([
    [0.0,         BEV_H - 1.0],
    [BEV_W - 1.0, BEV_H - 1.0],
    [BEV_W - 1.0, 0.0],
    [0.0,         0.0],
])


def _to_bev(px, py, img_w, img_h):
    p = Point()
    p.x = float(px) / img_w * BEV_W
    p.y = float(py) / img_h * BEV_H
    return p


class NXPTrackVision(Node):

    def __init__(self):
        super().__init__("nxp_track_vision")
        self.bridge = CvBridge()
        self.maskRectRatioWidthHeight = np.array([0.0, 0.0])

        self.declare_parameter("debug", True,
            ParameterDescriptor(type=ParameterType.PARAMETER_BOOL,
                                description='Publish debug image'))
        self.debug = self.get_parameter("debug").value

        self.imageHeight = 240
        self.imageWidth  = 320


        self.imageSub = self.create_subscription(
            sensor_msgs.msg.CompressedImage,
            'camera/image_raw/compressed',
            self.pixyImageCallback,
            qos_profile_sensor_data
        )
        self.edgeVectorsPub = self.create_publisher(EdgeVectors, "/edge_vectors", 10)
        if self.debug:
            self.debugPub = self.create_publisher(
                sensor_msgs.msg.CompressedImage, "nxp_cup/debug_image", 10)

        self.lineMethodsUsedCount = [0] * 7
        self.sortRightToLeft = False
        self.get_logger().info(
            f"NXPTrackVision ready  debug={self.debug}  → /edge_vectors")

    def findLines(self, img):
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        h, w = gray.shape[:2]

        thresh = cv2.bitwise_not(
            cv2.threshold(gray, 80, 255, cv2.THRESH_BINARY)[1])

        # Vehicle mask
        mask = np.ones(thresh.shape, dtype="uint8") * 255
        tl = (int(w*(1-self.maskRectRatioWidthHeight[0])/2),
              int(h*(1-self.maskRectRatioWidthHeight[1])))
        br = (int(w*(1+self.maskRectRatioWidthHeight[0])/2), int(h))
        mask = cv2.rectangle(mask, tl, br, 0, -1)
        masked = cv2.bitwise_and(thresh, thresh, mask=mask)

        cnts, _ = cv2.findContours(masked.copy(),
                                    cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        debug_img = img.copy()
        # Filter contours:
        #   - min area: ignore noise
        #   - max area: reject background blobs (walls/ceiling fill huge areas)
        #   - min height: track lines are tall relative to their width
        img_area = h * w
        valid = []
        for c in cnts:
            area = cv2.contourArea(c)
            if area < 100:                    # too small — noise
                continue
            if area > img_area * 0.15:        # too large — background blob
                continue
            x, y, cw, ch = cv2.boundingRect(c)
            if ch < h * 0.10:                 # too short — not a lane line
                continue
            valid.append(c)

        maxCnt = min(2, len(valid))
        cnt = sorted(valid, key=cv2.contourArea, reverse=True)[:maxCnt]
        if len(cnt) > 1:
            bbs = [cv2.boundingRect(c) for c in cnt]
            cnt, bbs = zip(*sorted(zip(cnt, bbs),
                                    key=lambda b: b[1][0],
                                    reverse=self.sortRightToLeft))

        lines = []
        for cn in cnt:
            if self.debug:
                cv2.fillPoly(debug_img, pts=[cn], color=(0, 0, 255))

            [vx, vy, lx, ly] = cv2.fitLine(cn, cv2.DIST_L2, 0, 0.01, 0.01)
            if lx < 1.0 or ly < 1.0:
                continue
            if vy == 0: vy = 1e-4
            if vx == 0: vx = 1e-4

            x0 = int((-ly*vx/vy) + lx);    y0 = 0
            x1 = int(((h-ly)*vx/vy) + lx); y1 = h
            m = 0

            if x0 <= 0 and x1 > w:
                m=1; y0=int((-lx*vy/vx)+ly); y1=int(((w-lx)*vy/vx)+ly); x0=0; x1=w
            elif x0 > w and x1 < 0:
                m=2; y0=int(((w-lx)*vy/vx)+ly); y1=int((-lx*vy/vx)+ly); x0=w; x1=0
            elif x0 <= 0 and x1 < w:
                m=3; y0=int((-lx*vy/vx)+ly); y1=h; x0=0; x1=int(((h-ly)*vx/vy)+lx)
            elif x0 > 0 and x1 > w:
                m=4; y0=0; y1=int(((w-lx)*vy/vx)+ly); x0=int((-ly*vx/vy)+lx); x1=w
            elif x0 > w and x1 > 0:
                m=5; y0=h; y1=int(((w-lx)*vy/vx)+ly); x0=int(((h-ly)*vx/vy)+lx); x1=w
            elif x0 < w and x1 < 0:
                m=6; y0=int((-lx*vy/vx)+ly); y1=0; x0=0; x1=int(ly*(-vx/vy)+lx)

            x0=max(0,min(w,x0)); x1=max(0,min(w,x1))
            y0=max(0,min(h,y0)); y1=max(0,min(h,y1))
            self.lineMethodsUsedCount[m] += 1
            lines.append((x0, y0, x1, y1, w, h))

        # Publish EdgeVectors
        ev = EdgeVectors()
        ev.image_width  = BEV_W
        ev.image_height = BEV_H
        ev.vector_count = 0
        for i, (x0, y0, x1, y1, iw, ih) in enumerate(lines[:2]):
            if i == 0:
                ev.vector_1[0] = _to_bev(x0, y0, iw, ih)
                ev.vector_1[1] = _to_bev(x1, y1, iw, ih)
            else:
                ev.vector_2[0] = _to_bev(x0, y0, iw, ih)
                ev.vector_2[1] = _to_bev(x1, y1, iw, ih)
            ev.vector_count += 1
        self.edgeVectorsPub.publish(ev)

        # Debug overlay
        if self.debug:
            cv2.rectangle(debug_img, tl, br, (128, 128, 0), 3)
            for (x0, y0, x1, y1, *_) in lines:
                cv2.line(debug_img, (x0, y0), (x1, y1), (255, 128, 128), 2)

        return debug_img

    def pixyImageCallback(self, data):
        scene = self.bridge.compressed_imgmsg_to_cv2(data, desired_encoding='bgr8')
        scene = cv2.rotate(scene, cv2.ROTATE_180)
        M = cv2.getPerspectiveTransform(BEV_SRC_POINTS, BEV_DST_POINTS)
        scene = cv2.warpPerspective(scene, M, (BEV_W, BEV_H))
        result = self.findLines(copy.deepcopy(scene))
        if self.debug:
            msg = self.bridge.cv2_to_compressed_imgmsg(result)
            msg.header.stamp = data.header.stamp
            self.debugPub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = NXPTrackVision()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()