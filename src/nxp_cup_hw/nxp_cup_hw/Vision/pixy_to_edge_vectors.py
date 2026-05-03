#!/usr/bin/env python3
"""
pixy_to_edge_vectors.py
-----------------------
Bridges nxp_cup_vision output to the format expected by b3rb_ros_mrac.

  Subscribes : /cerebri/in/pixy_vector   synapse_msgs/PixyVector
  Publishes  : /edge_vectors              synapse_msgs/EdgeVectors

PixyVector coordinate space: 78 × 51 px  (pixyImageWidth × pixyImageHeight)
EdgeVectors coordinate space: BEV_W × BEV_H = 400 × 300 px

Remapping is a simple linear scale so the MRAC's ratio-based lookahead
(y_far=0.52, y_near=0.82 of image_height) lands on sensible rows.

PixyVector field convention (nxp_cup_vision):
  m0 = left  line:  m0_x0, m0_y0  (top point)   m0_x1, m0_y1  (bottom point)
  m1 = right line:  m1_x0, m1_y0  (top point)   m1_x1, m1_y1  (bottom point)
  A line is considered absent when all four of its fields are zero.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point

try:
    from synapse_msgs.msg import PixyVector, EdgeVectors
except ImportError as exc:
    raise SystemExit(
        "synapse_msgs not found — source your b3rb workspace"
    ) from exc

PIXY_W = 78
PIXY_H = 51
BEV_W  = 400
BEV_H  = 300


def _remap(px: int, py: int) -> Point:
    """Scale a Pixy-space point into BEV coordinate space."""
    p = Point()
    p.x = float(px) / PIXY_W * BEV_W
    p.y = float(py) / PIXY_H * BEV_H
    return p


def _line_valid(x0, y0, x1, y1) -> bool:
    """A line is valid if at least one endpoint is non-zero."""
    return (x0, y0, x1, y1) != (0, 0, 0, 0)


class PixyToEdgeVectorsNode(Node):

    def __init__(self):
        super().__init__("pixy_to_edge_vectors")

        self._sub = self.create_subscription(
            PixyVector, "/cerebri/in/pixy_vector", self._cb, 10
        )
        self._pub = self.create_publisher(EdgeVectors, "/edge_vectors", 10)

        self.get_logger().info(
            "pixy_to_edge_vectors ready  "
            "/cerebri/in/pixy_vector → /edge_vectors"
        )

    def _cb(self, msg: PixyVector):
        valid_m0 = _line_valid(msg.m0_x0, msg.m0_y0, msg.m0_x1, msg.m0_y1)
        valid_m1 = _line_valid(msg.m1_x0, msg.m1_y0, msg.m1_x1, msg.m1_y1)

        ev = EdgeVectors()
        ev.image_width  = BEV_W
        ev.image_height = BEV_H
        ev.vector_count = 0

        if valid_m0:
            # m0_x0/y0 = top point, m0_x1/y1 = bottom point
            ev.vector_1[0] = _remap(msg.m0_x0, msg.m0_y0)   # top  (far)
            ev.vector_1[1] = _remap(msg.m0_x1, msg.m0_y1)   # bottom (near)
            ev.vector_count += 1

        if valid_m1:
            ev.vector_2[0] = _remap(msg.m1_x0, msg.m1_y0)
            ev.vector_2[1] = _remap(msg.m1_x1, msg.m1_y1)
            ev.vector_count += 1

        self._pub.publish(ev)


def main(args=None):
    rclpy.init(args=args)
    node = PixyToEdgeVectorsNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()