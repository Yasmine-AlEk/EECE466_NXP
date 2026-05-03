#!/usr/bin/env python3
"""
lane_to_edge_vectors.py
-----------------------
Bridges the hw vision pipeline output to the format expected by b3rb_ros_mrac.

  Subscribes : /nxp_cup/edge_vectors   std_msgs/String  (JSON from vision_chain)
  Publishes  : /edge_vectors            synapse_msgs/EdgeVectors

JSON format from vision_chain:
  {
    "stamp": float,
    "img_w": int, "img_h": int,
    "left":  {"valid": bool, "x0": f, "y0": f, "dx": f, "dy": f,
               "triplet": [[x,y], [x,y], [x,y]]},
    "right": { ... }
  }

EdgeVectors field mapping
-------------------------
  image_width  = img_w from JSON
  image_height = img_h from JSON
  vector_count = 0 / 1 / 2
  vector_1     = LEFT  edge  [bottom_pt, top_pt]   (high-y = near, low-y = far)
  vector_2     = RIGHT edge  [bottom_pt, top_pt]

  Two points per side are extracted from the triplet: the y-lowest (far) and
  y-highest (near) triplet vertices.  The MRAC's ordered_vector_points()
  re-sorts internally so order here does not matter.
"""

import json

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Point

try:
    from synapse_msgs.msg import EdgeVectors
except ImportError as exc:
    raise SystemExit(
        "synapse_msgs not found — source your b3rb workspace or install the package"
    ) from exc


def _triplet_to_endpoints(triplet: list) -> tuple[Point, Point]:
    """
    Given 3 triplet points [[x,y], ...], return (bottom_pt, top_pt)
    where bottom = highest y (nearest, front of car)
           top   = lowest  y (farthest, far ahead)
    """
    sorted_pts = sorted(triplet, key=lambda p: p[1])  # ascending y = top first
    top_raw    = sorted_pts[0]
    bottom_raw = sorted_pts[-1]

    top = Point()
    top.x = float(top_raw[0])
    top.y = float(top_raw[1])

    bottom = Point()
    bottom.x = float(bottom_raw[0])
    bottom.y = float(bottom_raw[1])

    return bottom, top


class LaneToEdgeVectorsNode(Node):

    def __init__(self):
        super().__init__("lane_to_edge_vectors")

        self._sub = self.create_subscription(
            String, "/edge_vectors", self._cb, 10
        )
        self._pub = self.create_publisher(EdgeVectors, "/edge_vectors", 10)

        self.get_logger().info("lane_to_edge_vectors bridge ready")

    def _cb(self, msg: String):
        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError as e:
            self.get_logger().warn(f"JSON parse error: {e}", throttle_duration_sec=1.0)
            return

        left_d  = data.get("left",  {})
        right_d = data.get("right", {})
        img_w   = int(data.get("img_w", 640))
        img_h   = int(data.get("img_h", 480))

        valid_left  = left_d.get("valid",  False) and len(left_d.get("triplet",  [])) >= 2
        valid_right = right_d.get("valid", False) and len(right_d.get("triplet", [])) >= 2

        ev = EdgeVectors()
        ev.image_width  = img_w
        ev.image_height = img_h
        ev.vector_count = 0

        if valid_left and valid_right:
            b, t = _triplet_to_endpoints(left_d["triplet"])
            ev.vector_1[0], ev.vector_1[1] = b, t

            b, t = _triplet_to_endpoints(right_d["triplet"])
            ev.vector_2[0], ev.vector_2[1] = b, t

            ev.vector_count = 2

        elif valid_left:
            b, t = _triplet_to_endpoints(left_d["triplet"])
            ev.vector_1[0], ev.vector_1[1] = b, t
            ev.vector_count = 1

        elif valid_right:
            b, t = _triplet_to_endpoints(right_d["triplet"])
            ev.vector_1[0], ev.vector_1[1] = b, t
            ev.vector_count = 1

        self._pub.publish(ev)


def main(args=None):
    rclpy.init(args=args)
    node = LaneToEdgeVectorsNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()