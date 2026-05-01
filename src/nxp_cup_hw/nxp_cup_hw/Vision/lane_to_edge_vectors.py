#!/usr/bin/env python3
"""
lane_to_edge_vectors.py
-----------------------
Bridges the hw vision pipeline output to the format expected by b3rb_ros_mrac.

  Subscribes : /nxp_cup/lane_chains   std_msgs/String  (JSON from vision_chain)
  Publishes  : /edge_vectors           synapse_msgs/EdgeVectors

The MRAC only needs two points per lane edge — it interpolates x at two
lookahead depths (y_far, y_near).  We give it the bottom point (nearest,
index 0 of the chain = front of car) and top point (farthest, last index)
of each chain.  The MRAC's ordered_vector_points() handles y-ordering
internally so the order here does not matter.

EdgeVectors field mapping
-------------------------
  image_width  = BEV_W from JSON  (400 px default)
  image_height = BEV_H from JSON  (300 px default)
  vector_count = 0 / 1 / 2  depending on how many valid chains are present
  vector_1     = LEFT  edge  [bottom_pt, top_pt]
  vector_2     = RIGHT edge  [bottom_pt, top_pt]

  Point.x = pixel column in BEV space
  Point.y = pixel row   in BEV space  (0 = top, BEV_H = bottom/nearest)
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


def _chain_endpoints(chain: list) -> tuple[Point, Point]:
    """
    Return (bottom_pt, top_pt) as geometry_msgs/Point from a chain list.

    Chain convention (from vision_chain.py):
        index 0  = nearest strip  (bottom of BEV, front of car, high y value)
        index -1 = farthest strip (top of BEV, far ahead,       low y value)
    """
    bottom = Point()
    bottom.x = float(chain[0][0])
    bottom.y = float(chain[0][1])

    top = Point()
    top.x = float(chain[-1][0])
    top.y = float(chain[-1][1])

    return bottom, top


class LaneToEdgeVectorsNode(Node):

    def __init__(self):
        super().__init__("lane_to_edge_vectors")

        self._sub = self.create_subscription(
            String, "/nxp_cup/lane_chains", self._cb, 10
        )
        self._pub = self.create_publisher(EdgeVectors, "/edge_vectors", 10)

        self.get_logger().info("lane_to_edge_vectors bridge ready")

    def _cb(self, msg: String):
        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError as e:
            self.get_logger().warn(f"JSON parse error: {e}", throttle_duration_sec=1.0)
            return

        left  = data.get("left",  [])
        right = data.get("right", [])
        bev_w = int(data.get("bev_w", 400))
        bev_h = int(data.get("bev_h", 300))

        valid_left  = len(left)  >= 2
        valid_right = len(right) >= 2

        ev = EdgeVectors()
        ev.image_width  = bev_w
        ev.image_height = bev_h
        ev.vector_count = 0

        if valid_left and valid_right:
            b, t = _chain_endpoints(left)
            ev.vector_1[0], ev.vector_1[1] = b, t

            b, t = _chain_endpoints(right)
            ev.vector_2[0], ev.vector_2[1] = b, t

            ev.vector_count = 2

        elif valid_left:
            b, t = _chain_endpoints(left)
            ev.vector_1[0], ev.vector_1[1] = b, t
            ev.vector_count = 1

        elif valid_right:
            # MRAC expects vector_1 for a single-edge case
            b, t = _chain_endpoints(right)
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