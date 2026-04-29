"""
edge_vectors_bridge.py
======================
Converts /edge_vectors  (synapse_msgs/EdgeVectors)
      -> /nxp_cup/lane_chains  (std_msgs/String, JSON)

so that runner_rl (PathMeasurementExtractor) can receive camera data.

JSON schema expected by PathMeasurementExtractor:
{
  "center": [[x_near, y_near], [x_far, y_far]],
  "left":   [[x_near, y_near]],   # only when left edge detected
  "right":  [[x_near, y_near]],   # only when right edge detected
  "valid":  {"left": bool, "right": bool}
}

EdgeVectors geometry (BEV image space, BEV_W=400, BEV_H=300):
  vector_1[0] = top  point (far  from car, small y)
  vector_1[1] = bottom point (near to car, large y)
  Left  vector: center_x < BEV_W/2 = 200
  Right vector: center_x >= BEV_W/2 = 200
"""

import json

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from synapse_msgs.msg import EdgeVectors

QOS_PROFILE_DEFAULT = 10

BEV_W = 400.0
BEV_H = 300.0
BEV_HALF = BEV_W / 2.0

# y rows used as placeholder in the JSON arrays (extractor only reads x)
Y_NEAR = BEV_H - 1.0   # bottom row  (closest to car)
Y_FAR  = 0.0            # top row     (lookahead)


class EdgeVectorsBridge(Node):

    def __init__(self):
        super().__init__('edge_vectors_bridge')

        self.sub = self.create_subscription(
            EdgeVectors,
            '/edge_vectors',
            self._cb,
            QOS_PROFILE_DEFAULT,
        )
        self.pub = self.create_publisher(
            String,
            '/nxp_cup/lane_chains',
            QOS_PROFILE_DEFAULT,
        )
        self.get_logger().info('[bridge] edge_vectors -> lane_chains active')

    def _cb(self, msg: EdgeVectors):
        n = int(msg.vector_count)

        left_near_x  = None
        left_far_x   = None
        right_near_x = None
        right_far_x  = None

        vectors = []
        if n >= 1:
            vectors.append(msg.vector_1)
        if n >= 2:
            vectors.append(msg.vector_2)

        for v in vectors:
            # v[0] = top/far point, v[1] = bottom/near point
            far_x  = float(v[0].x)
            near_x = float(v[1].x)
            center_x = (far_x + near_x) / 2.0

            if center_x < BEV_HALF:
                left_far_x  = far_x
                left_near_x = near_x
            else:
                right_far_x  = far_x
                right_near_x = near_x

        have_left  = left_near_x  is not None
        have_right = right_near_x is not None

        # --- compute centre line ---
        if have_left and have_right:
            center_near_x = (left_near_x + right_near_x) / 2.0
            center_far_x  = (left_far_x  + right_far_x)  / 2.0
        elif have_left:
            # mirror left edge across image centre to estimate centre
            center_near_x = BEV_HALF + (BEV_HALF - left_near_x)
            center_far_x  = BEV_HALF + (BEV_HALF - left_far_x)
        elif have_right:
            center_near_x = BEV_HALF - (right_near_x - BEV_HALF)
            center_far_x  = BEV_HALF - (right_far_x  - BEV_HALF)
        else:
            # no vectors — publish empty so extractor returns no measurement
            center_near_x = BEV_HALF
            center_far_x  = BEV_HALF

        payload = {
            "center": [[center_near_x, Y_NEAR], [center_far_x, Y_FAR]],
            "left":   [[left_near_x,   Y_NEAR]] if have_left  else [],
            "right":  [[right_near_x,  Y_NEAR]] if have_right else [],
            "valid":  {"left": have_left, "right": have_right},
        }

        # PathMeasurementExtractor returns no measurement when center is empty
        if not have_left and not have_right:
            payload["center"] = []

        out = String()
        out.data = json.dumps(payload)
        self.pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = EdgeVectorsBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
