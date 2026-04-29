import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import Joy

QOS_PROFILE_DEFAULT = 10


class GzCmdBridge(Node):
    """
    Bridges /nxp_cup/cmd_safe (TwistStamped) -> /cerebri/in/joy (Joy).

    Loaded only in the Gazebo SIL launch so the shared controller output
    reaches the cerebri autopilot running inside the simulator.
    """

    def __init__(self):
        super().__init__('gz_cmd_bridge')

        self.subscription = self.create_subscription(
            TwistStamped,
            '/nxp_cup/cmd_safe',
            self._cmd_callback,
            QOS_PROFILE_DEFAULT,
        )

        self.publisher = self.create_publisher(
            Joy,
            '/cerebri/in/joy',
            QOS_PROFILE_DEFAULT,
        )

    def _cmd_callback(self, msg: TwistStamped):
        joy = Joy()
        joy.buttons = [1, 0, 0, 0, 0, 0, 0, 1]
        joy.axes = [0.0, float(msg.twist.linear.x), 0.0, float(msg.twist.angular.z)]
        self.publisher.publish(joy)


def main(args=None):
    rclpy.init(args=args)
    node = GzCmdBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
