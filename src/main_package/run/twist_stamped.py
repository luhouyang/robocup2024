import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped


class CmdVelInspector(Node):

    def __init__(self):
        super().__init__('cmd_vel_inspector')

        # Create a single subscription for TwistStamped
        self.subscription = self.create_subscription(
            TwistStamped,
            '/omni_wheel_controller/cmd_vel',
            self.listener_callback,
            10)

    def listener_callback(self, msg):
        self.get_logger().info(f'Received TwistStamped: {msg}')


def main(args=None):
    rclpy.init(args=args)
    node = CmdVelInspector()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
