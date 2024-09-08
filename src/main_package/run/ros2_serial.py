import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import serial


class VelocityToSerial(Node):

    def __init__(self):
        super().__init__('velocity_to_serial')

        # Initialize serial connection
        self.ser = serial.Serial(
            '/dev/ttyACM0',
            9600,
            timeout=1)  # Adjust the port and baud rate as necessary

        # Create a subscription to the /cmd_vel topic
        self.subscription = self.create_subscription(Twist,
                                                     '/cmd_vel',
                                                     self.listener_callback,
                                                     10)

    def listener_callback(self, msg):
        # Format the data to a string suitable for serial communication
        formatted_data = self.format_data(msg)

        # Send the formatted data over serial
        if self.ser.is_open:
            self.ser.write(formatted_data.encode('utf-8'))
        else:
            self.get_logger().error('Serial port not open')

    def format_data(self, msg):
        # Extract linear and angular velocities
        linear_x = msg.linear.x
        linear_y = msg.linear.y
        linear_z = msg.linear.z
        angular_x = msg.angular.x
        angular_y = msg.angular.y
        angular_z = msg.angular.z

        # Create a string with the format:
        # "linear_x:linear_y:linear_z:angular_x:angular_y:angular_z\n"
        return f"{linear_x},{linear_y},{linear_z},{angular_x},{angular_y},{angular_z}\n"


def main(args=None):
    rclpy.init(args=args)
    node = VelocityToSerial()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.ser.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
