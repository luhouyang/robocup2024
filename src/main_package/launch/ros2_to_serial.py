import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import serial
import time


class JointStateToSerial(Node):

    def __init__(self):
        super().__init__('joint_state_to_serial')

        # Initialize serial connection
        self.ser = serial.Serial(
            '/dev/ttyACM0',
            9600,
            timeout=1)  # Adjust the port and baud rate as necessary

        # Create a subscription to the /joint_states topic
        self.subscription = self.create_subscription(JointState,
                                                     '/joint_states',
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
        # Create a string with the format:
        # "name:position:velocity:effort\n"
        names = ','.join(
            msg.name
        )  # wheel0_shaft_joint,wheel0_shaft_joint,wheel0_shaft_joint,wheel0_shaft_joint
        positions = ','.join(
            map(str,
                msg.position)
        )  # 0.82152306607873,0.82152306607873,0.82152306607873,0.82152306607873
        velocities = ','.join(map(str, msg.velocity))
        efforts = ','.join(map(str, msg.effort))

        return f"{names}:{positions}:{velocities}:{efforts}\n"


def main(args=None):
    rclpy.init(args=args)
    node = JointStateToSerial()

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
