import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge, CvBridgeError
import ball_tracker.process_image as proc
import time


class DetectPerson(Node):

    def __init__(self):
        super().__init__('detect_person')

        self.get_logger().info('Looking for the person...')

        # Subscription to image topic
        self.image_sub = self.create_subscription(
            Image,
            "/camera/camera/color/image_raw",  # Make sure this is the correct topic for your RealSense camera
            self.image_callback,
            rclpy.qos.qos_profile_sensor_data
        )

        # Publishers
        self.image_out_pub = self.create_publisher(Image, "/image_out", 1)
        self.image_tuning_pub = self.create_publisher(Image,
                                                      "/image_tuning",
                                                      1)
        self.cmd_vel_pub = self.create_publisher(Twist, "/cmd_vel", 1)

        # Declare parameters
        self.declare_parameter("rcv_timeout_secs", 1.0)
        self.declare_parameter("angular_chase_multiplier", 0.5)
        self.declare_parameter("forward_chase_speed", 0.5)
        self.declare_parameter("search_angular_speed", 0.25)
        self.declare_parameter("max_size_thresh", 0.1)
        self.declare_parameter("filter_value", 0.9)

        # Retrieve parameters
        self.rcv_timeout_secs = self.get_parameter(
            "rcv_timeout_secs").get_parameter_value().double_value
        self.angular_chase_multiplier = self.get_parameter(
            "angular_chase_multiplier").get_parameter_value().double_value
        self.forward_chase_speed = self.get_parameter(
            "forward_chase_speed").get_parameter_value().double_value
        self.search_angular_speed = self.get_parameter(
            "search_angular_speed").get_parameter_value().double_value
        self.max_size_thresh = self.get_parameter(
            "max_size_thresh").get_parameter_value().double_value
        self.filter_value = self.get_parameter(
            "filter_value").get_parameter_value().double_value

        self.lastrcvtime = time.time() - 10000
        self.bridge = CvBridge()
        self.cmd_vel = Twist()

    def image_callback(self, data):
        try:
            # Convert the ROS image message to OpenCV format
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            self.get_logger().info(
                f"Image received with resolution: {cv_image.shape[1]}x{cv_image.shape[0]}"
            )
        except CvBridgeError as e:
            self.get_logger().error(f"Failed to convert image: {e}")
            return

        try:
            # Call your processing function to get bounding box (cx, cy, w, h)
            cx, cy, w, h = proc.find_bounding_boxes(cv_image)

            # Calculate control commands
            image_center_x = cv_image.shape[1] / 2
            image_center_y = cv_image.shape[0] / 2
            error_x = cx - image_center_x
            error_y = cy - image_center_y

            # linear_x = self.forward_chase_speed * (error_y / image_center_y)
            linear_x = self.forward_chase_speed * (cv_image.shape[0] - h -
                                                   20) / cv_image.shape[0]
            angular_z = -self.angular_chase_multiplier * (error_x /
                                                          image_center_x)

            # Publish velocity commands
            self.cmd_vel.linear.x = linear_x
            self.cmd_vel.angular.z = angular_z
            self.cmd_vel_pub.publish(self.cmd_vel)

        except Exception as e:
            self.get_logger().error(f"Failed during image processing: {e}")


def main(args=None):
    rclpy.init(args=args)
    detect_person = DetectPerson()

    try:
        rclpy.spin(detect_person)
    except KeyboardInterrupt:
        pass

    # Destroy node when finished
    detect_person.destroy_node()
    rclpy.shutdown()
