import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from .end_line_tracker import EndLineTracker
from .stop_line_tracker import StopLineTracker
import cv_bridge
import cv2
import numpy as np


class LineDetector(Node):
    def __init__(self, end_line_tracker: EndLineTracker, stop_line_tracker: StopLineTracker):
        super().__init__('line_detector')

        # Tracker instances (end line / stop line)
        self.end_line_tracker = end_line_tracker
        self.stop_line_tracker = stop_line_tracker

        # Subscription: which vehicle is selected
        self.car_info_subscription_ = self.create_subscription(
            String,
            'car_info',
            self.car_info_listener_callback,
            10
        )

        # Publisher: end-line detection result
        self.end_issue_publisher_ = self.create_publisher(
            String,
            'end_issue',
            10
        )

        # Publisher: stop-line detection result
        self.stop_issue_publisher_ = self.create_publisher(
            String,
            'stop_issue',
            10
        )

        # Publisher: obstacle detection result
        self.obstacle_issue_publisher_ = self.create_publisher(
            String,
            'obstacle_issue',
            10
        )

        # Camera subscription is created once the vehicle is known
        self.image_subscription_ = None
        self.bridge = cv_bridge.CvBridge()

        # Obstacle detection state flag
        self.obstacle_detected = False

    def car_info_listener_callback(self, msg: String):
        car = msg.data
        # Subscribe to the selected vehicle's camera topic
        self.image_subscription_ = self.create_subscription(
            Image,
            '/demo/' + car + '_camera/image_raw',
            self.image_callback,
            10
        )

    def image_callback(self, image: Image):
        # Convert the ROS image to an OpenCV BGR image
        img = self.bridge.imgmsg_to_cv2(image, desired_encoding='bgr8')

        # End-line detection
        self.end_line_tracker.process(img)
        if self.end_line_tracker.delta is not None and self.end_line_tracker.delta < 30:
            end_msg = String()
            end_msg.data = '종료'
            self.end_issue_publisher_.publish(end_msg)

        # Stop-line detection
        self.stop_line_tracker.process(img)
        if self.stop_line_tracker.delta is not None and self.stop_line_tracker.delta < 3.0:
            stop_msg = String()
            stop_msg.data = '정지'
            self.stop_issue_publisher_.publish(stop_msg)
        else:
            stop_msg = String()
            stop_msg.data = ''
            self.stop_issue_publisher_.publish(stop_msg)

        # Obstacle detection
        self.obstacle_detected = self.detect_obstacle(img)
        obstacle_msg = String()
        if self.obstacle_detected:
            obstacle_msg.data = '장애물 감지'
        else:
            obstacle_msg.data = ''
        self.obstacle_issue_publisher_.publish(obstacle_msg)

    def detect_obstacle(self, img):
        """
        Detect a red obstacle in the camera image.

        Red wraps around both ends of the HSV hue axis (0-10 and 170-180),
        so two masks are combined. The obstacle is reported when more than
        5,000 pixels are red.
        """
        # Convert to HSV so color can be thresholded independently of brightness
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

        # Red, lower hue band (0-10)
        lower_red1 = np.array([0, 120, 70])
        upper_red1 = np.array([10, 255, 255])
        red_mask1 = cv2.inRange(hsv, lower_red1, upper_red1)

        # Red, upper hue band (170-180)
        lower_red2 = np.array([170, 120, 70])
        upper_red2 = np.array([180, 255, 255])
        red_mask2 = cv2.inRange(hsv, lower_red2, upper_red2)

        # Combine both red hue bands
        red_mask = cv2.bitwise_or(red_mask1, red_mask2)

        # Pixels classified as obstacle
        obstacle_mask = red_mask

        # Report an obstacle when the red area is large enough
        obstacle_area = cv2.countNonZero(obstacle_mask)
        return obstacle_area > 5000  # detection threshold in pixels


def main(args=None):
    rclpy.init(args=args)

    # Create the trackers
    end_line_tracker = EndLineTracker()
    stop_line_tracker = StopLineTracker()

    # Run the LineDetector node
    node = LineDetector(end_line_tracker, stop_line_tracker)

    rclpy.spin(node)

    # Shut down
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()